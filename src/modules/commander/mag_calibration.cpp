/****************************************************************************
 *
 *   Copyright (c) 2013-2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file mag_calibration.cpp
 *
 * Magnetometer calibration routine
 */

#include "mag_calibration.h"
#include "commander_helper.h"
#include "calibration_routines.h"
#include "calibration_messages.h"

#include <px4_platform_common/defines.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/time.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_tone_alarm.h>
#include <matrix/math.hpp>
#include <lib/conversion/rotation.h>
#include <lib/systemlib/mavlink_log.h>
#include <lib/parameters/param.h>
#include <lib/systemlib/err.h>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionBlocking.hpp>
#include <uORB/topics/sensor_mag.h>
#include <uORB/topics/sensor_gyro.h>

using namespace matrix;
using namespace time_literals;

static constexpr char sensor_name[] {"mag"};
static constexpr unsigned MAX_MAGS = 4;
static constexpr float mag_sphere_radius = 0.2f;
static constexpr unsigned int calibration_total_points = 240;		///< The total points per magnetometer
static constexpr unsigned int calibraton_duration_seconds = 42; 	///< The total duration the routine is allowed to take

static unsigned int calibration_sides = 6;			///< The total number of sides

calibrate_return mag_calibrate_all(orb_advert_t *mavlink_log_pub, int32_t cal_mask);

/// Data passed to calibration worker routine
typedef struct  {
	unsigned	done_count{0};
	unsigned int	calibration_points_perside;
	uint64_t	calibration_interval_perside_us;
	unsigned int	calibration_counter_total[MAX_MAGS];
	bool		side_data_collected[detect_orientation_side_count];
	float		*x[MAX_MAGS];
	float		*y[MAX_MAGS];
	float		*z[MAX_MAGS];
} mag_worker_data_t;

int do_mag_calibration(orb_advert_t *mavlink_log_pub)
{
	calibration_log_info(mavlink_log_pub, CAL_QGC_STARTED_MSG, sensor_name);

	int result = PX4_OK;

	// Collect: As defined by configuration
	// start with a full mask, all six bits set
	int32_t cal_mask = (1 << 6) - 1;
	param_get(param_find("CAL_MAG_SIDES"), &cal_mask);

	// Calibrate all mags at the same time
	switch (mag_calibrate_all(mavlink_log_pub, cal_mask)) {
	case calibrate_return_cancelled:
		// Cancel message already displayed, we're done here
		result = PX4_ERROR;
		break;

	case calibrate_return_ok:
		/* if there is a any preflight-check system response, let the barrage of messages through */
		px4_usleep(200000);

		calibration_log_info(mavlink_log_pub, CAL_QGC_PROGRESS_MSG, 100);
		px4_usleep(20000);
		calibration_log_info(mavlink_log_pub, CAL_QGC_DONE_MSG, sensor_name);
		px4_usleep(600000);
		break;

	default:
		calibration_log_critical(mavlink_log_pub, CAL_QGC_FAILED_MSG, sensor_name);
		px4_usleep(600000);
		break;
	}

	return result;
}

static bool reject_sample(matrix::Vector3f s, float x[], float y[], float z[], unsigned count, unsigned max_count)
{
	float min_sample_dist = fabsf(5.4f * mag_sphere_radius / sqrtf(max_count)) / 3.f;

	for (size_t i = 0; i < count; i++) {
		if (Vector3f{s - Vector3f{x[i], y[i], z[i]}} .norm() < min_sample_dist) {
			return true;
		}
	}

	return false;
}

static unsigned progress_percentage(mag_worker_data_t *worker_data)
{
	return 100 * ((float)worker_data->done_count) / calibration_sides;
}

static calibrate_return mag_calibration_worker(detect_orientation_return orientation, void *data)
{
	const hrt_abstime calibration_started = hrt_absolute_time();
	orb_advert_t mavlink_log_pub = nullptr;
	calibrate_return result = calibrate_return_ok;

	mag_worker_data_t *worker_data = (mag_worker_data_t *)(data);

	// notify user to start rotating
	set_tune(TONE_SINGLE_BEEP_TUNE);

	calibration_log_info(&mavlink_log_pub, "[cal] Rotate vehicle around the detected orientation");
	calibration_log_info(&mavlink_log_pub, "[cal] Continue rotation for %s %.1f s",
			     detect_orientation_str(orientation), worker_data->calibration_interval_perside_us / 1e6);

	/*
	 * Detect if the system is rotating.
	 *
	 * We're detecting this as a general rotation on any axis, not necessary on the one we
	 * asked the user for. This is because we really just need two roughly orthogonal axes
	 * for a good result, so we're not constraining the user more than we have to.
	 */

	const hrt_abstime detection_deadline = hrt_absolute_time() + worker_data->calibration_interval_perside_us * 5;
	hrt_abstime last_gyro = 0;
	float gyro_x_integral = 0.0f;
	float gyro_y_integral = 0.0f;
	float gyro_z_integral = 0.0f;

	const float gyro_int_thresh_rad = 0.5f;

	uORB::SubscriptionBlocking<sensor_gyro_s> gyro_sub{ORB_ID(sensor_gyro)};

	while (fabsf(gyro_x_integral) < gyro_int_thresh_rad &&
	       fabsf(gyro_y_integral) < gyro_int_thresh_rad &&
	       fabsf(gyro_z_integral) < gyro_int_thresh_rad) {

		/* abort on request */
		if (calibrate_cancel_check(&mavlink_log_pub, calibration_started)) {
			result = calibrate_return_cancelled;
			return result;
		}

		/* abort with timeout */
		if (hrt_absolute_time() > detection_deadline) {
			result = calibrate_return_error;
			PX4_ERR("int: %8.4f, %8.4f, %8.4f", (double)gyro_x_integral, (double)gyro_y_integral, (double)gyro_z_integral);
			calibration_log_critical(&mavlink_log_pub, "Failed: This calibration requires rotation.");
			break;
		}

		/* Wait clocking for new data on all gyro */
		sensor_gyro_s gyro{};

		if (gyro_sub.updateBlocking(gyro, 100_ms)) {

			/* ensure we have a valid first timestamp */
			if (last_gyro > 0) {

				/* integrate */
				float delta_t = (gyro.timestamp - last_gyro) / 1e6f;
				gyro_x_integral += gyro.x * delta_t;
				gyro_y_integral += gyro.y * delta_t;
				gyro_z_integral += gyro.z * delta_t;
			}

			last_gyro = gyro.timestamp;
		}
	}


	uORB::SubscriptionBlocking<sensor_mag_s> mag_sub[MAX_MAGS] {
		{ORB_ID(sensor_mag), 0, 0},
		{ORB_ID(sensor_mag), 0, 1},
		{ORB_ID(sensor_mag), 0, 2},
		{ORB_ID(sensor_mag), 0, 3},
	};

	unsigned last_mag_progress = 0;

	uint64_t calibration_deadline = hrt_absolute_time() + worker_data->calibration_interval_perside_us;
	unsigned poll_errcount = 0;
	unsigned int calibration_counter_side = 0;

	while (hrt_absolute_time() < calibration_deadline &&
	       calibration_counter_side < worker_data->calibration_points_perside) {

		if (calibrate_cancel_check(&mavlink_log_pub, calibration_started)) {
			result = calibrate_return_cancelled;
			break;
		}

		if (mag_sub[0].updatedBlocking(100_ms)) {

			bool rejected = false;
			Vector3f new_samples[MAX_MAGS] {};

			for (unsigned cur_mag = 0; cur_mag < MAX_MAGS; cur_mag++) {
				sensor_mag_s mag;

				if (mag_sub[cur_mag].update(&mag)) {
					// Check if this measurement is good to go in
					bool reject = reject_sample(matrix::Vector3f{mag.x, mag.y, mag.z},
								    worker_data->x[cur_mag], worker_data->y[cur_mag], worker_data->z[cur_mag],
								    worker_data->calibration_counter_total[cur_mag],
								    worker_data->calibration_points_perside * calibration_sides);

					if (reject) {
						rejected = true;
						PX4_DEBUG("Mag: %d rejected X: %.3f Y: %.3f Z: %.3f", cur_mag, (double)mag.x, (double)mag.y, (double)mag.z);

					} else {
						new_samples[cur_mag](0) = mag.x;
						new_samples[cur_mag](1) = mag.y;
						new_samples[cur_mag](2) = mag.z;
					}
				}
			}

			// Keep calibration of all mags in lockstep
			if (!rejected) {
				for (size_t cur_mag = 0; cur_mag < MAX_MAGS; cur_mag++) {
					if (mag_sub[cur_mag].advertised()) {
						worker_data->x[cur_mag][worker_data->calibration_counter_total[cur_mag]] = new_samples[cur_mag](0);
						worker_data->y[cur_mag][worker_data->calibration_counter_total[cur_mag]] = new_samples[cur_mag](1);
						worker_data->z[cur_mag][worker_data->calibration_counter_total[cur_mag]] = new_samples[cur_mag](2);
						worker_data->calibration_counter_total[cur_mag]++;
					}
				}

				calibration_counter_side++;

				unsigned new_progress = progress_percentage(worker_data) +
							(unsigned)((100 / calibration_sides) * ((float)calibration_counter_side / (float)
									worker_data->calibration_points_perside));

				if (new_progress - last_mag_progress > 3) {
					// Progress indicator for side
					calibration_log_info(&mavlink_log_pub, "[cal] %s side calibration: progress <%u>",
							     detect_orientation_str(orientation), new_progress);

					px4_usleep(10000);

					last_mag_progress = new_progress;
				}
			}

		} else {
			poll_errcount++;
		}

		if (poll_errcount > worker_data->calibration_points_perside * 3) {
			result = calibrate_return_error;
			calibration_log_info(&mavlink_log_pub, CAL_ERROR_SENSOR_MSG);
			break;
		}
	}

	if (result == calibrate_return_ok) {
		calibration_log_info(&mavlink_log_pub, "[cal] %s side done, rotate to a different side",
				     detect_orientation_str(orientation));

		worker_data->done_count++;
		px4_usleep(20000);
		calibration_log_info(&mavlink_log_pub, CAL_QGC_PROGRESS_MSG, progress_percentage(worker_data));
	}

	return result;
}

calibrate_return mag_calibrate_all(orb_advert_t *mavlink_log_pub, int32_t cal_mask)
{
	mag_worker_data_t worker_data{};
	worker_data.done_count = 0;
	worker_data.calibration_points_perside = calibration_total_points / detect_orientation_side_count;
	worker_data.calibration_interval_perside_us = (calibraton_duration_seconds / detect_orientation_side_count) * 1000 *
			1000;

	calibration_sides = 0;

	for (unsigned i = 0; i < (sizeof(worker_data.side_data_collected) / sizeof(worker_data.side_data_collected[0])); i++) {

		if ((cal_mask & (1 << i)) > 0) {
			// mark as missing
			worker_data.side_data_collected[i] = false;
			calibration_sides++;

		} else {
			// mark as completed from the beginning
			worker_data.side_data_collected[i] = true;

			calibration_log_info(mavlink_log_pub, "[cal] %s side done, rotate to a different side",
					     detect_orientation_str(static_cast<enum detect_orientation_return>(i)));
			px4_usleep(100000);
		}
	}

	const unsigned int calibration_points_maxcount = calibration_sides * worker_data.calibration_points_perside;

	calibrate_return result = calibrate_return_ok;

	int32_t device_ids[MAX_MAGS] {};
	int32_t rotation[MAX_MAGS] {};
	matrix::Vector3f scale_existing[MAX_MAGS] {};
	matrix::Vector3f offset_existing[MAX_MAGS] {};
	matrix::Vector3f off_diagonal_existing[MAX_MAGS] {};
	matrix::Vector3f power_compensation[MAX_MAGS] {};
	bool internal[MAX_MAGS] {true, true, true, true};

	ORB_PRIO device_prio_max = ORB_PRIO_UNINITIALIZED;
	int32_t device_id_primary = 0;

	for (uint8_t cur_mag = 0; cur_mag < MAX_MAGS; cur_mag++) {

		uORB::SubscriptionData<sensor_mag_s> mag_sub{ORB_ID(sensor_mag), cur_mag};
		mag_sub.update();

		device_ids[cur_mag] = mag_sub.get().device_id;
		internal[cur_mag] = !mag_sub.get().is_external;

		if (internal[cur_mag]) {
			rotation[cur_mag] = -1; // internal mags match have no configurable rotation

		} else {
			rotation[cur_mag] = 0; // external default rotation none
		}

		scale_existing[cur_mag] = Vector3f{1.f, 1.f, 1.f};
		offset_existing[cur_mag].zero();

		if (device_ids[cur_mag] != 0) {

			// Get priority
			ORB_PRIO prio = mag_sub.get_priority();

			if (prio > device_prio_max) {
				device_prio_max = prio;
				device_id_primary = device_ids[cur_mag];
			}

			// preserve any existing power compensation or configured rotation (external only)
			for (uint8_t mag_cal_index = 0; mag_cal_index < MAX_MAGS; mag_cal_index++) {
				char str[20] {};
				sprintf(str, "CAL_%s%u_ID", "MAG", cur_mag);
				int32_t mag_cal_device_id = 0;

				if (param_get(param_find(str), &mag_cal_device_id) == PX4_OK) {
					if (mag_cal_device_id == device_ids[cur_mag]) {
						// if external preserve configured rotation
						if (!internal[cur_mag]) {
							sprintf(str, "CAL_%s%u_ROT", "MAG", cur_mag);
							param_get(param_find(str), &rotation[cur_mag]);
						}

						for (int axis = 0; axis < 3; axis++) {
							char axis_char = 'X' + axis;

							// offsets
							sprintf(str, "CAL_%s%u_%cOFF", "MAG", cur_mag, axis_char);
							param_get(param_find(str), &offset_existing[cur_mag](axis));

							// scale
							sprintf(str, "CAL_%s%u_%cSCALE", "MAG", cur_mag, axis_char);
							param_get(param_find(str), &scale_existing[cur_mag](axis));

							// off diagonal factors
							sprintf(str, "CAL_%s%u_%cODIAG", "MAG", cur_mag, axis_char);
							param_get(param_find(str), &off_diagonal_existing[cur_mag](axis));

							// power compensation
							sprintf(str, "CAL_%s%u_%cCOMP", "MAG", cur_mag, axis_char);
							param_get(param_find(str), &power_compensation[cur_mag](axis));
						}
					}
				}
			}

			worker_data.x[cur_mag] = new float[calibration_points_maxcount];
			worker_data.y[cur_mag] = new float[calibration_points_maxcount];
			worker_data.z[cur_mag] = new float[calibration_points_maxcount];

			if (worker_data.x[cur_mag] == nullptr || worker_data.y[cur_mag] == nullptr || worker_data.z[cur_mag] == nullptr) {
				calibration_log_critical(mavlink_log_pub, "ERROR: out of memory");
				result = calibrate_return_error;
			}

			worker_data.calibration_counter_total[cur_mag] = 0;

		} else {
			break;
		}
	}

	if (result == calibrate_return_ok) {
		result = calibrate_from_orientation(mavlink_log_pub,                    // uORB handle to write output
						    worker_data.side_data_collected,    // Sides to calibrate
						    mag_calibration_worker,             // Calibration worker
						    &worker_data,			// Opaque data for calibration worked
						    true);				// true: lenient still detection
	}


	// Calculate calibration values for each mag
	float sphere_x[MAX_MAGS] {};
	float sphere_y[MAX_MAGS] {};
	float sphere_z[MAX_MAGS] {};
	float sphere_radius[MAX_MAGS] {0.2f, 0.2f, 0.2f, 0.2f};
	float diag_x[MAX_MAGS] {1.f, 1.f, 1.f, 1.f};
	float diag_y[MAX_MAGS] {1.f, 1.f, 1.f, 1.f};
	float diag_z[MAX_MAGS] {1.f, 1.f, 1.f, 1.f};
	float offdiag_x[MAX_MAGS] {};
	float offdiag_y[MAX_MAGS] {};
	float offdiag_z[MAX_MAGS] {};

	// Sphere fit the data to get calibration values
	if (result == calibrate_return_ok) {
		for (unsigned cur_mag = 0; cur_mag < MAX_MAGS; cur_mag++) {
			if (device_ids[cur_mag] != 0) {
				// Mag in this slot is available and we should have values for it to calibrate

				{
					float fitness = 1.e30f;
					float lambda = 1.f;

					for (int i = 0; i < 100; i++) {
						run_lm_sphere_fit(worker_data.x[cur_mag], worker_data.y[cur_mag], worker_data.z[cur_mag],
								  fitness, lambda, worker_data.calibration_counter_total[cur_mag],
								  sphere_x[cur_mag], sphere_y[cur_mag], sphere_z[cur_mag],
								  sphere_radius[cur_mag],
								  diag_x[cur_mag], diag_y[cur_mag], diag_z[cur_mag],
								  offdiag_x[cur_mag], offdiag_y[cur_mag], offdiag_z[cur_mag]);
					}
				}

				// Estimate only the offsets if two-sided calibration is selected, as the problem is not constrained
				// enough to reliably estimate both scales and offsets with 2 sides only (even if the existing calibration
				// is already close)
				const bool sphere_fit_only = (calibration_sides <= 2);

				if (!sphere_fit_only) {
					float fitness = 1.e30f;
					float lambda = 1.f;

					for (int i = 0; i < 100; i++) {
						run_lm_ellipsoid_fit(worker_data.x[cur_mag], worker_data.y[cur_mag], worker_data.z[cur_mag],
								     fitness, lambda, worker_data.calibration_counter_total[cur_mag],
								     sphere_x[cur_mag], sphere_y[cur_mag], sphere_z[cur_mag],
								     sphere_radius[cur_mag],
								     diag_x[cur_mag], diag_y[cur_mag], diag_z[cur_mag],
								     offdiag_x[cur_mag], offdiag_y[cur_mag], offdiag_z[cur_mag]
								    );
					}
				}

				// check calibration result

				// Make sure every parameter is finite
				const float must_be_finite[] {sphere_x[cur_mag], sphere_y[cur_mag], sphere_z[cur_mag],
							      sphere_radius[cur_mag],
							      diag_x[cur_mag], diag_y[cur_mag], diag_z[cur_mag],
							      offdiag_x[cur_mag], offdiag_y[cur_mag], offdiag_z[cur_mag]
							     };

				for (const auto &f : must_be_finite) {
					if (!PX4_ISFINITE(f)) {
						calibration_log_emergency(mavlink_log_pub, "ERROR: Retry calibration (sphere NaN, #%u)", cur_mag);
						result = calibrate_return_error;
					}
				}

				if (result == calibrate_return_ok) {
					// Notify if a parameter which should be positive is non-positive
					const float should_be_positive[] = {sphere_radius[cur_mag], diag_x[cur_mag], diag_y[cur_mag], diag_z[cur_mag]};

					for (const auto &p : should_be_positive) {
						if (p < 0.f) {
							calibration_log_critical(mavlink_log_pub, "Warning: %s mag %d with non-positive scale",
										 (internal[cur_mag]) ? "autopilot, internal" : "external", cur_mag);
						}
					}

					// save calibration

					// TODO: Update calibration
					// The formula for applying the calibration is:
					//   mag_value = (mag_readout - (offset_existing + offset_new/scale_existing)) * scale_existing * scale_new
					// x_offset = x_offset + sphere_x[cur_mag] / x_scale;
					// y_offset = y_offset + sphere_y[cur_mag] / y_scale;
					// z_offset = z_offset + sphere_z[cur_mag] / z_scale;
					// x_scale = x_scale * diag_x[cur_mag];
					// y_scale = y_scale * diag_y[cur_mag];
					// z_scale = z_scale * diag_z[cur_mag];


					Vector3f offset{sphere_x[cur_mag], sphere_y[cur_mag], sphere_z[cur_mag]};
					Vector3f scale{diag_x[cur_mag], diag_y[cur_mag], diag_z[cur_mag]};
					Vector3f offdiagonal{offdiag_x[cur_mag], offdiag_y[cur_mag], offdiag_z[cur_mag]};

					char str[20] {};

					sprintf(str, "CAL_%s%u_ID", "MAG", cur_mag);
					param_set_no_notification(param_find(str), &device_ids[cur_mag]);
					sprintf(str, "CAL_%s%u_ROT", "MAG", cur_mag);
					param_set_no_notification(param_find(str), &rotation[cur_mag]);

					for (int axis = 0; axis < 3; axis++) {
						char axis_char = 'X' + axis;

						// offsets
						sprintf(str, "CAL_%s%u_%cOFF", "MAG", cur_mag, axis_char);
						param_set_no_notification(param_find(str), &offset(axis));

						// scale
						sprintf(str, "CAL_%s%u_%cSCALE", "MAG", cur_mag, axis_char);
						param_get(param_find(str), &scale(axis));

						// off diagonal factors
						sprintf(str, "CAL_%s%u_%cODIAG", "MAG", cur_mag, axis_char);
						param_get(param_find(str), &offdiagonal(axis));

						// power compensation (preserved)
						sprintf(str, "CAL_%s%u_%cCOMP", "MAG", cur_mag, axis_char);
						param_set_no_notification(param_find(str), &power_compensation[cur_mag](axis));
					}

					PX4_INFO("[cal] %s #%u off: x:%.2f y:%.2f z:%.2f", "MAG", cur_mag, (double)offset(0), (double)offset(1),
						 (double)offset(2));
					PX4_INFO("[cal] %s #%u scale: x:%.2f y:%.2f z:%.2f", "MAG", cur_mag, (double)scale(0), (double)scale(1),
						 (double)scale(2));
					PX4_INFO("[cal] %s #%u offdiag: x:%.2f y:%.2f z:%.2f", "MAG", cur_mag, (double)offdiagonal(0), (double)offdiagonal(1),
						 (double)offdiagonal(2));

#if 0 // DO NOT REMOVE! Critical validation data!
					printf("RAW DATA:\n--------------------\n");

					if (worker_data.calibration_counter_total[cur_mag] == 0) {
						continue;
					}

					printf("RAW: MAG %u with %u samples:\n", (unsigned)cur_mag, (unsigned)worker_data.calibration_counter_total[cur_mag]);

					for (size_t i = 0; i < worker_data.calibration_counter_total[cur_mag]; i++) {
						float x = worker_data.x[cur_mag][i];
						float y = worker_data.y[cur_mag][i];
						float z = worker_data.z[cur_mag][i];
						printf("%8.4f, %8.4f, %8.4f\n", (double)x, (double)y, (double)z);
					}

					printf(">>>>>>>\n");

					printf("CALIBRATED DATA:\n--------------------\n");

					if (worker_data.calibration_counter_total[cur_mag] == 0) {
						continue;
					}

					printf("Calibrated: MAG %u with %u samples:\n", (unsigned)cur_mag,
					       (unsigned)worker_data.calibration_counter_total[cur_mag]);

					for (size_t i = 0; i < worker_data.calibration_counter_total[cur_mag]; i++) {
						float x = worker_data.x[cur_mag][i] - sphere_x[cur_mag];
						float y = worker_data.y[cur_mag][i] - sphere_y[cur_mag];
						float z = worker_data.z[cur_mag][i] - sphere_z[cur_mag];
						printf("%8.4f, %8.4f, %8.4f\n", (double)x, (double)y, (double)z);
					}

					printf("SPHERE RADIUS: %8.4f\n", (double)sphere_radius[cur_mag]);
					printf(">>>>>>>\n");
#endif // DO NOT REMOVE! Critical validation data!
				}
			}
		}
	}


	// Attempt to automatically determine external mag rotations
	if (result == calibrate_return_ok) {
		int32_t param_cal_mag_rot_auto = 0;
		param_get(param_find("CAL_MAG_ROT_AUTO"), &param_cal_mag_rot_auto);

		if ((calibration_sides >= 3) && (param_cal_mag_rot_auto == 1)) {

			// find first internal mag to use as reference
			int internal_index = -1;

			for (unsigned cur_mag = 0; cur_mag < MAX_MAGS; cur_mag++) {
				if (internal[cur_mag] && (device_ids[cur_mag] != 0)) {
					internal_index = cur_mag;
					break;
				}
			}

			// only proceed if there's a valid internal
			if (internal_index >= 0) {

				// apply new calibrations to all raw sensor data before comparison
				for (unsigned cur_mag = 0; cur_mag < MAX_MAGS; cur_mag++) {
					if (device_ids[cur_mag] != 0) {
						for (unsigned i = 0; i < worker_data.calibration_counter_total[cur_mag]; i++) {
							// apply calibration
							float x = (worker_data.x[cur_mag][i] - sphere_x[cur_mag]) * diag_x[cur_mag];
							float y = (worker_data.y[cur_mag][i] - sphere_y[cur_mag]) * diag_y[cur_mag];
							float z = (worker_data.z[cur_mag][i] - sphere_z[cur_mag]) * diag_z[cur_mag];

							// store back in work_data
							worker_data.x[cur_mag][i] = x;
							worker_data.y[cur_mag][i] = y;
							worker_data.z[cur_mag][i] = z;
						}
					}
				}

				// rotate internal mag data to board
				param_t board_rotation_h = param_find("SENS_BOARD_ROT");
				int32_t board_rotation_int = 0;
				param_get(board_rotation_h, &(board_rotation_int));
				const enum Rotation board_rotation_id = (enum Rotation)board_rotation_int;

				if (board_rotation_int != ROTATION_NONE) {
					for (unsigned i = 0; i < worker_data.calibration_counter_total[internal_index]; i++) {
						rotate_3f(board_rotation_id,
							  worker_data.x[internal_index][i],
							  worker_data.y[internal_index][i],
							  worker_data.z[internal_index][i]
							 );
					}
				}

				for (unsigned cur_mag = 0; cur_mag < MAX_MAGS; cur_mag++) {
					if (!internal[cur_mag] && (device_ids[cur_mag] != 0)) {

						const int last_sample_index = math::min(worker_data.calibration_counter_total[internal_index],
											worker_data.calibration_counter_total[cur_mag]);

						float diff_sum[ROTATION_MAX] {};

						float min_diff = FLT_MAX;
						int32_t best_rotation = ROTATION_NONE;

						for (int r = ROTATION_NONE; r < ROTATION_MAX; r++) {
							for (int i = 0; i < last_sample_index; i++) {

								float x = worker_data.x[cur_mag][i];
								float y = worker_data.y[cur_mag][i];
								float z = worker_data.z[cur_mag][i];
								rotate_3f((enum Rotation)r, x, y, z);

								Vector3f diff = Vector3f{x, y, z} - Vector3f{worker_data.x[internal_index][i], worker_data.y[internal_index][i], worker_data.z[internal_index][i]};

								diff_sum[r] += diff.norm();
							}

							if (diff_sum[r] < min_diff) {
								min_diff = diff_sum[r];
								best_rotation = r;
							}
						}


						// Check that the best rotation is at least twice as good as the next best
						bool smallest_check_passed = true;

						for (int r = ROTATION_NONE; r < ROTATION_MAX; r++) {
							if (r != best_rotation) {
								if (diff_sum[r] < (min_diff * 2.f)) {
									smallest_check_passed = false;
								}
							}
						}


						// Check that the average error across all samples (relative to internal mag) is less than the minimum earth field (~ 0.25 Gauss)
						const float mag_error_ga = (min_diff / last_sample_index);
						bool total_error_check_passed = (mag_error_ga < 0.25f);

						if (smallest_check_passed && total_error_check_passed) {
							// TODO: compare with existing rotation?
							PX4_INFO("External Mag: %d, determined rotation: %d", cur_mag, best_rotation);

							char str[20] {};
							sprintf(str, "CAL_MAG%u_ID", cur_mag);
							param_set_no_notification(param_find(str), &device_ids[cur_mag]);

							sprintf(str, "CAL_MAG%u_ROT", cur_mag);
							param_set_no_notification(param_find(str), &best_rotation);
						}

						for (int r = ROTATION_NONE; r < ROTATION_MAX; r++) {
							PX4_DEBUG("Mag: %d, rotation: %d error: %.6f", cur_mag, r, (double)diff_sum[r]);
						}
					}
				}
			}
		}
	}

	// Data points are no longer needed
	for (size_t cur_mag = 0; cur_mag < MAX_MAGS; cur_mag++) {
		delete[] worker_data.x[cur_mag];
		delete[] worker_data.y[cur_mag];
		delete[] worker_data.z[cur_mag];
	}

	if (result == calibrate_return_ok) {
		// Trigger a param set on the last step so the whole system updates
		param_set(param_find("CAL_MAG_PRIME"), &device_id_primary);
	}

	return result;
}
