/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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
 * @file gnss_yaw_control.cpp
 * Control functions for ekf GNSS yaw fusion
 */

#include "ekf.h"

void Ekf::controlGpsYawFusion()
{
	gpsYawSample gps_yaw_sample;
	if (_gps_yaw_buffer && _gps_yaw_buffer->pop_first_older_than(_imu_sample_delayed.time_us, &gps_yaw_sample)) {

		resetEstimatorAidStatus(_aid_src_gnss_yaw);

		const float yaw_offset = PX4_ISFINITE(gps_yaw_sample.yaw_offset) ? gps_yaw_sample.yaw_offset : 0.f;

		float yaw_accuracy = PX4_ISFINITE(gps_yaw_sample.yaw_accuracy) ? gps_yaw_sample.yaw_accuracy : _params.gps_heading_noise;
		const float observation_variance = sq(math::max(yaw_accuracy, _params.gps_heading_noise, 0.01f));




		// calculate the observed yaw angle of antenna array, converting a from body to antenna yaw measurement
		const float measured_hdg = wrap_pi(gps_yaw_sample.yaw + yaw_offset);

		// define the predicted antenna array vector and rotate into earth frame
		const Vector3f ant_vec_bf = {cosf(yaw_offset), sinf(yaw_offset), 0.f};
		const Vector3f ant_vec_ef = _R_to_earth * ant_vec_bf;

		// calculate predicted antenna yaw angle
		const float predicted_hdg = atan2f(ant_vec_ef(1), ant_vec_ef(0));

		// wrap the innovation to the interval between +-pi
		const float heading_innov = wrap_pi(predicted_hdg - measured_hdg);

		// using magnetic heading process noise
		// TODO extend interface to use yaw uncertainty provided by GPS if available
		const float R_YAW = observation_variance;

		_aid_src_gnss_yaw.timestamp_sample = gps_yaw_sample.time_us;
		_aid_src_gnss_yaw.observation = measured_hdg;
		_aid_src_gnss_yaw.observation_variance = R_YAW;
		_aid_src_gnss_yaw.innovation = heading_innov;







		const bool continuing_conditions_passing = (_params.gnss_ctrl & GnssCtrl::YAW)
					&& PX4_ISFINITE(gps_yaw_sample.yaw);


		const bool starting_conditions_passing = continuing_conditions_passing
				&& _control_status.flags.tilt_align
				&& isNewestSampleRecent(_time_last_gps_yaw_buffer_push, 2 * GPS_MAX_INTERVAL);

		if (_control_status.flags.gps_yaw) {
			// aid_src.fusion_enabled = true;

			if (continuing_conditions_passing) {

				fuseGpsYaw(gps_yaw_sample.time_us, gps_yaw_sample.yaw, yaw_offset, observation_variance);

				const bool is_fusion_failing = isTimedOut(_aid_src_gnss_yaw.time_last_fuse, _params.reset_timeout_max);

				if (is_fusion_failing) {
					if (_nb_gps_yaw_reset_available > 0) {
						// Data seems good, attempt a reset
						resetYawToGps(gps_yaw_sample.yaw, yaw_offset, observation_variance);

						if (_control_status.flags.in_air) {
							_nb_gps_yaw_reset_available--;
						}

					} else if (starting_conditions_passing) {
						// Data seems good, but previous reset did not fix the issue
						// something else must be wrong, declare the sensor faulty and stop the fusion
						_control_status.flags.gps_yaw_fault = true;
						stopGpsYawFusion();

					} else {
						// A reset did not fix the issue but all the starting checks are not passing
						// This could be a temporary issue, stop the fusion without declaring the sensor faulty
						stopGpsYawFusion();
					}

					// TODO: should we give a new reset credit when the fusion does not fail for some time?
				}

			} else {
				// Stop GPS yaw fusion but do not declare it faulty
				stopGpsYawFusion();
			}

		} else {
			if (starting_conditions_passing) {
				// Try to activate GPS yaw fusion
				if (resetYawToGps(gps_yaw_sample.yaw, yaw_offset, observation_variance)) {
					ECL_INFO("starting GPS yaw fusion");

					stopEvYawFusion();
					stopMagHdgFusion();
					stopMag3DFusion();

					_nb_gps_yaw_reset_available = 1;

					_control_status.flags.gps_yaw = true;
				}
			}
		}

	} else if (_control_status.flags.gps_yaw && !isNewestSampleRecent(_time_last_gps_yaw_buffer_push, _params.reset_timeout_max)) {
		// No yaw data in the message anymore. Stop until it comes back.
		stopGpsYawFusion();
	}

	// Before takeoff, we do not want to continue to rely on the current heading
	// if we had to stop the fusion
	if (!_control_status.flags.in_air
	    && !_control_status.flags.gps_yaw
	    && _control_status_prev.flags.gps_yaw) {

		_control_status.flags.yaw_align = false;
	}
}

void Ekf::stopGpsYawFusion()
{
	if (_control_status.flags.gps_yaw) {
		ECL_INFO("stopping GPS yaw fusion");
		resetEstimatorAidStatus(_aid_src_gnss_yaw);

		_control_status.flags.gps_yaw = false;
	}
}
