/****************************************************************************
 *
 *   Copyright (c) 2019-2022 PX4 Development Team. All rights reserved.
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

#include "VehicleIMU.hpp"

#include <px4_platform_common/log.h>

#include <uORB/topics/vehicle_imu_status.h>

using namespace matrix;

namespace sensors
{

static constexpr int clipping(const int16_t sample) { return (sample == INT16_MIN) || (sample == INT16_MAX); }

static constexpr int clipping(const int16_t samples[], int length)
{
	int clip_count = 0;

	for (int n = 0; n < length; n++) {
		if ((samples[n] == INT16_MIN) || (samples[n] == INT16_MAX)) {
			clip_count++;
		}
	}

	return clip_count;
}

VehicleIMU::VehicleIMU() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
{
}

VehicleIMU::~VehicleIMU()
{
	Stop();

	perf_free(_cycle_perf);
	perf_free(_filter_reset_perf);
	perf_free(_selection_changed_perf);
}

bool VehicleIMU::Start()
{
	// force initial updates
	ParametersUpdate(true);

	_imus[0].primary = true;


	ScheduleNow();

	return true;
}

void VehicleIMU::Stop()
{
	// clear all registered callbacks
	for (auto &imu_sub : _sensor_imu_fifo_subs) {
		imu_sub.sub.unregisterCallback();
	}

	Deinit();
}

void VehicleIMU::SensorBiasUpdate()
{
	// find corresponding estimated sensor bias
	if (_estimator_selector_status_sub.updated()) {
		estimator_selector_status_s estimator_selector_status;

		if (_estimator_selector_status_sub.copy(&estimator_selector_status)) {
			//_estimator_sensor_bias_sub.ChangeInstance(estimator_selector_status.primary_instance);

			// TODO:
			//  find corresponding primary accel and gyro device id
		}
	}

	for (auto &sub : _estimator_sensor_bias_subs) {
		estimator_sensor_bias_s estimator_sensor_bias;

		if (sub.update(&estimator_sensor_bias)) {

			for (auto &imu : _imus) {
				if (estimator_sensor_bias.accel_bias_valid
				    && (estimator_sensor_bias.accel_device_id != 0)
				    && (estimator_sensor_bias.accel_device_id == imu.accel.calibration.device_id())
				   ) {
					const Vector3f bias_var{estimator_sensor_bias.accel_bias_variance};

					if ((bias_var.norm() < imu.accel.estimated_bias_variance.norm())
					    || !imu.accel.estimated_bias_variance.longerThan(FLT_EPSILON)) {

						imu.accel.estimated_bias = Vector3f(estimator_sensor_bias.accel_bias);
						imu.accel.estimated_bias_variance = bias_var;
					}

				}

				if (estimator_sensor_bias.gyro_bias_valid
				    && (estimator_sensor_bias.gyro_device_id != 0)
				    && (estimator_sensor_bias.gyro_device_id == imu.gyro.calibration.device_id())
				   ) {
					const Vector3f bias_var{estimator_sensor_bias.gyro_bias_variance};

					if ((bias_var.norm() < imu.gyro.estimated_bias_variance.norm())
					    || !imu.gyro.estimated_bias_variance.longerThan(FLT_EPSILON)) {

						imu.gyro.estimated_bias = Vector3f(estimator_sensor_bias.gyro_bias);
						imu.gyro.estimated_bias_variance = bias_var;
					}
				}

			}
		}
	}
}

bool VehicleIMU::SensorSelectionUpdate(const hrt_abstime &time_now_us, bool force)
{

	return false;
}

void VehicleIMU::ParametersUpdate(bool force)
{
	// Check if parameters have changed
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		updateParams();

		for (auto &imu : _imus) {
			imu.accel.calibration.ParametersUpdate();
			imu.gyro.calibration.ParametersUpdate();
		}

		// IMU_GYRO_RATEMAX
		if (_param_imu_gyro_ratemax.get() <= 0) {
			const int32_t imu_gyro_ratemax = _param_imu_gyro_ratemax.get();
			_param_imu_gyro_ratemax.reset();
			PX4_WARN("IMU_GYRO_RATEMAX invalid (%" PRId32 "), resetting to default %" PRId32 ")", imu_gyro_ratemax,
				 _param_imu_gyro_ratemax.get());
		}

		// constrain IMU_GYRO_RATEMAX 50-10,000 Hz
		const int32_t imu_gyro_ratemax = constrain(_param_imu_gyro_ratemax.get(), (int32_t)50, (int32_t)10'000);

		if (imu_gyro_ratemax != _param_imu_gyro_ratemax.get()) {
			PX4_WARN("IMU_GYRO_RATEMAX updated %" PRId32 " -> %" PRIu32, _param_imu_gyro_ratemax.get(), imu_gyro_ratemax);
			_param_imu_gyro_ratemax.set(imu_gyro_ratemax);
			_param_imu_gyro_ratemax.commit_no_notification();
		}


	}
}

void VehicleIMU::Run()
{
	perf_begin(_cycle_perf);

	// backup schedule
	ScheduleDelayed(10_ms);

	const hrt_abstime time_now_us = hrt_absolute_time();

	ParametersUpdate();

	for (auto &imu : _imus) {
		imu.accel.calibration.SensorCorrectionsUpdate();
		imu.gyro.calibration.SensorCorrectionsUpdate();
	}

	SensorBiasUpdate();

	for (int sensor_instance = 0; sensor_instance < MAX_SENSOR_COUNT; sensor_instance++) {
		UpdateSensorImuFifo(sensor_instance);
		// UpdateSensorAccel(sensor_instance);
		// UpdateSensorGyro(sensor_instance);
	}

	// process all outstanding messages


	// force reselection on timeout
	// if (time_now_us > _last_publish + 500_ms) {
	// 	SensorSelectionUpdate(true);
	// }

	perf_end(_cycle_perf);
}

void VehicleIMU::UpdateSensorImuFifo(uint8_t sensor_instance)
{
	// sensor_imu_fifo
	sensor_imu_fifo_s sensor_imu_fifo;
	auto &s = _sensor_imu_fifo_subs[sensor_instance];

	while (s.sub.update(&sensor_imu_fifo)) {

		// TODO: find corresponding IMU slot
		IMU &imu = _imus[sensor_instance];

		// publication interval
		//  sensor output data rate or interval
		if (s.sub.get_last_generation() == s.last_generation + 1) {

			if ((s.timestamp_last != 0)
			    && (sensor_imu_fifo.timestamp > s.timestamp_last)
			    && (sensor_imu_fifo.timestamp_sample > imu.accel.timestamp_sample_last)
			    && (sensor_imu_fifo.timestamp_sample > imu.gyro.timestamp_sample_last)
			   ) {

				const float interval_us = sensor_imu_fifo.timestamp - s.timestamp_last;
				s.mean_interval_us.update(interval_us);

				const float accel_interval_sample_us = sensor_imu_fifo.timestamp_sample - imu.accel.timestamp_sample_last;
				imu.accel.mean_interval_us.update(accel_interval_sample_us / sensor_imu_fifo.samples);

				const float gyro_interval_sample_us = sensor_imu_fifo.timestamp_sample - imu.gyro.timestamp_sample_last;
				imu.gyro.mean_interval_us.update(gyro_interval_sample_us / sensor_imu_fifo.samples);

				// check and update sensor rate
				if (s.mean_interval_us.valid()
				    && imu.gyro.mean_interval_us.valid()
				   ) {

					// determine number of sensor samples that will get closest to the desired integration interval
					const float imu_integration_interval_us = 1e6f / _param_imu_integ_rate.get();
					const float pub_interval_avg_us = s.mean_interval_us.mean();
					uint8_t imu_publications = math::max(1, (int)roundf(imu_integration_interval_us / pub_interval_avg_us));

					// if samples exceeds queue depth, instead round to nearest even integer to improve scheduling options
					if (imu_publications > sensor_imu_fifo_s::ORB_QUEUE_LENGTH) {
						imu_publications = math::max(1, (int)roundf(imu_integration_interval_us / pub_interval_avg_us / 2) * 2);
					}

					uint32_t integration_interval_us = roundf(imu_publications * pub_interval_avg_us);

					if ((imu.gyro.integrator.reset_interval_us() - integration_interval_us) / integration_interval_us > 0.1f) {

						imu.gyro.integrator.set_reset_interval(integration_interval_us);
						imu.accel.integrator.set_reset_interval(integration_interval_us / 2.f);

						// number of samples per publication
						const float imu_sample_interval_avg_us = imu.gyro.mean_interval_us.mean();
						int interval_samples = roundf(integration_interval_us / imu_sample_interval_avg_us);
						imu.gyro.integrator.set_reset_samples(math::max(interval_samples, 1));

						imu.accel.integrator.set_reset_samples(1);

						//_backup_schedule_timeout_us = sensor_imu_fifo_s::ORB_QUEUE_LENGTH * interval_avg_us;

						// gyro: find largest integer multiple of gyro_integral_samples
						for (int n = sensor_imu_fifo_s::ORB_QUEUE_LENGTH; n > 0; n--) {
							if (imu_publications > sensor_imu_fifo_s::ORB_QUEUE_LENGTH) {
								imu_publications /= 2;
							}

							if (imu_publications % n == 0) {

								if (imu.primary) {
									s.sub.set_required_updates(1);

								} else {
									s.sub.set_required_updates(n);
								}

								s.sub.registerCallback();

								//_intervals_configured = true;
								//_update_integrator_config = false;

								PX4_INFO("IMU FIFO (%" PRIu32 "), FIFO samples: %" PRIu8 ", interval: %.1f",
									 imu.accel.calibration.device_id(), imu_publications, (double)integration_interval_us);

								PX4_INFO("IMU FIFO (%" PRIu32 "), interval: %.1f us, interval: %.1f us", (double)s.mean_interval_us.mean(),
									 (double)imu.accel.mean_interval_us.mean());

								break;
							}
						}

					}
				}
			}
		}

		s.last_generation = s.sub.get_last_generation();

		s.timestamp_last = sensor_imu_fifo.timestamp;

		imu.accel.timestamp_sample_last = sensor_imu_fifo.timestamp_sample;
		imu.gyro.timestamp_sample_last = sensor_imu_fifo.timestamp_sample;

		imu.accel.calibration.set_device_id(sensor_imu_fifo.device_id);
		imu.gyro.calibration.set_device_id(sensor_imu_fifo.device_id);

		imu.accel.error_count = sensor_imu_fifo.error_count;
		imu.gyro.error_count = sensor_imu_fifo.error_count;

		// temperature average
		if (PX4_ISFINITE(sensor_imu_fifo.temperature)) {
			imu.accel.temperature.update(sensor_imu_fifo.temperature);
			imu.gyro.temperature.update(sensor_imu_fifo.temperature);
		}

		const int N = sensor_imu_fifo.samples;

		const float dt_s = sensor_imu_fifo.dt * 1e-6f;
		const float inverse_dt_s = 1e6f / sensor_imu_fifo.dt;

		// integrate accel
		for (int n = 0; n < N; n++) {
			const Vector3f accel_raw{
				static_cast<float>(sensor_imu_fifo.accel_x[n]) *sensor_imu_fifo.accel_scale,
				static_cast<float>(sensor_imu_fifo.accel_y[n]) *sensor_imu_fifo.accel_scale,
				static_cast<float>(sensor_imu_fifo.accel_z[n]) *sensor_imu_fifo.accel_scale
			};
			imu.accel.integrator.put(accel_raw, dt_s);
			imu.accel.raw_mean.update(accel_raw);
		}

		// accel clipping
		{
			int clip_counter[3] {
				clipping(sensor_imu_fifo.accel_x, sensor_imu_fifo.samples),
				clipping(sensor_imu_fifo.accel_y, sensor_imu_fifo.samples),
				clipping(sensor_imu_fifo.accel_z, sensor_imu_fifo.samples),
			};

			if (clip_counter[0] > 0 || clip_counter[1] > 0 || clip_counter[2] > 0) {
				// rotate sensor clip counts into vehicle body frame
				const Vector3f clipping{imu.accel.calibration.rotation() *Vector3f{(float)clip_counter[0], (float)clip_counter[1], (float)clip_counter[2]}};

				// round to get reasonble clip counts per axis (after board rotation)
				const uint8_t clip_x = roundf(fabsf(clipping(0)));

				if (clip_x > 0) {
					imu.accel.clipping_total[0] += clip_x;
					imu.accel.clipping_flags |= vehicle_imu_s::CLIPPING_X;
				}

				const uint8_t clip_y = roundf(fabsf(clipping(1)));

				if (clip_y > 0) {
					imu.accel.clipping_total[1] += clip_y;
					imu.accel.clipping_flags |= vehicle_imu_s::CLIPPING_Y;
				}

				const uint8_t clip_z = roundf(fabsf(clipping(2)));

				if (clip_z > 0) {
					imu.accel.clipping_total[2] += clip_z;
					imu.accel.clipping_flags |= vehicle_imu_s::CLIPPING_Z;
				}
			}
		}

		// integrate gyro
		for (int n = 0; n < N; n++) {
			const Vector3f gyro_raw{
				static_cast<float>(sensor_imu_fifo.gyro_x[n]) *sensor_imu_fifo.gyro_scale,
				static_cast<float>(sensor_imu_fifo.gyro_y[n]) *sensor_imu_fifo.gyro_scale,
				static_cast<float>(sensor_imu_fifo.gyro_z[n]) *sensor_imu_fifo.gyro_scale
			};
			imu.gyro.integrator.put(gyro_raw, dt_s);
			imu.gyro.raw_mean.update(gyro_raw);
		}

		// gyro clipping
		{
			int clip_counter[3] {
				clipping(sensor_imu_fifo.gyro_x, sensor_imu_fifo.samples),
				clipping(sensor_imu_fifo.gyro_y, sensor_imu_fifo.samples),
				clipping(sensor_imu_fifo.gyro_z, sensor_imu_fifo.samples),
			};

			if (clip_counter[0] > 0 || clip_counter[1] > 0 || clip_counter[2] > 0) {
				// rotate sensor clip counts into vehicle body frame
				const Vector3f clipping{imu.gyro.calibration.rotation() *Vector3f{(float)clip_counter[0], (float)clip_counter[1], (float)clip_counter[2]}};

				// round to get reasonble clip counts per axis (after board rotation)
				const uint8_t clip_x = roundf(fabsf(clipping(0)));

				if (clip_x > 0) {
					imu.gyro.clipping_total[0] += clip_x;
					imu.gyro.clipping_flags |= vehicle_imu_s::CLIPPING_X;
				}

				const uint8_t clip_y = roundf(fabsf(clipping(1)));

				if (clip_y > 0) {
					imu.gyro.clipping_total[1] += clip_y;
					imu.gyro.clipping_flags |= vehicle_imu_s::CLIPPING_Y;
				}

				const uint8_t clip_z = roundf(fabsf(clipping(2)));

				if (clip_z > 0) {
					imu.gyro.clipping_total[2] += clip_z;
					imu.gyro.clipping_flags |= vehicle_imu_s::CLIPPING_Z;
				}
			}
		}

		if (imu.accel.integrator.integral_ready() && imu.gyro.integrator.integral_ready()) {
			PublishImu(imu);
		}

		// TODO: call to filter data
		if (imu.primary) {

			//_acceleration.updateSensorImuFifo(sensor_imu_fifo);
			_vehicle_angular_velocity.updateSensorImuFifo(imu, sensor_imu_fifo);

			// TODO: or do it axis by axis?
			//  float raw_data_array[]
		}
	}
}

void VehicleIMU::UpdateSensorAccel(uint8_t sensor_instance)
{
	// process all outstanding messages
	// sensor_accel_s sensor_accel;

	// while (_sensor_accel_sub.update(&sensor_accel)) {
	// 	if (PX4_ISFINITE(sensor_accel.x) && PX4_ISFINITE(sensor_accel.y) && PX4_ISFINITE(sensor_accel.z)) {

	// 		Vector3f acceleration_uncalibrated;

	// 		float raw_data_array[] {sensor_accel.x, sensor_accel.y, sensor_accel.z};

	// 		for (int axis = 0; axis < 3; axis++) {
	// 			// copy sensor sample to float array for filtering
	// 			float data[1] {raw_data_array[axis]};

	// 			// save last filtered sample
	// 			acceleration_uncalibrated(axis) = FilterAcceleration(axis, data);
	// 		}

	// 		// Publish
	// 		if (!_sensor_accel_sub.updated()) {
	// 			if (CalibrateAndPublishAcceleration(sensor_accel.timestamp_sample, acceleration_uncalibrated)) {

	// 				//perf_end(_cycle_perf);
	// 				//return;
	// 			}
	// 		}
	// 	}
	// }
}

void VehicleIMU::UpdateSensorGyro(uint8_t sensor_instance)
{
	// sensor_gyro_s sensor_gyro;

	// while (_sensor_gyro_sub.update(&sensor_gyro)) {
	// 	if (Vector3f(sensor_gyro.x, sensor_gyro.y, sensor_gyro.z).isAllFinite()) {

	// 		if (_timestamp_sample_last == 0 || (sensor_gyro.timestamp_sample <= _timestamp_sample_last)) {
	// 			_timestamp_sample_last = sensor_gyro.timestamp_sample - 1e6f / _filter_sample_rate_hz;
	// 		}

	// 		const float inverse_dt_s = 1.f / math::constrain(((sensor_gyro.timestamp_sample - _timestamp_sample_last) * 1e-6f),
	// 					   0.00002f, 0.02f);
	// 		_timestamp_sample_last = sensor_gyro.timestamp_sample;

	// 		Vector3f angular_velocity_uncalibrated;
	// 		Vector3f angular_acceleration_uncalibrated;

	// 		float raw_data_array[] {sensor_gyro.x, sensor_gyro.y, sensor_gyro.z};

	// 		for (int axis = 0; axis < 3; axis++) {
	// 			// copy sensor sample to float array for filtering
	// 			float data[1] {raw_data_array[axis]};

	// 			// save last filtered sample
	// 			angular_velocity_uncalibrated(axis) = FilterAngularVelocity(axis, data);
	// 			angular_acceleration_uncalibrated(axis) = FilterAngularAcceleration(axis, inverse_dt_s, data);
	// 		}

	// 		// Publish
	// 		if (!_sensor_gyro_sub.updated()) {
	// 			if (CalibrateAndPublishAngularVelocity(sensor_gyro.timestamp_sample,
	// 							       angular_velocity_uncalibrated, angular_acceleration_uncalibrated)) {


	// 				// shift last publish time forward, but don't let it get further behind than the interval
	// 				_angular_velocity_last_publish = math::constrain(_angular_velocity_last_publish + _publish_interval_min_us,
	// 								 sensor_gyro.timestamp_sample - _publish_interval_min_us, sensor_gyro.timestamp_sample);

	// 				perf_end(_cycle_perf);
	// 				return;
	// 			}
	// 		}
	// 	}
	// }

}

bool VehicleIMU::PublishImu(sensors::IMU &imu)
{
	bool updated = false;

	vehicle_imu_s vehicle_imu;
	Vector3f delta_angle;
	Vector3f delta_velocity;

	const Vector3f accumulated_coning_corrections = imu.gyro.integrator.accumulated_coning_corrections();

	if (imu.accel.integrator.reset(delta_velocity, vehicle_imu.delta_velocity_dt)
	    && imu.gyro.integrator.reset(delta_angle, vehicle_imu.delta_angle_dt)) {

		if (imu.accel.calibration.enabled() && imu.gyro.calibration.enabled()) {

			// delta angle: apply offsets, scale, and board rotation
			imu.gyro.calibration.SensorCorrectionsUpdate();
			const float gyro_dt_s = 1.e-6f * vehicle_imu.delta_angle_dt;
			const Vector3f angular_velocity{imu.gyro.calibration.Correct(delta_angle / gyro_dt_s)};

			// Gyro high frequency vibe = filtered length of (angular_velocity - angular_velocity_prev)
			imu.gyro.vibration_metric = 0.99f * imu.gyro.vibration_metric + 0.01f * Vector3f(angular_velocity -
						    imu.gyro.angular_velocity_prev).norm();
			imu.gyro.angular_velocity_prev = angular_velocity;

			const Vector3f delta_angle_corrected{angular_velocity * gyro_dt_s};

			// accumulate delta angle coning corrections
			imu.gyro.coning_norm_accum += accumulated_coning_corrections.norm() * gyro_dt_s;
			imu.gyro.coning_norm_accum_total_time_s += gyro_dt_s;

			// delta velocity: apply offsets, scale, and board rotation
			imu.accel.calibration.SensorCorrectionsUpdate();
			const float accel_dt_s = 1.e-6f * vehicle_imu.delta_velocity_dt;
			const Vector3f acceleration{imu.accel.calibration.Correct(delta_velocity / accel_dt_s)};

			// Accel high frequency vibe = filtered length of (acceleration - acceleration_prev)
			imu.accel.vibration_metric = 0.99f * imu.accel.vibration_metric + 0.01f * Vector3f(acceleration -
						     imu.accel.acceleration_prev).norm();
			imu.accel.acceleration_prev = acceleration;

			const Vector3f delta_velocity_corrected{acceleration * accel_dt_s};


			// vehicle_imu_status
			//  publish before vehicle_imu so that error counts are available synchronously if needed
			const bool status_publish_interval_exceeded = (hrt_elapsed_time(&imu.time_last_status_publication) >= 100_ms);

			if (imu.accel.raw_mean.valid() && imu.gyro.raw_mean.valid()
			    && imu.accel.mean_interval_us.valid() && imu.gyro.mean_interval_us.valid()
			    && (imu.publish_status || status_publish_interval_exceeded)
			   ) {

				vehicle_imu_status_s imu_status{};

				// Accel
				{
					imu_status.accel_device_id = imu.accel.calibration.device_id();

					imu_status.accel_error_count = imu.accel.error_count;

					imu_status.accel_clipping[0] = imu.accel.clipping_total[0];
					imu_status.accel_clipping[1] = imu.accel.clipping_total[1];
					imu_status.accel_clipping[2] = imu.accel.clipping_total[2];

					imu_status.accel_rate_hz = 1e6f / imu.accel.mean_interval_us.mean(); // TODO: should be for actual publication
					imu_status.accel_raw_rate_hz = 1e6f / imu.accel.mean_interval_us.mean();

					// accel mean and variance
					const Dcmf &R = imu.accel.calibration.rotation();
					Vector3f(R * imu.accel.raw_mean.mean()).copyTo(imu_status.mean_accel);

					// variance from R * COV * R^T
					const Matrix3f cov = R * imu.accel.raw_mean.covariance() * R.transpose();
					cov.diag().copyTo(imu_status.var_accel);

					imu_status.accel_vibration_metric = imu.accel.vibration_metric;

					// temperature
					if (imu.accel.temperature.valid()) {
						imu_status.temperature_accel = imu.accel.temperature.mean();

					} else {
						imu_status.temperature_accel = NAN;
					}
				}

				// Gyro
				{
					imu_status.gyro_device_id = imu.gyro.calibration.device_id();

					imu_status.gyro_error_count = imu.gyro.error_count;

					imu_status.gyro_clipping[0] = imu.gyro.clipping_total[0];
					imu_status.gyro_clipping[1] = imu.gyro.clipping_total[1];
					imu_status.gyro_clipping[2] = imu.gyro.clipping_total[2];

					imu_status.gyro_rate_hz = 1e6f / imu.gyro.mean_interval_us.mean(); // TODO: should be for actual publication
					imu_status.gyro_raw_rate_hz = 1e6f / imu.gyro.mean_interval_us.mean();

					// gyro mean and variance
					const Dcmf &R = imu.gyro.calibration.rotation();
					Vector3f(R * imu.gyro.raw_mean.mean()).copyTo(imu_status.mean_gyro);

					// variance from R * COV * R^T
					const Matrix3f cov = R * imu.gyro.raw_mean.covariance() * R.transpose();
					cov.diag().copyTo(imu_status.var_gyro);

					imu_status.gyro_vibration_metric = imu.gyro.vibration_metric;

					// Gyro delta angle coning metric = length of coning corrections averaged since last status publication
					imu_status.delta_angle_coning_metric = imu.gyro.coning_norm_accum / imu.gyro.coning_norm_accum_total_time_s;
					imu.gyro.coning_norm_accum = 0;
					imu.gyro.coning_norm_accum_total_time_s = 0;

					// temperature
					if (imu.gyro.temperature.valid()) {
						imu_status.temperature_gyro = imu.gyro.temperature.mean();

					} else {
						imu_status.temperature_gyro = NAN;
					}
				}

				// publish
				imu_status.timestamp = hrt_absolute_time();
				imu.vehicle_imu_status_pub.publish(imu_status);

				imu.time_last_status_publication = imu_status.timestamp;
				imu.publish_status = false;

				if (status_publish_interval_exceeded) {
					imu.accel.raw_mean.reset();
					imu.accel.raw_mean.reset();
				}
			}

			// publish vehicle_imu
			vehicle_imu.timestamp_sample = imu.gyro.timestamp_sample_last;
			vehicle_imu.accel_device_id = imu.accel.calibration.device_id();
			vehicle_imu.gyro_device_id = imu.gyro.calibration.device_id();
			delta_angle_corrected.copyTo(vehicle_imu.delta_angle);
			delta_velocity_corrected.copyTo(vehicle_imu.delta_velocity);
			vehicle_imu.delta_angle_clipping = imu.gyro.clipping_flags;
			vehicle_imu.delta_velocity_clipping = imu.accel.clipping_flags;
			vehicle_imu.accel_calibration_count = imu.accel.calibration.calibration_count();
			vehicle_imu.gyro_calibration_count = imu.gyro.calibration.calibration_count();
			vehicle_imu.timestamp = hrt_absolute_time();
			imu.vehicle_imu_pub.publish(vehicle_imu);

			// reset clipping flags
			imu.gyro.clipping_flags = 0;
			imu.accel.clipping_flags = 0;

			// record gyro publication latency and integrated samples
			//_gyro_publish_latency_mean_us.update(imu.timestamp - _gyro_timestamp_last);
			//_gyro_update_latency_mean_us.update(imu.timestamp - _gyro_timestamp_sample_last);

			updated = true;
		}
	}

	return updated;
}

void VehicleIMU::PrintStatus()
{
	// PX4_INFO_RAW("[vehicle_angular_velocity] selected sensor: %" PRIu32
	// 	     ", rate: %.1f Hz %s, estimated bias: [%.5f %.5f %.5f]\n",
	// 	     _gyro_calibration.device_id(), (double)_filter_sample_rate_hz, _fifo_available ? "FIFO" : "",
	// 	     (double)_gyro_bias(0), (double)_gyro_bias(1), (double)_gyro_bias(2));

	// _gyro_calibration.PrintStatus();


	// PX4_INFO_RAW("[vehicle_imu_fifo] %" PRIu8 " - IMU: %" PRIu32 ", interval: %.1f us (SD %.1f us)\n",
	// 	     _instance, _accel_calibration.device_id(), (double)_interval_us, (double)sqrtf(_interval_best_variance));

	// PX4_INFO_RAW("gyro update mean sample latency: %.6f s, publish latency %.6f s, gyro interval %.6f s",
	// 	     (double)_update_latency_mean.mean()(0),
	// 	     (double)_update_latency_mean.mean()(1),
	// 	     (double)(_interval_us * 1e-6f));

	// perf_print_counter(_imu_generation_gap_perf);

	// _accel_calibration.PrintStatus();
	// _gyro_calibration.PrintStatus();

	_vehicle_angular_velocity.PrintStatus();



	perf_print_counter(_cycle_perf);
	perf_print_counter(_filter_reset_perf);
	perf_print_counter(_selection_changed_perf);
}

} // namespace sensors
