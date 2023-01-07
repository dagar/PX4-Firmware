/****************************************************************************
 *
 *   Copyright (c) 2019-2023 PX4 Development Team. All rights reserved.
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

using namespace matrix;

namespace sensors
{

static constexpr bool clipping(const int16_t sample) { return (sample == INT16_MIN) || (sample == INT16_MAX); }

static constexpr int clipping(const int16_t samples[], int length)
{
	int clip_count = 0;

	for (int n = 0; n < length; n++) {
		if (clipping(samples[n])) {
			clip_count++;
		}
	}

	return clip_count;
}

static constexpr int32_t sum(const int16_t samples[], int length)
{
	int32_t sum = 0;

	for (int n = 0; n < length; n++) {
		sum += samples[n];
	}

	return sum;
}

static matrix::Vector3f AverageFifoAccel(const sensor_imu_fifo_s &sensor_imu_fifo)
{
	// X axis
	int32_t x = 0;

	for (int i = 0; i < sensor_imu_fifo.samples; i++) {
		x += sensor_imu_fifo.accel_x[i];
	}

	// Y axis
	int32_t y = 0;

	for (int i = 0; i < sensor_imu_fifo.samples; i++) {
		y += sensor_imu_fifo.accel_y[i];
	}

	// Z axis
	int32_t z = 0;

	for (int i = 0; i < sensor_imu_fifo.samples; i++) {
		z += sensor_imu_fifo.accel_z[i];
	}

	const float scale = sensor_imu_fifo.accel_scale / sensor_imu_fifo.samples;
	return matrix::Vector3f{x * scale, y * scale, z * scale};
}

static matrix::Vector3f AverageFifoGyro(const sensor_imu_fifo_s &sensor_imu_fifo)
{
	// X axis
	int32_t x = 0;

	for (int i = 0; i < sensor_imu_fifo.samples; i++) {
		x += sensor_imu_fifo.gyro_x[i];
	}

	// Y axis
	int32_t y = 0;

	for (int i = 0; i < sensor_imu_fifo.samples; i++) {
		y += sensor_imu_fifo.gyro_y[i];
	}

	// Z axis
	int32_t z = 0;

	for (int i = 0; i < sensor_imu_fifo.samples; i++) {
		z += sensor_imu_fifo.gyro_z[i];
	}

	const float scale = sensor_imu_fifo.gyro_scale / sensor_imu_fifo.samples;
	return matrix::Vector3f{x * scale, y * scale, z * scale};
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
	perf_free(_selection_changed_perf);
}

bool VehicleIMU::Start()
{
	// force initial updates
	ParametersUpdate(true);

	_imus[0].primary = true;

	for (auto &sub : _sensor_imu_fifo_subs) {
		sub.set_required_updates(sensor_imu_fifo_s::ORB_QUEUE_LENGTH);
		sub.registerCallback();
	}

	ScheduleNow();

	return true;
}

void VehicleIMU::Stop()
{
	// clear all registered callbacks
	for (auto &sub : _sensor_imu_fifo_subs) {
		sub.unregisterCallback();
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

		_vehicle_acceleration.ParametersUpdate();
		_vehicle_angular_velocity.ParametersUpdate();

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
	ScheduleDelayed(1.5e6 / _param_imu_gyro_ratemax.get()); // backup 150% of expected/desired interval

	//const hrt_abstime time_now_us = hrt_absolute_time();

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



	// TODO: IMU at reset (across all enabled)
	//  - if still don't reset gyro/accel welford mean, instead use it for calibration
	//   - delete gyro_calibraiton module


	//  accel integration on raw int16 data

	// sensor_imu_fifo -> imu instance
	//  sensor_accel + sensor_gyro -> imu instance
	//  sensors_combined, sensor selection



	perf_end(_cycle_perf);
}

int8_t VehicleIMU::findAccelInstance(uint32_t device_id)
{
	if (device_id == 0) {
		return -1;
	}

	// look for existing matching
	for (int i = 0; i < MAX_SENSOR_COUNT; i++) {
		if (_imus[i].accel.device_id == device_id) {
			return i;
		}
	}

	// look for empty slot
	for (int i = 0; i < MAX_SENSOR_COUNT; i++) {
		if (_imus[i].accel.device_id == 0) {
			return i;
		}
	}

	return -1;
}

void VehicleIMU::UpdateSensorImuFifo(uint8_t sensor_instance)
{
	// sensor_imu_fifo
	auto &sub = _sensor_imu_fifo_subs[sensor_instance];

	while (sub.updated()) {
		unsigned last_generation = sub.get_last_generation();
		sensor_imu_fifo_s sensor_imu_fifo;
		sub.copy(&sensor_imu_fifo);

		int8_t imu_instance = findAccelInstance(sensor_imu_fifo.device_id);

		if (imu_instance == -1) {
			return;
		}

		IMU &imu = _imus[imu_instance];

		// publication interval
		//  sensor output data rate or interval
		if (sub.get_last_generation() == last_generation + 1) {

			if ((sensor_imu_fifo.timestamp_sample > imu.accel.timestamp_sample_last)
			    && (sensor_imu_fifo.timestamp_sample > imu.gyro.timestamp_sample_last)
			   ) {

				const float interval_us = sensor_imu_fifo.timestamp_sample - imu.accel.timestamp_sample_last;
				const float sample_interval_us = interval_us / sensor_imu_fifo.samples;

				imu.accel.mean_publish_interval_us.update(interval_us);
				imu.accel.mean_sample_interval_us.update(sample_interval_us);

				imu.gyro.mean_publish_interval_us.update(interval_us);
				imu.gyro.mean_sample_interval_us.update(sample_interval_us);

				// check and update sensor rate
				if (imu.gyro.mean_publish_interval_us.valid() &&
				    (imu.gyro.mean_publish_interval_us.count() > 1000 || imu.gyro.mean_publish_interval_us.standard_deviation() < 100)
				   ) {

					// determine number of sensor samples that will get closest to the desired integration interval
					const float imu_integration_interval_us = 1e6f / _param_imu_integ_rate.get();
					const float pub_interval_avg_us = imu.gyro.mean_publish_interval_us.mean();

					const uint8_t imu_publications = roundf(imu_integration_interval_us / pub_interval_avg_us);

					const float integration_interval_us = roundf(imu_publications * pub_interval_avg_us);

					// TODO: variance
					if ((static_cast<float>(imu.gyro.integrator.reset_interval_us()) - integration_interval_us) >
					    imu.gyro.mean_publish_interval_us.standard_deviation()
					    // || primary_changed
					   ) {

						const auto accel_reset_samples_prev = imu.accel.integrator.get_reset_samples();
						const auto gyro_reset_samples_prev = imu.gyro.integrator.get_reset_samples();

						imu.gyro.integrator.set_reset_interval(integration_interval_us);
						imu.accel.integrator.set_reset_interval(integration_interval_us / 2.f);

						// number of samples per publication
						int interval_samples = roundf(integration_interval_us / imu.gyro.mean_sample_interval_us.mean());
						imu.gyro.integrator.set_reset_samples(math::max(interval_samples, 1));
						imu.accel.integrator.set_reset_samples(1);

						if (accel_reset_samples_prev != imu.accel.integrator.get_reset_samples()
						    || gyro_reset_samples_prev != imu.gyro.integrator.get_reset_samples()
						    || (static_cast<float>(imu.gyro.integrator.reset_interval_us()) - integration_interval_us > 100)
						   ) {

							if (imu.primary) {
								sub.set_required_updates(1);

							} else {
								// avg samples per publication
								float samples_per_pub = imu.gyro.mean_publish_interval_us.mean() / imu.gyro.mean_sample_interval_us.mean();
								uint8_t pubs_per_integrator_reset = floorf(imu.gyro.integrator.get_reset_samples() / samples_per_pub);

								sub.set_required_updates(math::constrain(pubs_per_integrator_reset, (uint8_t)1, sensor_imu_fifo_s::ORB_QUEUE_LENGTH));
							}

							sub.registerCallback();

							imu.accel.interval_configured = true;
							imu.gyro.interval_configured = true;

							PX4_INFO("IMU FIFO (%" PRIu32 "), publish interval: %.1f us (STD:%.1f us), integrator: %.1f us, %d samples",
								 imu.gyro.calibration.device_id(),
								 (double)imu.gyro.mean_publish_interval_us.mean(), (double)imu.gyro.mean_publish_interval_us.standard_deviation(),
								 (double)imu.gyro.integrator.reset_interval_us(), imu.gyro.integrator.get_reset_samples()
								);
						}

					}
				}
			}
		}

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
		//const float inverse_dt_s = 1e6f / sensor_imu_fifo.dt;

		// integrate accel
		for (int n = 0; n < N; n++) {
			const Vector3f accel_raw{
				static_cast<float>(sensor_imu_fifo.accel_x[n]) *sensor_imu_fifo.accel_scale,
				static_cast<float>(sensor_imu_fifo.accel_y[n]) *sensor_imu_fifo.accel_scale,
				static_cast<float>(sensor_imu_fifo.accel_z[n]) *sensor_imu_fifo.accel_scale};
			imu.accel.integrator.put(accel_raw, dt_s);
			//imu.accel.raw_mean.update(accel_raw);
		}


		// integrate accel

		// check for scale change
		if (fabsf(sensor_imu_fifo.accel_scale - imu.accel.fifo_scale) > FLT_EPSILON) {
			// rescale last sample on scale change
			const float rescale = imu.accel.fifo_scale / sensor_imu_fifo.accel_scale;

			imu.accel.last_fifo_sample[0] = roundf(imu.accel.last_fifo_sample[0] * rescale);
			imu.accel.last_fifo_sample[1] = roundf(imu.accel.last_fifo_sample[1] * rescale);
			imu.accel.last_fifo_sample[2] = roundf(imu.accel.last_fifo_sample[2] * rescale);

			imu.accel.fifo_scale = sensor_imu_fifo.accel_scale;
		}


		// trapezoidal integration (equally spaced)
		const float scale = dt_s * sensor_imu_fifo.accel_scale;
		imu.accel.integral(0) += (0.5f * (imu.accel.last_fifo_sample[0] + sensor_imu_fifo.accel_x[N - 2])
					  + sum(sensor_imu_fifo.accel_x, N - 1)) * scale;
		imu.accel.last_fifo_sample[0] = sensor_imu_fifo.accel_x[N - 1];

		imu.accel.integral(1) += (0.5f * (imu.accel.last_fifo_sample[1] + sensor_imu_fifo.accel_y[N - 2])
					  + sum(sensor_imu_fifo.accel_y, N - 1)) * scale;
		imu.accel.last_fifo_sample[1] = sensor_imu_fifo.accel_y[N - 1];

		imu.accel.integral(2) += (0.5f * (imu.accel.last_fifo_sample[2] + sensor_imu_fifo.accel_z[N - 2])
					  + sum(sensor_imu_fifo.accel_z, N - 1)) * scale;
		imu.accel.last_fifo_sample[2] = sensor_imu_fifo.accel_z[N - 1];





		const Vector3f accel_raw_avg{AverageFifoAccel(sensor_imu_fifo)};

		imu.accel.raw_mean.update(accel_raw_avg);

		if (imu.primary && imu.accel.interval_configured && imu.accel.mean_publish_interval_us.valid()) {
			float sample_rate_hz = 1e6f / imu.accel.mean_publish_interval_us.mean();
			_vehicle_acceleration.updateAccel(imu, accel_raw_avg, sample_rate_hz);
		}

		// accel clipping
		{
			const Vector3f clip_counter{
				(float)clipping(sensor_imu_fifo.accel_x, sensor_imu_fifo.samples),
				(float)clipping(sensor_imu_fifo.accel_y, sensor_imu_fifo.samples),
				(float)clipping(sensor_imu_fifo.accel_z, sensor_imu_fifo.samples)};

			if (clip_counter(0) > 0 || clip_counter(1) > 0 || clip_counter(2) > 0) {
				// rotate sensor clip counts into vehicle body frame
				const Vector3f clipping{imu.accel.calibration.rotation() *clip_counter};

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
				static_cast<float>(sensor_imu_fifo.gyro_z[n]) *sensor_imu_fifo.gyro_scale};
			imu.gyro.integrator.put(gyro_raw, dt_s);
			//imu.gyro.raw_mean.update(gyro_raw);
		}

		imu.gyro.raw_mean.update(AverageFifoGyro(sensor_imu_fifo));

		// gyro clipping
		{
			const Vector3f clip_counter{
				(float)clipping(sensor_imu_fifo.gyro_x, sensor_imu_fifo.samples),
				(float)clipping(sensor_imu_fifo.gyro_y, sensor_imu_fifo.samples),
				(float)clipping(sensor_imu_fifo.gyro_z, sensor_imu_fifo.samples)};


			if (clip_counter(0) > 0 || clip_counter(1) > 0 || clip_counter(2) > 0) {
				// rotate sensor clip counts into vehicle body frame
				const Vector3f clipping{imu.gyro.calibration.rotation() *clip_counter};

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


			// Vector3f delta_velocity{imu.accel.integral * imu.accel.fifo_scale * dt_s};
			// imu.accel.integral.zero();
			// uint16_t delta_velocity_dt = delta_angle_dt;

			// TODO: rescale if changed?



			PublishImu(imu);

			imu.accel.integral.zero();
		}

		// TODO: call to filter data
		if (imu.primary && imu.gyro.interval_configured) {

			// TODO: generic update?
			//_vehicle_angular_velocity.update(imu)?
			// for (int axis = 0; axis < 3; axis++) {
			//	_vehicle_angular_velocity.updateAxis(axis, raw_data[], length)?
			// }

			_vehicle_angular_velocity.updateSensorImuFifo(imu, sensor_imu_fifo);

			// TODO: or do it axis by axis?
			//  float raw_data_array[]

			// update per axis?
			// publish when
			if (!sub.updated()) {
				// _vehicle_angular_velocity.publish()?
			}
		}
	}
}

void VehicleIMU::UpdateSensorAccel(uint8_t sensor_instance)
{


}

void VehicleIMU::UpdateSensorGyro(uint8_t sensor_instance)
{


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
			delta_velocity = imu.accel.integral; // TODO
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
			    && imu.accel.mean_sample_interval_us.valid() && imu.gyro.mean_sample_interval_us.valid()
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

					imu_status.accel_rate_hz = 1e6f / imu.accel.mean_publish_interval_us.mean();
					imu_status.accel_raw_rate_hz = 1e6f / imu.accel.mean_sample_interval_us.mean();

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

					imu_status.gyro_rate_hz = 1e6f / imu.gyro.mean_publish_interval_us.mean();
					imu_status.gyro_raw_rate_hz = 1e6f / imu.gyro.mean_sample_interval_us.mean();

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

			if (imu.primary) {

				sensor_combined_s sensor_combined{};

				sensor_combined.timestamp = vehicle_imu.timestamp_sample;
				sensor_combined.accelerometer_m_s2[0] = acceleration(0);
				sensor_combined.accelerometer_m_s2[1] = acceleration(1);
				sensor_combined.accelerometer_m_s2[2] = acceleration(2);
				sensor_combined.accelerometer_integral_dt = vehicle_imu.delta_velocity_dt;
				sensor_combined.accelerometer_clipping = vehicle_imu.delta_velocity_clipping;
				sensor_combined.gyro_rad[0] = angular_velocity(0);
				sensor_combined.gyro_rad[1] = angular_velocity(1);
				sensor_combined.gyro_rad[2] = angular_velocity(2);
				sensor_combined.gyro_integral_dt = vehicle_imu.delta_angle_dt;
				sensor_combined.gyro_clipping = vehicle_imu.delta_angle_clipping;
				sensor_combined.accel_calibration_count = vehicle_imu.accel_calibration_count;
				sensor_combined.gyro_calibration_count = vehicle_imu.gyro_calibration_count;

				sensor_combined.accelerometer_timestamp_relative = (int32_t)((int64_t)imu.accel.timestamp_sample_last -
						(int64_t)imu.gyro.timestamp_sample_last);

				_sensor_combined_pub.publish(sensor_combined);
			}

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

	// uint32_t imu_fifo_index = 0;

	// for (auto &s : _sensor_imu_fifo_subs) {
	// 	if (s.mean_sample_interval_us.mean() > 0.f) {
	// 		PX4_INFO_RAW("[vehicle_imu] IMU FIFO (%" PRIu32 "), publish interval: %.1f us (STD:%.1f us)\n",
	// 			     imu_fifo_index++, (double)s.mean_sample_interval_us.mean(), (double)s.mean_sample_interval_us.standard_deviation());
	// 	}
	// }

	for (auto &imu : _imus) {
		if (imu.accel.calibration.device_id() != 0) {
			PX4_INFO_RAW("[vehicle_imu] IMU (%" PRIu32 "), pub:%.1f us (STD:%.1f us), sample:%.1f us (STD:%.1f us) %s\n",
				     imu.accel.calibration.device_id(),
				     (double)imu.accel.mean_publish_interval_us.mean(), (double)imu.accel.mean_publish_interval_us.standard_deviation(),
				     (double)imu.accel.mean_sample_interval_us.mean(), (double)imu.accel.mean_sample_interval_us.standard_deviation(),
				     imu.primary ? "*" : "");
		}
	}





	// PX4_INFO_RAW("[vehicle_imu_fifo] %" PRIu8 " - IMU: %" PRIu32 ", interval: %.1f us (SD %.1f us)\n",
	// 	     _instance, _accel_calibration.device_id(), (double)_interval_us, (double)sqrtf(_interval_best_variance));

	// PX4_INFO_RAW("gyro update mean sample latency: %.6f s, publish latency %.6f s, gyro interval %.6f s",
	// 	     (double)_update_latency_mean.mean()(0),
	// 	     (double)_update_latency_mean.mean()(1),
	// 	     (double)(_interval_us * 1e-6f));

	// perf_print_counter(_imu_generation_gap_perf);

	// _accel_calibration.PrintStatus();
	// _gyro_calibration.PrintStatus();

	_vehicle_acceleration.PrintStatus();
	_vehicle_angular_velocity.PrintStatus();

	perf_print_counter(_cycle_perf);
	perf_print_counter(_selection_changed_perf);
}

} // namespace sensors
