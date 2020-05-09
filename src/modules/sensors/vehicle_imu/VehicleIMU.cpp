/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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
using namespace time_literals;

namespace sensors
{

VehicleIMU::VehicleIMU(uint8_t accel_index, uint8_t gyro_index) :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::navigation_and_controllers),
	_sensor_accel_sub(this, ORB_ID(sensor_accel), accel_index),
	_sensor_gyro_sub(this, ORB_ID(sensor_gyro), gyro_index),
	_accel_corrections(this, SensorCorrections::SensorType::Accelerometer),
	_gyro_corrections(this, SensorCorrections::SensorType::Gyroscope)
{
}

VehicleIMU::~VehicleIMU()
{
	Stop();
	perf_free(_publish_interval_perf);
	perf_free(_accel_update_perf);
	perf_free(_gyro_update_perf);
	perf_free(_integrator_sync_error_perf);
}

bool VehicleIMU::Start()
{
	// force initial updates
	ParametersUpdate(true);

	//const float interval_us = 1e6f / _param_imu_integ_rate.get();
	//set_interval_us(interval_us);

	return _sensor_accel_sub.registerCallback() && _sensor_gyro_sub.registerCallback();
}

void VehicleIMU::Stop()
{
	// clear all registered callbacks
	_sensor_accel_sub.unregisterCallback();
	_sensor_gyro_sub.unregisterCallback();

	Deinit();
}

void VehicleIMU::ParametersUpdate(bool force)
{
	// Check if parameters have changed
	if (_params_sub.updated() || force) {
		// clear update
		parameter_update_s param_update;
		_params_sub.copy(&param_update);

		updateParams();

		_accel_corrections.ParametersUpdate();
		_gyro_corrections.ParametersUpdate();
	}
}

bool VehicleIMU::UpdateIntervalAverage(IntervalAverage &intavg, const hrt_abstime &timestamp_sample)
{
	bool updated = false;

	if ((intavg.timestamp_sample_last > 0) && (timestamp_sample > intavg.timestamp_sample_last)) {
		intavg.interval_sum += (timestamp_sample - intavg.timestamp_sample_last);
		intavg.interval_count++;

	} else {
		intavg.interval_sum = 0.f;
		intavg.interval_count = 0.f;
	}

	intavg.timestamp_sample_last = timestamp_sample;

	// periodically calculate sensor update rate
	if (intavg.interval_count > 10000 || ((intavg.timestamp_sample_last == 0) && intavg.interval_count > 100)) {

		const float sample_interval_avg = intavg.interval_sum / intavg.interval_count;

		if (PX4_ISFINITE(sample_interval_avg) && (sample_interval_avg > 0.f)) {
			// check if sample rate error is greater than 1%
			if ((fabsf(intavg.update_interval - sample_interval_avg) / intavg.update_interval) > 0.01f) {

				intavg.update_interval = sample_interval_avg;
				updated = true;
			}
		}

		// reset sample interval accumulator
		intavg.timestamp_sample_last = 0;
	}

	return updated;
}

void VehicleIMU::Run()
{
	ParametersUpdate();
	_accel_corrections.SensorCorrectionsUpdate();
	_gyro_corrections.SensorCorrectionsUpdate();

	while (_sensor_accel_sub.updated() || _sensor_gyro_sub.updated()) {

		sensor_accel_s accel;
		const bool accel_updated = _sensor_accel_sub.update(&accel);

		sensor_gyro_s gyro;
		const bool gyro_updated = _sensor_gyro_sub.update(&gyro);

		bool accel_reset = false;
		Vector3f delta_velocity;
		uint32_t accel_integral_dt = 0;

		if (accel_updated) {
			perf_count_interval(_accel_update_perf, accel.timestamp_sample);
			_accel_corrections.set_device_id(accel.device_id);

			const Vector3f accel_corrected{_accel_corrections.Correct(Vector3f{accel.x, accel.y, accel.z})};

			if (_accel_integrator.put(accel.timestamp_sample, accel_corrected, delta_velocity, accel_integral_dt)) {
				accel_reset = true;
				_integrator_reset_timestamp_accel = accel.timestamp_sample;
			}

			// collect sample interval average for filters
			if (UpdateIntervalAverage(_accel_interval, accel.timestamp_sample)) {
				const float configured_interval_us = 1e6f / _param_imu_integ_rate.get();
				const float ratio = roundf(configured_interval_us / _accel_interval.update_interval);
				const float interval_us = roundf(ratio * _accel_interval.update_interval);

				// update integrator reset to a multiple of actual sensor update
				_accel_integrator.set_autoreset_interval(interval_us);

				// uORB callback limit
				_sensor_accel_sub.set_required_updates(ratio);

				PX4_INFO("accel (%d) interval updated: %.0f us (%.1f us x %.0f)", _accel_corrections.get_device_id(),
					 (double)interval_us, (double)_accel_interval.update_interval, (double)ratio);
			}
		}

		bool gyro_reset = false;
		Vector3f delta_angle;
		uint32_t gyro_integral_dt = 0;

		if (gyro_updated) {
			perf_count_interval(_gyro_update_perf, gyro.timestamp_sample);
			_gyro_corrections.set_device_id(gyro.device_id);

			const Vector3f gyro_corrected{_gyro_corrections.Correct(Vector3f{gyro.x, gyro.y, gyro.z})};

			if (_gyro_integrator.put(gyro.timestamp_sample, gyro_corrected, delta_angle, gyro_integral_dt)) {
				gyro_reset = true;
				_integrator_reset_timestamp_gyro = gyro.timestamp_sample;
			}

			// collect sample interval average for filters
			if (UpdateIntervalAverage(_gyro_interval, gyro.timestamp_sample)) {
				const float configured_interval_us = 1e6f / _param_imu_integ_rate.get();
				const float ratio = roundf(configured_interval_us / _gyro_interval.update_interval);
				const float interval_us = roundf(ratio * _gyro_interval.update_interval);

				// update integrator reset to a multiple of actual sensor update
				_gyro_integrator.set_autoreset_interval(interval_us);

				// uORB callback limit
				_sensor_gyro_sub.set_required_updates(ratio);

				PX4_INFO("gyro (%d) interval updated: %.0f us (%.1f us x %.0f)", _gyro_corrections.get_device_id(), (double)interval_us,
					 (double)_gyro_interval.update_interval, (double)ratio);
			}
		}

		bool publish = false;

		if (accel_reset && gyro_reset) {
			publish = true;

		} else if (accel_reset && !gyro_reset) {
			// force gyro integrator reset to sync with accel
			delta_angle = _gyro_integrator.reset(gyro_integral_dt);
			_integrator_reset_timestamp_gyro = _gyro_interval.timestamp_sample_last;

			publish = true;

		} else if (!accel_reset && gyro_reset) {
			// force accel integrator reset to sync with gyro
			delta_velocity = _accel_integrator.reset(accel_integral_dt);
			_integrator_reset_timestamp_accel = _accel_interval.timestamp_sample_last;

			publish = true;
		}

		if (publish) {

			// skip invalid integrations
			if (accel_integral_dt == 0 || gyro_integral_dt == 0) {
				perf_count(_integrator_sync_error_perf);
				PX4_DEBUG("accel_integral_dt: %d gyro_integral_dt: %d", accel_integral_dt, gyro_integral_dt);
				return;
			}

			// publich vehicle_imu
			perf_count(_publish_interval_perf);

			vehicle_imu_s imu{};
			imu.timestamp_sample = (_integrator_reset_timestamp_accel + _integrator_reset_timestamp_gyro) / 2;
			imu.accel_device_id = _accel_corrections.get_device_id();
			imu.gyro_device_id = _gyro_corrections.get_device_id();

			delta_angle.copyTo(imu.delta_angle);
			delta_velocity.copyTo(imu.delta_velocity);

			imu.dt = (accel_integral_dt + gyro_integral_dt) / 2;
			imu.dt_accel = accel_integral_dt;
			imu.dt_gyro = gyro_integral_dt;
			//imu.clip_count = accel.clip_count;
			imu.timestamp = hrt_absolute_time();

			_vehicle_imu_pub.publish(imu);
			return;
		}
	}
}

void VehicleIMU::PrintStatus()
{
	PX4_INFO("selected IMU: accel: %d gyro: %d ", _accel_corrections.get_device_id(), _gyro_corrections.get_device_id());
	perf_print_counter(_publish_interval_perf);
	perf_print_counter(_accel_update_perf);
	perf_print_counter(_gyro_update_perf);
	perf_print_counter(_integrator_sync_error_perf);
	_accel_corrections.PrintStatus();
	_gyro_corrections.PrintStatus();
}

} // namespace sensors
