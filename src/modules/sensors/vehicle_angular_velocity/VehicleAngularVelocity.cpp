/****************************************************************************
 *
 *   Copyright (c) 2019-2020 PX4 Development Team. All rights reserved.
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

#include "VehicleAngularVelocity.hpp"

#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/topics/vehicle_imu_status.h>

#include <px4_platform_common/log.h>

using namespace matrix;
using namespace time_literals;

namespace sensors
{

VehicleAngularVelocity::VehicleAngularVelocity() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
{
	_lp_filter_velocity.set_cutoff_frequency(kInitialRateHz, _param_imu_gyro_cutoff.get());
	_notch_filter_velocity.setParameters(kInitialRateHz, _param_imu_gyro_nf_freq.get(), _param_imu_gyro_nf_bw.get());

	_lp_filter_acceleration.set_cutoff_frequency(kInitialRateHz, _param_imu_dgyro_cutoff.get());
}

VehicleAngularVelocity::~VehicleAngularVelocity()
{
	Stop();
}

bool VehicleAngularVelocity::Start()
{
	// force initial updates
	ParametersUpdate(true);

	// sensor_selection needed to change the active sensor if the primary stops updating
	if (!_sensor_selection_sub.registerCallback()) {
		PX4_ERR("sensor_selection callback registration failed");
		return false;
	}

	if (!SensorSelectionUpdate(true)) {
		ScheduleDelayed(10_ms);
	}

	return true;
}

void VehicleAngularVelocity::Stop()
{
	// clear all registered callbacks
	_sensor_sub.unregisterCallback();
	_sensor_selection_sub.unregisterCallback();

	Deinit();
}

void VehicleAngularVelocity::CheckFilters()
{
	bool reset_filters = false;

	if (!_filters_configured) {
		// get current sensor sample rate from vehicle_imu_status
		uORB::SubscriptionMultiArray<vehicle_imu_status_s, MAX_SENSOR_COUNT> vehicle_imu_status_subs{ORB_ID::vehicle_imu_status};

		for (auto &imu_status_sub : vehicle_imu_status_subs) {
			vehicle_imu_status_s vehicle_imu_status;

			if (imu_status_sub.copy(&vehicle_imu_status)) {

				if ((_selected_sensor_device_id != 0) && (vehicle_imu_status.gyro_device_id == _selected_sensor_device_id)) {
					if (vehicle_imu_status.gyro_rate_hz > 0) {
						_update_rate_hz = vehicle_imu_status.gyro_rate_hz;

						// check if sample rate error is greater than 1%
						if ((fabsf(_update_rate_hz - _filter_sample_rate) / _filter_sample_rate) > 0.01f) {
							reset_filters = true;
						}
					}
				}
			}
		}

		if (reset_filters || (_required_sample_updates == 0)) {
			if (_param_imu_gyro_rate_max.get() > 0) {
				// determine number of sensor samples that will get closest to the desired rate
				const float configured_interval_us = 1e6f / _param_imu_gyro_rate_max.get();
				const float interval_us = 1e6f / _update_rate_hz;
				const uint8_t samples = math::constrain(roundf(configured_interval_us / interval_us), 1.f,
									(float)sensor_gyro_s::ORB_QUEUE_LENGTH);

				_sensor_sub.set_required_updates(samples);
				_required_sample_updates = samples;

			} else {
				_sensor_sub.set_required_updates(1);
				_required_sample_updates = 1;
			}
		}

		// publish interval
		if (_param_imu_gyro_rate_max.get() > 0) {
			const uint64_t imu_gyro_interval = 1e6f / _param_imu_gyro_rate_max.get();
			const float interval_us = 1e6f / _update_rate_hz;
			_publish_interval_min_us = imu_gyro_interval - (interval_us / 2);

		} else {
			_publish_interval_min_us = 0;
		}
	}

	if (!reset_filters) {
		// gyro low pass cutoff frequency changed
		if (fabsf(_lp_filter_velocity.get_cutoff_freq() - _param_imu_gyro_cutoff.get()) > 0.01f) {
			reset_filters = true;
		}

		// gyro notch filter frequency or bandwidth changed
		if ((fabsf(_notch_filter_velocity.getNotchFreq() - _param_imu_gyro_nf_freq.get()) > 0.01f)
		    || (fabsf(_notch_filter_velocity.getBandwidth() - _param_imu_gyro_nf_bw.get()) > 0.01f)) {
			reset_filters = true;
		}

		// gyro derivative low pass cutoff changed
		if (fabsf(_lp_filter_acceleration.get_cutoff_freq() - _param_imu_dgyro_cutoff.get()) > 0.01f) {
			reset_filters = true;
		}
	}

	if (reset_filters) {
		PX4_DEBUG("resetting filters, sample rate: %.3f Hz -> %.3f Hz", (double)_filter_sample_rate, (double)_update_rate_hz);
		_filter_sample_rate = _update_rate_hz;

		// update software low pass filters
		_lp_filter_velocity.set_cutoff_frequency(_filter_sample_rate, _param_imu_gyro_cutoff.get());
		_lp_filter_velocity.reset(_angular_velocity_filtered_last);

		_notch_filter_velocity.setParameters(_filter_sample_rate, _param_imu_gyro_nf_freq.get(), _param_imu_gyro_nf_bw.get());
		_notch_filter_velocity.reset(_angular_velocity_filtered_last);

		_lp_filter_acceleration.set_cutoff_frequency(_filter_sample_rate, _param_imu_dgyro_cutoff.get());
		_lp_filter_acceleration.reset(_angular_acceleration_filtered_last);

		_filters_configured = true;
	}
}

void VehicleAngularVelocity::SensorBiasUpdate(bool force)
{
	if (_estimator_sensor_bias_sub.updated() || force) {
		estimator_sensor_bias_s bias;

		if (_estimator_sensor_bias_sub.copy(&bias)) {
			if (bias.gyro_device_id == _selected_sensor_device_id) {
				_bias = Vector3f{bias.gyro_bias};

			} else {
				_bias.zero();
			}
		}
	}
}

bool VehicleAngularVelocity::SensorSelectionUpdate(bool force)
{
	if (_sensor_selection_sub.updated() || (_selected_sensor_device_id == 0) || force) {
		sensor_selection_s sensor_selection{};
		_sensor_selection_sub.copy(&sensor_selection);

		if (_selected_sensor_device_id != sensor_selection.gyro_device_id) {

			// see if the selected sensor publishes sensor_gyro_fifo
			for (uint8_t i = 0; i < MAX_SENSOR_COUNT; i++) {
				uORB::SubscriptionData<sensor_gyro_fifo_s> sensor_gyro_fifo_sub{ORB_ID(sensor_gyro_fifo), i};

				if ((sensor_gyro_fifo_sub.get().device_id != 0)
				    && (sensor_gyro_fifo_sub.get().device_id == sensor_selection.gyro_device_id)) {

					if (_sensor_fifo_sub.ChangeInstance(i) && _sensor_fifo_sub.registerCallback()) {
						// record selected sensor (array index)
						_selected_sensor_sub_index = i;
						_selected_sensor_device_id = sensor_selection.gyro_device_id;

						// clear bias and corrections
						_bias.zero();

						_calibration.set_device_id(sensor_gyro_fifo_sub.get().device_id);

						// reset sample interval accumulator on sensor change
						_filters_configured = false;
						_required_sample_updates = 0;

						_fifo_available = true;

						return true;
					}
				}
			}

			for (uint8_t i = 0; i < MAX_SENSOR_COUNT; i++) {
				uORB::SubscriptionData<sensor_gyro_s> sensor_gyro_sub{ORB_ID(sensor_gyro), i};

				if ((sensor_gyro_sub.get().device_id != 0) && (sensor_gyro_sub.get().device_id == sensor_selection.gyro_device_id)) {

					if (_sensor_sub.ChangeInstance(i) && _sensor_sub.registerCallback()) {
						// record selected sensor (array index)
						_selected_sensor_sub_index = i;
						_selected_sensor_device_id = sensor_selection.gyro_device_id;

						// clear bias and corrections
						_bias.zero();

						_calibration.set_device_id(sensor_gyro_sub.get().device_id);

						// reset sample interval accumulator on sensor change
						_filters_configured = false;
						_required_sample_updates = 0;

						_fifo_available = false;

						return true;
					}
				}
			}

			PX4_ERR("unable to find or subscribe to selected sensor (%d)", sensor_selection.gyro_device_id);
			_selected_sensor_device_id = 0;
			_selected_sensor_sub_index = 0;
		}
	}

	return false;
}

void VehicleAngularVelocity::ParametersUpdate(bool force)
{
	// Check if parameters have changed
	if (_params_sub.updated() || force) {
		// clear update
		parameter_update_s param_update;
		_params_sub.copy(&param_update);

		updateParams();

		_calibration.ParametersUpdate();
	}
}

void VehicleAngularVelocity::CalibrateAndFilter(const Vector3f &gyro, const float dt)
{
	// apply calibration, rotation, and correct for in-run bias errors
	const Vector3f angular_velocity_calibrated{_calibration.Correct(gyro) - _bias};

	// Apply general notch filter (IMU_GYRO_NF_FREQ)
	const Vector3f angular_velocity_notched{_notch_filter_velocity.apply(angular_velocity_calibrated)};

	// Apply general low-pass filter (IMU_GYRO_CUTOFF)
	const Vector3f angular_velocity{_lp_filter_velocity.apply(angular_velocity_notched)};

	// Differentiate & apply specific angular acceleration (D-term) low-pass (IMU_DGYRO_CUTOFF)
	const Vector3f angular_acceleration_raw = (angular_velocity - _angular_velocity_filtered_last) / dt;
	_angular_acceleration_filtered_last = _lp_filter_acceleration.apply(angular_acceleration_raw);

	// save last filtered angular velocity
	_angular_velocity_filtered_last = angular_velocity;
}

void VehicleAngularVelocity::Run()
{
	// backup schedule
	ScheduleDelayed(10_ms);

	// update corrections first to set _selected_sensor
	bool selection_updated = SensorSelectionUpdate();

	_calibration.SensorCorrectionsUpdate(selection_updated);
	SensorBiasUpdate(selection_updated);
	ParametersUpdate();

	bool publish = false;

	if (_fifo_available) {
		// process all outstanding fifo messages
		sensor_gyro_fifo_s sensor_fifo_data;

		while (_sensor_fifo_sub.update(&sensor_fifo_data)) {
			const int N = sensor_fifo_data.samples;
			const float dt = sensor_fifo_data.dt;
			const enum Rotation fifo_rotation = static_cast<enum Rotation>(sensor_fifo_data.rotation);

			_timestamp_sample_last = sensor_fifo_data.timestamp_sample;

			for (int n = 0; n < N; n++) {
				// rotate raw FIFO sensor data
				float x = sensor_fifo_data.x[n];
				float y = sensor_fifo_data.y[n];
				float z = sensor_fifo_data.z[n];
				rotate_3f(fifo_rotation, x, y, z);

				CalibrateAndFilter(Vector3f{x, y, z} * sensor_fifo_data.scale, dt);
			}

			publish = true;
		}

	} else {
		// process all outstanding messages
		sensor_gyro_s sensor_data;

		while (_sensor_sub.update(&sensor_data)) {
			// Guard against too small (< 0.2ms) and too large (> 20ms) dt's.
			const float dt = math::constrain(((sensor_data.timestamp_sample - _timestamp_sample_last) / 1e6f), 0.0002f, 0.02f);
			_timestamp_sample_last = sensor_data.timestamp_sample;

			// get the sensor data and correct for thermal errors (apply offsets and scale)
			CalibrateAndFilter(Vector3f{sensor_data.x, sensor_data.y, sensor_data.z}, dt);

			publish = true;
		}
	}

	if (_param_imu_gyro_rate_max.get() > 0) {
		if (hrt_elapsed_time(&_last_publish) < _publish_interval_min_us) {
			publish = false;
		}
	}

	if (publish) {
		// Publish vehicle_angular_acceleration
		vehicle_angular_acceleration_s v_angular_acceleration;
		v_angular_acceleration.timestamp_sample = _timestamp_sample_last;
		_angular_acceleration_filtered_last.copyTo(v_angular_acceleration.xyz);
		v_angular_acceleration.timestamp = hrt_absolute_time();
		_vehicle_angular_acceleration_pub.publish(v_angular_acceleration);

		// Publish vehicle_angular_velocity
		vehicle_angular_velocity_s v_angular_velocity;
		v_angular_velocity.timestamp_sample = _timestamp_sample_last;
		_angular_velocity_filtered_last.copyTo(v_angular_velocity.xyz);
		v_angular_velocity.timestamp = hrt_absolute_time();
		_vehicle_angular_velocity_pub.publish(v_angular_velocity);

		_last_publish = _timestamp_sample_last;
	}
}

void VehicleAngularVelocity::PrintStatus()
{
	PX4_INFO("selected sensor: %d (%d), rate: %.1f Hz %s",
		 _selected_sensor_device_id, _selected_sensor_sub_index, (double)_update_rate_hz, _fifo_available ? "FIFO" : "");
	PX4_INFO("estimated bias: [%.4f %.4f %.4f]", (double)_bias(0), (double)_bias(1), (double)_bias(2));
	_calibration.PrintStatus();
}

} // namespace sensors
