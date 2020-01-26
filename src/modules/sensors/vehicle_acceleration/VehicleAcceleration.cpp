/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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

#include "VehicleAcceleration.hpp"

#include <px4_platform_common/log.h>

using namespace matrix;
using namespace time_literals;
using math::radians;

VehicleAcceleration::VehicleAcceleration() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::att_pos_ctrl)
{
	_lowpass_filter.set_cutoff_frequency(kINITIAL_RATE_HZ, _param_imu_accel_cutoff.get());
}

VehicleAcceleration::~VehicleAcceleration()
{
	Stop();

	perf_free(_interval_perf);
}

bool VehicleAcceleration::Start()
{
	// force initial updates
	ParametersUpdate(true);

	// sensor_selection needed to change the active sensor if the primary stops updating
	if (!_sensor_selection_sub.registerCallback()) {
		PX4_ERR("sensor_selection callback registration failed");
		return false;
	}

	ScheduleNow();
	return true;
}

void VehicleAcceleration::Stop()
{
	Deinit();

	// clear all registered callbacks
	for (auto &sub : _sensor_sub) {
		sub.unregisterCallback();
	}

	_sensor_selection_sub.unregisterCallback();
}

void VehicleAcceleration::CheckFilters()
{
	if ((hrt_elapsed_time(&_filter_check_last) > 100_ms)) {
		_filter_check_last = hrt_absolute_time();

		// calculate sensor update rate
		const float sample_interval_avg = perf_mean(_interval_perf);

		if (PX4_ISFINITE(sample_interval_avg) && (sample_interval_avg > 0.0f)) {

			const float update_rate_hz = 1.0f / sample_interval_avg;

			if ((fabsf(update_rate_hz) > 0.0f) && PX4_ISFINITE(update_rate_hz)) {
				_update_rate_hz = update_rate_hz;

				// check if sample rate error is greater than 1%
				if ((fabsf(_update_rate_hz - _filter_sample_rate) / _filter_sample_rate) > 0.01f) {
					++_sample_rate_incorrect_count;
				}
			}
		}

		const bool sample_rate_updated = (_sample_rate_incorrect_count > 50);
		const bool lowpass_updated = (fabsf(_lowpass_filter.get_cutoff_freq() - _param_imu_accel_cutoff.get()) > 0.01f);

		if (sample_rate_updated || lowpass_updated) {
			PX4_INFO("updating filter, sample rate: %.3f Hz -> %.3f Hz", (double)_filter_sample_rate, (double)_update_rate_hz);
			_filter_sample_rate = _update_rate_hz;

			// update software low pass filters
			_lowpass_filter.set_cutoff_frequency(_filter_sample_rate, _param_imu_accel_cutoff.get());
			_lowpass_filter.reset(_previous_sample);

			// reset state
			_sample_rate_incorrect_count = 0;
		}
	}
}

void VehicleAcceleration::SensorBiasUpdate(bool force)
{
	if (_sensor_bias_sub.updated() || force) {
		sensor_bias_s bias;

		if (_sensor_bias_sub.copy(&bias)) {
			if (bias.accel_device_id == _selected_sensor_device_id) {
				_bias = Vector3f{bias.accel_bias};

			} else {
				_bias.zero();
			}
		}
	}
}

void VehicleAcceleration::SensorCorrectionsUpdate(bool force)
{
	// check if the selected sensor has updated
	if (_sensor_correction_sub.updated() || force) {

		sensor_correction_s corrections{};
		_sensor_correction_sub.copy(&corrections);

		// selected sensor has changed, find updated index
		if ((_corrections_selected_instance < 0) || force) {
			_corrections_selected_instance = -1;

			// find sensor_corrections index
			for (int i = 0; i < MAX_SENSOR_COUNT; i++) {
				if (corrections.accel_device_ids[i] == _selected_sensor_device_id) {
					_corrections_selected_instance = i;
				}
			}
		}

		switch (_corrections_selected_instance) {
		case 0:
			_offset = Vector3f{corrections.accel_offset_0};
			_scale = Vector3f{corrections.accel_scale_0};
			break;
		case 1:
			_offset = Vector3f{corrections.accel_offset_1};
			_scale = Vector3f{corrections.accel_scale_1};
			break;
		case 2:
			_offset = Vector3f{corrections.accel_offset_2};
			_scale = Vector3f{corrections.accel_scale_2};
			break;
		default:
			_offset = Vector3f{0.f, 0.f, 0.f};
			_scale = Vector3f{1.f, 1.f, 1.f};
		}
	}
}

bool VehicleAcceleration::SensorSelectionUpdate(bool force)
{
	if (_sensor_selection_sub.updated() || (_selected_sensor_device_id == 0) || force) {
		sensor_selection_s sensor_selection{};
		_sensor_selection_sub.copy(&sensor_selection);

		if (_selected_sensor_device_id != sensor_selection.accel_device_id) {
			// clear all registered callbacks
			for (auto &sub : _sensor_sub) {
				sub.unregisterCallback();
			}

			for (int i = 0; i < MAX_SENSOR_COUNT; i++) {
				sensor_accel_s report{};
				_sensor_sub[i].copy(&report);

				if ((report.device_id != 0) && (report.device_id == sensor_selection.accel_device_id)) {
					if (_sensor_sub[i].registerCallback()) {
						PX4_DEBUG("selected sensor changed %d -> %d", _selected_sensor_sub_index, i);

						// record selected sensor (array index)
						_selected_sensor_sub_index = i;
						_selected_sensor_device_id = sensor_selection.accel_device_id;

						// clear bias and corrections
						_bias.zero();
						_offset = Vector3f{0.f, 0.f, 0.f};
						_scale = Vector3f{1.f, 1.f, 1.f};

						// force corrections reselection
						_corrections_selected_instance = -1;

						// reset sample rate monitor
						_sample_rate_incorrect_count = 0;

						return true;
					}
				}
			}

			PX4_ERR("unable to find or subscribe to selected sensor (%d)", sensor_selection.accel_device_id);
			_selected_sensor_device_id = 0;
			_selected_sensor_sub_index = 0;
		}
	}

	return false;
}

void VehicleAcceleration::ParametersUpdate(bool force)
{
	// Check if parameters have changed
	if (_params_sub.updated() || force) {
		// clear update
		parameter_update_s param_update;
		_params_sub.copy(&param_update);

		updateParams();

		// get transformation matrix from sensor/board to body frame
		const Dcmf board_rotation = get_rot_matrix((enum Rotation)_param_sens_board_rot.get());

		// fine tune the rotation
		const Dcmf board_rotation_offset(Eulerf(
				radians(_param_sens_board_x_off.get()),
				radians(_param_sens_board_y_off.get()),
				radians(_param_sens_board_z_off.get())));

		_board_rotation = board_rotation_offset * board_rotation;
	}
}

void VehicleAcceleration::Run()
{
	// update corrections first to set _selected_sensor
	bool selection_updated = SensorSelectionUpdate();

	SensorCorrectionsUpdate(selection_updated);
	SensorBiasUpdate(selection_updated);
	ParametersUpdate();

	bool sensor_updated = _sensor_sub[_selected_sensor_sub_index].updated();

	if (sensor_updated || selection_updated) {

		Vector3f accel_filtered{};

		sensor_accel_s sensor_data{};

		// process all outstanding messages
		while (sensor_updated || selection_updated) {

			if (_sensor_sub[_selected_sensor_sub_index].copy(&sensor_data)) {

				if (sensor_updated) {
					perf_count_interval(_interval_perf, sensor_data.timestamp_sample);
				}

				// get the sensor data and correct for thermal errors (apply offsets and scale)
				const Vector3f accel{sensor_data.x, sensor_data.y, sensor_data.z};

				// Filter: apply low-pass
				CheckFilters();
				accel_filtered = _lowpass_filter.apply(accel);

				_previous_sample = accel_filtered;
			}

			sensor_updated = _sensor_sub[_selected_sensor_sub_index].updated();
			selection_updated = false;
		}

		// apply offsets and scale
		Vector3f accel{(accel_filtered - _offset).emult(_scale)};

		// rotate corrected measurements from sensor to body frame
		accel = _board_rotation * accel;

		// correct for in-run bias errors
		accel -= _bias;

		// publish
		vehicle_acceleration_s out;

		out.timestamp_sample = sensor_data.timestamp_sample;
		accel.copyTo(out.xyz);
		out.timestamp = hrt_absolute_time();

		_vehicle_acceleration_pub.publish(out);
	}
}

void VehicleAcceleration::PrintStatus()
{
	PX4_INFO("selected sensor: %d (%d)", _selected_sensor_device_id, _selected_sensor_sub_index);
	PX4_INFO("bias: [%.3f %.3f %.3f]", (double)_bias(0), (double)_bias(1), (double)_bias(2));
	PX4_INFO("offset: [%.3f %.3f %.3f]", (double)_offset(0), (double)_offset(1), (double)_offset(2));
	PX4_INFO("scale: [%.3f %.3f %.3f]", (double)_scale(0), (double)_scale(1), (double)_scale(2));

	PX4_INFO("sample rate: %.3f Hz", (double)_update_rate_hz);
	PX4_INFO("low-pass filter cutoff: %.3f Hz", (double)_lowpass_filter.get_cutoff_freq());
}
