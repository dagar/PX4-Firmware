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

#include "VehicleIMU.hpp"

#include <px4_platform_common/log.h>

using namespace matrix;
using namespace time_literals;
using math::radians;

VehicleIMU::VehicleIMU() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::att_pos_ctrl)
{
}

VehicleIMU::~VehicleIMU()
{
	Stop();
}

bool
VehicleIMU::Start()
{
	// initialize thermal corrections as we might not immediately get a topic update (only non-zero values)
	_scale = Vector3f{1.0f, 1.0f, 1.0f};
	_offset.zero();
	_bias.zero();

	// force initial updates
	ParametersUpdate(true);
	SensorBiasUpdate(true);

	// needed to change the active sensor if the primary stops updating
	_sensor_selection_sub.registerCallback();

	if (SensorSelectionUpdate(true)) {
		SensorCorrectionsUpdate(true);
		ParametersUpdate(true);
		SensorBiasUpdate(true);

		return true;
	}

	return false;
}

void
VehicleIMU::Stop()
{
	Deinit();

	// clear all registered callbacks
	for (auto &sub : _sensor_gyro_sub) {
		sub.unregisterCallback();
	}

	_sensor_selection_sub.unregisterCallback();
}

void
VehicleIMU::SensorBiasUpdate(bool force)
{
	if (_sensor_bias_sub.updated() || force) {
		sensor_bias_s bias;

		if (_sensor_bias_sub.copy(&bias)) {
			// TODO: should be checking device ID
			_bias = Vector3f{bias.accel_bias};
		}
	}
}

void
VehicleIMU::SensorCorrectionsUpdate(bool force)
{
	// check if the selected sensor has updated
	if (_sensor_correction_sub.updated() || force) {

		sensor_correction_s corrections{};
		_sensor_correction_sub.copy(&corrections);

		// find in sensor corrections
		if ((_sensor_correction_index < 0)
		    || (corrections.accel_device_id[_sensor_correction_index] != _selected_sensor_device_id)) {

			for (int correction_index = 0; correction_index < MAX_SENSOR_COUNT; correction_index++) {
				if (corrections.accel_device_id[correction_index] == _selected_sensor_device_id) {
					_sensor_correction_index = correction_index;
				}
			}
		}

		if (_sensor_correction_index == 0) {
			_offset = Vector3f{corrections.accel_offset_0};
			_scale = Vector3f{corrections.accel_scale_0};

		} else if (_sensor_correction_index == 1) {
			_offset = Vector3f{corrections.accel_offset_1};
			_scale = Vector3f{corrections.accel_scale_1};

		} else if (_sensor_correction_index == 2) {
			_offset = Vector3f{corrections.accel_offset_2};
			_scale = Vector3f{corrections.accel_scale_2};

		} else {
			_offset = Vector3f{0.0f, 0.0f, 0.0f};
			_scale = Vector3f{1.0f, 1.0f, 1.0f};
		}
	}
}

bool
VehicleIMU::SensorSelectionUpdate(bool force)
{
	// check if the selected sensor has updated
	if (_sensor_selection_sub.updated() || force) {

		sensor_selection_s selection{};
		_sensor_selection_sub.copy(&selection);

		// update the latest sensor selection
		if ((_selected_sensor_device_id != selection.accel_device_id) || force) {

			// clear all registered callbacks
			for (auto &sub : _sensor_gyro_sub) {
				sub.unregisterCallback();
			}

			for (int i = 0; i < MAX_SENSOR_COUNT; i++) {
				sensor_accel_s report{};
				_sensor_gyro_sub[i].copy(&report);

				if ((report.device_id != 0) && (report.device_id == _selected_sensor_device_id)) {
					if (_sensor_gyro_sub[i].registerCallback()) {
						PX4_DEBUG("selected sensor changed %d -> %d", _selected_sensor_index, i);

						// record selected sensor
						_selected_sensor_index = i;

						return true;
					}
				}
			}
		}
	}

	return false;
}

void
VehicleIMU::ParametersUpdate(bool force)
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

void
VehicleIMU::Run()
{
	// check for updated selected sensor
	bool sensor_select_update = SensorSelectionUpdate();
	SensorCorrectionsUpdate(sensor_select_update);
	ParametersUpdate();
	SensorBiasUpdate();

	if (_sensor_gyro_sub[_selected_sensor_index].updated() || sensor_select_update) {

		sensor_accel_s sensor_accel_data;
		sensor_gyro_s sensor_gyro_data;

		_sensor_gyro_sub[_selected_sensor_index].copy(&sensor_gyro_data);
		_sensor_accel_sub[_selected_sensor_index].copy(&sensor_accel_data);

		//const Vector3f val{sensor_data.x, sensor_data.y, sensor_data.z};

		// apply offsets and scale (thermal correction)
		// rotate corrected measurements from sensor to vehicle body frame
		// correct for in-run bias errors
		//const Vector3f imu = _board_rotation * (val - _offset).emult(_scale) - _bias;

		// publish vehicle_acceleration
		vehicle_imu_s out;
		//accel.copyTo(out.xyz);
		out.timestamp = hrt_absolute_time();
		_vehicle_imu_pub.publish(out);
	}
}

void
VehicleIMU::PrintStatus()
{
	PX4_INFO_RAW("\n");

	PX4_INFO("selected sensor: %d (sensor_accel:%d)", _selected_sensor_device_id, _selected_sensor_index);

	PX4_INFO("offsets: %.5f %.5f %.5f", (double)_offset(0), (double)_offset(1), (double)_offset(2));
	PX4_INFO("scale: %.5f %.5f %.5f", (double)_scale(0), (double)_scale(1), (double)_scale(2));
	PX4_INFO("bias: %.5f %.5f %.5f", (double)_bias(0), (double)_bias(1), (double)_bias(2));
}
