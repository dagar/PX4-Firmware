/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

#include "Barometer.hpp"

#include "Utilities.hpp"

#include <lib/parameters/param.h>

using namespace matrix;
using namespace time_literals;

namespace calibration
{

Barometer::Barometer()
{
	Reset();
}

Barometer::Barometer(uint32_t device_id, bool external)
{
	Reset();
	set_device_id(device_id, external);
}

void Barometer::set_device_id(uint32_t device_id, bool external)
{
	if (_device_id != device_id || _external != external) {
		set_external(external);
		_device_id = device_id;
		ParametersUpdate();
		SensorCorrectionsUpdate(true);
	}
}

void Barometer::set_external(bool external)
{
	// update priority default appropriately if not set
	if (_calibration_index < 0 || _priority < 0) {
		if ((_priority < 0) || (_priority > 100)) {
			_priority = external ? DEFAULT_EXTERNAL_PRIORITY : DEFAULT_PRIORITY;

		} else if (!_external && external && (_priority == DEFAULT_PRIORITY)) {
			// internal -> external
			_priority = DEFAULT_EXTERNAL_PRIORITY;

		} else if (_external && !external && (_priority == DEFAULT_EXTERNAL_PRIORITY)) {
			// external -> internal
			_priority = DEFAULT_PRIORITY;
		}
	}

	_external = external;
}

void Barometer::SensorCorrectionsUpdate(bool force)
{
	// check if the selected sensor has updated
	if (_sensor_correction_sub.updated() || force) {

		// valid device id required
		if (_device_id == 0) {
			return;
		}

		sensor_correction_s corrections;

		if (_sensor_correction_sub.copy(&corrections)) {
			// find sensor_corrections index
			for (int i = 0; i < MAX_SENSOR_COUNT; i++) {
				if (corrections.baro_device_ids[i] == _device_id) {
					switch (i) {
					case 0:
						_thermal_offset = corrections.baro_offset_0;
						return;

					case 1:
						_thermal_offset = corrections.baro_offset_1;
						return;

					case 2:
						_thermal_offset = corrections.baro_offset_2;
						return;

					case 3:
						_thermal_offset = corrections.baro_offset_3;
						return;
					}
				}
			}
		}

		// zero thermal offset if not found
		_thermal_offset = 0.f;
	}
}

bool Barometer::set_offset(const float &offset)
{
	if (fabs(_offset - offset) > (0.1f)) {
		if (PX4_ISFINITE(offset)) {
			_offset = offset;
			_calibration_count++;
			return true;
		}
	}

	return false;
}

void Barometer::ParametersUpdate()
{
	if (_device_id == 0) {
		Reset();
		return;
	}

	_calibration_index = FindCalibrationIndex(SensorString(), _device_id);

	if (_calibration_index >= 0) {

		// CAL_BAROx_PRIO
		_priority = GetCalibrationParamInt32(SensorString(), "PRIO", _calibration_index);

		if ((_priority < 0) || (_priority > 100)) {
			// reset to default, -1 is the uninitialized parameter value
			int32_t new_priority = _external ? DEFAULT_EXTERNAL_PRIORITY : DEFAULT_PRIORITY;

			if (_priority != -1) {
				PX4_ERR("%s %d (%d) invalid priority %d, resetting to %d", SensorString(), _device_id, _calibration_index, _priority,
					new_priority);
			}

			SetCalibrationParam(SensorString(), "PRIO", _calibration_index, new_priority);
			_priority = new_priority;
		}

		// CAL_BAROx_TEMP
		set_temperature(GetCalibrationParamFloat(SensorString(), "TEMP", _calibration_index));

		// CAL_GYROx_OFF
		set_offset(GetCalibrationParamFloat(SensorString(), "OFF", _calibration_index));

	} else {
		Reset();
	}
}

void Barometer::Reset()
{
	_offset = 0;
	_thermal_offset = 0;
	_temperature = NAN;

	_priority = _external ? DEFAULT_EXTERNAL_PRIORITY : DEFAULT_PRIORITY;

	_calibration_index = -1;

	_calibration_count = 0;
}

bool Barometer::ParametersSave()
{
	if (_calibration_index >= 0) {
		// save calibration
		bool success = true;
		success &= SetCalibrationParam(SensorString(), "ID", _calibration_index, _device_id);
		success &= SetCalibrationParam(SensorString(), "PRIO", _calibration_index, _priority);
		success &= SetCalibrationParam(SensorString(), "OFF", _calibration_index, _offset);
		success &= SetCalibrationParam(SensorString(), "TEMP", _calibration_index, _temperature);

		return success;
	}

	return false;
}

void Barometer::PrintStatus()
{
	PX4_INFO("%s %d EN: %d, offset: %.4f, %.1f degC", SensorString(), device_id(), enabled(), (double)_offset,
		 (double)_temperature);

	if (fabsf(_thermal_offset) > 0.f) {
		PX4_INFO("%s %d temperature offset: %.4f", SensorString(), _device_id, (double)_thermal_offset);
	}
}

} // namespace calibration
