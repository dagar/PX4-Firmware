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

#include "DifferentialPressure.hpp"

#include "Utilities.hpp"

#include <lib/parameters/param.h>

using namespace matrix;
using namespace time_literals;

namespace calibration
{

DifferentialPressure::DifferentialPressure()
{
	Reset();
}

DifferentialPressure::DifferentialPressure(uint32_t device_id)
{
	Reset();
	set_device_id(device_id);
}

void DifferentialPressure::set_device_id(uint32_t device_id)
{
	if (_device_id != device_id) {
		_device_id = device_id;
		ParametersUpdate();
	}
}

bool DifferentialPressure::set_offset(const float &offset)
{
	if (fabsf(_offset - offset) > 0.01f) {
		_offset = offset;

		_calibration_count++;
		return true;
	}

	return false;
}

void DifferentialPressure::ParametersUpdate()
{
	if (_device_id == 0) {
		Reset();
		return;
	}

	_calibration_index = FindCalibrationIndex(SensorString(), _device_id);

	if (_calibration_index >= 0) {
		// CAL_DPRESx_PRIO
		_priority = GetCalibrationParam(SensorString(), "PRIO", _calibration_index);

		if ((_priority < 0) || (_priority > 100)) {
			// reset to default, -1 is the uninitialized parameter value
			int32_t new_priority = DEFAULT_PRIORITY;

			if (_priority != -1) {
				PX4_ERR("%s %d (%d) invalid priority %d, resetting to %d", SensorString(), _device_id, _calibration_index, _priority,
					new_priority);
			}

			SetCalibrationParam(SensorString(), "PRIO", _calibration_index, new_priority);
			_priority = new_priority;
		}

		// CAL_DPRESx_OFF
		set_offset(GetCalibrationParam(SensorString(), "OFF", _calibration_index));

	} else {
		Reset();
	}
}

void DifferentialPressure::Reset()
{
	_offset = 0;

	_priority = DEFAULT_PRIORITY;

	_calibration_index = -1;

	_calibration_count = 0;
}

bool DifferentialPressure::ParametersSave()
{
	if (_calibration_index >= 0) {
		// save calibration
		bool success = true;
		success &= SetCalibrationParam(SensorString(), "ID", _calibration_index, _device_id);
		success &= SetCalibrationParam(SensorString(), "PRIO", _calibration_index, _priority);
		success &= SetCalibrationParam(SensorString(), "OFF", _calibration_index, _offset);

		return success;
	}

	return false;
}

void DifferentialPressure::PrintStatus()
{
	PX4_INFO("%s %d EN: %d, offset: %.4f", SensorString(), device_id(), enabled(),
		 (double)_offset);
}

} // namespace calibration
