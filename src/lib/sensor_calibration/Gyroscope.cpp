/****************************************************************************
 *
 *   Copyright (c) 2020-2022 PX4 Development Team. All rights reserved.
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

#include "Gyroscope.hpp"

#include "Utilities.hpp"

#include <lib/parameters/param.h>

#include <px4_platform_common/param.h>

using namespace matrix;
using namespace time_literals;

namespace calibration
{

const param_t param_cal_id[Gyroscope::MAX_SENSOR_COUNT] {
	static_cast<param_t>(px4::params::CAL_GYRO0_ID),
	static_cast<param_t>(px4::params::CAL_GYRO1_ID),
	static_cast<param_t>(px4::params::CAL_GYRO2_ID),
	static_cast<param_t>(px4::params::CAL_GYRO3_ID),
};

const param_t param_cal_prio[Gyroscope::MAX_SENSOR_COUNT] {
	static_cast<param_t>(px4::params::CAL_GYRO0_PRIO),
	static_cast<param_t>(px4::params::CAL_GYRO1_PRIO),
	static_cast<param_t>(px4::params::CAL_GYRO2_PRIO),
	static_cast<param_t>(px4::params::CAL_GYRO3_PRIO),
};

const param_t param_cal_rot[Gyroscope::MAX_SENSOR_COUNT] {
	static_cast<param_t>(px4::params::CAL_GYRO0_ROT),
	static_cast<param_t>(px4::params::CAL_GYRO1_ROT),
	static_cast<param_t>(px4::params::CAL_GYRO2_ROT),
	static_cast<param_t>(px4::params::CAL_GYRO3_ROT),
};

const param_t param_cal_temp[Gyroscope::MAX_SENSOR_COUNT] {
	static_cast<param_t>(px4::params::CAL_GYRO0_TEMP),
	static_cast<param_t>(px4::params::CAL_GYRO1_TEMP),
	static_cast<param_t>(px4::params::CAL_GYRO2_TEMP),
	static_cast<param_t>(px4::params::CAL_GYRO3_TEMP),
};

static constexpr param_t param_cal_off[3][Gyroscope::MAX_SENSOR_COUNT] {
	{
		static_cast<param_t>(px4::params::CAL_GYRO0_XOFF),
		static_cast<param_t>(px4::params::CAL_GYRO1_XOFF),
		static_cast<param_t>(px4::params::CAL_GYRO2_XOFF),
		static_cast<param_t>(px4::params::CAL_GYRO3_XOFF),
	},
	{
		static_cast<param_t>(px4::params::CAL_GYRO0_YOFF),
		static_cast<param_t>(px4::params::CAL_GYRO1_YOFF),
		static_cast<param_t>(px4::params::CAL_GYRO2_YOFF),
		static_cast<param_t>(px4::params::CAL_GYRO3_YOFF),
	},
	{
		static_cast<param_t>(px4::params::CAL_GYRO0_ZOFF),
		static_cast<param_t>(px4::params::CAL_GYRO1_ZOFF),
		static_cast<param_t>(px4::params::CAL_GYRO2_ZOFF),
		static_cast<param_t>(px4::params::CAL_GYRO3_ZOFF),
	},
};

Gyroscope::Gyroscope()
{
	Reset();
}

Gyroscope::Gyroscope(uint32_t device_id)
{
	set_device_id(device_id);
}

void Gyroscope::set_device_id(uint32_t device_id)
{
	bool external = DeviceExternal(device_id);

	if (_device_id != device_id || _external != external) {

		_device_id = device_id;
		_external = external;

		Reset();

		ParametersUpdate();
		SensorCorrectionsUpdate(true);
	}
}

void Gyroscope::SensorCorrectionsUpdate(bool force)
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
				if (corrections.gyro_device_ids[i] == _device_id) {
					switch (i) {
					case 0:
						_thermal_offset = Vector3f{corrections.gyro_offset_0};
						return;
					case 1:
						_thermal_offset = Vector3f{corrections.gyro_offset_1};
						return;
					case 2:
						_thermal_offset = Vector3f{corrections.gyro_offset_2};
						return;
					case 3:
						_thermal_offset = Vector3f{corrections.gyro_offset_3};
						return;
					}
				}
			}
		}

		// zero thermal offset if not found
		_thermal_offset.zero();
	}
}

bool Gyroscope::set_offset(const Vector3f &offset)
{
	if (Vector3f(_offset - offset).longerThan(0.01f)) {
		if (PX4_ISFINITE(offset(0)) && PX4_ISFINITE(offset(1)) && PX4_ISFINITE(offset(2))) {
			_offset = offset;
			_calibration_count++;
			return true;
		}
	}

	return false;
}

void Gyroscope::set_rotation(Rotation rotation)
{
	_rotation_enum = rotation;

	// always apply board level adjustments
	_rotation = Dcmf(GetSensorLevelAdjustment()) * get_rot_matrix(rotation);
}

bool Gyroscope::set_calibration_index(int calibration_index)
{
	if ((calibration_index >= 0) && (calibration_index < MAX_SENSOR_COUNT)) {
		_calibration_index = calibration_index;
		return true;
	}

	return false;
}

void Gyroscope::ParametersUpdate()
{
	if (_device_id == 0) {
		return;
	}

	_calibration_index = FindCurrentCalibrationIndex(SensorString(), _device_id);

	if (_calibration_index == -1) {
		// no saved calibration available
		Reset();

	} else {
		ParametersLoad();
	}
}

bool Gyroscope::ParametersLoad()
{
	if (_calibration_index >= 0 && _calibration_index < MAX_SENSOR_COUNT) {
		// CAL_GYROx_ROT
		int32_t rotation_value = ROTATION_NONE;
		param_get(param_cal_rot[_calibration_index], &rotation_value);

		if (_external) {
			if ((rotation_value >= ROTATION_MAX) || (rotation_value < 0)) {
				// invalid rotation, resetting
				rotation_value = ROTATION_NONE;
			}

			set_rotation(static_cast<Rotation>(rotation_value));

		} else {
			// internal sensors follow board rotation
			set_rotation(GetBoardRotation());
		}

		// CAL_GYROx_PRIO
		param_get(param_cal_prio[_calibration_index], &_priority);

		if ((_priority < 0) || (_priority > 100)) {
			// reset to default, -1 is the uninitialized parameter value
			static constexpr int32_t CAL_PRIO_UNINITIALIZED = -1;

			if (_priority != CAL_PRIO_UNINITIALIZED) {
				PX4_ERR("%s %" PRIu32 " (%" PRId8 ") invalid priority %" PRId32 ", resetting", SensorString(), _device_id,
					_calibration_index, _priority);

				param_set_no_notification(param_cal_prio[_calibration_index], &CAL_PRIO_UNINITIALIZED);
			}

			_priority = _external ? DEFAULT_EXTERNAL_PRIORITY : DEFAULT_PRIORITY;
		}

		// CAL_GYROx_TEMP
		float cal_temp = TEMPERATURE_INVALID;
		param_get(param_cal_temp[_calibration_index], &cal_temp);

		if (cal_temp > TEMPERATURE_INVALID) {
			set_temperature(cal_temp);

		} else {
			set_temperature(NAN);
		}

		// CAL_GYROx_OFF{X,Y,Z}
		param_get(param_cal_off[0][_calibration_index], &_offset(0));
		param_get(param_cal_off[1][_calibration_index], &_offset(1));
		param_get(param_cal_off[2][_calibration_index], &_offset(2));

		return true;
	}

	return false;
}

void Gyroscope::Reset()
{
	if (_external) {
		set_rotation(ROTATION_NONE);

	} else {
		// internal sensors follow board rotation
		set_rotation(GetBoardRotation());
	}

	_offset.zero();

	_thermal_offset.zero();
	_temperature = NAN;

	_priority = _external ? DEFAULT_EXTERNAL_PRIORITY : DEFAULT_PRIORITY;

	_calibration_index = -1;

	_calibration_count = 0;
}

bool Gyroscope::ParametersSave(int desired_calibration_index, bool force)
{
	if (force && desired_calibration_index >= 0 && desired_calibration_index < MAX_SENSOR_COUNT) {
		_calibration_index = desired_calibration_index;

	} else if (!force || (_calibration_index < 0)
		   || (desired_calibration_index != -1 && desired_calibration_index != _calibration_index)) {

		// ensure we have a valid calibration slot (matching existing or first available slot)
		int8_t calibration_index_prev = _calibration_index;
		_calibration_index = FindAvailableCalibrationIndex(SensorString(), _device_id, desired_calibration_index);

		if (calibration_index_prev >= 0 && (calibration_index_prev != _calibration_index)) {
			PX4_WARN("%s %" PRIu32 " calibration index changed %" PRIi8 " -> %" PRIi8, SensorString(), _device_id,
				 calibration_index_prev, _calibration_index);
		}
	}

	if (_calibration_index >= 0 && _calibration_index < MAX_SENSOR_COUNT) {
		// save calibration
		int ret = PX4_OK;

		ret |= param_set_no_notification(param_cal_id[_calibration_index], &_device_id);
		ret |= param_set_no_notification(param_cal_prio[_calibration_index], &_priority);

		ret |= param_set_no_notification(param_cal_off[0][_calibration_index], &_offset(0));
		ret |= param_set_no_notification(param_cal_off[1][_calibration_index], &_offset(1));
		ret |= param_set_no_notification(param_cal_off[2][_calibration_index], &_offset(2));

		if (_external) {
			int32_t rot = static_cast<int32_t>(_rotation_enum);
			ret |= param_set_no_notification(param_cal_rot[_calibration_index], &rot);

		} else {
			int32_t rot = -1; // internal
			ret |= param_set_no_notification(param_cal_rot[_calibration_index], &rot);
		}

		if (PX4_ISFINITE(_temperature)) {
			ret |= param_set_no_notification(param_cal_temp[_calibration_index], &_temperature);

		} else {
			float temperature = TEMPERATURE_INVALID;
			ret |= param_set_no_notification(param_cal_temp[_calibration_index], &temperature);
		}

		return (ret == PX4_OK);
	}

	return false;
}

void Gyroscope::PrintStatus()
{
	if (external()) {
		PX4_INFO_RAW("%s %" PRIu32
			     " EN: %d, offset: [%05.3f %05.3f %05.3f], %.1f degC, Ext ROT: %d\n",
			     SensorString(), device_id(), enabled(),
			     (double)_offset(0), (double)_offset(1), (double)_offset(2),
			     (double)_temperature,
			     rotation_enum());

	} else {
		PX4_INFO_RAW("%s %" PRIu32 " EN: %d, offset: [%05.3f %05.3f %05.3f], %.1f degC, Internal\n",
			     SensorString(), device_id(), enabled(),
			     (double)_offset(0), (double)_offset(1), (double)_offset(2),
			     (double)_temperature);
	}

	if (_thermal_offset.norm() > 0.f) {
		PX4_INFO_RAW("%s %" PRIu32 " temperature offset: [%.4f %.4f %.4f]\n", SensorString(), _device_id,
			     (double)_thermal_offset(0), (double)_thermal_offset(1), (double)_thermal_offset(2));
	}
}

} // namespace calibration
