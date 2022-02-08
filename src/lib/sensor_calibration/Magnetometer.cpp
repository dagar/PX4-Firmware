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

#include "Magnetometer.hpp"

#include "Utilities.hpp"

#include <lib/parameters/param.h>

#include <px4_platform_common/param.h>

using namespace matrix;
using namespace time_literals;

namespace calibration
{

const param_t param_cal_id[Magnetometer::MAX_SENSOR_COUNT] {
	static_cast<param_t>(px4::params::CAL_MAG0_ID),
	static_cast<param_t>(px4::params::CAL_MAG1_ID),
	static_cast<param_t>(px4::params::CAL_MAG2_ID),
	static_cast<param_t>(px4::params::CAL_MAG3_ID),
};

const param_t param_cal_prio[Magnetometer::MAX_SENSOR_COUNT] {
	static_cast<param_t>(px4::params::CAL_MAG0_PRIO),
	static_cast<param_t>(px4::params::CAL_MAG1_PRIO),
	static_cast<param_t>(px4::params::CAL_MAG2_PRIO),
	static_cast<param_t>(px4::params::CAL_MAG3_PRIO),
};

const param_t param_cal_rot[Magnetometer::MAX_SENSOR_COUNT] {
	static_cast<param_t>(px4::params::CAL_MAG0_ROT),
	static_cast<param_t>(px4::params::CAL_MAG1_ROT),
	static_cast<param_t>(px4::params::CAL_MAG2_ROT),
	static_cast<param_t>(px4::params::CAL_MAG3_ROT),
};

const param_t param_cal_temp[Magnetometer::MAX_SENSOR_COUNT] {
	static_cast<param_t>(px4::params::CAL_MAG0_TEMP),
	static_cast<param_t>(px4::params::CAL_MAG1_TEMP),
	static_cast<param_t>(px4::params::CAL_MAG2_TEMP),
	static_cast<param_t>(px4::params::CAL_MAG3_TEMP),
};

static constexpr param_t param_cal_off[3][Magnetometer::MAX_SENSOR_COUNT] {
	{
		static_cast<param_t>(px4::params::CAL_MAG0_XOFF),
		static_cast<param_t>(px4::params::CAL_MAG1_XOFF),
		static_cast<param_t>(px4::params::CAL_MAG2_XOFF),
		static_cast<param_t>(px4::params::CAL_MAG3_XOFF),
	},
	{
		static_cast<param_t>(px4::params::CAL_MAG0_YOFF),
		static_cast<param_t>(px4::params::CAL_MAG1_YOFF),
		static_cast<param_t>(px4::params::CAL_MAG2_YOFF),
		static_cast<param_t>(px4::params::CAL_MAG3_YOFF),
	},
	{
		static_cast<param_t>(px4::params::CAL_MAG0_ZOFF),
		static_cast<param_t>(px4::params::CAL_MAG1_ZOFF),
		static_cast<param_t>(px4::params::CAL_MAG2_ZOFF),
		static_cast<param_t>(px4::params::CAL_MAG3_ZOFF),
	},
};

static constexpr param_t param_cal_scale[3][Magnetometer::MAX_SENSOR_COUNT] {
	{
		static_cast<param_t>(px4::params::CAL_MAG0_XSCALE),
		static_cast<param_t>(px4::params::CAL_MAG1_XSCALE),
		static_cast<param_t>(px4::params::CAL_MAG2_XSCALE),
		static_cast<param_t>(px4::params::CAL_MAG3_XSCALE),
	},
	{
		static_cast<param_t>(px4::params::CAL_MAG0_YSCALE),
		static_cast<param_t>(px4::params::CAL_MAG1_YSCALE),
		static_cast<param_t>(px4::params::CAL_MAG2_YSCALE),
		static_cast<param_t>(px4::params::CAL_MAG3_YSCALE),
	},
	{
		static_cast<param_t>(px4::params::CAL_MAG0_ZSCALE),
		static_cast<param_t>(px4::params::CAL_MAG1_ZSCALE),
		static_cast<param_t>(px4::params::CAL_MAG2_ZSCALE),
		static_cast<param_t>(px4::params::CAL_MAG3_ZSCALE),
	},
};

static constexpr param_t param_cal_odiag[3][Magnetometer::MAX_SENSOR_COUNT] {
	{
		static_cast<param_t>(px4::params::CAL_MAG0_XODIAG),
		static_cast<param_t>(px4::params::CAL_MAG1_XODIAG),
		static_cast<param_t>(px4::params::CAL_MAG2_XODIAG),
		static_cast<param_t>(px4::params::CAL_MAG3_XODIAG),
	},
	{
		static_cast<param_t>(px4::params::CAL_MAG0_YODIAG),
		static_cast<param_t>(px4::params::CAL_MAG1_YODIAG),
		static_cast<param_t>(px4::params::CAL_MAG2_YODIAG),
		static_cast<param_t>(px4::params::CAL_MAG3_YODIAG),
	},
	{
		static_cast<param_t>(px4::params::CAL_MAG0_ZODIAG),
		static_cast<param_t>(px4::params::CAL_MAG1_ZODIAG),
		static_cast<param_t>(px4::params::CAL_MAG2_ZODIAG),
		static_cast<param_t>(px4::params::CAL_MAG3_ZODIAG),
	},
};

static constexpr param_t param_cal_comp[3][Magnetometer::MAX_SENSOR_COUNT] {
	{
		static_cast<param_t>(px4::params::CAL_MAG0_XCOMP),
		static_cast<param_t>(px4::params::CAL_MAG1_XCOMP),
		static_cast<param_t>(px4::params::CAL_MAG2_XCOMP),
		static_cast<param_t>(px4::params::CAL_MAG3_XCOMP),
	},
	{
		static_cast<param_t>(px4::params::CAL_MAG0_YCOMP),
		static_cast<param_t>(px4::params::CAL_MAG1_YCOMP),
		static_cast<param_t>(px4::params::CAL_MAG2_YCOMP),
		static_cast<param_t>(px4::params::CAL_MAG3_YCOMP),
	},
	{
		static_cast<param_t>(px4::params::CAL_MAG0_ZCOMP),
		static_cast<param_t>(px4::params::CAL_MAG1_ZCOMP),
		static_cast<param_t>(px4::params::CAL_MAG2_ZCOMP),
		static_cast<param_t>(px4::params::CAL_MAG3_ZCOMP),
	},
};

Magnetometer::Magnetometer()
{
	Reset();
}

Magnetometer::Magnetometer(uint32_t device_id)
{
	set_device_id(device_id);
}

void Magnetometer::set_device_id(uint32_t device_id)
{
	bool external = DeviceExternal(device_id);

	if (_device_id != device_id || _external != external) {

		_device_id = device_id;
		_external = external;

		Reset();

		ParametersUpdate();
	}
}

bool Magnetometer::set_offset(const Vector3f &offset)
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

bool Magnetometer::set_scale(const Vector3f &scale)
{
	if (Vector3f(_scale.diag() - scale).longerThan(0.01f)) {
		if ((scale(0) > 0.f) && (scale(1) > 0.f) && (scale(2) > 0.f) &&
		    PX4_ISFINITE(scale(0)) && PX4_ISFINITE(scale(1)) && PX4_ISFINITE(scale(2))) {

			_scale(0, 0) = scale(0);
			_scale(1, 1) = scale(1);
			_scale(2, 2) = scale(2);

			_calibration_count++;
			return true;
		}
	}

	return false;
}

bool Magnetometer::set_offdiagonal(const Vector3f &offdiagonal)
{
	if (Vector3f(Vector3f{_scale(0, 1), _scale(0, 2), _scale(1, 2)} - offdiagonal).longerThan(0.01f)) {
		if (PX4_ISFINITE(offdiagonal(0)) && PX4_ISFINITE(offdiagonal(1)) && PX4_ISFINITE(offdiagonal(2))) {

			_scale(0, 1) = offdiagonal(0);
			_scale(1, 0) = offdiagonal(0);

			_scale(0, 2) = offdiagonal(1);
			_scale(2, 0) = offdiagonal(1);

			_scale(1, 2) = offdiagonal(2);
			_scale(2, 1) = offdiagonal(2);

			_calibration_count++;
			return true;
		}
	}

	return false;
}

void Magnetometer::set_rotation(Rotation rotation)
{
	_rotation_enum = rotation;

	// always apply board level adjustments
	_rotation = Dcmf(GetSensorLevelAdjustment()) * get_rot_matrix(rotation);
}

bool Magnetometer::set_calibration_index(int calibration_index)
{
	if ((calibration_index >= 0) && (calibration_index < MAX_SENSOR_COUNT)) {
		_calibration_index = calibration_index;
		return true;
	}

	return false;
}

void Magnetometer::ParametersUpdate()
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

bool Magnetometer::ParametersLoad()
{
	if (_calibration_index >= 0 && _calibration_index < MAX_SENSOR_COUNT) {
		// CAL_MAGx_ROT
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

		// CAL_MAGx_PRIO
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

		// CAL_MAGx_TEMP
		float cal_temp = TEMPERATURE_INVALID;
		param_get(param_cal_temp[_calibration_index], &cal_temp);

		if (cal_temp > TEMPERATURE_INVALID) {
			set_temperature(cal_temp);

		} else {
			set_temperature(NAN);
		}

		// CAL_MAGx_OFF{X,Y,Z}
		param_get(param_cal_off[0][_calibration_index], &_offset(0));
		param_get(param_cal_off[1][_calibration_index], &_offset(1));
		param_get(param_cal_off[2][_calibration_index], &_offset(2));

		// CAL_MAGx_SCALE{X,Y,Z}
		param_get(param_cal_scale[0][_calibration_index], &_scale(0, 0));
		param_get(param_cal_scale[1][_calibration_index], &_scale(1, 1));
		param_get(param_cal_scale[2][_calibration_index], &_scale(2, 2));

		// CAL_MAGx_ODIAG{X,Y,Z}
		Vector3f offdiagonal{0, 0, 0};
		param_get(param_cal_odiag[0][_calibration_index], &offdiagonal(0));
		param_get(param_cal_odiag[1][_calibration_index], &offdiagonal(1));
		param_get(param_cal_odiag[2][_calibration_index], &offdiagonal(2));
		set_offdiagonal(offdiagonal);

		// CAL_MAGx_COMP{X,Y,Z}
		param_get(param_cal_comp[0][_calibration_index], &_power_compensation(0));
		param_get(param_cal_comp[1][_calibration_index], &_power_compensation(1));
		param_get(param_cal_comp[2][_calibration_index], &_power_compensation(2));

		return true;
	}

	return false;
}

void Magnetometer::Reset()
{
	if (_external) {
		set_rotation(ROTATION_NONE);

	} else {
		// internal sensors follow board rotation
		set_rotation(GetBoardRotation());
	}

	_offset.zero();
	_scale.setIdentity();

	_power_compensation.zero();
	_power = 0.f;

	_temperature = NAN;

	_priority = _external ? DEFAULT_EXTERNAL_PRIORITY : DEFAULT_PRIORITY;

	_calibration_index = -1;

	_calibration_count = 0;
}

bool Magnetometer::ParametersSave(int desired_calibration_index, bool force)
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

		// scale (diagonal)
		ret |= param_set_no_notification(param_cal_scale[0][_calibration_index], &_scale(0, 0));
		ret |= param_set_no_notification(param_cal_scale[1][_calibration_index], &_scale(1, 1));
		ret |= param_set_no_notification(param_cal_scale[2][_calibration_index], &_scale(2, 2));

		// offdiagonal
		ret |= param_set_no_notification(param_cal_odiag[0][_calibration_index], &_scale(0, 1));
		ret |= param_set_no_notification(param_cal_odiag[1][_calibration_index], &_scale(0, 2));
		ret |= param_set_no_notification(param_cal_odiag[2][_calibration_index], &_scale(1, 2));

		// power compensation
		ret |= param_set_no_notification(param_cal_comp[0][_calibration_index], &_power_compensation(0));
		ret |= param_set_no_notification(param_cal_comp[1][_calibration_index], &_power_compensation(1));
		ret |= param_set_no_notification(param_cal_comp[2][_calibration_index], &_power_compensation(2));

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

void Magnetometer::PrintStatus()
{
	if (external()) {
		PX4_INFO_RAW("%s %" PRIu32
			     " EN: %d, offset: [%05.3f %05.3f %05.3f], scale: [%05.3f %05.3f %05.3f], %.1f degC, Ext ROT: %d\n",
			     SensorString(), device_id(), enabled(),
			     (double)_offset(0), (double)_offset(1), (double)_offset(2),
			     (double)_scale(0, 0), (double)_scale(1, 1), (double)_scale(2, 2),
			     (double)_temperature,
			     rotation_enum());

	} else {
		PX4_INFO_RAW("%s %" PRIu32
			     " EN: %d, offset: [%05.3f %05.3f %05.3f], scale: [%05.3f %05.3f %05.3f], %.1f degC, Internal\n",
			     SensorString(), device_id(), enabled(),
			     (double)_offset(0), (double)_offset(1), (double)_offset(2),
			     (double)_scale(0, 0), (double)_scale(1, 1), (double)_scale(2, 2),
			     (double)_temperature);
	}

#if defined(DEBUG_BUILD)
	_scale.print();
#endif // DEBUG_BUILD
}

} // namespace calibration
