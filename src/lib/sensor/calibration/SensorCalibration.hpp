/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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

#pragma once

#include <px4_platform_common/px4_config.h>

#include <lib/matrix/matrix/math.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/sensor_correction.h>

#include <lib/sensor/configuration/Accelerometer.hpp>

namespace sensor
{
namespace calibration
{



template <typename BASE, typename T = typename BASE::value_type>
struct CalibrationOffset : public BASE {

public:
	Vector3f offset{};

	bool set_offset(const matrix::Vector3f &offset) { _offset = offset; }
	const Vector3f &offset() const { return offset; }


	bool ParametersLoad() override
	{
		bool success = false;

		if (BASE::ParametersLoad()) {
			// CAL_{}n_{X,Y,Z}OFF
			Vector3f offset{
				GetCalibrationParamFloat(SensorString(), "XOFF", calibration_index()),
				GetCalibrationParamFloat(SensorString(), "YOFF", calibration_index()),
				GetCalibrationParamFloat(SensorString(), "ZOFF", calibration_index())
			};

			success = set_offset(offset);
		}

		return success;
	}

	bool ParametersSave(int desired_calibration_index, bool force) override
	{
		bool success = false;

		if (Base::ParametersSave(desired_calibration_index, force)) {
			// CAL_{}n_{X,Y,Z}OFF
			success &= SetCalibrationParam(SensorString(), "XOFF", calibration_index(), _offset(0));
			success &= SetCalibrationParam(SensorString(), "YOFF", calibration_index(), _offset(1));
			success &= SetCalibrationParam(SensorString(), "ZOFF", calibration_index(), _offset(2));
		}

		return success;
	}

	void Reset() override
	{
		_offset.zero();
		Base::Reset();
	}
};


//typedef CalibrationOffset< CalibrationBase > gyroCalibration;




// TODO: pass Configuration to Calibration
// TODO: internal/external sensor

// TODO: max instances
// TODO: axis

class Configuration
{
public:
	static constexpr int MAX_SENSOR_COUNT = 4;

	static constexpr uint8_t DEFAULT_PRIORITY = 50;
	static constexpr uint8_t DEFAULT_EXTERNAL_PRIORITY = 75;

	static constexpr const char *SensorString() { return "GYRO"; }

	Configuration();
	explicit Configuration(uint32_t device_id);

	~Configuration() = default;

	void PrintStatus();

	bool set_configuration_index(int configuration_index);
	void set_device_id(uint32_t device_id);

	void set_rotation(Rotation rotation);

	bool configured() const { return (_device_id != 0) && (_configuration_index >= 0); }
	uint8_t configuration_count() const { return _configuration_count; }
	int8_t configuration_index() const { return _configuration_index; }
	uint32_t device_id() const { return _device_id; }
	bool enabled() const { return (_priority > 0); }
	bool external() const { return _external; }

	const int32_t &priority() const { return _priority; }

	bool ParametersLoad();
	bool ParametersSave(int desired_configuration_index = -1, bool force = false);
	void ParametersUpdate();

	void Reset();

private:
	uint32_t _device_id{0}; // TODO Device.hpp? pretty print
	int32_t _priority{-1};

	bool _external{false};

	int8_t _configuration_index{-1};
	uint8_t _configuration_count{0};
};



class CalibrationBase
{
public:
	static constexpr int MAX_SENSOR_COUNT = 4;

	//static constexpr const char *SensorString() { return "ACC"; }

	CalibrationBase();
	explicit CalibrationBase(uint32_t device_id);

	~CalibrationBase() = default;

	void PrintStatus() {}

	bool set_calibration_index(int calibration_index) { return true;}

	bool calibrated() const { return (_calibration_index >= 0); }
	uint8_t calibration_count() const { return _calibration_count; }
	int8_t calibration_index() const { return _calibration_index; }

	// apply offsets and scale
	// rotate corrected measurements from sensor to body frame
	// inline matrix::Vector3f Correct(const matrix::Vector3f &data) const
	// {
	// 	return _configuration.rotation() * matrix::Vector3f{(data - _thermal_offset - _offset).emult(_scale)};
	// }

	// Compute sensor offset from bias (board frame)
	// matrix::Vector3f BiasCorrectedSensorOffset(const matrix::Vector3f &bias) const
	// {
	// 	// updated calibration offset = existing offset + bias rotated to sensor frame and unscaled
	// 	return _offset + (_configuration.rotation().I() * bias).edivide(_scale);
	// }

	bool ParametersLoad() { return true; }
	bool ParametersSave(int desired_calibration_index = -1, bool force = false) { return true; }
	void ParametersUpdate() {}

	void Reset() {}

	//void SensorCorrectionsUpdate(bool force = false);

protected:
	//sensor::configuration::Accelerometer _configuration{};

	//uORB::Subscription _sensor_correction_sub{ORB_ID(sensor_correction)};

	//matrix::Vector3f _offset;
	//matrix::Vector3f _scale;
	//matrix::Vector3f _thermal_offset;

	int8_t _calibration_index{-1};
	uint8_t _calibration_count{0};
};

} // namespace calibration
} // namespace sensor
