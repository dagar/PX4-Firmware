/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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
#include <px4_platform_common/log.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/sensor_correction.h>

namespace calibration
{
class Barometer
{
public:
	static constexpr int MAX_SENSOR_COUNT = 4;

	static constexpr uint8_t DEFAULT_PRIORITY = 50;
	static constexpr uint8_t DEFAULT_EXTERNAL_PRIORITY = 75;

	static constexpr const char *SensorString() { return "BARO"; }

	Barometer();
	explicit Barometer(uint32_t device_id);

	~Barometer() = default;

	void PrintStatus();

	bool set_calibration_index(int calibration_index);
	void set_device_id(uint32_t device_id);
	bool set_offset(const float &offset);

	bool calibrated() const { return (_device_id != 0) && (_calibration_index >= 0); }
	uint8_t calibration_count() const { return _calibration_count; }
	int8_t calibration_index() const { return _calibration_index; }
	uint32_t device_id() const { return _device_id; }
	bool enabled() const { return (_priority > 0); }
	bool external() const { return _external; }
	const float &offset() const { return _offset; }
	const int32_t &priority() const { return _priority; }
	const float &thermal_offset() const { return _thermal_offset; }

	// apply offsets
	inline float Correct(const float &data) const
	{
#if 0
		// Corrections for static pressure position error where Ps_error = Ps_meas - Ps_truth
		// Coef = Ps_error / Pdynamic, where Pdynamic = 1/2 * density * TAS**2

		// calculate static pressure error = Pmeas - Ptruth
		// model position error sensitivity as a body fixed ellipse with a different scale in the positive and
		// negative X and Y directions. Used to correct baro data for positional errors
		const matrix::Dcmf R_to_body{};//_output_new.quat_nominal.inversed());

		// Calculate airspeed in body frame
		const matrix::Vector3f velocity_earth{};// = _output_new.vel - _vel_imu_rel_body_ned;
		const matrix::Vector3f wind_velocity_earth{};//(_state.wind_vel(0), _state.wind_vel(1), 0.0f);
		const matrix::Vector3f airspeed_earth = velocity_earth - wind_velocity_earth;
		const matrix::Vector3f airspeed_body = R_to_body * airspeed_earth;

		const matrix::Vector3f K_pstatic_coef(airspeed_body(0) >= 0 ? _static_pressure_coef_xp : _static_pressure_coef_xn,
						      airspeed_body(1) >= 0 ? _static_pressure_coef_yp : _static_pressure_coef_yn,
						      _static_pressure_coef_z);

		const matrix::Vector3f airspeed_squared = airspeed_body.emult(airspeed_body); // TODO: constrain max_correction_airspeed

		const float pstatic_err = 0.5f * _air_density * (airspeed_squared.dot(K_pstatic_coef));

		// correct baro measurement using pressure error estimate and assuming sea level gravity
		return baro_alt_uncompensated + pstatic_err / (_air_density * CONSTANTS_ONE_G);
#endif
		return data - _thermal_offset - _offset;
	}

	inline float Uncorrect(const float &corrected_data) const
	{
		return corrected_data + _thermal_offset + _offset;
	}

	// Compute sensor offset from bias (board frame)
	float BiasCorrectedSensorOffset(const float &bias) const
	{
		return bias + _thermal_offset + _offset;
	}

	bool ParametersLoad();
	bool ParametersSave(int desired_calibration_index = -1, bool force = false);
	void ParametersUpdate();

	void Reset();

	void SensorCorrectionsUpdate(bool force = false);

private:
	uORB::Subscription _sensor_correction_sub{ORB_ID(sensor_correction)};

	float _offset{0};
	float _thermal_offset{0};

	float _static_pressure_coef_xp{0.0f};	// (-)
	float _static_pressure_coef_xn{0.0f};	// (-)
	float _static_pressure_coef_yp{0.0f};	// (-)
	float _static_pressure_coef_yn{0.0f};	// (-)
	float _static_pressure_coef_z{0.0f};	// (-)
	float _air_density{1.2f};

	int8_t _calibration_index{-1};
	uint32_t _device_id{0};
	int32_t _priority{-1};

	bool _external{false};

	uint8_t _calibration_count{0};
};
} // namespace calibration
