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

#include "GNSS.hpp"

#include "Utilities.hpp"

#include <lib/parameters/param.h>
#include <lib/sensor/Utilities.hpp>

using namespace matrix;
using namespace time_literals;
using namespace sensor::utilities;

namespace sensor
{
namespace configuration
{

GNSS::GNSS()
{
	Reset();
}

GNSS::GNSS(uint32_t device_id)
{
	set_device_id(device_id);
}

void GNSS::set_device_id(uint32_t device_id)
{
	if (_device_id != device_id) {

		_device_id = device_id;

		Reset();

		ParametersUpdate();
	}
}

bool GNSS::set_configuration_index(int configuration_index)
{
	if ((configuration_index >= 0) && (configuration_index < MAX_SENSOR_COUNT)) {
		_configuration_index = configuration_index;
		return true;
	}

	return false;
}

bool GNSS::set_position(const matrix::Vector3f &position)
{
	static constexpr float MIN_POSITION_CHANGE = 1e-6;

	if (position.isAllFinite()
	    && (Vector3f(_position - position).longerThan(MIN_POSITION_CHANGE) || (_configuration_count == 0))
	   ) {
		_position = position;

		_configuration_count++;
		return true;
	}

	return false;
}

void GNSS::ParametersUpdate()
{
	if (_device_id == 0) {
		return;
	}

	_configuration_index = FindCurrentConfigurationIndex(SensorString(), _device_id);

	if (_configuration_index == -1) {
		// no saved configuration available
		Reset();

	} else {
		ParametersLoad();
	}
}

bool GNSS::ParametersLoad()
{
	if (_configuration_index >= 0 && _configuration_index < MAX_SENSOR_COUNT) {
		// SENS_{}n_PRIO
		_priority = GetConfigurationParamInt32(SensorString(), "PRIO", _configuration_index);

		if ((_priority < 0) || (_priority > 100)) {
			// reset to default, -1 is the uninitialized parameter value
			static constexpr int32_t SENS_PRIO_UNINITIALIZED = -1;

			if (_priority != SENS_PRIO_UNINITIALIZED) {
				PX4_ERR("%s %" PRIu32 " (%" PRId8 ") invalid priority %" PRId32 ", resetting", SensorString(), _device_id,
					_configuration_index, _priority);

				SetConfigurationParam(SensorString(), "PRIO", _configuration_index, SENS_PRIO_UNINITIALIZED);
			}

			_priority = DEFAULT_PRIORITY;
		}

		// SENS_{}n_{X,Y,Z}POS:
		Vector3f position{0.f, 0.f, 0.f};
		position(0) = GetConfigurationParamFloat(SensorString(), "XPOS", _configuration_index);
		position(1) = GetConfigurationParamFloat(SensorString(), "YPOS", _configuration_index);
		position(2) = GetConfigurationParamFloat(SensorString(), "ZPOS", _configuration_index);
		set_position(position);

		return true;
	}

	return false;
}

void GNSS::Reset()
{
	_position.zero();

	_priority = DEFAULT_PRIORITY;

	_configuration_index = -1;

	_configuration_count = 0;
}

bool GNSS::ParametersSave(int desired_configuration_index, bool force)
{
	if (force && desired_configuration_index >= 0 && desired_configuration_index < MAX_SENSOR_COUNT) {
		_configuration_index = desired_configuration_index;

	} else if (!force || (_configuration_index < 0)
		   || (desired_configuration_index != -1 && desired_configuration_index != _configuration_index)) {

		// ensure we have a valid configuration slot (matching existing or first available slot)
		int8_t configuration_index_prev = _configuration_index;
		_configuration_index = FindAvailableConfigurationIndex(SensorString(), _device_id, desired_configuration_index);

		if (configuration_index_prev >= 0 && (configuration_index_prev != _configuration_index)) {
			PX4_WARN("%s %" PRIu32 " configuration index changed %" PRIi8 " -> %" PRIi8, SensorString(), _device_id,
				 configuration_index_prev, _configuration_index);
		}
	}

	if (_configuration_index >= 0 && _configuration_index < MAX_SENSOR_COUNT) {
		// save configuration
		bool success = true;

		success &= SetConfigurationParam(SensorString(), "ID", _configuration_index, _device_id);
		success &= SetConfigurationParam(SensorString(), "PRIO", _configuration_index, _priority);

		// SENS_{}n_{X,Y,Z}POS
		success &= SetConfigurationParam(SensorString(), "XPOS", _configuration_index, _position(0));
		success &= SetConfigurationParam(SensorString(), "YPOS", _configuration_index, _position(0));
		success &= SetConfigurationParam(SensorString(), "ZPOS", _configuration_index, _position(0));

		return success;
	}

	return false;
}

void GNSS::PrintStatus()
{
	PX4_INFO_RAW("%s %" PRIu32
		     " EN: %d, position: [%05.3f %05.3f %05.3f]\n",
		     SensorString(), device_id(), enabled(),
		     (double)_position(0), (double)_position(1), (double)_position(2));
}

} // namespace configuration
} // namespace sensor
