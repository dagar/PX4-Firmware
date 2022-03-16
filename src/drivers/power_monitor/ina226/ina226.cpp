/****************************************************************************
 *
 *   Copyright (c) 2019-2022 PX4 Development Team. All rights reserved.
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

/**
 * @file ina226.cpp
 * @author David Sidrane <david_s5@usa.net>
 *
 * Driver for the I2C attached INA226
 */

#include "ina226.h"

INA226::INA226(const I2CSPIDriverConfig &config, int battery_index) :
	I2C(config),
	ModuleParams(nullptr),
	I2CSPIDriver(config),
	_battery(battery_index, this, INA226_SAMPLE_INTERVAL_US, battery_status_s::BATTERY_SOURCE_POWER_MODULE)
{
	float fvalue = MAX_CURRENT;
	_max_current = fvalue;
	param_t ph = param_find("INA226_CURRENT");

	if (ph != PARAM_INVALID && param_get(ph, &fvalue) == PX4_OK) {
		_max_current = fvalue;
	}

	fvalue = INA226_SHUNT;
	_rshunt = fvalue;
	ph = param_find("INA226_SHUNT");

	if (ph != PARAM_INVALID && param_get(ph, &fvalue) == PX4_OK) {
		_rshunt = fvalue;
	}

	_current_lsb = _max_current / DN_MAX;
	_power_lsb = 25 * _current_lsb;

	// We need to publish immediately, to guarantee that the first instance of the driver publishes to uORB instance 0
	_battery.setConnected(false);
	_battery.updateVoltage(0.f);
	_battery.updateCurrent(0.f);
	_battery.updateAndPublishBatteryStatus(hrt_absolute_time());
}

INA226::~INA226()
{
	perf_free(_comms_errors);
	perf_free(_measure_errors);
}

int INA226::read(uint8_t address, int16_t &data)
{
	// read desired little-endian value via I2C
	uint16_t received_bytes;
	const int ret = transfer(&address, 1, (uint8_t *)&received_bytes, sizeof(received_bytes));

	if (ret == PX4_OK) {
		data = swap16(received_bytes);

	} else {
		perf_count(_comms_errors);
	}

	return ret;
}

int INA226::write(uint8_t address, uint16_t value)
{
	uint8_t data[3] = {address, ((uint8_t)((value & 0xFF00) >> 8)), (uint8_t)(value & 0xFF)};
	int ret = transfer(data, sizeof(data), nullptr, 0);

	if (ret != 0) {
		perf_count(_comms_errors);
	}

	return ret;
}

int INA226::init()
{
	// do I2C init (and probe) first
	if (I2C::init() != PX4_OK) {
		return PX4_ERROR;
	}

	write(INA226_REG_CONFIGURATION, INA226_RST);

	int16_t cal = INA226_CONST / (_current_lsb * _rshunt);

	if (write(INA226_REG_CALIBRATION, cal) < 0) {
		return PX4_ERROR;
	}

	// If we run in continuous mode then start it here
	if (write(INA226_REG_CONFIGURATION, INA226_CONFIG) == 0) {
		_initialized = true;
	}

	ScheduleOnInterval(INA226_SAMPLE_INTERVAL_US, INA226_SAMPLE_INTERVAL_US);

	return PX4_OK;
}

int INA226::force_init()
{
	int ret = init();
	ScheduleOnInterval(INA226_SAMPLE_INTERVAL_US, INA226_SAMPLE_INTERVAL_US);
	return ret;
}

int INA226::probe()
{
	int16_t value = 0;

	if (read(INA226_MFG_ID, value) != PX4_OK || value != INA226_MFG_ID_TI) {
		PX4_DEBUG("probe mfg ID %d", value);
		return -1;
	}

	if (read(INA226_MFG_DIEID, value) != PX4_OK || value != INA226_MFG_DIE) {
		PX4_DEBUG("probe die id %d", value);
		return -1;
	}

	return PX4_OK;
}

void INA226::RunImpl()
{
	if (_parameter_update_sub.updated()) {
		// Read from topic to clear updated flag
		parameter_update_s parameter_update;
		_parameter_update_sub.copy(&parameter_update);

		updateParams();
	}

	// read from the sensor
	// Note: If the power module is connected backwards, then the values of power, current, and shunt will be negative but otherwise valid.
	int16_t bus_voltage = 0;
	int16_t current = 0;

	if ((read(INA226_REG_BUSVOLTAGE, bus_voltage) == PX4_OK) && (read(INA226_REG_CURRENT, current) == PX4_OK)) {
		_battery.setConnected(true);
		_battery.updateVoltage(static_cast<float>(bus_voltage * INA226_VSCALE));
		_battery.updateCurrent(static_cast<float>(current * _current_lsb));
		_battery.updateAndPublishBatteryStatus(hrt_absolute_time());

	} else {
		_battery.setConnected(false);
		perf_count(_measure_errors);
	}

	// TODO: force reconfigure if there hasn't been a successful read
}

void INA226::print_status()
{
	I2CSPIDriverBase::print_status();

	if (_initialized) {
		perf_print_counter(_comms_errors);
		perf_print_counter(_measure_errors);

	} else {
		PX4_INFO("Device not initialized. Retrying every %d ms until battery is plugged in.",
			 INA226_INIT_RETRY_INTERVAL_US / 1000);
	}
}
