/****************************************************************************
 *
 *   Copyright (c) 2012-2021 PX4 Development Team. All rights reserved.
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
 * @file batt_smbus.h
 *
 * This is a basic, SBS v1.1 compliant implementation of
 * an SMBUS Smart Battery. This driver is to be used as a default,
 * or as a base-class for more specific implementations such as
 * the Rotoye Batmon or TI BQ40z50/80
 *
 */

#include <lib/drivers/sbs_battery/SBSBattery.hpp>

#include <lib/geo/geo.h>

// Convert a uint16_t to int16_t without relying on type punning pointers
inline static int16_t u2int16(uint16_t x)
{
	return static_cast<int16_t>(x);
}

int SBSBattery::populate_startup_data()
{
	int ret = PX4_OK;

	// There's no convenient way to auto-detect the number of cells via the SMBus Battery specification, so we just read it from a parameter for now
	_cell_count = uint8_t(_param_sbs_bat_n_cells.get());

	ret |= block_read(SBS_REG_MANUFACTURER_NAME, _manufacturer_name, MANUFACTURER_NAME_SIZE, true);
	_manufacturer_name[sizeof(_manufacturer_name) - 1] = '\0';

	ret |= read_word(SBS_REG_MANUFACTURE_DATE, _manufacture_date);

	ret |= read_word(SBS_REG_SERIAL_NUMBER, _serial_number);

	ret |= read_word(SBS_REG_CYCLE_COUNT, _cycle_count);

	uint16_t design_voltage_mv;
	ret |= read_word(SBS_REG_DESIGN_VOLTAGE, design_voltage_mv);
	_design_voltage = float(design_voltage_mv) * 0.001f;

	ret |= read_word(SBS_REG_DESIGN_CAPACITY, _design_capacity);
	_design_capacity *= _param_sbs_bat_c_mult.get();

	ret |= read_word(SBS_REG_FULL_CHARGE_CAPACITY, _actual_capacity);
	_actual_capacity *= _param_sbs_bat_c_mult.get();

	return ret;
}

int SBSBattery::populate_runtime_data(battery_status_s &data)
{
	// Temporary variable for storing SMBUS reads.
	uint16_t result;

	// Voltage (mV -> V)
	int ret = read_word(SBS_REG_VOLTAGE, result);
	data.voltage_v = float(result) * 0.001f;
	data.voltage_filtered_v = data.voltage_v;

	// Current (mA -> A, scaling)
	ret |= read_word(SBS_REG_CURRENT, result);
	data.current_a = float(u2int16(result)) * -0.001f * _param_sbs_bat_c_mult.get();
	data.current_filtered_a = data.current_a;

	// Average current (mA -> A, scaling)
	ret |= read_word(SBS_REG_AVERAGE_CURRENT, result);
	data.current_average_a = float(u2int16(result)) * -0.001f * _param_sbs_bat_c_mult.get();

	// Time to empty (seconds).
	ret |= read_word(SBS_REG_RUN_TIME_TO_EMPTY, result);
	data.time_remaining_s = result * 60;

	// Average time to empty (minutes).
	ret |= read_word(SBS_REG_AVERAGE_TIME_TO_EMPTY, result);
	data.average_time_to_empty = result;

	// Remaining amount
	ret |= read_word(SBS_REG_RELATIVE_SOC, result);
	data.remaining = float(result) / 100.0f;

	// Max Error
	ret |= read_word(SBS_REG_MAX_ERROR, result);
	data.max_error = result;

	// Battery temperature
	ret |= read_word(SBS_REG_TEMP, result);
	data.temperature = (float(result) / 10.0f) + CONSTANTS_ABSOLUTE_NULL_CELSIUS;

	// Static data
	// TODO: move to a new battery_info message?
	data.capacity = _actual_capacity;
	data.cycle_count = _cycle_count;
	data.serial_number = _serial_number;
	data.manufacture_date = _manufacture_date;

	return ret;
}

int SBSBattery::read_word(const uint8_t cmd_code, uint16_t &data)
{
	uint8_t buf[6];
	// 2 data bytes + pec byte
	int result = transfer(&cmd_code, 1, buf + 3, 3);

	if (result == PX4_OK) {
		data = buf[3] | ((uint16_t)buf[4] << 8);
		// Check PEC.
		//uint8_t addr = (get_device_address()) << 1;
		uint8_t addr = (0x0B) << 1;
		buf[0] = addr | 0x00;
		buf[1] = cmd_code;
		buf[2] = addr | 0x01;

		uint8_t pec = get_pec(buf, sizeof(buf) - 1);

		if (pec != buf[sizeof(buf) - 1]) {
			//result = -EINVAL;
			perf_count(_interface_errors);
		}

	} else {
		perf_count(_interface_errors);
	}

	return result;
}

int SBSBattery::write_word(const uint8_t cmd_code, uint16_t data)
{
	// 2 data bytes + pec byte
	uint8_t buf[5];
	buf[0] = (get_device_address() << 1) | 0x10;
	buf[1] = cmd_code;
	buf[2] = data & 0xff;
	buf[3] = (data >> 8) & 0xff;
	buf[4] = get_pec(buf, 4);

	{
		uint8_t pec_buf[5];
		pec_buf[0] = (0x0B << 1) | 0x10;
		pec_buf[1] = cmd_code;
		pec_buf[2] = data & 0xff;
		pec_buf[3] = (data >> 8) & 0xff;
		pec_buf[4] = get_pec(pec_buf, 4);

		buf[4] = pec_buf[4];
	}

	int result = transfer(&buf[1], 4, nullptr, 0);

	if (result != PX4_OK) {
		perf_count(_interface_errors);
	}

	return result;
}

int SBSBattery::block_read(const uint8_t cmd_code, void *data, const uint8_t length, const bool use_pec)
{
	uint8_t byte_count = 0;
	// addr(wr), cmd_code, addr(r), byte_count, data (MAX_BLOCK_LEN bytes max), pec
	uint8_t rx_data[MAX_BLOCK_LEN + 5];

	if (length > MAX_BLOCK_LEN) {
		return -EINVAL;
	}

	int result = transfer(&cmd_code, 1, (uint8_t *)&rx_data[3], length + 2);

	if (result != PX4_OK) {
		perf_count(_interface_errors);
		return result;
	}

	uint8_t device_address = get_device_address();
	rx_data[0] = (device_address << 1) | 0x00;
	rx_data[1] = cmd_code;
	rx_data[2] = (device_address << 1) | 0x01;
	byte_count = math::min(rx_data[3], MAX_BLOCK_LEN);

	// ensure data is not longer than given buffer
	memcpy(data, &rx_data[4], math::min(byte_count, length));

	if (use_pec) {
		rx_data[0] = (0x0B << 1) | 0x00;
		rx_data[2] = (0x0B << 1) | 0x01;

		uint8_t pec = get_pec(rx_data, byte_count + 4);

		if (pec != rx_data[byte_count + 4]) {
			//result = -EIO;
			perf_count(_interface_errors);
		}
	}

	return result;
}

int SBSBattery::block_write(const uint8_t cmd_code, const void *data, uint8_t byte_count, const bool use_pec)
{
	// cmd code[1], byte count[1], data[byte_count] (MAX_BLOCK_LEN max), pec[1] (optional)
	uint8_t buf[MAX_BLOCK_LEN + 2];

	if (byte_count > MAX_BLOCK_LEN) {
		return -EINVAL;
	}

	buf[0] = cmd_code;
	buf[1] = (uint8_t)byte_count;
	memcpy(&buf[2], data, byte_count);

	if (use_pec) {
		uint8_t pec = get_pec(buf, byte_count + 2);
		buf[byte_count + 2] = pec;
		byte_count++;
	}

	unsigned i = 0;
	int result = 0;

	// If block_write fails, try up to 10 times.
	while (i < 10 && ((result = transfer((uint8_t *)buf, byte_count + 2, nullptr, 0)) != PX4_OK)) {
		perf_count(_interface_errors);
		i++;
	}

	if (i == 10 || result) {
		result = -EINVAL;
	}

	return result;
}

uint8_t SBSBattery::get_pec(uint8_t *buff, const uint8_t len)
{
	// TODO: use "return crc8ccitt(buff, len);"

	// Initialise CRC to zero.
	uint8_t crc = 0;
	uint8_t shift_register = 0;
	bool invert_crc;

	// Calculate crc for each byte in the stream.
	for (uint8_t i = 0; i < len; i++) {
		// Load next data byte into the shift register
		shift_register = buff[i];

		// Calculate crc for each bit in the current byte.
		for (uint8_t j = 0; j < 8; j++) {
			invert_crc = (crc ^ shift_register) & 0x80;
			crc <<= 1;
			shift_register <<= 1;

			if (invert_crc) {
				crc ^= SMBUS_PEC_POLYNOMIAL;
			}
		}
	}

	return crc;
}
