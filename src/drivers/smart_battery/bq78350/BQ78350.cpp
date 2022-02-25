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

#include "BQ78350.hpp"

BQ78350::BQ78350(const I2CSPIDriverConfig &config) :
	SBSBattery(config),
	I2CSPIDriver(config)
{
	_battery_status_pub.advertise(); // reserve instance number

	set_device_type(DRV_BAT_DEVTYPE_BQ78350);
}

BQ78350::~BQ78350()
{
	perf_free(_cycle_perf);
}

int BQ78350::init()
{
	int ret = SBSBattery::init();

	if (ret != PX4_OK) {
		DEVICE_DEBUG("I2C::init failed (%i)", ret);
		return ret;
	}

	ScheduleOnInterval(100_ms);

	return 0;
}

void BQ78350::print_status()
{
	I2CSPIDriverBase::print_status();

	perf_print_counter(_cycle_perf);

	PX4_INFO_RAW("cell count: %d\n", _cell_count);
	PX4_INFO_RAW("design voltage: %.3f\n", (double)_design_voltage);
	PX4_INFO_RAW("design capacity: %d\n", _design_capacity);
	PX4_INFO_RAW("actual capacity: %d\n", _actual_capacity);
	PX4_INFO_RAW("cycle count: %d\n", _cycle_count);
	PX4_INFO_RAW("serial number: %d\n", _serial_number);
	PX4_INFO_RAW("manufacturer name: %s\n", _manufacturer_name);
	PX4_INFO_RAW("manufacturer date: %d\n", _manufacture_date);
}

int BQ78350::probe()
{
	for (int i = 0; i < 5; i++) {
		if (populate_startup_data() == PX4_OK) {
			return PX4_OK;
		}

		px4_usleep(20000);
	}

	// always start
	return PX4_OK;
}

void BQ78350::RunImpl()
{
	//const hrt_abstime now = hrt_absolute_time();

	perf_begin(_cycle_perf);

	battery_status_s new_report{};

	new_report.device_id = get_device_id();
	new_report.id = _battery_status_pub.get_instance();
	new_report.connected = true;
	new_report.cell_count = _cell_count;
	new_report.warning = battery_status_s::BATTERY_WARNING_NONE;

	// Read data from sensor.
	int ret = populate_runtime_data(new_report);

	if (new_report.remaining > _param_bat_low_thr.get()) {
		// Leave as battery_status_s::BATTERY_WARNING_NONE

	} else if (new_report.remaining > _param_bat_crit_thr.get()) {
		new_report.warning = battery_status_s::BATTERY_WARNING_LOW;

	} else if (new_report.remaining > _param_bat_emergen_thr.get()) {
		new_report.warning = battery_status_s::BATTERY_WARNING_CRITICAL;

	} else {
		new_report.warning = battery_status_s::BATTERY_WARNING_EMERGENCY;
	}

	new_report.interface_error = perf_event_count(_interface_errors);

	new_report.timestamp = hrt_absolute_time();

	// Only publish if no errors.
	if (!ret) {
		_battery_status_pub.publish(new_report);
	}

	perf_end(_cycle_perf);
}

int BQ78350::populate_cell_voltages(battery_status_s &data)
{
	uint8_t DAstatus1[32] {}; // 32 bytes of data

	if (PX4_OK != manufacturer_read(BQ78350_MAN_DASTATUS1, DAstatus1, sizeof(DAstatus1))) {
		return PX4_ERROR;
	}

	// Cells 1-15
	int cells = 0;

	for (int i = 0; i < 16; i++) {
		// convert mV -> volts
		float voltage = ((float)((DAstatus1[2 * i + 1] << 8) | DAstatus1[2 * i]) / 1000.0f);

		if (voltage > 0.f) {
			data.voltage_cell_v[i] = voltage;
			cells++;
		}
	}

	_cell_count = cells;

	return PX4_OK;
}

int BQ78350::set_protection_cuv(bool enable)
{
	uint8_t protections_a_tmp = enable ?
				    BQ78350_ENABLED_PROTECTIONS_A_DEFAULT :
				    BQ78350_ENABLED_PROTECTIONS_A_CUV_DISABLED;

	return dataflash_write(BQ78350_MAN_ENABLED_PROTECTIONS_A_ADDRESS, &protections_a_tmp, 1);
}

int BQ78350::dataflash_read(const uint16_t address, void *data, const unsigned length)
{
	if (length > MAC_DATA_BUFFER_SIZE) {
		return -EINVAL;
	}

	uint8_t code = BQ78350_REG_MANUFACTURER_BLOCK_ACCESS;

	uint8_t tx_buf[2] = {};
	tx_buf[0] = ((uint8_t *)&address)[0];
	tx_buf[1] = ((uint8_t *)&address)[1];

	uint8_t rx_buf[MAC_DATA_BUFFER_SIZE + 2];

	int ret = block_write(code, tx_buf, 2, false);

	if (ret != PX4_OK) {
		return ret;
	}

	// Always returns 32 bytes of data
	ret = block_read(code, rx_buf, 32 + 2, true);
	memcpy(data, &rx_buf[2], length); // remove the address bytes

	return ret;
}

int BQ78350::dataflash_write(const uint16_t address, void *data, const unsigned length)
{
	return manufacturer_write(address, data, length);
}

int BQ78350::manufacturer_read(const uint16_t cmd_code, void *data, const unsigned length)
{
	if (length > MAC_DATA_BUFFER_SIZE) {
		return -EINVAL;
	}

	uint8_t code = BQ78350_REG_MANUFACTURER_BLOCK_ACCESS;

	uint8_t address[2] = {};
	address[0] = ((uint8_t *)&cmd_code)[0];
	address[1] = ((uint8_t *)&cmd_code)[1];

	uint8_t rx_buf[MAC_DATA_BUFFER_SIZE + 2];

	int ret = block_write(code, address, 2, false);

	if (ret != PX4_OK) {
		return ret;
	}

	ret = block_read(code, rx_buf, length + 2, true);
	memcpy(data, &rx_buf[2], length); // remove the address bytes

	return ret;
}

int BQ78350::manufacturer_write(const uint16_t cmd_code, void *data, const unsigned length)
{
	uint8_t code = BQ78350_REG_MANUFACTURER_BLOCK_ACCESS;

	uint8_t tx_buf[MAC_DATA_BUFFER_SIZE + 2] = {};
	tx_buf[0] = cmd_code & 0xff;
	tx_buf[1] = (cmd_code >> 8) & 0xff;

	if (data != nullptr && length <= MAC_DATA_BUFFER_SIZE) {
		memcpy(&tx_buf[2], data, length);
	}

	int ret = block_write(code, tx_buf, length + 2, false);

	return ret;
}

int BQ78350::lifetime_flush()
{
	return manufacturer_write(BQ78350_MAN_LIFETIME_FLUSH, nullptr, 0);
}

int BQ78350::lifetime_read()
{
	uint8_t lifetime_block_one[32] = {}; // 32 bytes of data

	if (PX4_OK != manufacturer_read(BQ78350_MAN_LIFETIME_BLOCK_ONE, lifetime_block_one, sizeof(lifetime_block_one))) {
		return PX4_ERROR;
	}

	return PX4_OK;
}

int BQ78350::populate_startup_data()
{
	int ret = PX4_OK;

	uint16_t device_type;
	ret |= manufacturer_read(BQ78350_MAN_DEVICE_TYPE, &device_type, sizeof(device_type));

	if (ret != PX4_OK) {
		return -EIO;
	}

	// Get generic SBS startup information
	ret |= SBSBattery::populate_startup_data();

	if (device_type == 0x1e9b) {
		PX4_INFO("serial: %X, manufacturer: %s, date: %X", _serial_number, _manufacturer_name, _manufacture_date);

	} else {
		PX4_WARN("failed probe (ret: %d, device_type: 0x%04x)", ret, device_type);
		return -EIO;
	}

	uint8_t cell_configuration;
	ret |= dataflash_read(BQ78350_FLASH_CELL_CONFIGURATION, &cell_configuration, sizeof(cell_configuration));
	PX4_INFO("BQ78350_FLASH_CELL_CONFIGURATION: 0x%X", cell_configuration);

	uint16_t state_of_health;
	ret |= read_word(BQ78350_REG_STATE_OF_HEALTH, state_of_health);
	PX4_INFO("BQ78350_REG_STATE_OF_HEALTH: 0x%X", state_of_health);

	if (ret) {
		PX4_WARN("Failed to read startup info: %d", ret);
		//return ret;
	}

	_state_of_health = state_of_health;
	_cell_count = cell_configuration & 0x07;

	return PX4_OK;
}

int BQ78350::populate_runtime_data(battery_status_s &data)
{
	int ret = SBSBattery::populate_runtime_data(data);

	ret |= populate_cell_voltages(data);

	data.state_of_health = _state_of_health;

	return ret;
}
