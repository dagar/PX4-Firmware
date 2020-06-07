/****************************************************************************
 *
 *   Copyright (c) 2019-2020 PX4 Development Team. All rights reserved.
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

#include "AK09916.hpp"

using namespace time_literals;

static constexpr int16_t combine(uint8_t msb, uint8_t lsb)
{
	return (msb << 8u) | lsb;
}

AK09916::AK09916(I2CSPIBusOption bus_option, int bus, int bus_frequency, enum Rotation rotation) :
	I2C(DRV_MAG_DEVTYPE_AK09916, MODULE_NAME, bus, I2C_ADDRESS_DEFAULT, bus_frequency),
	I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(get_device_id()), bus_option, bus),
	_px4_mag(get_device_id(), external() ? ORB_PRIO_VERY_HIGH : ORB_PRIO_DEFAULT, rotation)
{
	_px4_mag.set_external(external());
}

AK09916::~AK09916()
{
	perf_free(_transfer_perf);
	perf_free(_bad_register_perf);
	perf_free(_bad_transfer_perf);
}

int AK09916::init()
{
	int ret = I2C::init();

	if (ret != PX4_OK) {
		DEVICE_DEBUG("I2C::init failed (%i)", ret);
		return ret;
	}

	return Reset() ? 0 : -1;
}

bool AK09916::Reset()
{
	_state = STATE::RESET;
	ScheduleClear();
	ScheduleNow();
	return true;
}

void AK09916::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_transfer_perf);
	perf_print_counter(_bad_register_perf);
	perf_print_counter(_bad_transfer_perf);
	_px4_mag.print_status();
}

int AK09916::probe()
{
	const uint8_t WAI = RegisterRead(Register::WAI);

	if (WAI != WHOAMI) {
		DEVICE_DEBUG("unexpected WAI 0x%02x", whoami);
		return PX4_ERROR;
	}

	return PX4_OK;
}

void AK09916::RunImpl()
{
	switch (_state) {
	case STATE::RESET:
		// CNTL3 SRST: Soft reset
		RegisterWrite(Register::CNTL3, CNTL3_BIT::SRST);
		_reset_timestamp = hrt_absolute_time();
		_state = STATE::WAIT_FOR_RESET;
		ScheduleDelayed(100_ms);
		break;

	case STATE::WAIT_FOR_RESET:
		if (RegisterRead(Register::WAI) == WHOAMI) {
			// if reset succeeded then configure
			if (!_sensitivity_adjustments_loaded) {
				// Set Fuse ROM Access mode (MODE[3:0]=“1111”) before reading Fuse ROM data.
				RegisterWrite(Register::CNTL1, CNTL1_BIT::BIT_16 | CNTL1_BIT::FUSE_ROM_ACCESS_MODE);
				_state = STATE::READ_SENSITIVITY_ADJUSTMENTS;
				ScheduleDelayed(100_ms);

			} else {
				RegisterWrite(Register::CNTL1, CNTL1_BIT::BIT_16 | CNTL1_BIT::CONTINUOUS_MODE_2);
				_state = STATE::CONFIGURE;
				ScheduleDelayed(10_ms);
			}

		} else {
			// RESET not complete
			if (hrt_elapsed_time(&_reset_timestamp) > 1000_ms) {
				PX4_DEBUG("Reset failed, retrying");
				_state = STATE::RESET;
				ScheduleDelayed(100_ms);

			} else {
				PX4_DEBUG("Reset not complete, check again in 10 ms");
				ScheduleDelayed(10_ms);
			}
		}

		break;

	case STATE::READ_SENSITIVITY_ADJUSTMENTS: {
			// read FUSE ROM (to get ASA corrections)
			uint8_t cmd = static_cast<uint8_t>(Register::ASAX);
			uint8_t response[3] {};
			transfer(&cmd, 1, buffer, 3);

			bool valid = true;

			for (int i = 0; i < 3; i++) {
				if (response[i] != 0 && response[i] != 0xFF) {
					_sensitivity[i] = ((float)(response[i] - 128) / 256.f) + 1.f;

				} else {
					valid = false;
				}
			}

			_sensitivity_adjustments_loaded = valid;

			// After reading fuse ROM data, set power-down mode (MODE[3:0]=“0000”) before the transition to another mode.
			RegisterWrite(Register::CNTL1, 0);
			_state = STATE::RESET;
			ScheduleDelayed(10_ms);
		}

		break;

	case STATE::CONFIGURE:
		if (Configure()) {
			// if configure succeeded then start reading
			_state = STATE::READ;
			ScheduleOnInterval(20_ms, 20_ms); // 50 Hz

		} else {
			// CONFIGURE not complete
			if (hrt_elapsed_time(&_reset_timestamp) > 1000_ms) {
				PX4_DEBUG("Configure failed, resetting");
				_state = STATE::RESET;

			} else {
				PX4_DEBUG("Configure failed, retrying");
			}

			ScheduleDelayed(10_ms);
		}

		break;

	case STATE::READ: {
			perf_begin(_transfer_perf);
			struct TransferBuffer {
				uint8_t ST1;
				uint8_t HXL;
				uint8_t HXH;
				uint8_t HYL;
				uint8_t HYH;
				uint8_t HZL;
				uint8_t HZH;
				uint8_t TMPS;
				uint8_t ST2;
			} buffer{};
			const hrt_abstime timestamp_sample = hrt_absolute_time();
			uint8_t cmd = static_cast<uint8_t>(Register::ST1);
			int ret = transfer(&cmd, 1, (uint8_t *)&buffer, sizeof(buffer));
			perf_end(_transfer_perf);

			bool success = false;

			if ((ret == PX4_OK) && (buffer.ST1 & ST1_BIT::DRDY) && !(buffer.ST2 & ST2_BIT::HOFL)) {
				// sensor's frame is +y forward (x), -x right, +z down
				int16_t x = combine(buffer.HYH, buffer.HYL); // +Y
				int16_t y = combine(buffer.HXH, buffer.HXL); // +X
				y = (y == INT16_MIN) ? INT16_MAX : -y; // flip y
				int16_t z = combine(buffer.HZH, buffer.HZL);

				const bool all_zero = (x == 0 && y == 0 && z == 0);
				const bool new_data = (_last_measurement[0] != x || _last_measurement[1] != y || _last_measurement[2] != z);

				if (!all_zero && new_data) {
					_px4_mag.update(timestamp_sample, x * _sensitivity[0], y * _sensitivity[1], z * _sensitivity[2]);

					_last_measurement[0] = x;
					_last_measurement[1] = y;
					_last_measurement[2] = z;

					success = true;
				}
			}

			if (!success || hrt_elapsed_time(&_last_config_check_timestamp) > 10_ms) {
				// check configuration registers periodically or immediately following any failure
				if (RegisterCheck(_register_cfg[_checked_register])) {
					_last_config_check_timestamp = timestamp_sample;
					_checked_register = (_checked_register + 1) % size_register_cfg;

				} else {
					// register check failed, force reset
					perf_count(_bad_register_perf);
					Reset();
				}
			}
		}

		break;
	}
}

bool AK09916::Configure()
{
	// first set and clear all configured register bits
	for (const auto &reg_cfg : _register_cfg) {
		RegisterSetAndClearBits(reg_cfg.reg, reg_cfg.set_bits, reg_cfg.clear_bits);
	}

	// now check that all are configured
	bool success = true;

	for (const auto &reg_cfg : _register_cfg) {
		if (!RegisterCheck(reg_cfg)) {
			success = false;
		}
	}

	// in 16-bit sampling mode the mag resolution is 1.5 milli Gauss per bit */
	_px4_mag.set_scale(1.5e-3f);

	return success;
}

bool AK09916::RegisterCheck(const register_config_t &reg_cfg)
{
	bool success = true;

	const uint8_t reg_value = RegisterRead(reg_cfg.reg);

	if (reg_cfg.set_bits && ((reg_value & reg_cfg.set_bits) != reg_cfg.set_bits)) {
		PX4_DEBUG("0x%02hhX: 0x%02hhX (0x%02hhX not set)", (uint8_t)reg_cfg.reg, reg_value, reg_cfg.set_bits);
		success = false;
	}

	if (reg_cfg.clear_bits && ((reg_value & reg_cfg.clear_bits) != 0)) {
		PX4_DEBUG("0x%02hhX: 0x%02hhX (0x%02hhX not cleared)", (uint8_t)reg_cfg.reg, reg_value, reg_cfg.clear_bits);
		success = false;
	}

	return success;
}

uint8_t AK09916::RegisterRead(Register reg)
{
	const uint8_t cmd = static_cast<uint8_t>(reg);
	uint8_t buffer{};
	transfer(&cmd, 1, &buffer, 1);
	return buffer;
}

void AK09916::RegisterWrite(Register reg, uint8_t value)
{
	uint8_t buffer[2] { (uint8_t)reg, value };
	transfer(buffer, sizeof(buffer), nullptr, 0);
}

void AK09916::RegisterSetAndClearBits(Register reg, uint8_t setbits, uint8_t clearbits)
{
	const uint8_t orig_val = RegisterRead(reg);
	uint8_t val = (orig_val & ~clearbits) | setbits;

	if (orig_val != val) {
		RegisterWrite(reg, val);
	}
}
