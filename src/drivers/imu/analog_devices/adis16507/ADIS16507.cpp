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

#include "ADIS16507.hpp"

using namespace time_literals;

static constexpr int16_t combine(uint8_t msb, uint8_t lsb)
{
	return (msb << 8u) | lsb;
}

ADIS16507::ADIS16507(const I2CSPIDriverConfig &config) :
	SPI(config),
	I2CSPIDriver(config),
	_drdy_gpio(config.drdy_gpio),
	_px4_accel(get_device_id(), config.rotation),
	_px4_gyro(get_device_id(), config.rotation)
{
	if (_drdy_gpio != 0) {
		_drdy_missed_perf = perf_alloc(PC_COUNT, MODULE_NAME": DRDY missed");
	}
}

ADIS16507::~ADIS16507()
{
	perf_free(_reset_perf);
	perf_free(_bad_register_perf);
	perf_free(_bad_transfer_perf);
	perf_free(_perf_crc_bad);
	perf_free(_drdy_missed_perf);
}

int ADIS16507::init()
{
	int ret = SPI::init();

	if (ret != PX4_OK) {
		DEVICE_DEBUG("SPI::init failed (%i)", ret);
		return ret;
	}

	return Reset() ? 0 : -1;
}

bool ADIS16507::Reset()
{
	_state = STATE::RESET;
	DataReadyInterruptDisable();
	ScheduleClear();
	ScheduleNow();
	return true;
}

void ADIS16507::exit_and_cleanup()
{
	DataReadyInterruptDisable();
	I2CSPIDriverBase::exit_and_cleanup();
}

void ADIS16507::print_status()
{
	I2CSPIDriverBase::print_status();

	perf_print_counter(_reset_perf);
	perf_print_counter(_bad_register_perf);
	perf_print_counter(_bad_transfer_perf);
	perf_print_counter(_perf_crc_bad);
	perf_print_counter(_drdy_missed_perf);

	perf_print_counter(_drdy_interval_perf);
	perf_print_counter(_run_interval_perf);
	perf_print_counter(_timestamp_sample_perf);
	perf_print_counter(_transfer_interval_perf);
	perf_print_counter(_tranfer_elapsed_perf);
}

int ADIS16507::probe()
{
	// Power-On Start-Up Time 310 ms
	if (hrt_absolute_time() < 310_ms) {
		PX4_WARN("required Power-On Start-Up Time 310 ms");
	}

	const uint16_t PROD_ID = RegisterRead(Register::PROD_ID);

	if (PROD_ID != Product_identification) {
		DEVICE_DEBUG("unexpected PROD_ID 0x%02x", PROD_ID);
		return PX4_ERROR;
	}

	const uint16_t SERIAL_NUM = RegisterRead(Register::SERIAL_NUM);
	const uint16_t FIRM_REV = RegisterRead(Register::FIRM_REV);
	const uint16_t FIRM_DM = RegisterRead(Register::FIRM_DM);
	const uint16_t FIRM_Y = RegisterRead(Register::FIRM_Y);

	PX4_INFO("Serial Number: 0x%X, Firmware revision: 0x%X Date: Y %X DM %X", SERIAL_NUM, FIRM_REV, FIRM_Y, FIRM_DM);

	return PX4_OK;
}

void ADIS16507::RunImpl()
{
	const hrt_abstime now = hrt_absolute_time();

	perf_count_interval(_run_interval_perf, now);

	switch (_state) {
	case STATE::RESET:
		perf_count(_reset_perf);
		// GLOB_CMD: software reset
		RegisterWrite(Register::GLOB_CMD, GLOB_CMD_BIT::Software_reset);
		_reset_timestamp = now;
		_failure_count = 0;
		_state = STATE::WAIT_FOR_RESET;
		ScheduleDelayed(255_ms); // 255 ms Reset Recovery Time
		break;

	case STATE::WAIT_FOR_RESET:

		if (_self_test_passed) {
			if ((RegisterRead(Register::PROD_ID) == Product_identification)) {
				// if reset succeeded then configure
				_state = STATE::CONFIGURE;
				ScheduleNow();

			} else {
				// RESET not complete
				if (hrt_elapsed_time(&_reset_timestamp) > 1000_ms) {
					PX4_DEBUG("Reset failed, retrying");
					_state = STATE::RESET;
					ScheduleDelayed(100_ms);

				} else {
					PX4_DEBUG("Reset not complete, check again in 100 ms");
					ScheduleDelayed(100_ms);
				}
			}

		} else {
			RegisterWrite(Register::GLOB_CMD, GLOB_CMD_BIT::Sensor_self_test);
			_state = STATE::SELF_TEST_CHECK;
			ScheduleDelayed(14_ms); // Self Test Time
		}

		break;

	case STATE::SELF_TEST_CHECK: {
			// read DIAG_STAT to check result
			const uint16_t DIAG_STAT = RegisterRead(Register::DIAG_STAT);

			if (DIAG_STAT != 0) {
				PX4_ERR("DIAG_STAT: %#X", DIAG_STAT);

			} else {
				PX4_DEBUG("self test passed");
				_self_test_passed = true;
				_state = STATE::RESET;
				ScheduleNow();
			}
		}
		break;

	case STATE::CONFIGURE:
		if (Configure()) {
			// if configure succeeded then start reading
			_state = STATE::READ;

			if (DataReadyInterruptConfigure()) {
				_data_ready_interrupt_enabled = true;

				// backup schedule as a watchdog timeout
				ScheduleDelayed(100_ms);

			} else {
				_data_ready_interrupt_enabled = false;
				ScheduleOnInterval(SAMPLE_INTERVAL_US, SAMPLE_INTERVAL_US);
			}

		} else {
			// CONFIGURE not complete
			if (hrt_elapsed_time(&_reset_timestamp) > 1000_ms) {
				PX4_DEBUG("Configure failed, resetting");
				_state = STATE::RESET;

			} else {
				PX4_DEBUG("Configure failed, retrying");
			}

			ScheduleDelayed(100_ms);
		}

		break;

	case STATE::READ: {
			hrt_abstime timestamp_sample = now;

			if (_data_ready_interrupt_enabled) {
				// scheduled from interrupt if _drdy_timestamp_sample was set as expected
				const hrt_abstime drdy_timestamp_sample = _drdy_timestamp_sample.fetch_and(0);

				if ((now - drdy_timestamp_sample) < SAMPLE_INTERVAL_US * 1.5) {
					timestamp_sample = drdy_timestamp_sample;

				} else {
					perf_count(_drdy_missed_perf);
				}

				// push backup schedule back
				ScheduleDelayed(SAMPLE_INTERVAL_US * 2);
			}

			bool success = false;

			// TODO: review and test BURST_SEL = 1
			//  16-Bit Burst Mode with BURST_SEL = 1
			//  In 16-bit burst mode with BURST_SEL = 1, a burst contains
			//  calibrated delta angle and delta velocity data in 16-bit format.

			struct BurstRead {
				uint16_t cmd;
				uint16_t DIAG_STAT;
				int16_t X_GYRO_OUT;
				int16_t Y_GYRO_OUT;
				int16_t Z_GYRO_OUT;
				int16_t X_ACCL_OUT;
				int16_t Y_ACCL_OUT;
				int16_t Z_ACCL_OUT;
				int16_t TEMP_OUT;
				uint16_t DATA_CNTR;
				uint16_t checksum;
			} buffer{};

			// ADIS16507 burst report should be 176 bits
			static_assert(sizeof(BurstRead) == (176 / 8), "ADIS16507 report not 176 bits");

			buffer.cmd = static_cast<uint16_t>(Register::GLOB_CMD) << 8;
			set_frequency(SPI_SPEED_BURST);

			perf_count(_transfer_interval_perf);
			perf_begin(_tranfer_elapsed_perf);

			if (transferhword((uint16_t *)&buffer, (uint16_t *)&buffer, sizeof(buffer) / sizeof(uint16_t)) == PX4_OK) {

				perf_end(_tranfer_elapsed_perf);

				// Calculate checksum and compare

				// Checksum = DIAG_STAT, Bits[15:8] + DIAG_STAT, Bits[7:0] +
				//  X_GYRO_OUT, Bits[15:8] + X_GYRO_OUT, Bits[7:0] +
				//  Y_GYRO_OUT, Bits[15:8] + Y_GYRO_OUT, Bits[7:0] +
				//  Z_GYRO_OUT, Bits[15:8] + Z_GYRO_OUT, Bits[7:0] +
				//  X_ACCL_OUT, Bits[15:8] + X_ACCL_OUT, Bits[7:0] +
				//  Y_ACCL_OUT, Bits[15:8] + Y_ACCL_OUT, Bits[7:0] +
				//  Z_ACCL_OUT, Bits[15:8] + Z_ACCL_OUT, Bits[7:0] +
				//  TEMP_OUT, Bits[15:8] + TEMP_OUT, Bits[7:0] +
				//  DATA_CNTR, Bits[15:8] + DATA_CNTR, Bits[7:0]
				uint8_t *checksum_helper = (uint8_t *)&buffer.DIAG_STAT;

				uint16_t checksum = 0;

				for (int i = 0; i < 18; i++) {
					checksum += checksum_helper[i];
				}

				if (buffer.checksum != checksum) {
					//PX4_DEBUG("adis_report.checksum: %X vs calculated: %X", buffer.checksum, checksum);
					perf_count(_bad_transfer_perf);
					perf_count(_perf_crc_bad);
				}

				if (buffer.DIAG_STAT != DIAG_STAT_BIT::Data_path_overrun) {
					// Data path overrun. A 1 indicates that one of the
					// data paths have experienced an overrun condition.
					// If this occurs, initiate a reset,

					//Reset();
					//return;
				}

				// Check all Status/Error Flag Indicators (DIAG_STAT)
				if (buffer.DIAG_STAT != 0) {
					perf_count(_bad_transfer_perf);
				}

				// temperature 1 LSB = 0.1°C
				const float temperature = buffer.TEMP_OUT * 0.1f;
				_px4_accel.set_temperature(temperature);
				_px4_gyro.set_temperature(temperature);


				int16_t accel_x = buffer.X_ACCL_OUT;
				int16_t accel_y = buffer.Y_ACCL_OUT;
				int16_t accel_z = buffer.Z_ACCL_OUT;

				// sensor's frame is +x forward, +y left, +z up
				//  flip y & z to publish right handed with z down (x forward, y right, z down)
				accel_y = (accel_y == INT16_MIN) ? INT16_MAX : -accel_y;
				accel_z = (accel_z == INT16_MIN) ? INT16_MAX : -accel_z;


				// TODO:
				// Group Delay with No Filtering: Accelerometer 1.57 ms
				const uint64_t accel_group_delay_us = 1'570;
				_px4_accel.update(timestamp_sample - accel_group_delay_us, accel_x, accel_y, accel_z);


				int16_t gyro_x = buffer.X_GYRO_OUT;
				int16_t gyro_y = buffer.Y_GYRO_OUT;
				int16_t gyro_z = buffer.Z_GYRO_OUT;
				// sensor's frame is +x forward, +y left, +z up
				//  flip y & z to publish right handed with z down (x forward, y right, z down)
				gyro_y = (gyro_y == INT16_MIN) ? INT16_MAX : -gyro_y;
				gyro_z = (gyro_z == INT16_MIN) ? INT16_MAX : -gyro_z;

				// TODO:
				// Group Delay with No Filtering:
				//  Gyroscope (X-Axis) 1.51 ms
				//  Gyroscope (Y-Axis) 1.51 ms
				//  Gyroscope (Z-Axis) 1.29 ms
				const uint64_t gyro_group_delay_us = (1'510 + 1'510 + 1'290) / 3;
				_px4_gyro.update(timestamp_sample - gyro_group_delay_us, gyro_x, gyro_y, gyro_z);



				perf_count_interval(_timestamp_sample_perf, timestamp_sample);

				success = true;

				if (_failure_count > 0) {
					_failure_count--;
				}

			} else {
				perf_count(_bad_transfer_perf);
			}

			if (!success) {
				_failure_count++;

				// full reset if things are failing consistently
				if (_failure_count > 10) {
					Reset();
					return;
				}
			}

			if (!success || hrt_elapsed_time(&_last_config_check_timestamp) > 100_ms) {
				// check configuration registers periodically or immediately following any failure
				if (RegisterCheck(_register_cfg[_checked_register])) {
					_last_config_check_timestamp = now;
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

bool ADIS16507::Configure()
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

	// accel: ±392 m/sec^2
	_px4_accel.set_range(392);
	_px4_accel.set_scale(392.f / 32'000.f); // 32,000 -> 392 m/sec^2

	// Gyroscope measurement range
	// Range Identifier (RANG_MDL)
	const uint16_t RANG_MDL = RegisterRead(Register::RANG_MDL);

	// sanity check RANG_MDL [1:0] Reserved, binary value = 11
	if (RANG_MDL & (Bit1 | Bit0)) {
		const uint16_t gyro_range = (RANG_MDL & (Bit3 | Bit2)) >> 2;

		if (gyro_range == 0b11) {
			// 11 = ±2000°/sec (ADIS16507-3BMLZ)
			_px4_gyro.set_range(math::radians(2000.f));
			_px4_gyro.set_scale(math::radians(1.f / 10.f)); // scaling 10 LSB/°/sec -> rad/s per LSB

		} else if (gyro_range == 0b01) {
			// 01 = ±500°/sec (ADIS16507-2BMLZ)
			_px4_gyro.set_range(math::radians(500.f));
			_px4_gyro.set_scale(math::radians(1.f / 40.f)); // scaling 40 LSB/°/sec -> rad/s per LSB

		} else if (gyro_range == 0b00) {
			// 00 = ±125°/sec (ADIS16507-1BMLZ)
			_px4_gyro.set_range(math::radians(500.f));
			_px4_gyro.set_scale(math::radians(1.f / 40.f)); // scaling 40 LSB/°/sec -> rad/s per LSB
		}
	}

	return success;
}

int ADIS16507::DataReadyInterruptCallback(int irq, void *context, void *arg)
{
	static_cast<ADIS16507 *>(arg)->DataReady();
	return 0;
}

void ADIS16507::DataReady()
{
	const hrt_abstime now = hrt_absolute_time();
	_drdy_timestamp_sample.store(now);
	perf_count_interval(_drdy_interval_perf, now);
	ScheduleNow();
}

bool ADIS16507::DataReadyInterruptConfigure()
{
	if (_drdy_gpio == 0) {
		return false;
	}

	// Setup data ready on falling edge
	return px4_arch_gpiosetevent(_drdy_gpio, false, true, false, &DataReadyInterruptCallback, this) == 0;
}

bool ADIS16507::DataReadyInterruptDisable()
{
	if (_drdy_gpio == 0) {
		return false;
	}

	return px4_arch_gpiosetevent(_drdy_gpio, false, false, false, nullptr, nullptr) == 0;
}

bool ADIS16507::RegisterCheck(const register_config_t &reg_cfg)
{
	bool success = true;

	const uint16_t reg_value = RegisterRead(reg_cfg.reg);

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

uint16_t ADIS16507::RegisterRead(Register reg)
{
	set_frequency(SPI_SPEED);

	uint16_t cmd[1];
	cmd[0] = (static_cast<uint16_t>(reg) << 8);

	transferhword(cmd, nullptr, 1);
	px4_udelay(SPI_STALL_PERIOD);
	transferhword(nullptr, cmd, 1);

	return cmd[0];
}

void ADIS16507::RegisterWrite(Register reg, uint16_t value)
{
	set_frequency(SPI_SPEED);

	uint16_t cmd[2];
	cmd[0] = (((static_cast<uint16_t>(reg))     | DIR_WRITE) << 8) | ((0x00FF & value));
	cmd[1] = (((static_cast<uint16_t>(reg) + 1) | DIR_WRITE) << 8) | ((0xFF00 & value) >> 8);

	transferhword(cmd, nullptr, 1);
	px4_udelay(SPI_STALL_PERIOD);
	transferhword(cmd + 1, nullptr, 1);
}

void ADIS16507::RegisterSetAndClearBits(Register reg, uint16_t setbits, uint16_t clearbits)
{
	const uint16_t orig_val = RegisterRead(reg);

	uint16_t val = (orig_val & ~clearbits) | setbits;

	if (orig_val != val) {
		RegisterWrite(reg, val);
	}
}
