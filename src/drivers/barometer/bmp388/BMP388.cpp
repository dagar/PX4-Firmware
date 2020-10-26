/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#include "BMP388.hpp"

using namespace time_literals;

static constexpr int16_t combine(uint8_t msb, uint8_t lsb)
{
	return (msb << 8u) | lsb;
}

BMP388::BMP388(I2CSPIBusOption bus_option, int bus, int bus_frequency, enum Rotation rotation) :
	I2C(DRV_MAG_DEVTYPE_BMP388, MODULE_NAME, bus, I2C_ADDRESS_DEFAULT, bus_frequency),
	I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(get_device_id()), bus_option, bus),
	_px4_baro(get_device_id(), rotation)
{
}

BMP388::~BMP388()
{
	perf_free(_reset_perf);
	perf_free(_bad_register_perf);
	perf_free(_bad_transfer_perf);
	perf_free(_overflow_perf);
}

int BMP388::init()
{
	int ret = I2C::init();

	if (ret != PX4_OK) {
		DEVICE_DEBUG("I2C::init failed (%i)", ret);
		return ret;
	}

	return Reset() ? 0 : -1;
}

bool BMP388::Reset()
{
	RegisterWrite(Register::POWER_CONTROL, 0);
	_state = STATE::RESET;
	ScheduleClear();
	ScheduleDelayed(2_ms);
	return true;
}

void BMP388::print_status()
{
	I2CSPIDriverBase::print_status();

	perf_print_counter(_reset_perf);
	perf_print_counter(_bad_register_perf);
	perf_print_counter(_bad_transfer_perf);
}

int BMP388::probe()
{
	const uint8_t POWER_CONTROL = RegisterRead(Register::POWER_CONTROL);
	const uint8_t CHIP_ID = RegisterRead(Register::CHIP_ID);

	PX4_DEBUG("POWER_CONTROL: 0x%02hhX, CHIP_ID: 0x%02hhX", POWER_CONTROL, CHIP_ID);

	// either power control bit is set and chip ID can be read, or both registers are 0x00
	if ((POWER_CONTROL & POWER_CONTROL_BIT::PowerControl) && (CHIP_ID == chip_identification_number)) {
		return PX4_OK;

	} else if ((POWER_CONTROL == 0) && (CHIP_ID == 0)) {
		return PX4_OK;
	}

	return PX4_ERROR;
}

/*!
 * @brief This internal API is used to compensate the raw pressure data and
 * return the compensated pressure data in double data type.
 * For e.g. returns pressure in Pascal p = 95305.295 which is 953.05295 hecto pascal
 */
static double compensate_pressure(const struct bmp3_uncomp_data *uncomp_data, const struct bmp3_calib_data *calib_data)
{
	const struct bmp3_quantized_calib_data *quantized_calib_data = &calib_data->quantized_calib_data;

	double partial_data1 = quantized_calib_data->par_p6 * quantized_calib_data->t_lin;
	double partial_data2 = quantized_calib_data->par_p7 * pow_bmp3(quantized_calib_data->t_lin, 2);
	double partial_data3 = quantized_calib_data->par_p8 * pow_bmp3(quantized_calib_data->t_lin, 3);
	double partial_out1 = quantized_calib_data->par_p5 + partial_data1 + partial_data2 + partial_data3;
	partial_data1 = quantized_calib_data->par_p2 * quantized_calib_data->t_lin;
	partial_data2 = quantized_calib_data->par_p3 * pow_bmp3(quantized_calib_data->t_lin, 2);
	partial_data3 = quantized_calib_data->par_p4 * pow_bmp3(quantized_calib_data->t_lin, 3);
	double partial_out2 = uncomp_data->pressure * (quantized_calib_data->par_p1 + partial_data1 + partial_data2 +
			      partial_data3);
	partial_data1 = pow_bmp3((double)uncomp_data->pressure, 2);
	partial_data2 = quantized_calib_data->par_p9 + quantized_calib_data->par_p10 * quantized_calib_data->t_lin;
	partial_data3 = partial_data1 * partial_data2;
	double partial_data4 = partial_data3 + pow_bmp3((double)uncomp_data->pressure, 3) * quantized_calib_data->par_p11;
	double comp_press = partial_out1 + partial_out2 + partial_data4;

	return comp_press;
}

/*!
 * @brief This internal API is used to compensate the raw temperature data and
 * return the compensated temperature data in double data type.
 * for e.g. returns temperature 24.26 deg Celsius
 */
static double compensate_temperature(const struct bmp3_uncomp_data *uncomp_data, struct bmp3_calib_data *calib_data)
{
	uint32_t uncomp_temp = uncomp_data->temperature;

	double partial_data1 = (double)(uncomp_temp - calib_data->quantized_calib_data.par_t1);
	double partial_data2 = (double)(partial_data1 * calib_data->quantized_calib_data.par_t2);

	/* Update the compensated temperature in calib structure since this is
	 * needed for pressure calculation */
	calib_data->quantized_calib_data.t_lin = partial_data2 + (partial_data1 * partial_data1) *
			calib_data->quantized_calib_data.par_t3;

	/* Returns compensated temperature */
	return calib_data->quantized_calib_data.t_lin;
}

void BMP388::RunImpl()
{
	const hrt_abstime now = hrt_absolute_time();

	switch (_state) {
	case STATE::RESET:
		// POWER_CONTROL: soft reset
		RegisterWrite(Register::PWR_CTRL, PWR_CTRL_BIT::normal_mode);
		_reset_timestamp = now;
		_failure_count = 0;
		_state = STATE::WAIT_FOR_RESET;
		perf_count(_reset_perf);
		ScheduleDelayed(3_ms); // 3.0 ms start-up time from suspend to sleep
		break;

	case STATE::WAIT_FOR_RESET:

		// Soft reset always brings the device into sleep mode (power off -> suspend -> sleep)
		if ((RegisterRead(Register::CHIP_ID) == chip_identification_number)
		    && (RegisterRead(Register::POWER_CONTROL) == POWER_CONTROL_BIT::PowerControl)
		    && (RegisterRead(Register::OP_MODE) == OP_MODE_BIT::Opmode_Sleep)) {

			_state = STATE::READ_TRIM;
			ScheduleDelayed(10_ms);

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

	case STATE::READ_CALIBRATION_DATA: {
			uint8_t BMP3_LEN_CALIB_DATA = 21;
			uint8_t reg_addr = BMP3_REG_CALIB_DATA;

			/* Array to store calibration data */
			uint8_t calib_data[BMP3_LEN_CALIB_DATA] {};

			/* Read the calibration data from the sensor */
			//rslt = bmp3_get_regs(reg_addr, calib_data, BMP3_LEN_CALIB_DATA);

			/* Parse calibration data and store it in device structure */
			_calib_data.par_t1 = BMP3_CONCAT_BYTES(reg_data[1], reg_data[0]);
			_calib_data.par_t2 = BMP3_CONCAT_BYTES(reg_data[3], reg_data[2]);
			_calib_data.par_t3 = (int8_t)reg_data[4];
			_calib_data.par_p1 = (int16_t)BMP3_CONCAT_BYTES(reg_data[6], reg_data[5]);
			_calib_data.par_p2 = (int16_t)BMP3_CONCAT_BYTES(reg_data[8], reg_data[7]);
			_calib_data.par_p3 = (int8_t)reg_data[9];
			_calib_data.par_p4 = (int8_t)reg_data[10];
			_calib_data.par_p5 = BMP3_CONCAT_BYTES(reg_data[12], reg_data[11]);
			_calib_data.par_p6 = BMP3_CONCAT_BYTES(reg_data[14], reg_data[13]);
			_calib_data.par_p7 = (int8_t)reg_data[15];
			_calib_data.par_p8 = (int8_t)reg_data[16];
			_calib_data.par_p9 = (int16_t)BMP3_CONCAT_BYTES(reg_data[18], reg_data[17]);
			_calib_data.par_p10 = (int8_t)reg_data[19];
			_calib_data.par_p11 = (int8_t)reg_data[20];
		}
		break;

	case STATE::CONFIGURE:
		if (Configure()) {
			// if configure succeeded then start reading every 50 ms (20 Hz)
			_state = STATE::READ;
			ScheduleOnInterval(50_ms, 50_ms);

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
			struct TransferBuffer {
				STATUS;
				PRESS_XLSB_7_0;
				PRESS_LSB_15_8;
				PRESS_MSB_23_16;
				TEMP_XLSB_7_0;
				TEMP_LSB_15_8;
				TEMP_MSB_23_16;
			} buffer{};

			bool success = false;
			// 0x42 to 0x4A with a burst read.
			uint8_t cmd = static_cast<uint8_t>(Register::STATUS);

			if (transfer(&cmd, 1, (uint8_t *)&buffer, sizeof(buffer)) == PX4_OK) {

				uint32_t press_data_xlsb = buffer.PRESS_XLSB_7_0;
				uint32_t press_data_lsb = buffer.PRESS_LSB_15_8 << 8;
				uint32_t press_data_msb = buffer.PRESS_MSB_23_16 << 16;
				uint64_t pressure_raw = press_data_msb | press_data_lsb | press_data_xlsb;

				uint32_t temp_data_xlsb = buffer.PRESS_XLSB_7_0;
				uint32_t temp_data_lsb = buffer.PRESS_LSB_15_8 << 8;
				uint32_t temp_data_msb = buffer.PRESS_MSB_23_16 << 16;
				uint64_t temperature_raw = temp_data_msb | temp_data_lsb | temp_data_xlsb;

				if (data_ready) {
					_px4_baro.set_error_count(perf_event_count(_bad_register_perf) + perf_event_count(_bad_transfer_perf));
					_px4_baro.temperature(temperature);
					_px4_baro.update(now, pressure);

					success = true;

					if (_failure_count > 0) {
						_failure_count--;
					}
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

bool BMP388::Configure()
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

	// TODO: check ERR_REG
	const uint8_t ERR_REG = RegisterRead(Register::ERR_REG);

	if (ERR_REG & Bit0) {
		// Fatal error
		PX4_ERR("fatal error");
		return false;
	}

	if (ERR_REG & Bit2) {
		// sensor configuration error detected
		PX4_ERR("sensor configuration error detected");
		return false;
	}

	return success;
}

bool BMP388::RegisterCheck(const register_config_t &reg_cfg)
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

uint8_t BMP388::RegisterRead(Register reg)
{
	const uint8_t cmd = static_cast<uint8_t>(reg);
	uint8_t buffer{};
	int ret = transfer(&cmd, 1, &buffer, 1);

	if (ret != PX4_OK) {
		PX4_DEBUG("register read 0x%02hhX failed, ret = %d", cmd, ret);
		return -1;
	}

	return buffer;
}

void BMP388::RegisterWrite(Register reg, uint8_t value)
{
	uint8_t buffer[2] { (uint8_t)reg, value };
	int ret = transfer(buffer, sizeof(buffer), nullptr, 0);

	if (ret != PX4_OK) {
		PX4_DEBUG("register write 0x%02hhX failed, ret = %d", (uint8_t)reg, ret);
	}
}

void BMP388::RegisterSetAndClearBits(Register reg, uint8_t setbits, uint8_t clearbits)
{
	const uint8_t orig_val = RegisterRead(reg);
	uint8_t val = (orig_val & ~clearbits) | setbits;

	if (orig_val != val) {
		RegisterWrite(reg, val);
	}
}
