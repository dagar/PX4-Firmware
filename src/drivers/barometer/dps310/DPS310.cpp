/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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

#include "DPS310.hpp"

static constexpr uint32_t DPS310_CONVERSION_INTERVAL{27600};	// microseconds

DPS310::DPS310(device::Device *interface) :
	ScheduledWorkItem(px4::device_bus_to_wq(interface->get_device_id())),
	_px4_barometer(interface->get_device_id()),
	_interface(interface),
	_sample_perf(perf_alloc(PC_ELAPSED, "dps310: read")),
	_sample_interval_perf(perf_alloc(PC_INTERVAL, "dps310: read interval")),
	_comms_errors(perf_alloc(PC_COUNT, "dps310: comm errors"))
{
	_px4_barometer.set_device_type(DRV_BARO_DEVTYPE_DPS310);
}

DPS310::~DPS310()
{
	// make sure we are truly inactive
	stop();

	// free perf counters
	perf_free(_sample_perf);
	perf_free(_sample_interval_perf);
	perf_free(_comms_errors);

	delete _interface;
}

int
DPS310::init()
{
	uint8_t id = RegisterRead(Register::Product_ID);

	if (Infineon_DPS310::ID != id) {
		return PX4_ERROR;
	}

	if (reset() != OK) {
		return PX4_ERROR;
	}

	start();

	return PX4_OK;
}

/*
  handle bit width for 16 bit config registers
 */
static void fix_config_bits16(int16_t &v, uint8_t bits)
{
	if (v > int16_t((1U << (bits - 1)) - 1)) {
		v = v - (1U << bits);
	}
}

/*
  handle bit width for 32 bit config registers
 */
static void fix_config_bits32(int32_t &v, uint8_t bits)
{
	if (v > int32_t((1U << (bits - 1)) - 1)) {
		v = v - (1U << bits);
	}
}

int
DPS310::reset()
{
	// read calibration data
	uint8_t buf[18] {};

	if (_interface->read((uint8_t)Register::COEF, &buf, 18)) {
		return false;
	}

	_calibration.C0  = (buf[0] << 4) + ((buf[1] >> 4) & 0x0F);
	_calibration.C1  = (buf[2] + ((buf[1] & 0x0F) << 8));
	_calibration.C00 = ((buf[4] << 4) + (buf[3] << 12)) + ((buf[5] >> 4) & 0x0F);
	_calibration.C10 = ((buf[5] & 0x0F) << 16) + buf[7] + (buf[6] << 8);
	_calibration.C01 = (buf[9] + (buf[8] << 8));
	_calibration.C11 = (buf[11] + (buf[10] << 8));
	_calibration.C20 = (buf[13] + (buf[12] << 8));
	_calibration.C21 = (buf[15] + (buf[14] << 8));
	_calibration.C30 = (buf[17] + (buf[16] << 8));

	fix_config_bits16(_calibration.C0, 12);
	fix_config_bits16(_calibration.C1, 12);
	fix_config_bits32(_calibration.C00, 20);
	fix_config_bits32(_calibration.C10, 20);
	fix_config_bits16(_calibration.C01, 16);
	fix_config_bits16(_calibration.C11, 16);
	fix_config_bits16(_calibration.C20, 16);
	fix_config_bits16(_calibration.C21, 16);
	fix_config_bits16(_calibration.C30, 16);

	// get calibration source
	if (_interface->read((uint8_t)Register::COEF_SRCE, &_calibration.temp_source, 1)) {
		return PX4_ERROR;
	}

	_calibration.temp_source &= 0x80;



	RegisterWrite(Register::CFG_REG, 0x0C);		// shift for 16x oversampling
	RegisterWrite(Register::PRS_CFG, 0x54);		// 32 Hz, 16x oversample
	RegisterWrite(Register::TMP_CFG, 0x54);		// 32 Hz, 16x oversample
	RegisterWrite(Register::MEAS_CFG, 0x07);	// continuous temp and pressure

	return PX4_OK;
}

void
DPS310::start()
{
	// reset the report ring and state machine
	_collect_phase = false;

	ScheduleOnInterval(DPS310_CONVERSION_INTERVAL);
}

void
DPS310::stop()
{
	ScheduleClear();
}

void
DPS310::calculate_PT(int32_t UT, int32_t UP, float &pressure, float &temperature)
{
	// scaling for 16x oversampling
	const float scaling_16 = 1.0f / 253952;

	float temp_scaled = float(UT) * scaling_16;

	temperature = _calibration.C0 * 0.5f + _calibration.C1 * temp_scaled;

	float press_scaled = float(UP) * scaling_16;

	pressure = _calibration.C00;
	pressure += press_scaled * (_calibration.C10 + press_scaled * (_calibration.C20 + press_scaled * _calibration.C30));
	pressure += temp_scaled * _calibration.C01;
	pressure += temp_scaled * press_scaled * (_calibration.C11 + press_scaled * _calibration.C21);
}

void
DPS310::Run()
{
	perf_begin(_sample_perf);
	perf_count(_sample_interval_perf);

	uint8_t ready = RegisterRead(Register::MEAS_CFG);

	if (!(ready & (1U << 4))) {
		// pressure not ready
		return;
	}

	uint8_t buf[6];
	const hrt_abstime timestamp_sample = hrt_absolute_time();

	if (_interface->read((uint8_t)Register::PSR_B0, buf, 3) != PX4_OK) {
		perf_count(_comms_errors);
		return;
	}

	if (_interface->read((uint8_t)Register::TMP_B0, &buf[3], 3) != PX4_OK) {
		perf_count(_comms_errors);
		return;
	}

	int32_t press = (buf[2]) + (buf[1] << 8) + (buf[0] << 16);
	int32_t temp  = (buf[5]) + (buf[4] << 8) + (buf[3] << 16);
	fix_config_bits32(press, 24);
	fix_config_bits32(temp, 24);

	float pressure = 0.0f;
	float temperature = 0.0f;

	calculate_PT(temp, press, pressure, temperature);

	//float P_raw_sc = 0.0f;
	//float T_raw_sc = 0.0f;

	//float c00, c10, c20, c30, c01, c11, c21 = 0.0f;

	//float P_comp_Pa = c00 + P_raw_sc * (c10 + P_raw_sc * (c20 + P_raw_sc * c30)) + T_raw_sc * c01 + T_raw_sc * P_raw_sc * (c11 + P_raw_sc * c21);

	_px4_barometer.set_error_count(perf_event_count(_comms_errors));
	_px4_barometer.set_temperature(temperature);
	_px4_barometer.update(timestamp_sample, pressure);

	perf_end(_sample_perf);
}

uint8_t
DPS310::RegisterRead(Register reg)
{
	uint8_t buf{};
	_interface->read((uint8_t)reg, &buf, 1);

	return buf;
}

void
DPS310::RegisterWrite(Register reg, uint8_t value)
{
	_interface->write((uint8_t)reg, &value, 1);
}

void
DPS310::RegisterClearBits(Register reg, uint8_t clearbits)
{
	uint8_t val = RegisterRead(reg);

	// only write if necessary
	if (val & clearbits) {
		val &= !clearbits;
		RegisterWrite(reg, val);
	}
}

void
DPS310::RegisterSetBits(Register reg, uint8_t setbits)
{
	uint8_t val = RegisterRead(reg);

	// only write if necessary
	if (!(val & setbits)) {
		val |= setbits;
		RegisterWrite(reg, val);
	}
}

void
DPS310::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	printf("poll interval: %u \n", _measure_interval);

	_px4_barometer.print_status();
}
