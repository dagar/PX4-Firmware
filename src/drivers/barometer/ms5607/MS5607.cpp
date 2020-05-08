/****************************************************************************
 *
 *   Copyright (c) 2012-2020 PX4 Development Team. All rights reserved.
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

#include "MS5607.hpp"

using namespace time_literals;

template<typename T>
static constexpr T sq(T x) { return x * x; }

static constexpr int16_t combine(uint8_t msb, uint8_t lsb) { return (msb << 8u) | lsb; }

MS5607::MS5607(device::Device *interface, I2CSPIBusOption bus_option, int bus) :
	I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(interface->get_device_id()), bus_option, bus),
	_interface(interface),
	_px4_barometer(interface->get_device_id())
{
}

MS5607::~MS5607()
{
	perf_free(_bad_transfer_perf);
	perf_free(_read_pressure_interval_perf);
	perf_free(_read_temperature_interval_perf);
	perf_free(_bad_pressure_perf);
	perf_free(_bad_temperature_perf);
}

int MS5607::init()
{
	if (!ReadProm()) {
		PX4_DEBUG("prom readout failed");
		return PX4_ERROR;
	}

	_state = STATE::RESET;
	ScheduleNow();

	return PX4_OK;
}

uint32_t MS5607::ReadADC()
{
	// 24 bit raw D1/B2
	uint8_t raw[3] {};

	if (_interface->read(Register::ADC_Read, raw, 3) != PX4_OK) {
		return 0;
	}

	return (raw[0] << 16) + (raw[1] << 8) + raw[2];
}

bool MS5607::ReadProm()
{
	uint8_t buffer[16] {};

	for (int i = 0; i < 16; i = i + 2) {
		_interface->read(Register::PROM_Read + i, &buffer[i], 2);
	}

	_prom.factory_setup = combine(buffer[0], buffer[1]);
	_prom.C1 = combine(buffer[2], buffer[3]);
	_prom.C2 = combine(buffer[4], buffer[5]);
	_prom.C3 = combine(buffer[6], buffer[7]);
	_prom.C4 = combine(buffer[8], buffer[9]);
	_prom.C5 = combine(buffer[10], buffer[11]);
	_prom.C6 = combine(buffer[12], buffer[13]);
	_prom.serial_and_crc = combine(buffer[14], buffer[15]);

	return crc4((uint16_t *)&_prom);
}

void MS5607::RunImpl()
{
	switch (_state) {
	case STATE::RESET:
		_interface->write(Register::Reset);
		_state = STATE::READ_PROM;
		ScheduleDelayed(RESET_TIME_US);
		break;

	case STATE::READ_PROM:
		if (ReadProm()) {
			_state = STATE::CONVERT_TEMPERATURE;
			ScheduleNow();

		} else {
			// ERROR
			perf_count(_bad_transfer_perf);
			_state = STATE::RESET;
			ScheduleDelayed(100_ms);
		}

		break;

	case STATE::CONVERT_TEMPERATURE:
		if (_interface->write(Register::Convert_D2_OSR4096) == PX4_OK) {
			_state = STATE::READ_TEMPERATURE;
			ScheduleDelayed(OSR4096_MAX_CONVERSION_TIME_US);

		} else {
			perf_count(_bad_transfer_perf);
			_state = STATE::RESET;
			ScheduleDelayed(100_ms);
		}

		break;

	case STATE::READ_TEMPERATURE: {
			perf_count(_read_temperature_interval_perf);
			const uint32_t D2 = ReadADC();

			if (D2 == 0 || D2 == 0xFFFFFF) {
				// error, reset and try again
				perf_count(_bad_transfer_perf);
				_state = STATE::RESET;
				ScheduleDelayed(100_ms);
				return;
			}

			// temperature offset (in ADC units)
			int32_t dT = (int32_t)D2 - ((int32_t)_prom.C5 << 8);

			// absolute temperature in centidegrees - note intermediate value is outside 32-bit range
			int32_t TEMP = 2000 + (int32_t)(((int64_t)dT * _prom.C6) >> 23);

			// base sensor scale/offset values
			// Perform MS5607 Caculation
			_OFF  = ((int64_t)_prom.C2 << 17) + (((int64_t)_prom.C4 * dT) >> 6);
			_SENS = ((int64_t)_prom.C1 << 16) + (((int64_t)_prom.C3 * dT) >> 7);

			/* MS5607 temperature compensation */
			if (TEMP < 2000) {
				int32_t T2 = sq(dT) >> 31;

				int64_t f = sq((int64_t)TEMP - 2000);
				int64_t OFF2 = 61 * f >> 4;
				int64_t SENS2 = 2 * f;

				if (TEMP < -1500) {
					int64_t f2 = sq(TEMP + 1500);
					OFF2 += 15 * f2;
					SENS2 += 8 * f2;
				}

				TEMP -= T2;
				_OFF  -= OFF2;
				_SENS -= SENS2;
			}

			const float temperature = TEMP / 100.0f;

			if (PX4_ISFINITE(temperature) && (temperature >= TEMPERATURE_MIN) && (temperature <= TEMPERATURE_MAX)) {
				_last_temperature_update = hrt_absolute_time();
				_px4_barometer.set_temperature(temperature);

				_state = STATE::CONVERT_PRESSURE;
				ScheduleNow();

			} else {
				// invalid data, reset and try again
				perf_count(_bad_temperature_perf);
				_state = STATE::RESET;
				ScheduleDelayed(100_ms);
			}
		}

		break;

	case STATE::CONVERT_PRESSURE:
		if (_interface->write(Register::Convert_D1_OSR4096) == PX4_OK) {
			// save timestamp_sample, approximately the middle of the averaged data
			_timestamp_sample = hrt_absolute_time() + (OSR4096_MAX_CONVERSION_TIME_US / 2);
			_state = STATE::READ_PRESSURE;
			ScheduleDelayed(OSR4096_MAX_CONVERSION_TIME_US);

		} else {
			// transfer error, reset and try again
			perf_count(_bad_transfer_perf);
			_state = STATE::RESET;
			ScheduleDelayed(100_ms);
		}

		break;

	case STATE::READ_PRESSURE: {
			perf_count(_read_pressure_interval_perf);
			const uint32_t D1 = ReadADC();

			if (D1 == 0 || D1 == 0xFFFFFF) {
				// error, reset and try again
				perf_count(_bad_transfer_perf);
				_state = STATE::RESET;
				ScheduleDelayed(100_ms);
				return;
			}

			const int64_t P = (((D1 * _SENS) >> 21) - _OFF) >> 15;

			const float pressure = P / 100.0f; // convert to millibar

			if (PX4_ISFINITE(pressure) && (pressure >= OPERATING_PRESSURE_MIN) && (pressure <= OPERATING_PRESSURE_MAX)) {

				_px4_barometer.update(_timestamp_sample, pressure);

				// update temperature ~= 5 Hz, otherwise schedule another pressure conversion
				if (hrt_elapsed_time(&_last_temperature_update) >= 200_ms) {
					_state = STATE::CONVERT_TEMPERATURE;

				} else {
					_state = STATE::CONVERT_PRESSURE;
				}

				ScheduleNow();

			} else {
				// invalid data, reset and try again
				perf_count(_bad_pressure_perf);
				_state = STATE::RESET;
				ScheduleDelayed(100_ms);
			}
		}
		break;
	}
}

void MS5607::print_status()
{
	I2CSPIDriverBase::print_status();

	perf_print_counter(_bad_transfer_perf);
	perf_print_counter(_read_pressure_interval_perf);
	perf_print_counter(_read_temperature_interval_perf);
	perf_print_counter(_bad_pressure_perf);
	perf_print_counter(_bad_temperature_perf);

	_px4_barometer.print_status();
}

bool MS5607::crc4(uint16_t *n_prom)
{
	int16_t cnt;
	uint16_t n_rem;
	uint16_t crc_read;
	uint8_t n_bit;

	n_rem = 0x00;

	/* save the read crc */
	crc_read = n_prom[7];

	/* remove CRC byte */
	n_prom[7] = (0xFF00 & (n_prom[7]));

	for (cnt = 0; cnt < 16; cnt++) {
		/* uneven bytes */
		if (cnt & 1) {
			n_rem ^= (uint8_t)((n_prom[cnt >> 1]) & 0x00FF);

		} else {
			n_rem ^= (uint8_t)(n_prom[cnt >> 1] >> 8);
		}

		for (n_bit = 8; n_bit > 0; n_bit--) {
			if (n_rem & 0x8000) {
				n_rem = (n_rem << 1) ^ 0x3000;

			} else {
				n_rem = (n_rem << 1);
			}
		}
	}

	/* final 4 bit remainder is CRC value */
	n_rem = (0x000F & (n_rem >> 12));
	n_prom[7] = crc_read;

	/* return true if CRCs match */
	return (0x000F & crc_read) == (n_rem ^ 0x00);
}
