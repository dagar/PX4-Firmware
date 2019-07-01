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

#include "MAX7315.hpp"

MAX7315::MAX7315(int bus, int address) :
	I2C("MAX7315", nullptr, bus, address, 400000),
	ScheduledWorkItem(px4::device_bus_to_wq(get_device_id())),
	_sample_perf(perf_alloc(PC_ELAPSED, "max7315: read")),
	_comms_errors(perf_alloc(PC_COUNT, "max7315: communication error"))
{
	//_debug_enabled = true;

	_retries = 10;
}

MAX7315::~MAX7315()
{
	// make sure we are truly inactive
	stop();

	// free perf counters
	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int
MAX7315::init()
{
	PX4_INFO("init");

	// do I2C init (and probe) first
	if (I2C::init() != OK) {
		return PX4_ERROR;
	}

	uint8_t data{};



	// Master
	// Set the master, O8 intensity register 0x0E to any value from 0x10 to 0xFF
	if (writeRegister(Register::MASTER, 0xFF) != PX4_OK) {
		PX4_ERR("MASTER failed");
		return PX4_ERROR;
	}

	readRegister(Register::MASTER, &data);

	if (data != 0xFF) {
		PX4_ERR("MASTER setup failed: %X", data);
		return PX4_ERROR;
	}



	// PORT_CONFIG
	// configure all as outputs
	if (writeRegister(Register::PORT_CONFIG, 0x00) != PX4_OK) {
		PX4_ERR("PORT_CONFIG failed");
		return PX4_ERROR;
	}

	readRegister(Register::PORT_CONFIG, &data);

	if (data != 0x00) {
		PX4_ERR("PORT_CONFIG setup failed: %X", data);
		return PX4_ERROR;
	}



	// Clear global intensity G bit to 0 in the configuration register to disable global intensity control.
	if (writeRegister(Register::CONFIG, 0x00) != PX4_OK) {
		PX4_ERR("CONFIG failed");
		return PX4_ERROR;
	}

	readRegister(Register::CONFIG, &data);

	if (data != 0x00) {
		PX4_ERR("CONFIG setup failed: %X", data);
		return PX4_ERROR;
	}



	// disable BLINK
	writeRegister(Register::BLINK_PHASE_0, 0xFF);
	writeRegister(Register::BLINK_PHASE_1, 0x00);


	// For the static outputs, set the output intensity value to 0xF.
	// For the PWM outputs, set the output intensity value in the range 0x0 to 0xE.
	writeRegister(Register::INT_76, 0x0);
	writeRegister(Register::INT_54, 0x0);
	writeRegister(Register::INT_32, 0x0);
	writeRegister(Register::INT_10, 0x0);

	start();

	return PX4_OK;
}

int
MAX7315::probe()
{
	uint8_t data{};
	readRegister(Register::CONFIG, &data);

	if (data != 0xC) {
		PX4_ERR("probe CONFIG: %X", data);
		return PX4_ERROR;
	}

	return OK;
}

void
MAX7315::test(int p)
{
	// toggle port P
	if (p >= 0 && p <= 7) {

		if (p >= 0 && p <= 1) {
			uint8_t data{};
			readRegister(Register::INT_10, &data);

			uint8_t msb = ((data & 0xF0) >> 4);
			uint8_t lsb = (data & 0x0F);

			if (p == 0) {
				PX4_INFO("toggling p0");

				uint8_t new_data = (msb << 4) | ~lsb;

				PX4_INFO("writing register INT_10 %X", new_data);
				writeRegister(Register::INT_10, new_data);

			} else if (p == 1) {
				PX4_INFO("toggling p1");
				// toggle MSB (P1)
				uint8_t new_data = (~msb << 4) | lsb;
				PX4_INFO("writing register INT_10 %X", new_data);
				writeRegister(Register::INT_10, new_data);
			}
		}

		if (p >= 2 && p <= 3) {
			uint8_t data{};
			readRegister(Register::INT_32, &data);

			uint8_t msb = ((data & 0xF0) >> 4);
			uint8_t lsb = (data & 0x0F);

			if (p == 2) {
				PX4_INFO("toggling p2");
				// toggle LSB (P0)
				uint8_t new_data = (msb << 4) | ~lsb;
				PX4_INFO("writing register INT_32 %X", new_data);
				writeRegister(Register::INT_32, new_data);

			} else if (p == 3) {
				PX4_INFO("toggling p3");
				// toggle MSB (P1)
				uint8_t new_data = (~msb << 4) | lsb;
				PX4_INFO("writing register INT_32 %X", new_data);
				writeRegister(Register::INT_32, new_data);
			}
		}

		if (p >= 4 && p <= 5) {
			uint8_t data{};
			readRegister(Register::INT_54, &data);

			uint8_t msb = ((data & 0xF0) >> 4);
			uint8_t lsb = (data & 0x0F);

			if (p == 4) {
				PX4_INFO("toggling p4");
				// toggle LSB (P0)
				uint8_t new_data = (msb << 4) | ~lsb;
				PX4_INFO("writing register INT_54 %X", new_data);
				writeRegister(Register::INT_54, new_data);

			} else if (p == 5) {
				PX4_INFO("toggling p5");
				// toggle MSB (P1)
				uint8_t new_data = (~msb << 4) | lsb;
				PX4_INFO("writing register INT_54 %X", new_data);
				writeRegister(Register::INT_54, new_data);
			}
		}

	}

	uint8_t data{};

	readRegister(Register::INPUTS, &data);
	PX4_INFO("INPUTS: %X", data);

	readRegister(Register::BLINK_PHASE_0, &data);
	PX4_INFO("BLINK_PHASE_0: %X", data);

	readRegister(Register::BLINK_PHASE_1, &data);
	PX4_INFO("BLINK_PHASE_1: %X", data);

	readRegister(Register::PORT_CONFIG, &data);
	PX4_INFO("PORT_CONFIG: %X", data);

	readRegister(Register::MASTER, &data);
	PX4_INFO("MASTER: %X", data);

	readRegister(Register::CONFIG, &data);
	PX4_INFO("CONFIG: %X", data);

	readRegister(Register::INT_10, &data);
	PX4_INFO("INT10: %X", data);

	readRegister(Register::INT_32, &data);
	PX4_INFO("INT32: %X", data);

	readRegister(Register::INT_54, &data);
	PX4_INFO("INT54: %X", data);

	readRegister(Register::INT_76, &data);
	PX4_INFO("INT76: %X", data);
}

uint8_t
MAX7315::readRegister(Register reg, uint8_t *data)
{
	uint8_t cmd = static_cast<uint8_t>(reg);
	int ret = transfer(&cmd, 1, data, 1);

	return ret;
}

uint8_t
MAX7315::writeRegister(Register reg, uint8_t data)
{
	uint8_t value[2] {static_cast<uint8_t>(reg), data};

	return transfer(value, sizeof(value), nullptr, 0);
}

void
MAX7315::start()
{
	ScheduleClear();

	/* schedule a cycle to start things */
	ScheduleDelayed(5);
}

void
MAX7315::stop()
{
	ScheduleClear();
}

void
MAX7315::Run()
{
	perf_begin(_sample_perf);


	ScheduleDelayed(MAX7315_CONVERSION_INTERVAL);

	perf_end(_sample_perf);
}

void
MAX7315::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
}
