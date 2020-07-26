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

#include "PCA9505.hpp"

PCA9505::PCA9505(int bus, int address) :
	I2C(DRV_GPIO_DEVTYPE_PCA9505, MODULE_NAME, bus, address, 400000),
	_sample_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": read")),
	_comms_errors(perf_alloc(PC_COUNT, MODULE_NAME": communication error"))
{
	//_debug_enabled = true;
	_retries = 1;
}

PCA9505::~PCA9505()
{
	// free perf counters
	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int
PCA9505::init()
{
	PX4_INFO("init");

	// do I2C init (and probe) first
	if (I2C::init() != OK) {
		return PX4_ERROR;
	}

	// initial configuration

	// IO0 set all to input
	writeRegister(Register::IOC0, 0b1111111);

	// IO1 set all to input
	writeRegister(Register::IOC1, 0b1111111);

	// IO2 set low 7 to inputs; NB the 9505 internally pulls them
	// up through a 100K
	// b7 left as an output because we set it 0 to turn the smart sensor
	// IR led off
	writeRegister(Register::IOC2, 0b01111111);
	// IR LEDs on IO2_7 is active high. The control must be high to gate the 3902's pulsing
	// thereof. Note that this means you must physically remove the 3901's IR LED or else
	// it'll get hot enough to grill meat
	// God knows what's on 6
	// 5-0 are XSHUTs
	writeRegister(Register::OP2, 0b10000000); // IR LEDs enabd, XSHUTs asserted

	// IO3 set all to output. low 5 are opt flow resets and the high 3
	// are chip selects going somewhere not apparent on the hub bd s
	writeRegister(Register::IOC3, 0b00000000);
	writeRegister(Register::OP3, 0b00000000); // /reset the opt flows
	usleep(2000);
	writeRegister(Register::OP3, 0b00011111); // deassert their /resets

	// IO4 set all to output
	writeRegister(Register::IOC4, 0b00000000);

	//start();

	return PX4_OK;
}

int
PCA9505::probe()
{
	return OK;
}

uint8_t
PCA9505::readRegister(Register reg)
{
	uint8_t data{};

	uint8_t cmd = static_cast<uint8_t>(reg);
	transfer(&cmd, 1, &data, 1);

	return data;
}

uint8_t
PCA9505::writeRegister(Register reg, uint8_t data)
{
	uint8_t value[2] { static_cast<uint8_t>(reg), data };
	return transfer(value, sizeof(value), nullptr, 0);
}

void
PCA9505::modifyRegister(Register reg, uint8_t setbits)
{
	uint8_t	val = readRegister(reg);
	val |= setbits;
	writeRegister(reg, val);
}

void
PCA9505::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);

	PX4_INFO("IP0: %X", readRegister(Register::IP0));
	PX4_INFO("IP1: %X", readRegister(Register::IP1));
	PX4_INFO("IP2: %X", readRegister(Register::IP2));
	PX4_INFO("IP3: %X", readRegister(Register::IP3));
	PX4_INFO("IP4: %X", readRegister(Register::IP4));

	PX4_INFO("OP0: %X", readRegister(Register::OP0));
	PX4_INFO("OP1: %X", readRegister(Register::OP1));
	PX4_INFO("OP2: %X", readRegister(Register::OP2));
	PX4_INFO("OP3: %X", readRegister(Register::OP3));
	PX4_INFO("OP4: %X", readRegister(Register::OP4));

	PX4_INFO("IOC0: %X", readRegister(Register::IOC0));
	PX4_INFO("IOC1: %X", readRegister(Register::IOC1));
	PX4_INFO("IOC2: %X", readRegister(Register::IOC2));
	PX4_INFO("IOC3: %X", readRegister(Register::IOC3));
	PX4_INFO("IOC4: %X", readRegister(Register::IOC4));
}

void
PCA9505::EnableSensor(DIST_SENS s)
{
	switch (s) {

	// Cs
	case DIST_SENS::Forward_0:
		// Front Right C
		writeRegister(Register::OP4, SensorArm::FrontRight);
		writeRegister(Register::OP3, 0b10011111);	// IO3_{7} (CS_C)s
		break;

	case DIST_SENS::Right_1:
		// Back Right C
		writeRegister(Register::OP4, SensorArm::RearRight);
		writeRegister(Register::OP3, 0b10011111);	// IO3_{7} (CS_C)s
		break;

	case DIST_SENS::Backward_1:
		// Back Left C
		writeRegister(Register::OP4, SensorArm::RearLeft);
		writeRegister(Register::OP3, 0b10011111);	// IO3_{7} (CS_C)s
		break;

	case DIST_SENS::Left_1:
		// Front Left C
		writeRegister(Register::OP4, SensorArm::FrontLeft);
		writeRegister(Register::OP3, 0b10011111);	// IO3_{7} (CS_C)s
		break;



	// Bs
	case DIST_SENS::ForwardRight:
		// Front Right B
		writeRegister(Register::OP4, SensorArm::FrontRight);
		writeRegister(Register::OP3, 0b11011111);	// IO3_{7, 6} (CS_C + CS_B)
		break;

	case DIST_SENS::BackwardRight:
		// Back Right B
		writeRegister(Register::OP4, SensorArm::RearRight);
		writeRegister(Register::OP3, 0b11011111);	// IO3_{7, 6} (CS_C + CS_B)
		break;

	case DIST_SENS::BackwardLeft:
		// Front Left B
		writeRegister(Register::OP4, SensorArm::RearLeft);
		writeRegister(Register::OP3, 0b11011111);	// IO3_{7, 6} (CS_C + CS_B)
		break;

	case DIST_SENS::ForwardLeft:
		// Front Left B
		writeRegister(Register::OP4, SensorArm::FrontLeft);
		writeRegister(Register::OP3, 0b11011111);	// IO3_{7, 6} (CS_C + CS_B)
		break;


	// As
	case DIST_SENS::Right_0:
		// Front Right A
		writeRegister(Register::OP4, SensorArm::FrontRight);
		writeRegister(Register::OP3, 0b11111111);	// IO3_{7, 6, 5} (CS_C + CS_B + CS_A)
		break;

	case DIST_SENS::Backward_0:
		// Back Right A
		writeRegister(Register::OP4, SensorArm::RearRight);
		writeRegister(Register::OP3, 0b11111111);	// IO3_{7, 6, 5} (CS_C + CS_B + CS_A)
		break;

	case DIST_SENS::Left_0:
		// Back Right A
		writeRegister(Register::OP4, SensorArm::RearLeft);
		writeRegister(Register::OP3, 0b11111111);	// IO3_{7, 6, 5} (CS_C + CS_B + CS_A)
		break;

	case DIST_SENS::Forward_1:
		// Front Left A
		writeRegister(Register::OP4, SensorArm::FrontLeft);
		writeRegister(Register::OP3, 0b11111111);	// IO3_{7, 6, 5} (CS_C + CS_B + CS_A)
		break;
	}
}

void
PCA9505::EnableArm(uint8_t arm)
{
	writeRegister(Register::OP4, arm);
}
