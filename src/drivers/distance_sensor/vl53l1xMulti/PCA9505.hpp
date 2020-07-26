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

/**
 * @file PCA9505.hpp
 *
 * Driver for the I2C attached PCA9505
 */

#pragma once

#include <string.h>

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/getopt.h>
#include <drivers/device/i2c.h>
#include <lib/perf/perf_counter.h>
#include <drivers/drv_hrt.h>
#include <uORB/uORB.h>
#include <uORB/topics/power_monitor.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

using namespace time_literals;

static constexpr uint32_t PCA9505_CONVERSION_INTERVAL = 10_ms;

/* Configuration Constants */
static constexpr uint8_t PCA9505_BUS_DEFAULT = 4; // TODO: modernize
static constexpr uint8_t PCA9505_BASEADDR = 0x20; // 0b010000

class PCA9505 : public device::I2C
{
public:
	PCA9505(int bus = PCA9505_BUS_DEFAULT, int address = PCA9505_BASEADDR);
	virtual ~PCA9505();

	virtual int 		  	init();

	void				print_info();

	void				test(int p);

	// OP4
	enum SensorArm : uint8_t {
		FrontRight	= 0b00010001,	// EN1 + CS_CS1
		FrontLeft	= 0b00100010,	// EN2 + CS_CS2
		RearRight	= 0b01000100,	// EN3 + CS_CS3
		RearLeft	= 0b10001000,	// EN4 + CS_CS4
	};

	// Sensor
	enum class DIST_SENS {
		// front right arm
		Forward_0	= 0,
		ForwardRight	= 1,
		Right_0		= 2,

		// back right arm
		Right_1		= 3,
		BackwardRight	= 4,
		Backward_0	= 5,

		// back left arm
		Backward_1	= 6,
		BackwardLeft	= 7,
		Left_0		= 8,

		// front left arm
		Left_1		= 9,
		ForwardLeft	= 10,
		Forward_1	= 11,
	};

	void				EnableSensor(DIST_SENS sensor);
	void				EnableArm(uint8_t arm);

protected:

	virtual int	  		probe();

private:

	static constexpr uint8_t Bit0 = (1 << 0);
	static constexpr uint8_t Bit1 = (1 << 1);
	static constexpr uint8_t Bit2 = (1 << 2);
	static constexpr uint8_t Bit3 = (1 << 3);
	static constexpr uint8_t Bit4 = (1 << 4);
	static constexpr uint8_t Bit5 = (1 << 5);
	static constexpr uint8_t Bit6 = (1 << 6);
	static constexpr uint8_t Bit7 = (1 << 7);

	enum class Register : uint8_t {

		IP0 = 0x00,	// 00 0000 - Input Port register bank 0
		IP1 = 0x01,
		IP2 = 0x02,
		IP3 = 0x03,
		IP4 = 0x04,

		OP0 = 0x08,	// 00 1000 - Output Port register bank 0
		OP1 = 0x09,
		OP2 = 0x0A,
		OP3 = 0x0B,
		OP4 = 0x0C,

		IOC0 = 0x18,	// 01 1000 - Configuration register bank 0
		IOC1 = 0x19,
		IOC2 = 0x1A,
		IOC3 = 0x1B,
		IOC4 = 0x1C,

	};

	uint8_t				readRegister(Register reg);
	uint8_t				writeRegister(Register reg, uint8_t data);
	void				modifyRegister(Register reg, uint8_t setbits);

	perf_counter_t			_sample_perf;
	perf_counter_t			_comms_errors;

};
