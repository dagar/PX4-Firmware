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
 * @file MAX7315.hpp
 *
 * Driver for the I2C attached MAX7315
 */

#pragma once

#include <string.h>

#include <px4_config.h>
#include <px4_getopt.h>
#include <drivers/device/i2c.h>
#include <lib/perf/perf_counter.h>
#include <drivers/drv_hrt.h>
#include <uORB/uORB.h>
#include <uORB/topics/power_monitor.h>
#include <px4_work_queue/ScheduledWorkItem.hpp>

using namespace time_literals;

static constexpr uint32_t MAX7315_CONVERSION_INTERVAL = 10_ms;

/* Configuration Constants */
static constexpr uint8_t MAX7315_BUS_DEFAULT = PX4_I2C_BUS_EXPANSION;
static constexpr uint8_t MAX7315_BASEADDR =
	0x28;	// The 7-bit i2c slave address (note: this can be different based on the device connections)

class MAX7315 : public device::I2C, px4::ScheduledWorkItem
{
public:
	MAX7315(int bus = MAX7315_BUS_DEFAULT, int address = MAX7315_BASEADDR);
	virtual ~MAX7315();

	virtual int 		  	init();

	void				print_info();


	void				test(int p);

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
		INPUTS = 0x00,
		BLINK_PHASE_0 = 0x01,	// Blink phase 0 outputs P7–P0
		BLINK_PHASE_1 = 0x09,	// Blink phase 1 outputs P7–P0
		PORT_CONFIG = 0x03,	// Ports configuration P7–P0
		MASTER = 0x0E,		// Master and global/O8 intensity register
		CONFIG = 0x0F,		// Configuration register
		INT_10 = 0x10,		// Outputs intensity P1, P0
		INT_32 = 0x11,		// Outputs intensity P3, P2
		INT_54 = 0x12,		// Outputs intensity P5, P4
		INT_76 = 0x13		// Outputs intensity P7, P6
	};

	enum
	CONFIG_BIT : uint8_t {
		GLOBAL_INTENSITY_CONTROL = Bit2,
	};

	uint8_t				readRegister(Register reg, uint8_t *data);
	uint8_t				writeRegister(Register reg, uint8_t data);

	void				start();
	void				stop();

	void				Run() override;


	perf_counter_t			_sample_perf;
	perf_counter_t			_comms_errors;

};
