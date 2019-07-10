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

#pragma once

#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>

#include "Bosch_BMI088_Accelerometer_Registers.hpp"

#include <drivers/device/spi.h>

class BMI088_Accelerometer : public device::SPI
{
public:
	BMI088_Accelerometer(int bus, uint32_t device, enum Rotation rotation);
	virtual ~BMI088_Accelerometer();

	virtual int     init();

	// Diagnostics - print some basic information about the driver.
	void            print_info();

	void            print_registers();

protected:

	virtual int     probe();

private:

	PX4Accelerometer	_px4_accel;

	perf_counter_t      _sample_perf;
	perf_counter_t      _measure_interval;
	perf_counter_t      _bad_transfers;
	perf_counter_t      _bad_registers;
	perf_counter_t      _duplicates;

	// this is used to support runtime checking of key
	// configuration registers to detect SPI bus errors and sensor
	// reset
	static constexpr int BMI088_ACCEL_NUM_CHECKED_REGISTERS = 5;
	static const uint8_t    _checked_registers[BMI088_ACCEL_NUM_CHECKED_REGISTERS];
	uint8_t         _checked_values[BMI088_ACCEL_NUM_CHECKED_REGISTERS];
	uint8_t         _checked_bad[BMI088_ACCEL_NUM_CHECKED_REGISTERS];

	int         	reset();

	/**
	 * Fetch measurements from the sensor
	 */
	void            measure();

	/**
	 * Modify a register in the BMI088_accel
	 *
	 * Bits are cleared before bits are set.
	 *
	 * @param reg       The register to modify.
	 * @param clearbits Bits in the register to clear.
	 * @param setbits   Bits in the register to set.
	 */
	void            modify_reg(unsigned reg, uint8_t clearbits, uint8_t setbits);

	/**
	 * Write a register in the BMI088_accel, updating _checked_values
	 *
	 * @param reg       The register to write.
	 * @param value     The new value to write.
	 */
	void            write_checked_reg(unsigned reg, uint8_t value);

	/**
	 * Set the BMI088_accel measurement range.
	 *
	 * @param max_g     The maximum G value the range must support.
	 * @return      OK if the value can be supported, -EINVAL otherwise.
	 */
	int         set_accel_range(unsigned max_g);

	/*
	 * check that key registers still have the right value
	 */
	void		check_registers();

};
