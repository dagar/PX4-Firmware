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

/**
 * @file Bosch_BMP388_registers.hpp
 *
 * Bosch BMP388 registers.
 *
 */

#pragma once

#include <cstdint>

// TODO: move to a central header
static constexpr uint8_t Bit0 = (1 << 0);
static constexpr uint8_t Bit1 = (1 << 1);
static constexpr uint8_t Bit2 = (1 << 2);
static constexpr uint8_t Bit3 = (1 << 3);
static constexpr uint8_t Bit4 = (1 << 4);
static constexpr uint8_t Bit5 = (1 << 5);
static constexpr uint8_t Bit6 = (1 << 6);
static constexpr uint8_t Bit7 = (1 << 7);

namespace Bosch_BMP388
{
static constexpr uint32_t I2C_SPEED = 400 * 1000; // 400 kHz I2C serial interface
static constexpr uint8_t I2C_ADDRESS_DEFAULT = 0b1110110; // or 0b1110111

static constexpr uint8_t chip_identification_number = 0x32;

static constexpr int16_t OVERFLOW_XYAXES = -4096;
static constexpr int16_t OVERFLOW_ZAXIS  = -16384;

enum class Register : uint8_t {
	CHIP_ID         = 0x00, // chip identification code

	ERR_REG         = 0x02, // Sensor Error conditions
	STATUS          = 0x03, // Status register
	PRESS_XLSB_7_0  = 0x04,
	PRESS_LSB_15_8  = 0x05,
	PRESS_MSB_23_16 = 0x06,
	TEMP_XLSB_7_0   = 0x07,
	TEMP_LSB_15_8   = 0x08,
	TEMP_MSB_23_16  = 0x09,

	PWR_CTRL        = 0x1B, // enables or disables pressure and temperature measurement
	OSR             = 0x1C, // controls the oversampling settings for pressure and temperature measurements
	ODR             = 0x1D, // set the configuration of the output data rates by means of setting the subdivision/subsampling.

	CALIB_DATA      = 0x31,

	CMD             = 0x7E, //

};

// PWR_CTRL
enum PWR_CTRL_BIT : uint8_t {

	// Bit 5..4
	normal_mode = Bit5|Bit4,

	temp_en     = Bit1, // Enables the temperature sensor.
	press_en    = Bit0, // Enables the pressure sensor.
};

// OSR
enum OSR_BIT : uint8_t {
	// Bit 5..3 osr4_t Oversampling setting temperature measurement
	osr4_tx32 = Bit5 | Bit3, // 0b101 x32 oversampling

	// Bit 2..0 osr_p Oversampling setting pressure measurement
	osr_px32  = Bit2 | Bit0, // 0b101 x32 oversampling
	osr_px16  = Bit2,        // 0b100 x32 oversampling
};

// ODR
enum ODR_BIT : uint8_t {
	ODR_25 = 0x03, // ODR 25Hz
};

// STATUS
enum STATUS_BIT : uint8_t {
	Overflow = Bit6, // one or more axes exceeded maximum range of the device
};

// CMD
enum CMD_BIT : uint8_t {
	softreset = 0xB6, // Triggers a reset
};

} // namespace Bosch_BMP388
