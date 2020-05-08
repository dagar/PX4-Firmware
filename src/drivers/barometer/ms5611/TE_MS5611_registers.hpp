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
 * @file TE_MS5611_registers.hpp
 *
 * TE MS5611 registers.
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

namespace TE_MS5611
{
static constexpr uint32_t I2C_SPEED = 400 * 1000; // 400kHz I2C
static constexpr uint32_t SPI_SPEED = 20 * 1000 * 1000; // 10MHz SPI serial interface
static constexpr uint8_t DIR_READ = 0x80;

// I2C address 111011Cx

static constexpr uint32_t RESET_TIME_US{2800};

static constexpr uint32_t OSR4096_MAX_CONVERSION_TIME_US{9040};

static constexpr float OPERATING_PRESSURE_MIN{450}; // mbar
static constexpr float OPERATING_PRESSURE_MAX{1100}; // mbar

static constexpr float TEMPERATURE_MIN{-40.f};
static constexpr float TEMPERATURE_MAX{85.f};

enum Register : uint8_t {
	ADC_Read           = 0x00,

	Reset              = 0x1E,

	// D1: pressure
	Convert_D1_OSR256  = 0x40,
	Convert_D1_OSR512  = 0x42,
	Convert_D1_OSR1024 = 0x44,
	Convert_D1_OSR2048 = 0x46,
	Convert_D1_OSR4096 = 0x48,

	// D2: temperature
	Convert_D2_OSR256  = 0x50,
	Convert_D2_OSR512  = 0x52,
	Convert_D2_OSR1024 = 0x54,
	Convert_D2_OSR2048 = 0x56,
	Convert_D2_OSR4096 = 0x58,

	PROM_Read          = 0xA0,
};

struct PROM {
	uint16_t factory_setup;
	uint16_t C1; // Pressure sensitivity | SENS_T1
	uint16_t C2; // Pressure offset | OFF_T1
	uint16_t C3; // Temperature coefficient of pressure sensitivity | TCS
	uint16_t C4; // Temperature coefficient of pressure offset | TCO
	uint16_t C5; // Reference temperature | T_REF
	uint16_t C6; // Temperature coefficient of the temperature | TEMPSENS
	uint16_t serial_and_crc;
};

} // namespace InvenSense_ICM20602
