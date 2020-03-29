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
 * @file ST_LSM303D_registers.hpp
 *
 * ST LSM303D registers.
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

namespace ST_LSM303D
{
static constexpr uint32_t SPI_SPEED = 10 * 1000 * 1000; // 10MHz SPI serial interface

static constexpr uint8_t DIR_READ = Bit7;
static constexpr uint8_t AUTO_INCREMENT = Bit6;

static constexpr uint8_t WHOAMI = 0b01001001; // Who I am ID

static constexpr uint32_t LA_ODR  = 1600; // Accelerometer output data rate

enum class Register : uint8_t {
	TEMP_OUT_L    = 0x05,
	TEMP_OUT_H    = 0x06,
	STATUS_M      = 0x07,
	OUT_X_L_M     = 0x08,

	WHO_AM_I      = 0x0F,

	CTRL0         = 0x1F,
	CTRL1         = 0x20,
	CTRL2         = 0x21,
	CTRL3         = 0x22,
	CTRL4         = 0x23,
	CTRL5         = 0x24,
	CTRL6         = 0x25,
	CTRL7         = 0x26,
	STATUS_A      = 0x27,
	OUT_X_L_A     = 0x28,

	FIFO_CTRL     = 0x2E,
	FIFO_SRC      = 0x2F,
};

// CTRL0
enum CTRL0_BIT : uint8_t {
	BOOT    = Bit7,
	FIFO_EN = Bit6,
	FTH_EN  = Bit5,
};

// CTRL1
enum CTRL1_BIT : uint8_t {
	AODR_1600Hz = Bit7 | Bit5, // 1010 1600 Hz
	BDU = Bit3, // Block data update
};

// CTRL2
enum CTRL2_BIT : uint8_t {
	AFS_16G = Bit5, // 100 ± 16g
};

// CTRL3
enum CTRL3_BIT : uint8_t {
	INT1_DRDY_A = Bit2,
};

// CTRL4
enum CTRL4_BIT : uint8_t {
	INT2_DRDY_A  = Bit3,

	INT2_Overrun = Bit1,
	INT2_FTH     = Bit0,
};

// CTRL5
enum CTRL5_BIT : uint8_t {
	TEMP_EN      = Bit7,
	M_RES_HIGH   = Bit6 | Bit5,
	M_ODR_50_HZ  = Bit4,        // 100  50Hz
	M_ODR_100_HZ = Bit4 | Bit2, // 101 100Hz
};

// CTRL6
enum CTRL6_BIT : uint8_t {
	MFS_2_GAUSS  = 0,
	MFS_4_GAUSS  = Bit5,
	MFS_8_GAUSS  = Bit6,
	MFS_12_GAUSS = Bit6 | Bit5,
};

// CTRL7
enum CTRL7_BIT : uint8_t {
	MD = Bit1 | Bit0, // Magnetic sensor mode selection
};

// FIFO_CTRL
enum FIFO_CTRL_BIT : uint8_t {
	FIFO_mode = Bit5,
	Bypass_mode = Bit7 | Bit6 | Bit5, // 000
	// FTH [4:0]: FIFO threshold level.
};

// FIFO_SRC_REG
enum FIFO_SRC_BIT : uint8_t {
	FTH   = Bit7, // FIFO threshold status
	OVRN  = Bit6, // FIFO overrun status
	EMPTY = Bit5, // Empty status.
	// FSS [4:0]: FIFO stored data level.
};

namespace FIFO
{
//  LSM303D embeds 32 slots of 16-bit data FIFO for each of the three output channels
static constexpr size_t SIZE = 32 * 2 * 3;

struct DATA {
	uint8_t OUT_X_L_A;
	uint8_t OUT_X_H_A;
	uint8_t OUT_Y_L_A;
	uint8_t OUT_Y_H_A;
	uint8_t OUT_Z_L_A;
	uint8_t OUT_Z_H_A;
};

}

} // namespace ST_LSM303D
