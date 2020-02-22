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
 * @file InvenSense_ICM20948_registers.hpp
 *
 * Invensense ICM-20602 registers.
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

namespace InvenSense_ICM20948
{
static constexpr uint32_t SPI_SPEED = 7 * 1000 * 1000; // 7 MHz SPI
static constexpr uint8_t DIR_READ = 0x80;

static constexpr uint8_t WHOAMI = 0xEA;

static constexpr float TEMPERATURE_SENSITIVITY = 326.8f; // LSB/C
static constexpr float ROOM_TEMPERATURE_OFFSET = 25.f; // C

namespace Register
{

enum class BANK_0 : uint8_t {
	WHO_AM_I             = 0x00,

	USER_CTRL            = 0x03,

	PWR_MGMT_1           = 0x06,

	INT_ENABLE_1         = 0x11,

	I2C_MST_STATUS       = 0x17,

	TEMP_OUT_H           = 0x39,
	TEMP_OUT_L           = 0x3A,
	EXT_SLV_SENS_DATA_00 = 0x3B,

	FIFO_EN_2            = 0x67,
	FIFO_RST             = 0x68,

	FIFO_COUNTH          = 0x70,
	FIFO_COUNTL          = 0x71,
	FIFO_R_W             = 0x72,

	REG_BANK_SEL         = 0x7F,
};

enum class BANK_2 : uint8_t {
	GYRO_CONFIG_1        = 0x01,

	ACCEL_CONFIG         = 0x14,

	REG_BANK_SEL         = 0x7F,
};

enum class BANK_3 : uint8_t {
	I2C_MST_ODR_CONFIG   = 0x00,
	I2C_MST_CTRL         = 0x01,

	I2C_SLV0_ADDR        = 0x03,
	I2C_SLV0_REG         = 0x04,
	I2C_SLV0_CTRL        = 0x05,
	I2C_SLV0_DO          = 0x06,
	I2C_SLV1_ADDR        = 0x07,

	REG_BANK_SEL         = 0x7F,
};
};


//---------------- BANK0 Register bits
// USER_CTRL
enum USER_CTRL_BIT : uint8_t {
	FIFO_EN     = Bit6,
	I2C_MST_EN  = Bit5, // Enable the I2C Master I/F module
	I2C_IF_DIS  = Bit4, // Reset I2C Slave module and put the serial interface in SPI mode only

	I2C_MST_RST = Bit1, // Reset I2C Master module.
};

// PWR_MGMT_1
enum PWR_MGMT_1_BIT : uint8_t {
	DEVICE_RESET = Bit7,
	SLEEP        = Bit6,

	CLKSEL_2     = Bit2,
	CLKSEL_1     = Bit1,
	CLKSEL_0     = Bit0,
};

// INT_ENABLE_1
enum INT_ENABLE_1_BIT : uint8_t {
	RAW_DATA_0_RDY_EN = Bit0,
};

// FIFO_EN_2
enum FIFO_EN_2_BIT : uint8_t {
	ACCEL_FIFO_EN  = Bit4,
	GYRO_Z_FIFO_EN = Bit3,
	GYRO_Y_FIFO_EN = Bit2,
	GYRO_X_FIFO_EN = Bit1,
	TEMP_FIFO_EN   = Bit0,
};

// FIFO_RST
enum FIFO_RST_BIT : uint8_t {
	FIFO_RESET = Bit0,
};

// REG_BANK_SEL
enum REG_BANK_SEL_BIT : uint8_t {
	USER_BANK_0 = 0,           // 0: Select USER BANK 0.
	USER_BANK_1 = Bit3,        // 1: Select USER BANK 0.
	USER_BANK_2 = Bit4,        // 2: Select USER BANK 0.
	USER_BANK_3 = Bit4 | Bit3, // 3: Select USER BANK 0.
};

//---------------- BANK2 Register bits
// GYRO_CONFIG_1
enum GYRO_CONFIG_1_BIT : uint8_t {
	// GYRO_FS_SEL[1:0]
	GYRO_FS_SEL_250_DPS  = 0,           // 0b00 = ±250 dps
	GYRO_FS_SEL_500_DPS  = Bit1,        // 0b01 = ±500 dps
	GYRO_FS_SEL_1000_DPS = Bit2,        // 0b10 = ±1000 dps
	GYRO_FS_SEL_2000_DPS = Bit2 | Bit1, // 0b11 = ±2000 dps

	GYRO_FCHOICE         = Bit0,        // 0 – Bypass gyro DLPF
};

// ACCEL_CONFIG
enum ACCEL_CONFIG_BIT : uint8_t {
	// ACCEL_FS_SEL[1:0]
	ACCEL_FS_SEL_2G  = 0,           // 0b00: ±2g
	ACCEL_FS_SEL_4G  = Bit1,        // 0b01: ±4g
	ACCEL_FS_SEL_8G  = Bit2,        // 0b10: ±8g
	ACCEL_FS_SEL_16G = Bit2 | Bit1, // 0b11: ±16g

	ACCEL_FCHOICE    = Bit0,        // 0: Bypass accel DLPF
};

namespace FIFO
{
static constexpr size_t SIZE = 512;

// FIFO_DATA layout when FIFO_EN has ACCEL_FIFO_EN and GYRO_{Z, Y, X}_FIFO_EN set
struct DATA {
	uint8_t ACCEL_XOUT_H;
	uint8_t ACCEL_XOUT_L;
	uint8_t ACCEL_YOUT_H;
	uint8_t ACCEL_YOUT_L;
	uint8_t ACCEL_ZOUT_H;
	uint8_t ACCEL_ZOUT_L;
	uint8_t GYRO_XOUT_H;
	uint8_t GYRO_XOUT_L;
	uint8_t GYRO_YOUT_H;
	uint8_t GYRO_YOUT_L;
	uint8_t GYRO_ZOUT_H;
	uint8_t GYRO_ZOUT_L;
};
}
} // namespace InvenSense_ICM20948
