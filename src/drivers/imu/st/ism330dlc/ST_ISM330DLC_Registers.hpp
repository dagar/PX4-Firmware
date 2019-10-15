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
 * @file ST_ISM330DLC_registers.hpp
 *
 * ST ISM330DLC registers.
 *
 */

#pragma once

// TODO: move to a central header
static constexpr uint8_t Bit0 = (1 << 0);
static constexpr uint8_t Bit1 = (1 << 1);
static constexpr uint8_t Bit2 = (1 << 2);
static constexpr uint8_t Bit3 = (1 << 3);
static constexpr uint8_t Bit4 = (1 << 4);
static constexpr uint8_t Bit5 = (1 << 5);
static constexpr uint8_t Bit6 = (1 << 6);
static constexpr uint8_t Bit7 = (1 << 7);

namespace ST_ISM330DLC
{

static constexpr uint8_t DIR_READ	= 0x80;
static constexpr uint8_t DIR_WRITE	= 0x00;

static constexpr uint8_t ISM330DLC_WHO_AM_I = 0b01101010; // Who I am ID

static constexpr uint32_t SPI_SPEED = 10 * 1000 * 1000; // 10 MHz SPI clock frequency

static constexpr uint32_t LA_ODR = 6664;	// Linear acceleration output data rate
static constexpr uint32_t G_ODR = 6664;		// Angular rate output data rate

enum class
Register : uint8_t {
	FIFO_CTRL1	= 0x06,	// FIFO threshold level setting.

	WHO_AM_I	= 0x0F,

	FIFO_CTRL5	= 0x0A,

	INT1_CTRL	= 0x0D,

	CTRL1_XL	= 0x10,
	CTRL2_G		= 0x11,
	CTRL3_C		= 0x12,
	CTRL4_C		= 0x13,

	CTRL6_C		= 0x15,

	OUT_TEMP_L	= 0x20,
	OUT_TEMP_H	= 0x21,

	FIFO_STATUS1	= 0x3A,
	FIFO_STATUS2	= 0x3B,

	FIFO_DATA_OUT_L	= 0x3E,
	FIFO_DATA_OUT_H	= 0x3F,
};

// FIFO_CTRL5
enum
FIFO_CTRL5_BIT : uint8_t {
	FIFO_MODE_CONTINUOUS	= 0b110,
	ODR_FIFO_6_66_KHZ	= 0b1010, // FIFO ODR is set to 6.66 kHz
};

// INT1_CTRL
enum
INT1_CTRL_BIT : uint8_t {
	INT1_FULL_FLAG	= Bit5,
	INT1_FIFO_OVR	= Bit4,
};

// CTRL1_XL
enum
CTRL1_XL_BIT : uint8_t {
	ODR_XL_6_66KHZ	= Bit7 | Bit5,

	FS_XL_16	= Bit3 | Bit2,
};

// CTRL2_G
enum
CTRL2_G_BIT : uint8_t {
	ODR_G_6_66KHZ	= Bit7 | Bit5,

	FS_G_2000	= Bit3 | Bit2,
	FS_G_1000	= Bit3
};

// CTRL3_C
enum
CTRL3_C_BIT : uint8_t {
	SW_RESET	= Bit0
};


// CTRL6_C
enum
CTRL6_C_BIT : uint8_t {
	FTYPE_GYRO_LPF_BW_937_HZ = Bit1 | Bit0
};


namespace FIFO
{
static constexpr size_t SIZE = 4096;

// FIFO_DATA layout when FIFO_EN has both {X, Y, Z}G_FIFO_EN and ACCEL_FIFO_EN set
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
static_assert(sizeof(DATA) == 12);
}

} // namespace ST_ISM330DLC
