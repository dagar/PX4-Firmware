/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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
 * @file PNI_RM3100_registers.hpp
 *
 * PNI Sensor Corporation RM3100 registers.
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

namespace PNI_RM3100
{
static constexpr uint32_t I2C_SPEED = 400 * 1000; // 400 kHz I2C serial interface
static constexpr uint8_t I2C_ADDRESS_DEFAULT = 0x0E;

static constexpr uint8_t Device_ID = 0x10;

enum class Register : uint8_t {
	POLL       = 0x00, // Polls for a Single Measurement
	CMM        = 0x01, // Initiates Continuous Measurement Mode

	TMRC       = 0x0B, // Sets Continuous Measurement Mode Data Rate

	BIST       = 0x33, // Built-In Self Test
	STATUS     = 0x34, // Status of DRDY
	HSHAKE     = 0x35, // Handshake Register
	REVID      = 0x36, // MagI2C Revision Identification
};

// CMM
enum CMM_BIT : uint8_t {
	CMZ   = Bit6,
	CMY   = Bit5,
	CMX   = Bit4,

	START = Bit0, // A “1” in this bit position initiates Continuous Measurement Mode.
};

// TMRC
enum TMRC_BIT : uint8_t {
	// TMRC Value 0x96, ~27 ms, ~37 Hz
	CMM_37HZ_SET   = Bit2 | Bit1,
	CMM_37HZ_CLEAR = Bit3 | Bit0,
};

// AVGCNTL
enum AVGCNTL_BIT : uint8_t {
	// 5:3 Average times for y sensor data. Times of average will be done before switch to next channel
	Y_16TIMES_SET   = Bit5, // 3’b100 average by 16 times (ODRmax=166Hz)
	Y_16TIMES_CLEAR = Bit4 | Bit3,

	// 2:0 Average times for x & z sensor data. Times of average will be done before switch to next channel
	XZ_16TIMES_SET   = Bit2, // average by 16 times (ODRmax=166Hz)
	XZ_16TIMES_CLEAR = Bit1 | Bit0,
};

// PDCNTL
enum PDCNTL_BIT : uint8_t {
	// 7:6 Pulse duration
	PULSE_NORMAL = Bit7 | Bit6, // Normal (please use this setting)
};

} // namespace iSentek_RM3100
