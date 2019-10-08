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

namespace Infineon_DPS310
{

static constexpr uint8_t ID = 0x10;

enum class
Register : uint8_t {

	PSR_B2		= 0x00,
	PSR_B1		= 0x01,
	PSR_B0		= 0x02,
	TMP_B2		= 0x03,
	TMP_B1		= 0x04,
	TMP_B0		= 0x05,
	PRS_CFG		= 0x06,
	TMP_CFG		= 0x07,
	MEAS_CFG	= 0x08,

	RESET		= 0x0C,
	ID		= 0x0D,

	COEF		= 0x10,
	//	c0	= 0x10
	//	 .
	//	c30	= 0x21

	COEF_SRCE	= 0x28,

};


enum PRS_CFG : uint8_t
{
	PM_RATE_32HZ	= Bit6 | Bit4,	// 101 - 32 measurements pr. sec.
	PM_PRC_32	= Bit1 | Bit0,	// 0011 // 8 times
};

struct CalibrationCoefficients {
	int16_t c0;	// 12bit
	int16_t c1;	// 12bit
	int32_t c00;	// 20bit
	int32_t c10;	// 20bit
	int16_t c01;	// 16bit
	int16_t c11;	// 16bit
	int16_t c20;	// 16bit
	int16_t c21;	// 16bit
	int16_t c30;	// 16bit
};

} // namespace Infineon_DPS310
