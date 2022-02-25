/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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
 * @file SmartBatteryStandard_registers.hpp
 *
 * Smart Battery Standard registers.
 *
 */

#pragma once

#include <cstdint>

// TODO: move to a central header
// TODO: move to a central header
static constexpr uint16_t Bit0  = (1 << 0);
static constexpr uint16_t Bit1  = (1 << 1);
static constexpr uint16_t Bit2  = (1 << 2);
static constexpr uint16_t Bit3  = (1 << 3);
static constexpr uint16_t Bit4  = (1 << 4);
static constexpr uint16_t Bit5  = (1 << 5);
static constexpr uint16_t Bit6  = (1 << 6);
static constexpr uint16_t Bit7  = (1 << 7);
static constexpr uint16_t Bit8  = (1 << 8);
static constexpr uint16_t Bit9  = (1 << 9);
static constexpr uint16_t Bit10 = (1 << 10);
static constexpr uint16_t Bit11 = (1 << 11);
static constexpr uint16_t Bit12 = (1 << 12);
static constexpr uint16_t Bit13 = (1 << 13);
static constexpr uint16_t Bit14 = (1 << 14);
static constexpr uint16_t Bit15 = (1 << 15);

namespace SmartBatteryStandard
{
static constexpr uint32_t I2C_SPEED = 100 * 1000; // 100 kHz I2C serial interface

enum class Register : uint16_t {
	ManufacturerAccess     = 0x0000, // OPTIONAL
	RemainingCapacityAlarm = 0x0001,
	RemainingTimeAlarm     = 0x0002,
	BatteryMode            = 0x0003,
	AtRate                 = 0x0004,
	AtRateTimeToFull       = 0x0005,
	AtRateTimeToEmpty      = 0x0006,
	AtRateOK               = 0x0007,
	Temperature            = 0x0008,
	Voltage                = 0x0009,
	Current                = 0x000A,
	AverageCurrent         = 0x000B,
	MaxError               = 0x000C,
	RelativeStateOfCharge  = 0x000D,
	AbsoluteStateOfCharge  = 0x000E,
	RemainingCapacity      = 0x000F,
	FullChargeCapacity     = 0x0010,
	RunTimeToEmpty         = 0x0011,
	AverageTimeToEmpty     = 0x0012,
	AverageTimeToFull      = 0x0013,
	ChargingCurrent        = 0x0014,
	ChargingVoltage        = 0x0015,
	BatteryStatus          = 0x0016,
	CycleCount             = 0x0017,
	DesignCapacity         = 0x0018,
	DesignVoltage          = 0x0019,
	SpecificationInfo      = 0x001A,
	ManufacturerDate       = 0x001B,
	SerialNumber           = 0x001C,
	// reserved            = 0x001D
	// reserved            = 0x001E
	// reserved            = 0x001F
	ManufacturerName       = 0x0020,
	DeviceName             = 0x0021,
	DeviceChemistry        = 0x0022,
	ManufacturerData       = 0x0023,

	// reserved            = 0x0025
	//  ...
	// reserved            = 0x002E
	OptionalMfgFunction5   = 0x002F,
	// reserved            = 0x0030,
	//  ...
	// reserved            = 0x003B
	OptionalMfgFunction4   = 0x003C,
	OptionalMfgFunction3   = 0x003D,
	OptionalMfgFunction2   = 0x003E,
	OptionalMfgFunction1   = 0x003F,
};

// BatteryMode
enum BatteryMode_BIT : uint16_t {
	INTERNAL_CHARGE_CONTROLLER = Bit0, // OPTIONAL
	PRIMARY_BATTERY_SUPPORT    = Bit1,
	// Reserved 2-6
	CONDITION_FLAG             = Bit7,
	CHARGE_CONTROLLER_ENABLED  = Bit8,
	PRIMARY_BATTERY            = Bit9, // OPTIONAL
	// Reserved 10-12
	ALARM_MODE                 = Bit13,
	CHARGER_MODE               = Bit14,
	CAPACITY_MODE              = Bit15,
};

// BatteryStatus (0x0016)
enum BatteryStatus_BIT : uint16_t {
	OVER_CHARGED_ALARM        = Bit15, //
	TERMINATE_CHARGE_ALARM    = Bit14,
	// Reserved               = Bit13,
	OVER_TEMP_ALARM           = Bit12,
	TERMINATE_DISCHARGE_ALARM = Bit11,
	// Reserved               = Bit10,
	REMAINING_CAPACITY_ALARM  = Bit9,
	REMAINING_TIME_ALARM      = Bit8,
	INITIALIZED               = Bit7,
	DISCHARGING               = Bit6,
	FULLY_CHARGED             = Bit5,
	FULLY_DISCHARGEd          = Bit4,
	// Error Codes 3-0
};


// TODO:


// Devices which continuously poll the Smart Battery at high rate risk missing AlarmWarning and
// ChargingVoltage/ChargingCurrent broadcasts from the Smart Battery and so should therefore read these
// values at least once per ten (10) seconds to insure proper notification of these conditions and values.

// review MaxError()

// RunTimeToEmpty() AverageTimeToEmpty()


} // namespace iSentek_IST8310
