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
 * @file TI_BQ40ZXY_registers.hpp
 *
 * TI BQ40ZXY registers.
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

namespace TI_BQ40ZXY
{

// ManufacturerAccess() Command List
enum class Register : uint16_t {
	DeviceType                = 0x0001,
	FirmwareVersion           = 0x0002,
	HardwareVersion           = 0x0003,
	IFChecksum                = 0x0004,
	StaticDFSignature         = 0x0005,
	ChemID                    = 0x0006,

	StaticChemDFSignature     = 0x0008,
	AllDFSignature            = 0x0009,
	ShutdownMode              = 0x0010,
	SleepMode                 = 0x0011,

	AutoCCOfset               = 0x0013,

	FuseToggle                = 0x001D,
	PrechargeFETToggle        = 0x001E,
	ChargeFETToggle           = 0x001F,
	DischargeFETToggle        = 0x0020,
	Gauging                   = 0x0021,
	FETControl                = 0x0022,
	LifetimeDataCollection    = 0x0023,
	PermanentFailure          = 0x0024,
	BlackBoxRecorder          = 0x0025,
	Fuse                      = 0x0026,
	LEDDisplayEnable          = 0x0027,
	LifetimeDataReset         = 0x0028,
	PermanentFailureDataReset = 0x0029,
	BlackBoxRecorderReset     = 0x002A,
	LEDToggle                 = 0x002B,
	LEDDisplayPress           = 0x002C,
	CalibrationMode           = 0x002D,
	LifetimeDataFlush         = 0x002E,
	LifetimeDataSpeedUpMode   = 0x002F,
	SealDevice                = 0x0030,

	SecurityKeys              = 0x0035,

	AuthenticationKey         = 0x0037,

	DeviceReset               = 0x0041,

	SafetyAlert               = 0x0050,
	SafetyStatus              = 0x0051,
	PFAlert                   = 0x0052,
	PFStatus                  = 0x0053,
	OperationStatus           = 0x0054,
	ChargingStatus            = 0x0055,
	GaugingStatus             = 0x0056,
	ManufacturingStatus       = 0x0057,
	AFERegister               = 0x0058,

	NoLoadRemCap              = 0x005A,

	LifetimeDataBlock1        = 0x0060,
	LifetimeDataBlock2        = 0x0061,
	LifetimeDataBlock3        = 0x0062,
	LifetimeDataBlock4        = 0x0063,
	LifetimeDataBlock5        = 0x0064,

	ManufacturerInfo          = 0x0070,
	DAStatus1                 = 0x0071,
	DAStatus2                 = 0x0072,
	GaugeStatus1              = 0x0073,
	GaugeStatus2              = 0x0074,
	GaugeStatus3              = 0x0075,
	CBStatus                  = 0x0076,
	StateofHealth             = 0x0077,
	FilterCapacity            = 0x0078,
	RSOC_Write                = 0x0079,
	ManufacturerInfoB         = 0x007A,

	IATA_SHUTDOWN             = 0x00F0,
	IATA_RM                   = 0x00F1,
	IATA_FCC                  = 0x00F2,

	ROMMode                   = 0x0F00,

	ExitCalibrationOutput     = 0xF080,
	Output_CCADC_Cal          = 0xF081,
	Output_Shorted_CCADC_Cal  = 0xF082,
};

// additional SBS commands (in addition to smart battery standard)
enum class Register : uint16_t {

	Authenticate           = 0x002F, // OptionalMfgFunction5
	// reserved            = 0x0030,
	//  ...
	// reserved            = 0x003B
	CellVoltage4           = 0x003C, // OptionalMfgFunction4
	CellVoltage3           = 0x003D, // OptionalMfgFunction3
	CellVoltage2           = 0x003E, // OptionalMfgFunction2
	CellVoltage1           = 0x003F, // OptionalMfgFunction1

	// TI?
	BTPDischargeSet        = 0x004A,
	BTPChargeSet           = 0x004B,

	State_of_Health        = 0x004F,
	SafetyAlert            = 0x0050,
	SafetyStatus           = 0x0051,
	PFAlert                = 0x0052,
	PFStatus               = 0x0053,
	OperationStatus        = 0x0054,
	ChargingStatus         = 0x0055,
	GaugingStatus          = 0x0056,
	ManufacturingStatus    = 0x0057,
	AFE_Register           = 0x0058,
	MaxTurboPwr            = 0x0059,
	SusTurboPwr            = 0x005A,
	TURBO_PACK_R           = 0x005B,
	TURBO_SYS_R            = 0x005C,
	TURBO_EDV              = 0x005D,
	MaxTurboCurr           = 0x005E,
	SusTurboCurr           = 0x005F,
	Lifetime_Data_Block_1  = 0x0060,
	Lifetime_Data_Block_2  = 0x0061,
	Lifetime_Data_Block_3  = 0x0062,
	Lifetime_Data_Block_4  = 0x0063,
	Lifetime_Data_Block_5  = 0x0064,

	ManufacturerInfo       = 0x0070,
	DAStatus1              = 0x0071,
	DAStatus2              = 0x0072,
	GaugeStatus1           = 0x0073,
	GaugeStatus2           = 0x0074,
	GaugeStatus3           = 0x0075,
	CBStatus               = 0x0076,
	State_of_Health        = 0x0077,
	FilteredCapacity       = 0x0078,

};



} // namespace iSentek_IST8310
