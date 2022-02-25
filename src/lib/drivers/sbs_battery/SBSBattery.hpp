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
 * @file SBSBattery.hpp
 *
 * Header for a battery monitor connected via SMBus (I2C).
 * Designed for SMBUS SBS v1.1-compatible Smart Batteries
 *
 */

#pragma once

#include <drivers/drv_hrt.h>
#include <mathlib/mathlib.h>
#include <perf/perf_counter.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/param.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/battery_status.h>

#include <board_config.h>

#include <drivers/device/i2c.h>
#include <perf/perf_counter.h>
#include <string.h>

#define SMBUS_PEC_POLYNOMIAL	0x07	///< Polynomial for calculating PEC

static constexpr uint8_t MAX_BLOCK_LEN = 34;

using namespace time_literals;

class SBSBattery : public ModuleParams, public device::I2C
{
public:

	SBSBattery(const I2CSPIDriverConfig &config) :
		ModuleParams(nullptr),
		I2C(config)
	{
	}

	~SBSBattery()
	{
	}

	virtual int populate_startup_data();

	virtual int populate_runtime_data(battery_status_s &msg);

protected:
	static constexpr hrt_abstime MEASUREMENT_INTERVAL_US =	100_ms;	///< time in microseconds, measure at 10Hz

	static constexpr uint8_t BATT_ADDR = 			0x0B;	///< Default 7 bit address I2C address. 8 bit = 0x16

	enum SBS_Register {
		SBS_REG_TEMP =                                  0x08,   ///< temperature register
		SBS_REG_VOLTAGE =                               0x09,   ///< voltage register
		SBS_REG_CURRENT =                               0x0A,   ///< current register
		SBS_REG_AVERAGE_CURRENT =                       0x0B,   ///< average current register
		SBS_REG_MAX_ERROR =                             0x0C,   ///< max error
		SBS_REG_RELATIVE_SOC =                          0x0D,   ///< Relative State Of Charge
		SBS_REG_ABSOLUTE_SOC =                          0x0E,   ///< Absolute State of charge
		SBS_REG_REMAINING_CAPACITY =                    0x0F,   ///< predicted remaining battery capacity as a percentage
		SBS_REG_FULL_CHARGE_CAPACITY =                  0x10,   ///< capacity when fully charged
		SBS_REG_RUN_TIME_TO_EMPTY =                     0x11,   ///< predicted remaining battery capacity based on the present rate of discharge in min
		SBS_REG_AVERAGE_TIME_TO_EMPTY =                 0x12,   ///< predicted remaining battery capacity based on the present rate of discharge in min
		SBS_REG_CYCLE_COUNT =                           0x17,   ///< number of cycles the battery has experienced
		SBS_REG_DESIGN_CAPACITY =                       0x18,   ///< design capacity register
		SBS_REG_DESIGN_VOLTAGE =                        0x19,   ///< design voltage register
		SBS_REG_MANUFACTURER_NAME =                     0x20,   ///< manufacturer name
		SBS_REG_MANUFACTURE_DATE =                      0x1B,   ///< manufacture date register
		SBS_REG_SERIAL_NUMBER =                         0x1C,   ///< serial number register
		SBS_REG_MANUFACTURER_ACCESS =                   0x00,
		SBS_REG_MANUFACTURER_DATA =                     0x23,
	};

	static constexpr size_t MANUFACTURER_NAME_SIZE =        21;     ///< manufacturer name data size


	/**
	 * @brief Sends a block write command.
	 * @param cmd_code The command code.
	 * @param data The data to be written.
	 * @param length The number of bytes being written. Maximum is SMBus::MAX_BLOCK_LEN.
	 * @return Returns PX4_OK on success, -errno on failure.
	 */
	int block_write(const uint8_t cmd_code, const void *data, uint8_t byte_count, const bool use_pec);

	/**
	 * @brief Sends a block read command.
	 * @param cmd_code The command code.
	 * @param data The returned data.
	 * @param length The number of bytes being read. Maximum is SMBus::MAX_BLOCK_LEN.
	 * @return Returns PX4_OK on success, -errno on failure.
	 */
	int block_read(const uint8_t cmd_code, void *data, const uint8_t length, const bool use_pec);

	/**
	 * @brief Sends a read word command.
	 * @param cmd_code The command code.
	 * @param data The 2 bytes of returned data plus a 1 byte CRC if used.
	 * @return Returns PX4_OK on success, -errno on failure.
	 */
	int read_word(const uint8_t cmd_code, uint16_t &data);

	/**
	 * @brief Sends a write word command.
	 * @param cmd_code The command code.
	 * @param data The 2 bytes of data to be transfered.
	 * @return Returns PX4_OK on success, -errno on failure.
	 */
	int write_word(const uint8_t cmd_code, uint16_t data);

	/**
	 * @brief Calculates the PEC from the data.
	 * @param buffer The buffer that stores the data to perform the CRC over.
	 * @param length The number of bytes being written.
	 * @return Returns PX4_OK on success, -errno on failure.
	 */
	uint8_t get_pec(uint8_t *buffer, uint8_t length);

	perf_counter_t _interface_errors{perf_alloc(PC_COUNT, MODULE_NAME": errors")};





	uORB::PublicationMulti<battery_status_s> _battery_status_pub{ORB_ID(battery_status)};

	uint8_t _cell_count{0};
	float _design_voltage{0};
	uint16_t _design_capacity{0};
	uint16_t _actual_capacity{0};
	uint16_t _cycle_count{0};
	uint16_t _serial_number{0};
	char _manufacturer_name[MANUFACTURER_NAME_SIZE + 1] {};	// Plus one for terminator
	uint16_t _manufacture_date{0};

	SBSBattery(const SBSBattery &) = delete;
	SBSBattery operator=(const SBSBattery &) = delete;

	DEFINE_PARAMETERS_CUSTOM_PARENT(
		ModuleParams,
		(ParamFloat<px4::params::BAT_CRIT_THR>)		_param_bat_crit_thr,
		(ParamFloat<px4::params::BAT_LOW_THR>)  	_param_bat_low_thr,
		(ParamFloat<px4::params::BAT_EMERGEN_THR>)	_param_bat_emergen_thr,
		(ParamInt<px4::params::SBS_BAT_N_CELLS>)	_param_sbs_bat_n_cells,
		(ParamFloat<px4::params::SBS_BAT_C_MULT>)	_param_sbs_bat_c_mult
	)
};
