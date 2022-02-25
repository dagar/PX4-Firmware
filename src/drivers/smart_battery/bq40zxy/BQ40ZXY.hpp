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
 * @file BQ40ZXY.hpp
 *
 * Driver for TI BQ40ZXY connected via SMBus (I2C).
 *
 */

#pragma once

#include <lib/drivers/sbs_battery/SBSBattery.hpp>

#include <mathlib/mathlib.h>
#include <perf/perf_counter.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/param.h>
#include <uORB/Publication.hpp>

#include <board_config.h>

using namespace time_literals;

class BQ40ZXY : public SBSBattery, public I2CSPIDriver<BQ40ZXY>
{
public:
	BQ40ZXY(const I2CSPIDriverConfig &config);
	~BQ40ZXY() override;

	static void print_usage();

	void RunImpl();

	int init() override;
	void print_status() override;




	static constexpr const char *MOD_NAME = "bq40zxy";
	static constexpr uint8_t BATT_ADDR = 0x0B;		///< Default 7 bit address I2C address. 8 bit = 0x16

	static constexpr size_t MAC_DATA_BUFFER_SIZE = 32;

	static const uint8_t MAX_NUM_CELLS = 7;

	enum BQ40ZXY_FLASH {
		BQ40ZXY_FLASH_CELL_CONFIGURATION = 0x4D05,
	};

	enum BQ40ZXY_REG {
		BQ40ZXY_REG_STATE_OF_HEALTH =                    	0x4F,
		BQ40ZXY_REG_MANUFACTURER_BLOCK_ACCESS =			0x44,
	};

	enum BQ40ZXY_MAN_DATA {
		BQ40ZXY_MAN_DEVICE_TYPE =                         	0x0001,
		BQ40ZXY_MAN_SECURITY_KEYS =                         	0x0035,
		BQ40ZXY_MAN_LIFETIME_FLUSH =                        	0x002E,
		BQ40ZXY_MAN_LIFETIME_BLOCK_ONE =                    	0x0060,
		BQ40ZXY_MAN_ENABLED_PROTECTIONS_A_ADDRESS =         	0x4938,
		BQ40ZXY_MAN_SEAL =                                  	0x0030,
		BQ40ZXY_MAN_DASTATUS1 =                             	0x0071,
		BQ40ZXY_MAN_DASTATUS2 =                             	0x0072,
		BQ40ZXY_MAN_DASTATUS3 =                             	0x007B,
		BQ40ZXY_MAN_MFC_ENABLE =				0x270C,
		BQ40ZXY_MAN_MFC_SHUTDOWN =       			0x043D,
	};

	static constexpr uint8_t BQ40ZXY_ENABLED_PROTECTIONS_A_DEFAULT =	0xCF;
	static constexpr uint8_t BQ40ZXY_ENABLED_PROTECTIONS_A_CUV_DISABLED =	0xCE;

	int populate_startup_data() override;

	int populate_runtime_data(battery_status_s &msg) override;

private:

	int probe() override;


	// TODO: cleanup

	/**
	 * @brief Reads data from flash.
	 * @param address The address to start the read from.
	 * @param data The returned data.
	 * @return Returns PX4_OK on success, PX4_ERROR on failure.
	 */
	int dataflash_read(const uint16_t address, void *data, const unsigned length);

	/**
	 * @brief Writes data to flash.
	 * @param address The start address of the write.
	 * @param data The data to be written.
	 * @param length The number of bytes being written.
	 * @return Returns PX4_OK on success, PX4_ERROR on failure.
	 */
	int dataflash_write(const uint16_t address, void *data, const unsigned length);

	/**
	 * @brief Performs a ManufacturerBlockAccess() read command.
	 * @param cmd_code The command code.
	 * @param data The returned data.
	 * @param length The number of bytes being written.
	 * @return Returns PX4_OK on success, PX4_ERROR on failure.
	 */
	int manufacturer_read(const uint16_t cmd_code, void *data, const unsigned length);

	/**
	 * @brief Performs a ManufacturerBlockAccess() write command.
	 * @param cmd_code The command code.
	 * @param data The sent data.
	 * @param length The number of bytes being written.
	 * @return Returns PX4_OK on success, PX4_ERROR on failure.
	 */
	int manufacturer_write(const uint16_t cmd_code, void *data, const unsigned length);

	/**
	 * @brief This command flushes lifetime data from RAM to data flash, where it can be read out from
	 * @return Returns PX4_OK on success, PX4_ERROR on failure.
	 */
	int lifetime_flush();

	/**
	 * @brief Reads the lifetime data from block 1.
	 * @return Returns PX4_OK on success, PX4_ERROR on failure.
	 */
	int lifetime_read();

	/**
	 * @brief Reads the cell voltages.
	 * @return Returns PX4_OK on success or associated read error code on failure.
	 */
	int populate_cell_voltages(battery_status_s &data);

	/**
	 * @brief Enables or disables the cell under voltage protection emergency shut off.
	 */
	int set_protection_cuv(bool enable);

	int set_fet_state(bool enable);

	/** @param _state_of_health state of health as read on connection  */
	float _state_of_health{0.f};

	/** @param _cell_undervoltage_protection_status false if protection disabled, true if enabled */
	bool _cell_undervoltage_protection_status{true};

	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};

};
