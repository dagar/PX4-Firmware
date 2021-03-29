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
 * @file ICM20948_I2C.hpp
 *
 * Driver for the Invensense ICM20948 connected via SPI.
 *
 */

#pragma once

#include "InvenSense_ICM20948_registers.hpp"

#include <drivers/drv_hrt.h>
#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/drivers/device/i2c.h>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>
#include <lib/ecl/geo/geo.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/atomic.h>
#include <px4_platform_common/i2c_spi_buses.h>

using namespace InvenSense_ICM20948;

class ICM20948_I2C : public device::I2C, public I2CSPIDriver<ICM20948_I2C>
{
public:
	ICM20948_I2C(I2CSPIBusOption bus_option, int bus, uint32_t device, enum Rotation rotation, int bus_frequency,
		     int address);
	~ICM20948_I2C() override;

	static I2CSPIDriverBase *instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
					     int runtime_instance);
	static void print_usage();

	void RunImpl();

	int init() override;
	void print_status() override;

private:
	// Sensor Configuration
	static constexpr float FIFO_SAMPLE_DT{1e6f / 1125.f};
	static constexpr float GYRO_RATE{1e6f / FIFO_SAMPLE_DT};  // 1125 Hz gyro
	static constexpr float ACCEL_RATE{1e6f / FIFO_SAMPLE_DT}; // 1125 Hz accel

	// maximum FIFO samples per transfer is limited to the size of sensor_accel_fifo/sensor_gyro_fifo
	static constexpr uint32_t FIFO_MAX_SAMPLES{math::min(math::min(FIFO::SIZE / sizeof(FIFO::DATA), sizeof(sensor_gyro_fifo_s::x) / sizeof(sensor_gyro_fifo_s::x[0])), sizeof(sensor_accel_fifo_s::x) / sizeof(sensor_accel_fifo_s::x[0]) * (int)(GYRO_RATE / ACCEL_RATE))};

	// Transfer data
	struct FIFOTransferBuffer {
		uint8_t FIFO_COUNTH{0};
		uint8_t FIFO_COUNTL{0};
		FIFO::DATA f[FIFO_MAX_SAMPLES] {};
	};
	// ensure no struct padding
	static_assert(sizeof(FIFOTransferBuffer) == (2 + FIFO_MAX_SAMPLES *sizeof(FIFO::DATA)));

	struct register_bank0_config_t {
		Register::BANK_0 reg;
		uint8_t set_bits{0};
		uint8_t clear_bits{0};
	};

	struct register_bank2_config_t {
		Register::BANK_2 reg;
		uint8_t set_bits{0};
		uint8_t clear_bits{0};
	};

	int probe() override;

	bool Reset();

	bool Configure();
	void ConfigureAccel();
	void ConfigureGyro();
	void ConfigureSampleRate(int sample_rate);

	void SelectRegisterBank(enum REG_BANK_SEL_BIT bank);
	void SelectRegisterBank(Register::BANK_0 reg) { SelectRegisterBank(REG_BANK_SEL_BIT::USER_BANK_0); }
	void SelectRegisterBank(Register::BANK_2 reg) { SelectRegisterBank(REG_BANK_SEL_BIT::USER_BANK_2); }

	template <typename T> bool RegisterCheck(const T &reg_cfg);

	template <typename T> uint8_t RegisterRead(T reg);
	template <typename T> void RegisterWrite(T reg, uint8_t value);
	template <typename T> void RegisterSetAndClearBits(T reg, uint8_t setbits, uint8_t clearbits);

	uint16_t FIFOReadCount();
	bool FIFORead(const hrt_abstime &timestamp_sample, uint8_t samples);
	void FIFOReset();

	void ProcessAccel(const hrt_abstime &timestamp_sample, const FIFO::DATA fifo[], const uint8_t samples);
	void ProcessGyro(const hrt_abstime &timestamp_sample, const FIFO::DATA fifo[], const uint8_t samples);
	void UpdateTemperature();

	PX4Accelerometer _px4_accel;
	PX4Gyroscope _px4_gyro;

	perf_counter_t _bad_register_perf{perf_alloc(PC_COUNT, MODULE_NAME": bad register")};
	perf_counter_t _bad_transfer_perf{perf_alloc(PC_COUNT, MODULE_NAME": bad transfer")};
	perf_counter_t _fifo_empty_perf{perf_alloc(PC_COUNT, MODULE_NAME": FIFO empty")};
	perf_counter_t _fifo_overflow_perf{perf_alloc(PC_COUNT, MODULE_NAME": FIFO overflow")};
	perf_counter_t _fifo_reset_perf{perf_alloc(PC_COUNT, MODULE_NAME": FIFO reset")};

	hrt_abstime _reset_timestamp{0};
	hrt_abstime _last_config_check_timestamp{0};
	hrt_abstime _temperature_update_timestamp{0};
	int _failure_count{0};

	enum REG_BANK_SEL_BIT _last_register_bank {REG_BANK_SEL_BIT::USER_BANK_0};

	enum class STATE : uint8_t {
		RESET,
		WAIT_FOR_RESET,
		CONFIGURE,
		FIFO_READ,
	} _state{STATE::RESET};

	uint16_t _fifo_empty_interval_us{1250}; // default 1250 us / 800 Hz transfer interval
	uint32_t _fifo_gyro_samples{static_cast<uint32_t>(_fifo_empty_interval_us / (1000000 / GYRO_RATE))};

	uint8_t _checked_register_bank0{0};
	static constexpr uint8_t size_register_bank0_cfg{4};
	register_bank0_config_t _register_bank0_cfg[size_register_bank0_cfg] {
		// Register                             | Set bits, Clear bits
		{ Register::BANK_0::USER_CTRL,          USER_CTRL_BIT::FIFO_EN, USER_CTRL_BIT::I2C_MST_EN | USER_CTRL_BIT::I2C_IF_DIS | USER_CTRL_BIT::DMP_EN },
		{ Register::BANK_0::PWR_MGMT_1,         PWR_MGMT_1_BIT::CLKSEL_0, PWR_MGMT_1_BIT::DEVICE_RESET | PWR_MGMT_1_BIT::SLEEP },
		{ Register::BANK_0::FIFO_EN_2,          FIFO_EN_2_BIT::ACCEL_FIFO_EN | FIFO_EN_2_BIT::GYRO_Z_FIFO_EN | FIFO_EN_2_BIT::GYRO_Y_FIFO_EN | FIFO_EN_2_BIT::GYRO_X_FIFO_EN, FIFO_EN_2_BIT::TEMP_FIFO_EN },
		{ Register::BANK_0::FIFO_MODE,          FIFO_MODE_BIT::Snapshot, 0 },
	};

	uint8_t _checked_register_bank2{0};
	static constexpr uint8_t size_register_bank2_cfg{2};
	register_bank2_config_t _register_bank2_cfg[size_register_bank2_cfg] {
		// Register                             | Set bits, Clear bits
		//{ Register::BANK_2::GYRO_SMPLRT_DIV,    0, 0 },
		{ Register::BANK_2::GYRO_CONFIG_1,      GYRO_CONFIG_1_BIT::GYRO_DLPFCFG | GYRO_CONFIG_1_BIT::GYRO_FS_SEL_2000_DPS | GYRO_CONFIG_1_BIT::GYRO_FCHOICE, 0 },
		{ Register::BANK_2::ACCEL_CONFIG,       ACCEL_CONFIG_BIT::ACCEL_DLPFCFG | ACCEL_CONFIG_BIT::ACCEL_FS_SEL_16G | ACCEL_CONFIG_BIT::ACCEL_FCHOICE, 0 },
	};
};
