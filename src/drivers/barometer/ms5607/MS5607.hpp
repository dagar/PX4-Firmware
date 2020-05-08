/****************************************************************************
 *
 *   Copyright (c) 2012-2020 PX4 Development Team. All rights reserved.
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
 * @file MS5607.hpp
 * Driver for the MS5607 barometric pressure sensor connected via I2C or SPI.
 */

#pragma once

#include <drivers/drv_hrt.h>
#include <lib/drivers/device/Device.hpp>
#include <lib/drivers/barometer/PX4Barometer.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/i2c_spi_buses.h>

#include "TE_MS5607_registers.hpp"

using namespace TE_MS5607;

class MS5607 : public I2CSPIDriver<MS5607>
{
public:
	MS5607(device::Device *interface, I2CSPIBusOption bus_option, int bus);
	~MS5607() override;

	static I2CSPIDriverBase *instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
					     int runtime_instance);
	static void print_usage();

	int init();
	void print_status() override;
	void RunImpl();

private:
	uint32_t ReadADC();
	bool ReadProm();

	// MS5607 crc4 cribbed from the datasheet
	bool crc4(uint16_t *n_prom);

	device::Device *_interface;
	PX4Barometer _px4_barometer;

	enum class STATE : uint8_t {
		RESET,
		READ_PROM,
		CONVERT_TEMPERATURE,
		READ_TEMPERATURE,
		CONVERT_PRESSURE,
		READ_PRESSURE,
	} _state{STATE::RESET};

	hrt_abstime _timestamp_sample{0};
	hrt_abstime _last_temperature_update{0};

	TE_MS5607::PROM _prom{};

	// intermediate temperature values per MS5607 datasheet
	int64_t _OFF{0};
	int64_t _SENS{0};

	perf_counter_t _bad_transfer_perf{perf_alloc(PC_COUNT, MODULE_NAME": bad transfer")};
	perf_counter_t _read_pressure_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": read pressure interval")};
	perf_counter_t _read_temperature_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": read temperature interval")};
	perf_counter_t _bad_pressure_perf{perf_alloc(PC_COUNT, MODULE_NAME": bad pressure")};
	perf_counter_t _bad_temperature_perf{perf_alloc(PC_COUNT, MODULE_NAME": bad temperature")};
};
