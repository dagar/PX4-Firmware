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
 * @file ICM20602.cpp
 *
 * Driver for the Invensense ICM20602 connected via SPI.
 *
 * When the device is on the SPI bus the hrt is used to provide thread of
 * execution to the driver.
 *
 */

#pragma once

#include "InvenSense_ICM20602_registers.hpp"

#include <drivers/drv_hrt.h>
#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/drivers/device/spi.h>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>
#include <lib/ecl/geo/geo.h>
#include <lib/perf/perf_counter.h>
#include <px4_config.h>
#include <px4_getopt.h>
#include <px4_time.h>
#include <px4_work_queue/ScheduledWorkItem.hpp>
#include <lib/systemlib/conversions.h>
#include <lib/systemlib/px4_macros.h>

#include <uORB/Publication.hpp>
#include <uORB/topics/sensor_accel_fifo.h>
#include <uORB/topics/sensor_gyro_fifo.h>

using InvenSense_ICM20602::Register;

class ICM20602 : public device::SPI, public px4::ScheduledWorkItem
{
public:
	ICM20602(int bus, uint32_t device, enum Rotation rotation = ROTATION_NONE);
	virtual ~ICM20602();

	virtual int		init();

	void			start();
	void			stop();
	int			reset();

	void			print_info();
	void			print_registers();

protected:
	virtual int		probe();

private:

	static int		data_ready_interrupt(int irq, void *context, void *arg);

	void			Run() override;

	int			measure();

	uint8_t			registerRead(Register reg);
	void			registerWrite(Register reg, uint8_t value);
	void			registerSetBits(Register reg, uint8_t setbits);
	void			registerClearBits(Register reg, uint8_t clearbits);

	bool			resetFIFO();
	bool			configureAccel();
	bool			configureGyro();
	bool			selfTest();

	uint8_t			*_dma_data_buffer{nullptr};

	PX4Accelerometer	_px4_accel;
	PX4Gyroscope		_px4_gyro;

	uORB::Publication<sensor_accel_fifo_s>	_sensor_accel_fifo_pub{ORB_ID(sensor_accel_fifo)};
	uORB::Publication<sensor_gyro_fifo_s>	_sensor_gyro_fifo_pub{ORB_ID(sensor_gyro_fifo)};

	unsigned		_fifo_error_count{0};

	perf_counter_t		_interval_perf;
	perf_counter_t		_transfer_perf;
	perf_counter_t		_cycle_perf;
	perf_counter_t		_bad_transfers_perf;
	perf_counter_t		_fifo_empty_perf;
	perf_counter_t		_fifo_overflow_perf;
	perf_counter_t		_fifo_reset_perf;
	perf_counter_t		_reset_perf;
	perf_counter_t		_accel_process_perf;
	perf_counter_t		_gyro_process_perf;

};
