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
 * @file AFBRS50.hpp
 *
 * Driver for the Broadcom AFBR-S50 connected via SPI.
 *
 */
#pragma once

#include <termios.h>

#include "argus.h"

#include "main.h"

#include <drivers/drv_hrt.h>
#include <lib/drivers/rangefinder/PX4Rangefinder.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

using namespace time_literals;

#define AFBRS50_FIELD_OF_VIEW        (0.105f) // 6 deg cone angle.
#define AFBRS50_MAX_DISTANCE         30.0f
#define AFBRS50_MIN_DISTANCE         0.01f
#define AFBRS50_MEASURE_INTERVAL     100_ms // 10Hz

/*! Define the SPI slave (to be used in the SPI module). */
#define SPI_SLAVE 2
/*! Define the SPI baud rate (to be used in the SPI module). */
#define SPI_BAUD_RATE 5000000


class AFBRS50 : public px4::ScheduledWorkItem
{
public:
	AFBRS50(const uint8_t device_orientation = distance_sensor_s::ROTATION_DOWNWARD_FACING);
	~AFBRS50() override;

	int init();

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void print_info();

	/**
	 * Initialise the automatic measurement state machine and start it.
	 */
	void start();

	/**
	 * Stop the automatic measurement state machine.
	 */
	void stop();

private:

	/**
	 * Reads the data measrurement from serial UART.
	 */
	int collect();

	/**
	 * Sends a data request message to the sensor.
	 */
	int measure();

	void Run() override;

	static void hardware_init(void);
	static status_t measurement_ready_callback(status_t status, void * data);

	PX4Rangefinder _px4_rangefinder;

	hrt_abstime _measurement_time{0};

	argus_hnd_t * _hnd;

	perf_counter_t _comms_error{perf_alloc(PC_COUNT, MODULE_NAME": comms_error")};
	perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": sample")};
};
