/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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
 * @file nra15.cpp
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Greg Hulands
 * @author Ayush Gaud <ayush.gaud@gmail.com>
 * @author Christoph Tobler <christoph@px4.io>
 * @author Mohammed Kabir <mhkabir@mit.edu>
 *
 * Driver for the Benewake TFmini laser rangefinder series
 */

#pragma once

#include <termios.h>

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <lib/drivers/rangefinder/PX4Rangefinder.hpp>
#include <uORB/topics/distance_sensor.h>

#include "nra15_parser.h"

#define NRA15_DEFAULT_PORT	"/dev/ttyS3"

using namespace time_literals;

class NRA15 : public px4::ScheduledWorkItem
{
public:
	NRA15(const char *port, uint8_t rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING);
	virtual ~NRA15();

	int init();

	void print_info();

private:

	void Run() override;

	void start();
	void stop();

	bool ParseBuffer(const uint8_t *buffer, const size_t len);

	PX4Rangefinder	_px4_rangefinder;


	uORB::PublicationMultiData<distance_sensor_s> _distance_sensor_pub{ORB_ID(distance_sensor)};

	uORB::PublicationMulti<distance_sensor_s> _distance_sensor_pubs[8] {
		{ORB_ID(distance_sensor)},
		{ORB_ID(distance_sensor)},
		{ORB_ID(distance_sensor)},
		{ORB_ID(distance_sensor)},
		{ORB_ID(distance_sensor)},
		{ORB_ID(distance_sensor)},
		{ORB_ID(distance_sensor)},
		{ORB_ID(distance_sensor)}
	};

	NRA15_PARSE_STATE _parse_state {NRA15_PARSE_STATE::STATE0_UNSYNC};

	uint8_t _linebuf[1024] {};
	unsigned int _linebuf_index{0};

	char _port[20] {};

	static constexpr int kCONVERSIONINTERVAL{20_ms};

	unsigned _bytes_read{0};
	unsigned _bytes_written{0};

	int _fd{-1};

	px4::atomic_bool _print_debug{false};

	hrt_abstime _last_read{0};

	perf_counter_t _comms_errors{perf_alloc(PC_COUNT, MODULE_NAME": com_err")};
	perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": read")};
	perf_counter_t _poll_timeout_perf{perf_alloc(PC_COUNT, MODULE_NAME": poll timeout")};
	perf_counter_t _poll_error_perf{perf_alloc(PC_COUNT, MODULE_NAME": poll error")};
	perf_counter_t _read_error_perf{perf_alloc(PC_COUNT, MODULE_NAME": read error")};
	perf_counter_t _write_error_perf{perf_alloc(PC_COUNT, MODULE_NAME": write error")};

	perf_counter_t _checksum_good_perf{perf_alloc(PC_COUNT, MODULE_NAME": checksum good")};
	perf_counter_t _checksum_bad_perf{perf_alloc(PC_COUNT, MODULE_NAME": checksum bad")};

};
