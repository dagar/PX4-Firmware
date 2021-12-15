/****************************************************************************
 *
 *   Copyright (c) 2012-2021 PX4 Development Team. All rights reserved.
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

#include <stdint.h>
#include <stdbool.h>

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/micro_hal.h>

#include <lib/perf/perf_counter.h>

#include <lib/drivers/device/device.h>
#include <modules/px4iofirmware/protocol.h>

class PX4IO_serial
{
public:
	PX4IO_serial();
	~PX4IO_serial();

	int init();

	int read_io(unsigned offset, void *data, unsigned count = 1);
	int write_io(unsigned address, void *data, unsigned count = 1);

private:
	int _io_fd{-1};

	/*
	 * XXX tune this value
	 *
	 * At 1.5Mbps each register takes 13.3µs, and we always transfer a full packet.
	 * Packet overhead is 26µs for the four-byte header.
	 *
	 * 32 registers = 451µs
	 *
	 * Maybe we can just send smaller packets (e.g. 8 regs) and loop for larger (less common)
	 * transfers? Could cause issues with any regs expecting to be written atomically...
	 */
	static IOPacket _io_buffer px4_cache_aligned_data();

	/**
	 * Performance counters.
	 */
	perf_counter_t _pc_txns{perf_alloc(PC_ELAPSED, MODULE_NAME": txns")};
	perf_counter_t _pc_retries{perf_alloc(PC_COUNT, MODULE_NAME": retries")};
	perf_counter_t _pc_timeouts{perf_alloc(PC_COUNT, MODULE_NAME": timeouts")};
	perf_counter_t _pc_crcerrs{perf_alloc(PC_COUNT, MODULE_NAME": crcerrs")};
	perf_counter_t _pc_protoerrs{perf_alloc(PC_COUNT, MODULE_NAME": protoerrs")};
	perf_counter_t _pc_uerrs{perf_alloc(PC_COUNT, MODULE_NAME": uarterrs")};
	perf_counter_t _pc_idle{perf_alloc(PC_COUNT, MODULE_NAME": idle")};
	perf_counter_t _pc_badidle{perf_alloc(PC_COUNT, MODULE_NAME": badidle")};
};

PX4IO_serial *PX4IO_serial_interface();
