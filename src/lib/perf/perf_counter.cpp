/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
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
 * @file perf_counter.cpp
 *
 * @brief Performance measuring tools.
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/queue.h>
#include <drivers/drv_hrt.h>
#include <math.h>
#include <pthread.h>
#include <systemlib/err.h>

#include "perf_counter.h"

#include "PerfCounter.hpp"

#include <containers/BlockingList.hpp>

/* latency histogram */
const uint16_t latency_bucket_count = LATENCY_BUCKET_COUNT;
const uint16_t	latency_buckets[LATENCY_BUCKET_COUNT] = { 1, 2, 5, 10, 20, 50, 100, 1000 };
__EXPORT uint32_t	latency_counters[LATENCY_BUCKET_COUNT + 1];


#ifdef __PX4_QURT
// There is presumably no dprintf on QURT. Therefore use the usual output to mini-dm.
#define dprintf(_fd, _text, ...) ((_fd) == 1 ? PX4_INFO((_text), ##__VA_ARGS__) : (void)(_fd))
#endif

/**
 * List of all known counters.
 */
BlockingList<PerfCounter *> *_perf_counters_list{nullptr};

// FIXME: the BlockingList does **not** protect against access to/from the perf
// counter's data. It can still happen that a counter is updated while it is
// printed. This can lead to inconsistent output, or completely bogus values
// (especially the 64bit values which are in general not atomically updated).

void
perf_init(void)
{
	_perf_counters_list = new BlockingList<PerfCounter *>();
}

perf_counter_t
perf_alloc(enum perf_counter_type type, const char *name)
{
	perf_counter_t ctr = nullptr;

	switch (type) {
	case PC_COUNT:
		ctr = new PerfCounter(name);
		break;

	case PC_ELAPSED:
		ctr = new PerfCounterElapsed(name);
		break;

	case PC_INTERVAL:
		ctr = new PerfCounterInterval(name);
		break;

	default:
		break;
	}

	return ctr;
}

void
perf_free(perf_counter_t handle)
{
	delete handle;
}

void
perf_count(perf_counter_t handle)
{
	if (handle == nullptr) {
		return;
	}

	handle->count();
}

void
perf_begin(perf_counter_t handle)
{
	if (handle == nullptr) {
		return;
	}

	switch (handle->type) {
	case PC_ELAPSED:
		((PerfCounterElapsed *)handle)->begin();
		break;

	default:
		break;
	}
}

void
perf_end(perf_counter_t handle)
{
	if (handle == nullptr) {
		return;
	}

	switch (handle->type) {
	case PC_ELAPSED:
		((PerfCounterElapsed *)handle)->end();
		break;

	default:
		break;
	}
}

void
perf_set_elapsed(perf_counter_t handle, int64_t elapsed)
{
	if (handle == nullptr) {
		return;
	}

	switch (handle->type) {
	case PC_ELAPSED:
		((PerfCounterElapsed *)handle)->set_elapsed(elapsed);
		break;

	default:
		break;
	}
}

void
perf_set_count(perf_counter_t handle, uint64_t count)
{
	if (handle == nullptr) {
		return;
	}

	switch (handle->type) {
	case PC_COUNT:
		((PerfCounter *)handle)->set_count(count);
		break;

	default:
		break;
	}

}

void
perf_cancel(perf_counter_t handle)
{
	if (handle == nullptr) {
		return;
	}

	switch (handle->type) {
	case PC_ELAPSED:
		((PerfCounterElapsed *)handle)->cancel();
		break;

	default:
		break;
	}
}

void
perf_reset(perf_counter_t handle)
{
	if (handle == nullptr) {
		return;
	}

	handle->reset();
}

void
perf_print_counter(perf_counter_t handle)
{
	if (handle == nullptr) {
		return;
	}

	perf_print_counter_fd(1, handle);
}

void
perf_print_counter_fd(int fd, perf_counter_t handle)
{
	if (handle == nullptr) {
		return;
	}

	handle->print();
}

int
perf_print_counter_buffer(char *buffer, int length, perf_counter_t handle)
{
	if (handle == nullptr) {
		return 0;
	}

	return handle->print_buffer(buffer, length);
}

uint64_t
perf_event_count(perf_counter_t handle)
{
	if (handle == nullptr) {
		return 0;
	}

	return handle->event_count();
}

void
perf_iterate_all(perf_callback cb, void *user)
{
	if (_perf_counters_list == nullptr) {
		return;
	}

	for (auto pc : *_perf_counters_list) {
		cb(pc, user);
	}
}

void
perf_print_all(int fd)
{
	if (_perf_counters_list == nullptr) {
		return;
	}

	for (auto pc : *_perf_counters_list) {
		pc->print_fd(fd);
	}
}

void
perf_print_latency(int fd)
{
	dprintf(fd, "bucket [us] : events\n");

	for (int i = 0; i < latency_bucket_count; i++) {
		dprintf(fd, "       %4i : %li\n", latency_buckets[i], (long int)latency_counters[i]);
	}

	// print the overflow bucket value
	dprintf(fd, " >%4i : %i\n", latency_buckets[latency_bucket_count - 1], latency_counters[latency_bucket_count]);
}

void
perf_reset_all(void)
{
	if (_perf_counters_list == nullptr) {
		return;
	}

	for (auto pc : *_perf_counters_list) {
		pc->reset();
	}

	for (int i = 0; i <= latency_bucket_count; i++) {
		latency_counters[i] = 0;
	}
}
