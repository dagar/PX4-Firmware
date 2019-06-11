/****************************************************************************
 *
 *   Copyright (C) 2019 PX4 Development Team. All rights reserved.
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

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "perf_counter.h"

#include <containers/BlockingList.hpp>
#include <drivers/drv_hrt.h>

extern BlockingList<PerfCounter *> *_perf_counters_list;

class PerfCounter : public ListNode<PerfCounter *>
{
public:
	explicit PerfCounter(const char *name) : _name(name)
	{
		type = PC_COUNT;

		if (name) {
			if (_perf_counters_list) {

				for (auto pc : *_perf_counters_list) {
					if (strcmp(pc->name(), name) == 0) {
						_instance++;
					}
				}

				_perf_counters_list->add(this);
			}
		}
	}
	PerfCounter() = delete;

	virtual ~PerfCounter()
	{
		if (_perf_counters_list) {
			_perf_counters_list->remove(this);
		}
	};

	virtual void count() { _event_count++; }
	void set_count(uint64_t count) { _event_count = count; }
	uint64_t event_count() const { return _event_count; }

	virtual void reset() { _event_count = 0; }

	virtual void print_fd(int fd)
	{
		dprintf(fd, "%s (%d): %llu events\n", _name, _instance, (unsigned long long)_event_count);
	}

	virtual int print_buffer(char *buffer, int length)
	{
		int num_written = snprintf(buffer, length, "%s (%d): %llu events", _name, _instance, (unsigned long long)_event_count);
		buffer[length - 1] = 0; // ensure 0-termination
		return num_written;
	}

	void print() { print_fd(1); }

	const char *name() const { return _name; }

	enum perf_counter_type	type;	/**< counter type */

protected:
	uint64_t	_event_count{0};

	const char	*_name{nullptr};	/**< counter name */

	uint8_t		_instance{0};
};

class PerfCounterElapsed final : public PerfCounter
{
public:
	explicit PerfCounterElapsed(const char *name) : PerfCounter(name) { type = PC_ELAPSED; }
	PerfCounterElapsed() = delete;

	virtual ~PerfCounterElapsed() override = default;

	void begin() { _time_start = hrt_absolute_time(); }

	void end()
	{
		if (_time_start != 0) {
			const int64_t elapsed = hrt_elapsed_time(&_time_start);

			if (elapsed >= 0) {

				_event_count++;
				_time_total += elapsed;

				if ((_time_least > (uint32_t)elapsed) || (_time_least == 0)) {
					_time_least = elapsed;
				}

				if (_time_most < (uint32_t)elapsed) {
					_time_most = elapsed;
				}

				// maintain mean and variance of the elapsed time in seconds
				// Knuth/Welford recursive mean and variance of update intervals (via Wikipedia)
				float dt = elapsed / 1e6f;
				float delta_intvl = dt - _mean;
				_mean += delta_intvl / _event_count;
				_M2 += delta_intvl * (dt - _mean);

				_time_start = 0;
			}
		}
	}

	void cancel() { _time_start = 0; }

	void reset() override
	{
		_event_count = 0;
		_time_start = 0;
		_time_total = 0;
		_time_least = 0;
		_time_most = 0;
		_mean = 0.0f;
		_M2 = 0.0f;
	}

	void print_fd(int fd) override
	{
		float rms = sqrtf(_M2 / (_event_count - 1));
		dprintf(fd, "%s (%d): %llu events, %lluus elapsed, %.2fus avg, min %lluus max %lluus %5.3fus rms\n",
			_name, _instance,
			(unsigned long long)_event_count,
			(unsigned long long)_time_total,
			(_event_count == 0) ? 0 : (double)_time_total / (double)_event_count,
			(unsigned long long)_time_least,
			(unsigned long long)_time_most,
			(double)(1e6f * rms));
	}

	int print_buffer(char *buffer, int length) override
	{
		float rms = sqrtf(_M2 / (_event_count - 1));
		int num_written = snprintf(buffer, length,
					   "%s (%d): %llu events, %lluus elapsed, %.2fus avg, min %lluus max %lluus %5.3fus rms",
					   _name, _instance,
					   (unsigned long long)_event_count,
					   (unsigned long long)_time_total,
					   (_event_count == 0) ? 0 : (double)_time_total / (double)_event_count,
					   (unsigned long long)_time_least,
					   (unsigned long long)_time_most,
					   (double)(1e6f * rms));

		buffer[length - 1] = 0; // ensure 0-termination
		return num_written;
	}

	void set_elapsed(int64_t elapsed)
	{
		if (elapsed >= 0) {

			_event_count++;
			_time_total += elapsed;

			if ((_time_least > (uint32_t)elapsed) || (_time_least == 0)) {
				_time_least = elapsed;
			}

			if (_time_most < (uint32_t)elapsed) {
				_time_most = elapsed;
			}

			// maintain mean and variance of the elapsed time in seconds
			// Knuth/Welford recursive mean and variance of update intervals (via Wikipedia)
			float dt = elapsed / 1e6f;
			float delta_intvl = dt - _mean;
			_mean += delta_intvl / _event_count;
			_M2 += delta_intvl * (dt - _mean);

			_time_start = 0;
		}
	}

protected:

	uint64_t	_time_start{0};
	uint64_t	_time_total{0};

	uint32_t	_time_least{0};
	uint32_t	_time_most{0};

	float		_mean{0.0f};
	float		_M2{0.0f};

};

class PerfCounterInterval final : public PerfCounter
{
public:
	explicit PerfCounterInterval(const char *name) : PerfCounter(name) { type = PC_ELAPSED; }
	PerfCounterInterval() = delete;

	virtual ~PerfCounterInterval() override = default;

	void count() override
	{
		const hrt_abstime now = hrt_absolute_time();

		switch (_event_count) {
		case 0:
			_time_first = now;
			break;

		case 1:
			_time_least = (uint32_t)(now - _time_last);
			_time_most = (uint32_t)(now - _time_last);
			_mean = _time_least / 1e6f;
			_M2 = 0;
			break;

		default: {
				hrt_abstime interval = now - _time_last;

				if ((uint32_t)interval < _time_least) {
					_time_least = (uint32_t)interval;
				}

				if ((uint32_t)interval > _time_most) {
					_time_most = (uint32_t)interval;
				}

				// maintain mean and variance of interval in seconds
				// Knuth/Welford recursive mean and variance of update intervals (via Wikipedia)
				float dt = interval / 1e6f;
				float delta_intvl = dt - _mean;
				_mean += delta_intvl / _event_count;
				_M2 += delta_intvl * (dt - _mean);
				break;
			}
		}

		_time_last = now;
		_event_count++;
	}

	void reset() override
	{
		_event_count = 0;
		_time_event = 0;
		_time_first = 0;
		_time_last = 0;
		_time_least = 0;
		_time_most = 0;
		_mean = 0.0f;
		_M2 = 0.0f;
	}

	void print_fd(int fd) override
	{
		float rms = sqrtf(_M2 / (_event_count - 1));

		dprintf(fd, "%s (%d): %llu events, %.2fus avg, min %lluus max %lluus %5.3fus rms\n",
			_name, _instance,
			(unsigned long long)_event_count,
			(_event_count == 0) ? 0 : (double)(_time_last - _time_first) / (double)_event_count,
			(unsigned long long)_time_least,
			(unsigned long long)_time_most,
			(double)(1e6f * rms));
	}

	int print_buffer(char *buffer, int length) override
	{
		float rms = sqrtf(_M2 / (_event_count - 1));

		int num_written = snprintf(buffer, length, "%s (%d): %llu events, %.2f avg, min %lluus max %lluus %5.3fus rms",
					   _name, _instance,
					   (unsigned long long)_event_count,
					   (_event_count == 0) ? 0 : (double)(_time_last - _time_first) / (double)_event_count,
					   (unsigned long long)_time_least,
					   (unsigned long long)_time_most,
					   (double)(1e6f * rms));
		buffer[length - 1] = 0; // ensure 0-termination
		return num_written;
	}

protected:

	uint64_t	_time_event{0};
	uint64_t	_time_first{0};
	uint64_t	_time_last{0};

	uint32_t	_time_least{0};
	uint32_t	_time_most{0};

	float		_mean{0.0f};
	float		_M2{0.0f};

};
