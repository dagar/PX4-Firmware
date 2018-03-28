/****************************************************************************
 *
 *   Copyright (c) 2014-2017 PX4 Development Team. All rights reserved.
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
 * @file mavlink_stream.cpp
 * Mavlink messages stream implementation.
 *
 * @author Anton Babushkin <anton.babushkin@me.com>
 * @author Lorenz Meier <lorenz@px4.io>
 */

#include <stdlib.h>

#include "mavlink_stream.h"
#include "mavlink_main.h"

#include <mathlib/mathlib.h>

MavlinkStream::MavlinkStream(Mavlink *mavlink) : _mavlink(mavlink)
{
	_last_sent = hrt_absolute_time();
}

/**
 * Set messages interval in ms
 */
void
MavlinkStream::set_interval(const uint32_t interval)
{
	if (interval != _interval) {
		_mavlink->set_update_total_stream_rate();
		_interval = interval;
	}
}

/**
 * Update subscriptions and send message if necessary
 */
bool
MavlinkStream::update(const uint64_t &now, const float scale)
{
	// One of the previous iterations sent the update
	// already before the deadline
	if (_last_sent >= now) {
		return false;
	}

	int32_t interval = _interval;

	if (!const_rate()) {
		interval = static_cast<float>(interval) * scale;
	}

#ifndef __PX4_QURT // TODO: QuRT hack still needed?

	const int64_t dt = now - _last_sent;

	if (dt > interval) {
		// interval expired, send message

		// If the interval is non-zero do not use the actual time but
		// increment at a fixed rate, so that processing delays do not
		// distort the average rate
		if (send(now)) {
			// don't let it fall too far behind
			_last_sent = math::constrain(_last_sent + interval, now - interval, now);

			// return the delta required time to hit the interval
			return true;
		}

	} else if (interval == 0) {
		send(now);
		_last_sent = now;
	}

#endif /* __PX4_QURT */

	return false;
}
