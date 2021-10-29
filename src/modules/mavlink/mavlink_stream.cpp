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

MavlinkStream::MavlinkStream(Mavlink *mavlink) :
	_mavlink(mavlink)
{
	_last_sent = hrt_absolute_time();
}

/**
 * Update subscriptions and send message if necessary
 */
int
MavlinkStream::update(const hrt_abstime &t)
{
	update_data();

	bool sent = false;

	if (_interval == 0) {
		// We don't need to send anything if the inverval is 0. send() will be called manually.
		return 0;

	} else if (_last_sent == 0) {
		// If the message has never been sent before we want
		// to send it immediately and can return right away
		if (send()) {
			_last_sent = hrt_absolute_time();
			sent = true;
		}

	} else if (_interval < 0) {
		// unlimited rate
		if (send()) {
			_last_sent = t;
			sent = true;
		}

	} else if (const_rate()) {
		if (t >= _last_sent + _interval) {
			if (send()) {
				_last_sent = math::constrain(_last_sent + _interval, t - _interval, t);
				sent = true;
			}
		}

	} else {
		int interval = _interval * _mavlink->get_rate_div();

		if (t >= _last_sent + interval) {
			if (send()) {
				_last_sent = math::constrain(_last_sent + interval, t - interval, t);
				sent = true;
			}
		}
	}

	if (sent) {
		if (!_first_message_sent) {
			_first_message_sent = true;
		}

		return 0;
	}

	return -1;
}
