/****************************************************************************
 *
 *   Copyright (c) 2014, 2015 PX4 Development Team. All rights reserved.
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
 * @file mavlink_orb_subscription.cpp
 * uORB subscription implementation.
 *
 * @author Anton Babushkin <anton.babushkin@me.com>
 * @author Lorenz Meier <lorenz@px4.io>
 */

#include "mavlink_orb_subscription.h"

MavlinkOrbSubscription::MavlinkOrbSubscription(const orb_id_t topic, int instance) :
	_sub(topic, instance),
	_subscribe_from_beginning(false),
	_last_pub_check(0)
{
}

bool
MavlinkOrbSubscription::update(void *data)
{
	if (!is_published()) {
		return false;
	}

	return _sub.copy(data);
}

bool
MavlinkOrbSubscription::update_if_changed(void *data)
{
	if (!is_published()) {
		return false;
	}

	return _sub.update(data);
}

bool
MavlinkOrbSubscription::is_published()
{
	// If we marked it as published no need to check again
	if (_sub.published()) {
		return true;
	}

	hrt_abstime now = hrt_absolute_time();

	if (now - _last_pub_check < 300000) {
		return false;
	}

	// We are checking now
	_last_pub_check = now;

	// We don't want to subscribe to anything that does not exist
	// in order to save memory and file descriptors.
	// However, for some topics like vehicle_command_ack, we want to subscribe
	// from the beginning in order not to miss the first publish respective advertise.
	if (!_subscribe_from_beginning && orb_exists(get_topic(), get_instance())) {
		return false;
	}

	if (!_sub.published()) {
		return _sub.init();
	}

	return _sub.published();
}

void
MavlinkOrbSubscription::subscribe_from_beginning(bool from_beginning)
{
	_subscribe_from_beginning = from_beginning;
}
