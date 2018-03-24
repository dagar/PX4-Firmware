/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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


#include "listener_print.hpp"

#include <drivers/drv_hrt.h>
#include <px4_posix.h>

bool listener_print(uORB::SubscriptionBase &subscription, unsigned num_msgs)
{
	// print first message immediately
	subscription.print();

	px4_pollfd_struct_t fds[1] = {};
	fds[0].fd = subscription.getHandle();
	fds[0].events = POLLIN;

	hrt_abstime last_timestamp = hrt_absolute_time();

	unsigned i = 1;

	while (i < num_msgs) {
		int ret = px4_poll(fds, sizeof(fds) / sizeof(fds[0]), 100);

		if (!(fds[0].revents & POLLIN)) {

			if (hrt_elapsed_time(&last_timestamp) > 2 * 1000 * 1000) {
				PX4_WARN("Waited for 2 seconds without a message. Giving up");
				return false;
			}

			// no new data
			continue;
		}

		if (ret < 0) {
			// Poll error, sleep and try again
			usleep(10000);
			continue;

		} else if (ret == 0) {
			// Poll timeout or no new data, do nothing
			continue;

		} else {
			subscription.update();
			subscription.print();
			last_timestamp = hrt_absolute_time();

			i++;
		}
	}

	return (i == num_msgs);
}
