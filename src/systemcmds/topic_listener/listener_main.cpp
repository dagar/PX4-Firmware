/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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
 * @file listener_main.cpp
 */

#include <px4_getopt.h>
#include <px4_module.h>
#include <px4_log.h>

#include "listener_generated.hpp"

static void	usage();

extern "C" {
	__EXPORT int listener_main(int argc, char *argv[]);
}

int
listener_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage();
		return -1;
	}

	char *name = argv[1];

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	unsigned number_msgs = 1;
	unsigned rate = 0;
	unsigned instance = 0;

	while ((ch = px4_getopt(argc, argv, "n:r:i:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {

		case 'n':
			// number of messages
			number_msgs = strtol(myoptarg, nullptr, 0);
			PX4_INFO("printing %d", number_msgs);
			break;

		case 'r':
			// subscription rate
			rate = strtol(myoptarg, nullptr, 0);
			PX4_INFO("subscribing at %d", rate);
			break;

		case 'i':
			// instance (defaults to 0)
			instance = strtol(myoptarg, nullptr, 0);
			break;

		default:
			usage();
			return -1;
			break;
		}
	}

	int interval = 0;

	if (rate > 0 && rate < 1000) {
		interval = 1000 / rate;
	}

	return listener_generated(name, number_msgs, interval, instance);
}

static void
usage()
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
Utility to listen on uORB topics and print the data to the console.

Limitation: it can only listen to the first instance of a topic.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME_SIMPLE("listener", "command");
	PRINT_MODULE_USAGE_ARG("<topic_name>", "Select vehicle type", false);
	//PRINT_MODULE_USAGE_ARG("<topic_name> [<num_msgs>]", "uORB topic name and optionally number of messages (default=1)", false);
	PRINT_MODULE_USAGE_PARAM_INT('n', 1, 1, 1000, "number of messages to print", true);
	PRINT_MODULE_USAGE_PARAM_INT('r', 0, 0, 1000, "subscription rate", true);
	PRINT_MODULE_USAGE_PARAM_INT('i', 0, 0, 3, "uORB subscription instance", true);

}
