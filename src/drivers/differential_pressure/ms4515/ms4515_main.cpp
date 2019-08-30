/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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

#include <px4_config.h>
#include <px4_getopt.h>

#include <string.h>

#include "MS4515.hpp"

// I2C bus address is 1010001x
static constexpr uint8_t I2C_ADDRESS_MS4515DO = 0x46;

// Local functions in support of the shell command.
namespace ms4515
{
MS4515 *g_dev = nullptr;

int start();
int start_bus(uint8_t i2c_bus, uint8_t address);
int stop();
void usage();

/**
 * Attempt to start driver on all available I2C busses.
 *
 * This function will return as soon as the first sensor
 * is detected on one of the available busses or if no
 * sensors are detected.
 *
 */
int
start()
{
	for (unsigned i = 0; i < NUM_I2C_BUS_OPTIONS; i++) {
		if (start_bus(i2c_bus_options[i], I2C_ADDRESS_MS4515DO) == PX4_OK) {
			return PX4_OK;
		}
	}

	return PX4_ERROR;
}

/**
 * Start the driver on a specific bus.
 *
 * This function only returns if the sensor is up and running
 * or could not be detected successfully.
 */
int
start_bus(uint8_t i2c_bus, uint8_t address)
{
	if (g_dev != nullptr) {
		PX4_WARN("already started");
		return PX4_ERROR;
	}

	// create the driver
	g_dev = new MS4515(i2c_bus, address);

	if (g_dev == nullptr) {
		return PX4_ERROR;
	}

	// try to initialize
	if (g_dev->init() != PX4_OK) {
		delete g_dev;
		g_dev = nullptr;
		return PX4_ERROR;
	}

	return PX4_OK;
}

// stop the driver
int stop()
{
	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;

	} else {
		PX4_ERR("driver not running");
		return PX4_ERROR;
	}

	return PX4_OK;
}

void usage()
{
	PX4_INFO("usage: ms4515 command [options]");
	PX4_INFO("options:");
	PX4_INFO("\t-b --bus i2cbus");
	PX4_INFO("\t-a --all");
	PX4_INFO("command:");
	PX4_INFO("\tstart|stop");
}

} // namespace ms4515

// Driver 'main' command.
extern "C" __EXPORT int ms4515_main(int argc, char *argv[]);

int
ms4515_main(int argc, char *argv[])
{
	uint8_t i2c_bus = PX4_I2C_BUS_EXPANSION;

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	bool start_all = false;

	while ((ch = px4_getopt(argc, argv, "b:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'b':
			i2c_bus = atoi(myoptarg);
			break;

		default:
			ms4515::usage();
			return 0;
		}
	}

	if (myoptind >= argc) {
		ms4515::usage();
		return -1;
	}

	// Start/load the driver.
	if (!strcmp(argv[myoptind], "start")) {
		if (start_all) {
			return ms4515::start();

		} else {
			return ms4515::start_bus(i2c_bus, I2C_ADDRESS_MS4515DO);
		}
	}

	// Stop the driver
	if (!strcmp(argv[myoptind], "stop")) {
		return ms4515::stop();
	}

	ms4515::usage();
	return 0;
}
