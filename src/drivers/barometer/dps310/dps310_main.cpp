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

#include "DPS310.hpp"

#include <px4_getopt.h>

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int dps310_main(int argc, char *argv[]);


enum DPS310_BUS {
	DPS310_BUS_ALL = 0,
	DPS310_BUS_I2C_INTERNAL,
	DPS310_BUS_I2C_EXTERNAL,
	DPS310_BUS_SPI
};

/* interface factories */
extern device::Device *DPS310_SPI_interface(int bus);
extern device::Device *DPS310_I2C_interface(int bus);
typedef device::Device *(*DPS310_constructor)(int);

/**
 * Local functions in support of the shell command.
 */
namespace dps310
{

/*
  list of supported bus configurations
 */
struct dps310_bus_option {
	enum DPS310_BUS busid;
	DPS310_constructor interface_constructor;
	uint8_t busnum;
	DPS310	*dev;
} bus_options[] = {
	{ DPS310_BUS_SPI, &DPS310_SPI_interface, PX4_SPI_BUS_2, NULL },
#ifdef PX4_I2C_BUS_ONBOARD
	{ DPS310_BUS_I2C_INTERNAL, &DPS310_I2C_interface, PX4_I2C_BUS_ONBOARD, NULL },
#endif
};
#define NUM_BUS_OPTIONS (sizeof(bus_options)/sizeof(bus_options[0]))

struct dps310_bus_option *find_bus(enum DPS310_BUS busid);

bool	start_bus(struct dps310_bus_option &bus);

int	start(enum DPS310_BUS busid);
int	info();
int	usage();

/**
 * start driver for a specific bus option
 */
bool
start_bus(struct dps310_bus_option &bus)
{
	if (bus.dev != nullptr) {
		errx(1, "bus option already started");
	}

	device::Device *interface = bus.interface_constructor(bus.busnum);

	if (interface->init() != OK) {
		delete interface;
		warnx("no device on bus %u", (unsigned)bus.busid);
		return false;
	}

	bus.dev = new DPS310(interface);

	if (bus.dev != nullptr && OK != bus.dev->init()) {
		delete bus.dev;
		bus.dev = NULL;
		return false;
	}

	return true;
}

/**
 * Start the driver.
 *
 * This function call only returns once the driver
 * is either successfully up and running or failed to start.
 */
int
start(enum DPS310_BUS busid)
{
	bool started = false;

	for (unsigned i = 0; i < NUM_BUS_OPTIONS; i++) {
		if (busid == DPS310_BUS_ALL && bus_options[i].dev != NULL) {
			// this device is already started
			continue;
		}

		if (busid != DPS310_BUS_ALL && bus_options[i].busid != busid) {
			// not the one that is asked for
			continue;
		}

		started |= start_bus(bus_options[i]);
	}

	if (!started) {
		return PX4_ERROR;
	}

	return PX4_OK;
}

/**
 * find a bus structure for a busid
 */
struct dps310_bus_option *find_bus(enum DPS310_BUS busid)
{
	for (unsigned i = 0; i < NUM_BUS_OPTIONS; i++) {
		if ((busid == DPS310_BUS_ALL ||
		     busid == bus_options[i].busid) && bus_options[i].dev != NULL) {

			return &bus_options[i];
		}
	}

	PX4_ERR("bus %u not started", (unsigned)busid);
	return nullptr;
}

/**
 * Print a little info about the driver.
 */
int
info()
{
	for (uint8_t i = 0; i < NUM_BUS_OPTIONS; i++) {
		struct dps310_bus_option &bus = bus_options[i];

		if (bus.dev != nullptr) {
			bus.dev->print_info();
		}
	}

	return 0;
}

int
usage()
{
	PX4_INFO("missing command: try 'start', 'info', 'test', 'reset'");
	PX4_INFO("options:");
	PX4_INFO("    -X    (external I2C bus)");
	PX4_INFO("    -I    (internal I2C bus)");
	PX4_INFO("    -S    (external SPI bus)");
	PX4_INFO("    -s    (internal SPI bus)");

	return PX4_OK;
}

} // namespace

int
dps310_main(int argc, char *argv[])
{
	enum DPS310_BUS busid = DPS310_BUS_ALL;

	int ch = 0;
	int myoptind = 1;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "XIS:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'I':
			busid = DPS310_BUS_I2C_INTERNAL;
			break;

		case 'X':
			busid = DPS310_BUS_I2C_EXTERNAL;
			break;

		case 'S':
			busid = DPS310_BUS_SPI;
			break;

		default:
			return dps310::usage();
		}
	}

	if (myoptind >= argc) {
		return dps310::usage();
	}

	const char *verb = argv[myoptind];

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(verb, "start")) {
		return dps310::start(busid);
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(verb, "info")) {
		return dps310::info();
	}

	return dps310::usage();
}
