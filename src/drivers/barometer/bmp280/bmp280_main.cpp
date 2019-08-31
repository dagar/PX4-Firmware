/****************************************************************************
 *
 *   Copyright (c) 2012-2019 PX4 Development Team. All rights reserved.
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
 * @file bmp280.cpp
 * Driver for the BMP280 barometric pressure sensor connected via I2C TODO or SPI.
 */

#include "BMP280.hpp"
#include "IBMP280.hpp"

enum BMP280_BUS {
	BMP280_BUS_ALL = 0,
	BMP280_BUS_I2C_INTERNAL,
	BMP280_BUS_I2C_EXTERNAL,
	BMP280_BUS_SPI_INTERNAL,
	BMP280_BUS_SPI_EXTERNAL
};

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int bmp280_main(int argc, char *argv[]);

/**
 * Local functions in support of the shell command.
 */
namespace bmp280
{

/*
  list of supported bus configurations
 */
struct bmp280_bus_option {
	enum BMP280_BUS busid;
	BMP280_constructor interface_constructor;
	uint8_t busnum;
	uint32_t device;
	bool external;
	BMP280 *dev;
} bus_options[] = {
#if defined(PX4_SPIDEV_EXT_BARO) && defined(PX4_SPI_BUS_EXT)
	{ BMP280_BUS_SPI_EXTERNAL, &bmp280_spi_interface, PX4_SPI_BUS_EXT, PX4_SPIDEV_EXT_BARO, true, NULL },
#endif
#if defined(PX4_SPIDEV_BARO)
#  if defined(PX4_SPIDEV_BARO_BUS)
	{ BMP280_BUS_SPI_INTERNAL, &bmp280_spi_interface, PX4_SPIDEV_BARO_BUS, PX4_SPIDEV_BARO, false, NULL },
#  else
	{ BMP280_BUS_SPI_INTERNAL, &bmp280_spi_interface, PX4_SPI_BUS_SENSORS, PX4_SPIDEV_BARO, false, NULL },
#  endif
#endif
#if defined(PX4_I2C_BUS_ONBOARD) && defined(PX4_I2C_OBDEV_BMP280)
	{ BMP280_BUS_I2C_INTERNAL, &bmp280_i2c_interface, PX4_I2C_BUS_ONBOARD, PX4_I2C_OBDEV_BMP280, false, NULL },
#endif
#if defined(PX4_I2C_BUS_EXPANSION) && defined(PX4_I2C_OBDEV_BMP280)
	{ BMP280_BUS_I2C_EXTERNAL, &bmp280_i2c_interface, PX4_I2C_BUS_EXPANSION, PX4_I2C_OBDEV_BMP280, true, NULL },
#endif
};
#define NUM_BUS_OPTIONS (sizeof(bus_options)/sizeof(bus_options[0]))

bool	start_bus(struct bmp280_bus_option &bus);
struct bmp280_bus_option &find_bus(enum BMP280_BUS busid);
void	start(enum BMP280_BUS busid);
void	info();
void	usage();

/**
 * Start the driver.
 */
bool
start_bus(struct bmp280_bus_option &bus)
{
	if (bus.dev != nullptr) {
		PX4_ERR("bus option already started");
		exit(1);
	}

	bmp280::IBMP280 *interface = bus.interface_constructor(bus.busnum, bus.device, bus.external);

	if (interface->init() != OK) {
		delete interface;
		PX4_WARN("no device on bus %u", (unsigned)bus.busid);
		return false;
	}

	bus.dev = new BMP280(interface);

	if (bus.dev == nullptr) {
		return false;
	}

	if (OK != bus.dev->init()) {
		delete bus.dev;
		bus.dev = nullptr;
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
void
start(enum BMP280_BUS busid)
{
	bool started = false;
	uint8_t i = 0;

	for (i = 0; i < NUM_BUS_OPTIONS; i++) {
		if (busid == BMP280_BUS_ALL && bus_options[i].dev != NULL) {
			// this device is already started
			continue;
		}

		if (busid != BMP280_BUS_ALL && bus_options[i].busid != busid) {
			// not the one that is asked for
			continue;
		}

		started |= start_bus(bus_options[i]);
	}

	if (!started) {
		PX4_WARN("bus option number is %d", i);
		PX4_ERR("driver start failed");
		exit(1);
	}

	// one or more drivers started OK
	exit(0);
}

/**
 * find a bus structure for a busid
 */
struct bmp280_bus_option &find_bus(enum BMP280_BUS busid)
{
	for (uint8_t i = 0; i < NUM_BUS_OPTIONS; i++) {
		if ((busid == BMP280_BUS_ALL ||
		     busid == bus_options[i].busid) && bus_options[i].dev != NULL) {
			return bus_options[i];
		}
	}

	PX4_ERR("bus %u not started", (unsigned)busid);
	exit(1);
}

/**
 * Print a little info about the driver.
 */
void
info()
{
	for (uint8_t i = 0; i < NUM_BUS_OPTIONS; i++) {
		struct bmp280_bus_option &bus = bus_options[i];

		if (bus.dev != nullptr) {
			bus.dev->print_info();
		}
	}

	exit(0);
}

void
usage()
{
	PX4_WARN("missing command: try 'start', 'info', 'test', 'test2', 'reset'");
	PX4_WARN("options:");
	PX4_WARN("    -X    (external I2C bus TODO)");
	PX4_WARN("    -I    (internal I2C bus TODO)");
	PX4_WARN("    -S    (external SPI bus)");
	PX4_WARN("    -s    (internal SPI bus)");
}

} // namespace

int
bmp280_main(int argc, char *argv[])
{
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;
	enum BMP280_BUS busid = BMP280_BUS_ALL;

	while ((ch = px4_getopt(argc, argv, "XISs", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'X':
			busid = BMP280_BUS_I2C_EXTERNAL;
			break;

		case 'I':
			busid = BMP280_BUS_I2C_INTERNAL;
			break;

		case 'S':
			busid = BMP280_BUS_SPI_EXTERNAL;
			break;

		case 's':
			busid = BMP280_BUS_SPI_INTERNAL;
			break;

		default:
			bmp280::usage();
			return 0;
		}
	}

	if (myoptind >= argc) {
		bmp280::usage();
		return -1;
	}

	const char *verb = argv[myoptind];

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(verb, "start")) {
		bmp280::start(busid);
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(verb, "info")) {
		bmp280::info();
	}

	PX4_ERR("unrecognized command, try 'start', or 'info'");

	return -1;
}
