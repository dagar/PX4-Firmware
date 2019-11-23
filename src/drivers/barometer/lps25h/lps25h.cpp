/****************************************************************************
 *
 *   Copyright (c) 2016-2019 PX4 Development Team. All rights reserved.
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
 * @file lps25h.cpp
 *
 * Driver for the LPS25H barometer connected via I2C or SPI.
 */

#include "lps25h.h"

/*
 * LPS25H internal constants and data structures.
 */

/* Max measurement rate is 25Hz */
#define LPS25H_CONVERSION_INTERVAL	(1000000 / 25)	/* microseconds */

#define ADDR_REF_P_XL		0x08
#define ADDR_REF_P_L		0x09
#define ADDR_REF_P_H		0x0A
#define ADDR_WHO_AM_I		0x0F
#define ADDR_RES_CONF		0x10
#define ADDR_CTRL_REG1		0x20
#define ADDR_CTRL_REG2		0x21
#define ADDR_CTRL_REG3		0x22
#define ADDR_CTRL_REG4		0x23
#define ADDR_INT_CFG		0x24
#define ADDR_INT_SOURCE		0x25

#define ADDR_STATUS_REG		0x27
#define ADDR_P_OUT_XL		0x28
#define ADDR_P_OUT_L		0x29
#define ADDR_P_OUT_H		0x2A
#define ADDR_TEMP_OUT_L		0x2B
#define ADDR_TEMP_OUT_H		0x2C

#define ADDR_FIFO_CTRL		0x2E
#define ADDR_FIFO_STATUS	0x2F
#define ADDR_THS_P_L		0x30
#define ADDR_THS_P_H		0x31

#define ADDR_RPDS_L		0x39
#define ADDR_RPDS_H		0x3A

/* Data sheet is ambigious if AVGT or AVGP is first */
#define RES_CONF_AVGT_8		0x00
#define RES_CONF_AVGT_32	0x01
#define RES_CONF_AVGT_128	0x02
#define RES_CONF_AVGT_512	0x03
#define RES_CONF_AVGP_8		0x00
#define RES_CONF_AVGP_32	0x04
#define RES_CONF_AVGP_128	0x08
#define RES_CONF_AVGP_512	0x0C

#define CTRL_REG1_SIM		(1 << 0)
#define CTRL_REG1_RESET_AZ	(1 << 1)
#define CTRL_REG1_BDU		(1 << 2)
#define CTRL_REG1_DIFF_EN	(1 << 3)
#define CTRL_REG1_PD		(1 << 7)
#define CTRL_REG1_ODR_SINGLE	(0 << 4)
#define CTRL_REG1_ODR_1HZ	(1 << 4)
#define CTRL_REG1_ODR_7HZ	(2 << 4)
#define CTRL_REG1_ODR_12HZ5	(3 << 4)
#define CTRL_REG1_ODR_25HZ	(4 << 4)

#define CTRL_REG2_ONE_SHOT	(1 << 0)
#define CTRL_REG2_AUTO_ZERO	(1 << 1)
#define CTRL_REG2_SWRESET	(1 << 2)
#define CTRL_REG2_FIFO_MEAN_DEC	(1 << 4)
#define CTRL_REG2_WTM_EN	(1 << 5)
#define CTRL_REG2_FIFO_EN	(1 << 6)
#define CTRL_REG2_BOOT		(1 << 7)

#define CTRL_REG3_INT1_S_DATA	0x0
#define CTRL_REG3_INT1_S_P_HIGH	0x1
#define CTRL_REG3_INT1_S_P_LOW	0x2
#define CTRL_REG3_INT1_S_P_LIM	0x3
#define CTRL_REG3_PP_OD		(1 << 6)
#define CTRL_REG3_INT_H_L	(1 << 7)

#define CTRL_REG4_P1_DRDY	(1 << 0)
#define CTRL_REG4_P1_OVERRUN	(1 << 1)
#define CTRL_REG4_P1_WTM	(1 << 2)
#define CTRL_REG4_P1_EMPTY	(1 << 3)

#define INTERRUPT_CFG_PH_E	(1 << 0)
#define INTERRUPT_CFG_PL_E	(1 << 1)
#define INTERRUPT_CFG_LIR	(1 << 2)

#define INT_SOURCE_PH		(1 << 0)
#define INT_SOURCE_PL		(1 << 1)
#define INT_SOURCE_IA		(1 << 2)

#define STATUS_REG_T_DA		(1 << 0)
#define STATUS_REG_P_DA		(1 << 1)
#define STATUS_REG_T_OR		(1 << 4)
#define STATUS_REG_P_OR		(1 << 5)

#define FIFO_CTRL_WTM_FMEAN_2	0x01
#define FIFO_CTRL_WTM_FMEAN_4	0x03
#define FIFO_CTRL_WTM_FMEAN_8	0x07
#define FIFO_CTRL_WTM_FMEAN_16	0x0F
#define FIFO_CTRL_WTM_FMEAN_32	0x1F
#define FIFO_CTRL_F_MODE_BYPASS	(0x0 << 5)
#define FIFO_CTRL_F_MODE_FIFO	(0x1 << 5)
#define FIFO_CTRL_F_MODE_STREAM	(0x2 << 5)
#define FIFO_CTRL_F_MODE_SFIFO	(0x3 << 5)
#define FIFO_CTRL_F_MODE_BSTRM	(0x4 << 5)
#define FIFO_CTRL_F_MODE_FMEAN	(0x6 << 5)
#define FIFO_CTRL_F_MODE_BFIFO	(0x7 << 5)

#define FIFO_STATUS_EMPTY	(1 << 5)
#define FIFO_STATUS_FULL	(1 << 6)
#define FIFO_STATUS_WTM		(1 << 7)

enum LPS25H_BUS {
	LPS25H_BUS_ALL = 0,
	LPS25H_BUS_I2C_INTERNAL,
	LPS25H_BUS_I2C_EXTERNAL,
	LPS25H_BUS_SPI
};

class LPS25H : public px4::ScheduledWorkItem
{
public:
	LPS25H(device::Device *interface, const char *path);
	virtual ~LPS25H();

	int			init();

	void			print_info();


private:
	device::Device			*_interface;

	unsigned		_measure_interval{0};

	bool			_collect_phase{false};

	perf_counter_t		_sample_perf;
	perf_counter_t		_comms_errors;

	void			start();
	void			stop();
	int			reset();
	void			Run() override;


	int			write_reg(uint8_t reg, uint8_t val);
	int			read_reg(uint8_t reg, uint8_t &val);

	int			measure();
	int			collect();
};

LPS25H::LPS25H(device::Device *interface) :
	ScheduledWorkItem(MODULE_NAME, px4::device_bus_to_wq(interface->get_device_id())),
	_interface(interface),
	_sample_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": read")),
	_comms_errors(perf_alloc(PC_COUNT, MODULE_NAME": comms errors"))
{
	_px4_baro.set_device_type(DRV_BARO_DEVTYPE_LPS25H);
}

LPS25H::~LPS25H()
{
	stop();

	perf_free(_sample_perf);
	perf_free(_comms_errors);

	delete _interface;
}

int
LPS25H::init()
{
	return reset();
}

void
LPS25H::start()
{
	/* reset the report ring and state machine */
	_collect_phase = false;

	/* schedule a cycle to start things */
	ScheduleNow();
}

void
LPS25H::stop()
{
	ScheduleClear();
}

int
LPS25H::reset()
{
	int ret = 0;

	// Power on
	ret = write_reg(ADDR_CTRL_REG1, CTRL_REG1_PD);
	usleep(1000);

	// Reset
	ret = write_reg(ADDR_CTRL_REG2, CTRL_REG2_BOOT | CTRL_REG2_SWRESET);
	usleep(5000);

	// Power on
	ret = write_reg(ADDR_CTRL_REG1, CTRL_REG1_PD);
	usleep(1000);

	return ret;
}

void
LPS25H::Run()
{
	/* collection phase? */
	if (_collect_phase) {

		/* perform collection */
		if (OK != collect()) {
			PX4_DEBUG("collection error");
			/* restart the measurement state machine */
			start();
			return;
		}

		/* next phase is measurement */
		_collect_phase = false;

		/*
		 * Is there a collect->measure gap?
		 */
		if (_measure_interval > (LPS25H_CONVERSION_INTERVAL)) {

			/* schedule a fresh cycle call when we are ready to measure again */
			ScheduleDelayed(_measure_interval - LPS25H_CONVERSION_INTERVAL);

			return;
		}
	}

	/* measurement phase */
	if (OK != measure()) {
		PX4_DEBUG("measure error");
	}

	/* next phase is collection */
	_collect_phase = true;

	/* schedule a fresh cycle call when the measurement is done */
	ScheduleDelayed(LPS25H_CONVERSION_INTERVAL);
}

int
LPS25H::measure()
{
	// Send the command to begin a 16-bit measurement.
	int ret = write_reg(ADDR_CTRL_REG2, CTRL_REG2_ONE_SHOT);

	if (OK != ret) {
		perf_count(_comms_errors);
	}

	return ret;
}

int
LPS25H::collect()
{
#pragma pack(push, 1)
	struct {
		uint8_t		status;
		uint8_t		p_xl, p_l, p_h;
		int16_t		t;
	} report;
#pragma pack(pop)

	perf_begin(_sample_perf);

	/*
	 * @note  We could read the status register 1 here, which could tell us that
	 *        we were too early and that the output registers are still being
	 *        written.  In the common case that would just slow us down, and
	 *        we're better off just never being early.
	 */

	/* get measurements from the device : MSB enables register address auto-increment */
	const hrt_abstime timestamp_sample = hrt_absolute_time();
	int ret = _interface->read(ADDR_STATUS_REG | (1 << 7), (uint8_t *)&report, sizeof(report));

	if (ret != OK) {
		perf_count(_comms_errors);
		perf_end(_sample_perf);
		return ret;
	}

	const float temperature = 42.5f + (report.t / 480);

	// raw pressure
	uint32_t raw = report.p_xl + (report.p_l << 8) + (report.p_h << 16);

	// Pressure and MSL in mBar
	float p = raw / 4096.0f;

	_px4_baro.set_error_count(perf_event_count(_comms_errors));
	_px4_baro.set_temperature(temperature);
	_px4_baro.update(timestamp_sample, p);

	perf_end(_sample_perf);

	return PX4_OK;
}

int
LPS25H::write_reg(uint8_t reg, uint8_t val)
{
	uint8_t buf = val;
	return _interface->write(reg, &buf, 1);
}

int
LPS25H::read_reg(uint8_t reg, uint8_t &val)
{
	uint8_t buf = val;
	int ret = _interface->read(reg, &buf, 1);
	val = buf;
	return ret;
}

void
LPS25H::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);

	_px4_baro.print_status();
}

/**
 * Local functions in support of the shell command.
 */
namespace lps25h
{

/*
  list of supported bus configurations
 */
struct lps25h_bus_option {
	enum LPS25H_BUS busid;
	LPS25H_constructor interface_constructor;
	uint8_t busnum;
	LPS25H	*dev;
} bus_options[] = {
	{ LPS25H_BUS_I2C_EXTERNAL, &LPS25H_I2C_interface, PX4_I2C_BUS_EXPANSION, nullptr },
#ifdef PX4_I2C_BUS_EXPANSION1
	{ LPS25H_BUS_I2C_EXTERNAL, &LPS25H_I2C_interface, PX4_I2C_BUS_EXPANSION1, nullptr },
#endif
#ifdef PX4_I2C_BUS_EXPANSION2
	{ LPS25H_BUS_I2C_EXTERNAL, &LPS25H_I2C_interface, PX4_I2C_BUS_EXPANSION2, nullptr },
#endif
#ifdef PX4_I2C_BUS_ONBOARD
	{ LPS25H_BUS_I2C_INTERNAL, &LPS25H_I2C_interface, PX4_I2C_BUS_ONBOARD, nullptr },
#endif
#ifdef PX4_SPIDEV_HMC
	{ LPS25H_BUS_SPI, &LPS25H_SPI_interface, PX4_SPI_BUS_SENSORS, nullptr },
#endif
};

/**
 * start driver for a specific bus option
 */
bool
start_bus(struct lps25h_bus_option &bus)
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

	bus.dev = new LPS25H(interface, bus.devpath);

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
void
start(enum LPS25H_BUS busid)
{
	bool started = false;

	for (unsigned i = 0; i < NUM_BUS_OPTIONS; i++) {
		if (busid == LPS25H_BUS_ALL && bus_options[i].dev != NULL) {
			// this device is already started
			continue;
		}

		if (busid != LPS25H_BUS_ALL && bus_options[i].busid != busid) {
			// not the one that is asked for
			continue;
		}

		started |= start_bus(bus_options[i]);
	}

	if (!started) {
		exit(1);
	}
}

/**
 * find a bus structure for a busid
 */
struct lps25h_bus_option &find_bus(enum LPS25H_BUS busid)
{
	for (unsigned i = 0; i < NUM_BUS_OPTIONS; i++) {
		if ((busid == LPS25H_BUS_ALL ||
		     busid == bus_options[i].busid) && bus_options[i].dev != NULL) {
			return bus_options[i];
		}
	}

	errx(1, "bus %u not started", (unsigned)busid);
}

void
reset(enum LPS25H_BUS busid)
{
	struct lps25h_bus_option &bus = find_bus(busid);
	const char *path = bus.devpath;
}

/**
 * Print a little info about the driver.
 */
void
info()
{
	for (uint8_t i = 0; i < NUM_BUS_OPTIONS; i++) {
		struct lps25h_bus_option &bus = bus_options[i];

		if (bus.dev != nullptr) {
			warnx("%s", bus.devpath);
			bus.dev->print_info();
		}
	}

	exit(0);
}

void
usage()
{
	warnx("missing command: try 'start', 'info', 'test', 'reset'");
	warnx("options:");
	warnx("    -X    (external I2C bus)");
	warnx("    -I    (internal I2C bus)");
	warnx("    -S    (external SPI bus)");
	warnx("    -s    (internal SPI bus)");
}

} // namespace

int
lps25h_main(int argc, char *argv[])
{
	enum LPS25H_BUS busid = LPS25H_BUS_ALL;

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "XIS:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
#if (PX4_I2C_BUS_ONBOARD || PX4_SPIDEV_HMC)

		case 'I':
			busid = LPS25H_BUS_I2C_INTERNAL;
			break;
#endif

		case 'X':
			busid = LPS25H_BUS_I2C_EXTERNAL;
			break;

		case 'S':
			busid = LPS25H_BUS_SPI;
			break;

		default:
			lps25h::usage();
			exit(0);
		}
	}

	if (myoptind >= argc) {
		lps25h::usage();
		exit(0);
	}

	const char *verb = argv[myoptind];

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(verb, "start")) {
		lps25h::start(busid);
	}

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(verb, "test")) {
		lps25h::test(busid);
	}

	/*
	 * Reset the driver.
	 */
	if (!strcmp(verb, "reset")) {
		lps25h::reset(busid);
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(verb, "info")) {
		lps25h::info();
	}

	errx(1, "unrecognised command, try 'start', 'test', 'reset' or 'info'");
}
