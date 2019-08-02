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
 * @file ms5611.cpp
 * Driver for the MS5611 and MS5607 barometric pressure sensor connected via I2C or SPI.
 */

#include "ms5611.h"

#include <lib/drivers/barometer/PX4Barometer.hpp>

enum MS56XX_DEVICE_TYPES {
	MS56XX_DEVICE   = 0,
	MS5611_DEVICE	= 5611,
	MS5607_DEVICE	= 5607,
};

enum MS5611_BUS {
	MS5611_BUS_ALL = 0,
	MS5611_BUS_I2C_INTERNAL,
	MS5611_BUS_I2C_EXTERNAL,
	MS5611_BUS_SPI_INTERNAL,
	MS5611_BUS_SPI_EXTERNAL
};

/* helper macro for handling report buffer indices */
#define INCREMENT(_x, _lim)	do { __typeof__(_x) _tmp = _x+1; if (_tmp >= _lim) _tmp = 0; _x = _tmp; } while(0)

/* helper macro for arithmetic - returns the square of the argument */
#define POW2(_x)		((_x) * (_x))

/*
 * MS5611/MS5607 internal constants and data structures.
 */
#define ADDR_CMD_CONVERT_D1_OSR256		0x40	/* write to this address to start pressure conversion */
#define ADDR_CMD_CONVERT_D1_OSR512		0x42	/* write to this address to start pressure conversion */
#define ADDR_CMD_CONVERT_D1_OSR1024		0x44	/* write to this address to start pressure conversion */
#define ADDR_CMD_CONVERT_D1_OSR2048		0x46	/* write to this address to start pressure conversion */
#define ADDR_CMD_CONVERT_D1_OSR4096		0x48	/* write to this address to start pressure conversion */
#define ADDR_CMD_CONVERT_D2_OSR256		0x50	/* write to this address to start temperature conversion */
#define ADDR_CMD_CONVERT_D2_OSR512		0x52	/* write to this address to start temperature conversion */
#define ADDR_CMD_CONVERT_D2_OSR1024		0x54	/* write to this address to start temperature conversion */
#define ADDR_CMD_CONVERT_D2_OSR2048		0x56	/* write to this address to start temperature conversion */
#define ADDR_CMD_CONVERT_D2_OSR4096		0x58	/* write to this address to start temperature conversion */

/*
  use an OSR of 1024 to reduce the self-heating effect of the
  sensor. Information from MS tells us that some individual sensors
  are quite sensitive to this effect and that reducing the OSR can
  make a big difference
 */
#define ADDR_CMD_CONVERT_D1			ADDR_CMD_CONVERT_D1_OSR1024
#define ADDR_CMD_CONVERT_D2			ADDR_CMD_CONVERT_D2_OSR1024

/*
 * Maximum internal conversion time for OSR 1024 is 2.28 ms. We set an update
 * rate of 100Hz which is be very safe not to read the ADC before the
 * conversion finished
 */
#define MS5611_CONVERSION_INTERVAL	10000	/* microseconds */
#define MS5611_MEASUREMENT_RATIO	3	/* pressure measurements per temperature measurement */

class MS5611 : public px4::ScheduledWorkItem
{
public:
	MS5611(device::Device *interface, ms5611::prom_u &prom_buf, enum MS56XX_DEVICE_TYPES device_type);
	~MS5611();

	virtual int		init();

	void			print_info();

protected:

	PX4Barometer		_px4_barometer;

	device::Device		*_interface{nullptr};

	ms5611::prom_s		_prom;

	unsigned		_measure_interval{0};

	enum MS56XX_DEVICE_TYPES _device_type;
	bool			_collect_phase{false};
	unsigned		_measure_phase;

	/* intermediate temperature values per MS5611/MS5607 datasheet */
	int32_t			_TEMP{0};
	int64_t			_OFF{0};
	int64_t			_SENS{0};
	float			_P{0.0f};
	float			_T{0.0f};

	perf_counter_t		_sample_perf;
	perf_counter_t		_measure_perf;
	perf_counter_t		_comms_errors;


	void			start();
	void			stop();
	bool			reset();

	/**
	 * Perform a poll cycle; collect from the previous measurement
	 * and start a new one.
	 *
	 * This is the heart of the measurement state machine.  This function
	 * alternately starts a measurement, or collects the data from the
	 * previous measurement.
	 *
	 * When the interval between measurements is greater than the minimum
	 * measurement interval, a gap is inserted between collection
	 * and measurement to provide the most recent measurement possible
	 * at the next interval.
	 */
	void			Run() override;

	/**
	 * Issue a measurement command for the current state.
	 *
	 * @return		OK if the measurement command was successful.
	 */
	virtual int		measure();

	/**
	 * Collect the result of the most recent measurement.
	 */
	virtual int		collect();
};

MS5611::MS5611(device::Device *interface, ms5611::prom_u &prom_buf, enum MS56XX_DEVICE_TYPES device_type) :
	ScheduledWorkItem(px4::device_bus_to_wq(interface->get_device_id())),
	_px4_barometer(interface->get_device_id()),
	_interface(interface),
	_prom(prom_buf.s),
	_device_type(device_type),
	_sample_perf(perf_alloc(PC_ELAPSED, "ms5611_read")),
	_measure_perf(perf_alloc(PC_ELAPSED, "ms5611_measure")),
	_comms_errors(perf_alloc(PC_COUNT, "ms5611_com_err"))
{
}

MS5611::~MS5611()
{
	// make sure we are truly inactive
	stop();

	// free perf counters
	perf_free(_sample_perf);
	perf_free(_measure_perf);
	perf_free(_comms_errors);

	delete _interface;
}

int
MS5611::init()
{
	int ret = PX4_OK;

	// do a first measurement cycle to populate reports with valid data
	_measure_phase = 0;

	bool autodetect = false;

	if (_device_type == MS56XX_DEVICE) {
		autodetect = true;
		// try first with MS5611 and fallback to MS5607
		_device_type = MS5611_DEVICE;
	}

	while (true) {
		// do temperature first
		if (OK != measure()) {
			ret = -EIO;
			break;
		}

		px4_usleep(MS5611_CONVERSION_INTERVAL);

		if (OK != collect()) {
			ret = -EIO;
			break;
		}

		// now do a pressure measurement
		if (OK != measure()) {
			ret = -EIO;
			break;
		}

		px4_usleep(MS5611_CONVERSION_INTERVAL);

		if (OK != collect()) {
			ret = -EIO;
			break;
		}

		sensor_baro_s brp{};

		if (autodetect) {
			if (_device_type == MS5611_DEVICE) {
				if (brp.pressure < 520.0f) {
					/* This is likely not this device, try again */
					_device_type = MS5607_DEVICE;
					_measure_phase = 0;

					continue;
				}

			} else if (_device_type == MS5607_DEVICE) {
				if (brp.pressure < 520.0f) {
					/* Both devices returned a very low pressure;
					 * have fun on Everest using MS5611 */
					_device_type = MS5611_DEVICE;
				}
			}
		}

		switch (_device_type) {
		default:

		/* fall through */
		case MS5611_DEVICE:
			_interface->set_device_type(DRV_BARO_DEVTYPE_MS5611);
			break;

		case MS5607_DEVICE:
			_interface->set_device_type(DRV_BARO_DEVTYPE_MS5607);
			break;
		}

		/* ensure correct devid */
		brp.device_id = _interface->get_device_id();

		break;
	}

	return ret;
}

void
MS5611::start()
{
	// reset the report ring and state machine
	_collect_phase = false;
	_measure_phase = 0;

	// schedule a cycle to start things
	ScheduleNow();
}

void
MS5611::stop()
{
	ScheduleClear();
}

bool
MS5611::reset()
{
	//uint8_t cmd = ADDR_RESET_CMD | DIR_WRITE;

	//return  _transfer(&cmd, nullptr, 1);

	return true;
}

void
MS5611::Run()
{
	int ret = PX4_ERROR;

	// collection phase?
	if (_collect_phase) {

		// perform collection
		ret = collect();

		if (ret != OK) {
			if (ret == -6) {
				/*
				 * The ms5611 seems to regularly fail to respond to
				 * its address; this happens often enough that we'd rather not
				 * spam the console with a message for this.
				 */
			} else {
				//DEVICE_LOG("collection error %d", ret);
			}

			/* issue a reset command to the sensor */
			//_interface->ioctl(IOCTL_RESET, dummy);
			reset();

			/* reset the collection state machine and try again - we need
			 * to wait 2.8 ms after issuing the sensor reset command
			 * according to the MS5611 datasheet
			 */
			ScheduleDelayed(2800);

			return;
		}

		/* next phase is measurement */
		_collect_phase = false;

		/*
		 * Is there a collect->measure gap?
		 * Don't inject one after temperature measurements, so we can keep
		 * doing pressure measurements at something close to the desired rate.
		 */
		if ((_measure_phase != 0) &&
		    (_measure_interval > MS5611_CONVERSION_INTERVAL)) {

			/* schedule a fresh cycle call when we are ready to measure again */
			ScheduleDelayed(_measure_interval - MS5611_CONVERSION_INTERVAL);

			return;
		}
	}

	// measurement phase
	ret = measure();

	if (ret != OK) {
		// issue a reset command to the sensor
		reset();

		// reset the collection state machine and try again
		start();
		return;
	}

	// next phase is collection
	_collect_phase = true;

	// schedule a fresh cycle call when the measurement is done
	ScheduleDelayed(MS5611_CONVERSION_INTERVAL);
}

int
MS5611::measure()
{
	perf_begin(_measure_perf);

	// In phase zero, request temperature; in other phases, request pressure.
	unsigned addr = (_measure_phase == 0) ? ADDR_CMD_CONVERT_D2 : ADDR_CMD_CONVERT_D1;

	// Send the command to begin measuring.
	int ret = _interface->ioctl(IOCTL_MEASURE, addr);

	if (OK != ret) {
		perf_count(_comms_errors);
	}

	perf_end(_measure_perf);

	return ret;
}

int
MS5611::collect()
{
	uint32_t raw;

	perf_begin(_sample_perf);

	/* this should be fairly close to the end of the conversion, so the best approximation of the time */
	const hrt_abstime timestamp_sample = hrt_absolute_time();

	// read the most recent measurement - read offset/size are hardcoded in the interface
	int ret = _interface->read(0, (void *)&raw, 0);

	if (ret < 0) {
		perf_count(_comms_errors);
		perf_end(_sample_perf);
		return ret;
	}

	_px4_barometer.set_error_count(perf_event_count(_comms_errors));

	// handle a measurement
	if (_measure_phase == 0) {

		/* temperature offset (in ADC units) */
		int32_t dT = (int32_t)raw - ((int32_t)_prom.c5_reference_temp << 8);

		/* absolute temperature in centidegrees - note intermediate value is outside 32-bit range */
		_TEMP = 2000 + (int32_t)(((int64_t)dT * _prom.c6_temp_coeff_temp) >> 23);

		/* base sensor scale/offset values */
		if (_device_type == MS5611_DEVICE) {

			/* Perform MS5611 Caculation */

			_OFF  = ((int64_t)_prom.c2_pressure_offset << 16) + (((int64_t)_prom.c4_temp_coeff_pres_offset * dT) >> 7);
			_SENS = ((int64_t)_prom.c1_pressure_sens << 15) + (((int64_t)_prom.c3_temp_coeff_pres_sens * dT) >> 8);

			/* MS5611 temperature compensation */

			if (_TEMP < 2000) {

				int32_t T2 = POW2(dT) >> 31;

				int64_t f = POW2((int64_t)_TEMP - 2000);
				int64_t OFF2 = 5 * f >> 1;
				int64_t SENS2 = 5 * f >> 2;

				if (_TEMP < -1500) {

					int64_t f2 = POW2(_TEMP + 1500);
					OFF2 += 7 * f2;
					SENS2 += 11 * f2 >> 1;
				}

				_TEMP -= T2;
				_OFF  -= OFF2;
				_SENS -= SENS2;
			}

		} else if (_device_type == MS5607_DEVICE) {

			/* Perform MS5607 Caculation */

			_OFF  = ((int64_t)_prom.c2_pressure_offset << 17) + (((int64_t)_prom.c4_temp_coeff_pres_offset * dT) >> 6);
			_SENS = ((int64_t)_prom.c1_pressure_sens << 16) + (((int64_t)_prom.c3_temp_coeff_pres_sens * dT) >> 7);

			/* MS5607 temperature compensation */

			if (_TEMP < 2000) {

				int32_t T2 = POW2(dT) >> 31;

				int64_t f = POW2((int64_t)_TEMP - 2000);
				int64_t OFF2 = 61 * f >> 4;
				int64_t SENS2 = 2 * f;

				if (_TEMP < -1500) {
					int64_t f2 = POW2(_TEMP + 1500);
					OFF2 += 15 * f2;
					SENS2 += 8 * f2;
				}

				_TEMP -= T2;
				_OFF  -= OFF2;
				_SENS -= SENS2;
			}
		}

	} else {

		/* pressure calculation, result in Pa */
		int32_t P = (((raw * _SENS) >> 21) - _OFF) >> 15;
		_P = P * 0.01f;
		_T = _TEMP * 0.01f;

		/* generate a new report */
		float temperature = _TEMP / 100.0f;
		_px4_barometer.set_temperature(temperature);

		float pressure = P / 100.0f;		/* convert to millibar */

		_px4_barometer.update(timestamp_sample, pressure);
	}

	// update the measurement state machine
	INCREMENT(_measure_phase, MS5611_MEASUREMENT_RATIO + 1);

	perf_end(_sample_perf);

	return OK;
}

void
MS5611::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	printf("poll interval:  %u\n", _measure_interval);

	printf("device:         %s\n", _device_type == MS5611_DEVICE ? "ms5611" : "ms5607");
}

/**
 * Local functions in support of the shell command.
 */
namespace ms5611
{

/*
  list of supported bus configurations
 */
struct ms5611_bus_option {
	enum MS5611_BUS busid;
	MS5611_constructor interface_constructor;
	uint8_t busnum;
	MS5611 *dev;
} bus_options[] = {
#if defined(PX4_SPIDEV_EXT_BARO) && defined(PX4_SPI_BUS_EXT)
	{ MS5611_BUS_SPI_EXTERNAL, &MS5611_spi_interface, PX4_SPI_BUS_EXT, NULL },
#endif
#ifdef PX4_SPIDEV_BARO
	{ MS5611_BUS_SPI_INTERNAL, &MS5611_spi_interface, PX4_SPI_BUS_BARO, NULL },
#endif
#ifdef PX4_I2C_BUS_ONBOARD
	{ MS5611_BUS_I2C_INTERNAL, &MS5611_i2c_interface, PX4_I2C_BUS_ONBOARD, NULL },
#endif
#ifdef PX4_I2C_BUS_EXPANSION
	{ MS5611_BUS_I2C_EXTERNAL, &MS5611_i2c_interface, PX4_I2C_BUS_EXPANSION, NULL },
#endif
#ifdef PX4_I2C_BUS_EXPANSION1
	{ MS5611_BUS_I2C_EXTERNAL, &MS5611_i2c_interface, PX4_I2C_BUS_EXPANSION1, NULL },
#endif
#ifdef PX4_I2C_BUS_EXPANSION2
	{ MS5611_BUS_I2C_EXTERNAL, &MS5611_i2c_interface, PX4_I2C_BUS_EXPANSION2, NULL },
#endif
};
#define NUM_BUS_OPTIONS (sizeof(bus_options)/sizeof(bus_options[0]))

bool	start_bus(struct ms5611_bus_option &bus, enum MS56XX_DEVICE_TYPES device_type);
struct ms5611_bus_option *find_bus(enum MS5611_BUS busid);
int	start(enum MS5611_BUS busid, enum MS56XX_DEVICE_TYPES device_type);
int	test(enum MS5611_BUS busid);
int	reset(enum MS5611_BUS busid);
int	info();
int	usage();

/**
 * MS5611 crc4 cribbed from the datasheet
 */
bool crc4(uint16_t *n_prom)
{
	int16_t cnt;
	uint16_t n_rem;
	uint16_t crc_read;
	uint8_t n_bit;

	n_rem = 0x00;

	/* save the read crc */
	crc_read = n_prom[7];

	/* remove CRC byte */
	n_prom[7] = (0xFF00 & (n_prom[7]));

	for (cnt = 0; cnt < 16; cnt++) {
		/* uneven bytes */
		if (cnt & 1) {
			n_rem ^= (uint8_t)((n_prom[cnt >> 1]) & 0x00FF);

		} else {
			n_rem ^= (uint8_t)(n_prom[cnt >> 1] >> 8);
		}

		for (n_bit = 8; n_bit > 0; n_bit--) {
			if (n_rem & 0x8000) {
				n_rem = (n_rem << 1) ^ 0x3000;

			} else {
				n_rem = (n_rem << 1);
			}
		}
	}

	/* final 4 bit remainder is CRC value */
	n_rem = (0x000F & (n_rem >> 12));
	n_prom[7] = crc_read;

	/* return true if CRCs match */
	return (0x000F & crc_read) == (n_rem ^ 0x00);
}

/**
 * Start the driver.
 */
bool start_bus(struct ms5611_bus_option &bus, enum MS56XX_DEVICE_TYPES device_type)
{
	if (bus.dev != nullptr) {
		PX4_ERR("bus option already started");
		return false;
	}

	prom_u prom_buf;
	device::Device *interface = bus.interface_constructor(prom_buf, bus.busnum);

	if (interface->init() != OK) {
		delete interface;
		PX4_WARN("no device on bus %u", (unsigned)bus.busid);
		return false;
	}

	bus.dev = new MS5611(interface, prom_buf, device_type);

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
int start(enum MS5611_BUS busid, enum MS56XX_DEVICE_TYPES device_type)
{
	uint8_t i;
	bool started = false;

	for (i = 0; i < NUM_BUS_OPTIONS; i++) {
		if (busid == MS5611_BUS_ALL && bus_options[i].dev != NULL) {
			// this device is already started
			continue;
		}

		if (busid != MS5611_BUS_ALL && bus_options[i].busid != busid) {
			// not the one that is asked for
			continue;
		}

		started = started | start_bus(bus_options[i], device_type);
	}

	if (!started) {
		return 1;
	}

	// one or more drivers started OK
	return 0;
}

/**
 * find a bus structure for a busid
 */
struct ms5611_bus_option *find_bus(enum MS5611_BUS busid)
{
	for (uint8_t i = 0; i < NUM_BUS_OPTIONS; i++) {
		if ((busid == MS5611_BUS_ALL ||
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
		struct ms5611_bus_option *bus = &bus_options[i];

		if (bus->dev != nullptr) {
			bus->dev->print_info();
		}
	}

	return 0;
}

int usage()
{
	warnx("missing command: try 'start', 'info', 'reset'");
	warnx("options:");
	warnx("    -X    (external I2C bus)");
	warnx("    -I    (intternal I2C bus)");
	warnx("    -S    (external SPI bus)");
	warnx("    -s    (internal SPI bus)");
	warnx("    -T    5611|5607 (default 5611)");
	warnx("    -T    0 (autodetect version)");

	return 0;
}

} // namespace

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int ms5611_main(int argc, char *argv[]);

int ms5611_main(int argc, char *argv[])
{
	enum MS5611_BUS busid = MS5611_BUS_ALL;
	enum MS56XX_DEVICE_TYPES device_type = MS5611_DEVICE;
	int ch = 0;
	int myoptind = 1;
	const char *myoptarg = NULL;

	/* jump over start/off/etc and look at options first */
	while ((ch = px4_getopt(argc, argv, "T:XISs", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'X':
			busid = MS5611_BUS_I2C_EXTERNAL;
			break;

		case 'I':
			busid = MS5611_BUS_I2C_INTERNAL;
			break;

		case 'S':
			busid = MS5611_BUS_SPI_EXTERNAL;
			break;

		case 's':
			busid = MS5611_BUS_SPI_INTERNAL;
			break;

		case 'T': {
				int val = atoi(myoptarg);

				if (val == 5611) {
					device_type = MS5611_DEVICE;
					break;

				} else if (val == 5607) {
					device_type = MS5607_DEVICE;
					break;

				} else if (val == 0) {
					device_type = MS56XX_DEVICE;
					break;
				}
			}

		/* FALLTHROUGH */

		default:
			return ms5611::usage();
		}
	}

	if (myoptind >= argc) {
		return ms5611::usage();
		return -1;
	}

	const char *verb = argv[myoptind];

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(verb, "start")) {
		return ms5611::start(busid, device_type);
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(verb, "info")) {
		return ms5611::info();
	}

	PX4_ERR("unrecognised command, try 'start', 'reset' or 'info'");
	return -1;
}
