/****************************************************************************
 *
 *   Copyright (c) 2018-2019 PX4 Development Team. All rights reserved.
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
 * @file ADIS16448.cpp
 */

#include "ADIS16448.h"

ADIS16448::ADIS16448(int bus, uint32_t device, enum Rotation rotation) :
	SPI("ADIS16448", "/dev/adis16448", bus, device, SPIDEV_MODE3, SPI_BUS_SPEED),
	ScheduledWorkItem(px4::device_bus_to_wq(this->get_device_id())),
	_px4_accel(get_device_id(), (external() ? ORB_PRIO_MAX : ORB_PRIO_HIGH), rotation),
	_px4_baro(get_device_id(), (external() ? ORB_PRIO_MAX : ORB_PRIO_HIGH)),
	_px4_gyro(get_device_id(), (external() ? ORB_PRIO_MAX : ORB_PRIO_HIGH), rotation),
	_px4_mag(get_device_id(), (external() ? ORB_PRIO_MAX : ORB_PRIO_HIGH), rotation)
{
	_px4_accel.set_device_type(DRV_ACC_DEVTYPE_ADIS16448);
	_px4_baro.set_device_type(DRV_ACC_DEVTYPE_ADIS16448); // not a typo, baro uses accel device id
	_px4_gyro.set_device_type(DRV_GYR_DEVTYPE_ADIS16448);
	_px4_mag.set_device_type(DRV_MAG_DEVTYPE_ADIS16448);
}

ADIS16448::~ADIS16448()
{
	// Ensure the driver is inactive.
	stop();

	// Delete the perf counter.
	perf_free(_sample_perf);
	perf_free(_bad_transfers);
}

int
ADIS16448::init()
{
	// Do SPI init (and probe) first.
	int ret = SPI::init();

	if (ret != PX4_OK) {
		DEVICE_DEBUG("SPI setup failed");

		// If probe/setup failed, return result.
		return ret;
	}

	if (reset() != PX4_OK) {
		return ret;
	}

	// Obtain an initial set of measurements for advertisement.
	measure();

	return OK;
}

int ADIS16448::reset()
{
	// Set digital FIR filter tap.
	_set_dlpf_filter(BITS_FIR_16_TAP_CFG);

	// Set IMU sample rate.
	_set_sample_rate(_sample_rate);

	_px4_accel.set_scale(ACCEL_INITIAL_SENSITIVITY * CONSTANTS_ONE_G);

	// Set gyroscope scale to default value.
	_set_gyro_dyn_range(GYRO_INITIAL_SENSITIVITY);

	_px4_gyro.set_scale(GYRO_INITIAL_SENSITIVITY * M_DEG_TO_RAD_F);

	_px4_mag.set_scale(MAG_INITIAL_SENSITIVITY);

	// Settling time.
	usleep(50000);

	return OK;
}

int
ADIS16448::probe()
{
	// Retry 5 time to get the ADIS16448 PRODUCT ID number.
	for (size_t i = 0; i < 5; i++) {
		// Read product ID.
		_product_ID = read_reg16(ADIS16448_PRODUCT_ID);

		if (_product_ID != 0) {
			break;
		}
	}

	// Recognize product serial number.
	uint16_t serial_number = (read_reg16(ADIS16334_SERIAL_NUMBER) & 0xfff);

	// Verify product ID.
	switch (_product_ID) {
	case ADIS16448_Product:
		DEVICE_DEBUG("ADIS16448 is detected ID: 0x%02x, Serial: 0x%02x", _product_ID, serial_number);
		modify_reg16(ADIS16448_GPIO_CTRL, 0x0200, 0x0002);  // Turn on ADIS16448 adaptor board led.
		return OK;
	}

	DEVICE_DEBUG("unexpected ID 0x%02x", _product_ID);

	return -EIO;
}

void
ADIS16448::_set_sample_rate(uint16_t desired_sample_rate_hz)
{
	uint16_t smpl_prd = 0;

	if (desired_sample_rate_hz <= 51) {
		smpl_prd = BITS_SMPL_PRD_16_TAP_CFG;

	} else if (desired_sample_rate_hz <= 102) {
		smpl_prd = BITS_SMPL_PRD_8_TAP_CFG;

	} else if (desired_sample_rate_hz <= 204) {
		smpl_prd = BITS_SMPL_PRD_4_TAP_CFG;

	} else if (desired_sample_rate_hz <= 409) {
		smpl_prd = BITS_SMPL_PRD_2_TAP_CFG;

	} else {
		smpl_prd = BITS_SMPL_PRD_NO_TAP_CFG;
	}

	modify_reg16(ADIS16448_SMPL_PRD, 0x1f00, smpl_prd);

	if ((read_reg16(ADIS16448_SMPL_PRD) & 0x1f00) != smpl_prd) {
		DEVICE_DEBUG("failed to set IMU sample rate");
	}
}

void
ADIS16448::_set_dlpf_filter(uint16_t desired_filter_tap)
{
	// Set the DLPF FIR filter tap. This affects both accelerometer and gyroscope.
	modify_reg16(ADIS16448_SENS_AVG, 0x0007, desired_filter_tap);

	// Verify data write on the IMU.
	if ((read_reg16(ADIS16448_SENS_AVG) & 0x0007) != desired_filter_tap) {
		DEVICE_DEBUG("failed to set IMU filter");
	}
}

void
ADIS16448::_set_factory_defaults()
{
	write_reg16(ADIS16448_GLOB_CMD, 0x02);
}

void
ADIS16448::_set_gyro_dyn_range(uint16_t desired_gyro_dyn_range)
{
	uint16_t gyro_range_selection = 0;

	if (desired_gyro_dyn_range <= 250) {
		gyro_range_selection = BITS_GYRO_DYN_RANGE_250_CFG;

	} else if (desired_gyro_dyn_range <= 500) {
		gyro_range_selection = BITS_GYRO_DYN_RANGE_500_CFG;

	} else {
		gyro_range_selection = BITS_GYRO_DYN_RANGE_1000_CFG;
	}

	modify_reg16(ADIS16448_SENS_AVG, 0x0700, gyro_range_selection);

	// Verify data write on the IMU.
	if ((read_reg16(ADIS16448_SENS_AVG) & 0x0700) != gyro_range_selection) {
		DEVICE_DEBUG("failed to set gyro range");

	} else {
		_px4_gyro.set_scale((gyro_range_selection >> 8) / 100.0f);
		//_gyro_range_rad_s  = (float)(gyro_range_selection >> 8) * 250.0f * M_DEG_TO_RAD_F;
	}
}

void
ADIS16448::print_calibration_data()
{
	uint16_t XACCL_OFF = read_reg16(ADIS16448_XACCL_OFF);
	uint16_t YACCL_OFF = read_reg16(ADIS16448_YACCL_OFF);
	uint16_t ZACCL_OFF = read_reg16(ADIS16448_ZACCL_OFF);

	uint16_t XGYRO_OFF = read_reg16(ADIS16448_XGYRO_OFF);
	uint16_t YGYRO_OFF = read_reg16(ADIS16448_YGYRO_OFF);
	uint16_t ZGYRO_OFF = read_reg16(ADIS16448_ZGYRO_OFF);

	uint16_t XMAGN_HIC = read_reg16(ADIS16448_XMAGN_HIC);
	uint16_t YMAGN_HIC = read_reg16(ADIS16448_YMAGN_HIC);
	uint16_t ZMAGN_HIC = read_reg16(ADIS16448_ZMAGN_HIC);

	uint16_t XMAGN_SIC = read_reg16(ADIS16448_XMAGN_SIC);
	uint16_t YMAGN_SIC = read_reg16(ADIS16448_YMAGN_SIC);
	uint16_t ZMAGN_SIC = read_reg16(ADIS16448_ZMAGN_SIC);

	PX4_INFO("single calibration value read:");
	PX4_INFO("XACCL_OFF =:  \t%8.4x\t", XACCL_OFF);
	PX4_INFO("YACCL_OFF =:  \t%8.4x\t", YACCL_OFF);
	PX4_INFO("ZACCL_OFF =:  \t%8.4x\t", ZACCL_OFF);

	PX4_INFO("XGYRO_OFF =:  \t%8.4x\t", XGYRO_OFF);
	PX4_INFO("YGYRO_OFF =:  \t%8.4x\t", YGYRO_OFF);
	PX4_INFO("ZGYRO_OFF =:  \t%8.4x\t", ZGYRO_OFF);

	PX4_INFO("XMAGN_HIC =:  \t%8.4x\t", XMAGN_HIC);
	PX4_INFO("YMAGN_HIC =:  \t%8.4x\t", YMAGN_HIC);
	PX4_INFO("ZMAGN_HIC =:  \t%8.4x\t", ZMAGN_HIC);

	PX4_INFO("XMAGN_SIC =:  \t%8.4x\t", XMAGN_SIC);
	PX4_INFO("YMAGN_SIC =:  \t%8.4x\t", YMAGN_SIC);
	PX4_INFO("ZMAGN_SIC =:  \t%8.4x\t", ZMAGN_SIC);
}

void
ADIS16448::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_bad_transfers);

	_px4_accel.print_status();
	_px4_baro.print_status();
	_px4_gyro.print_status();
	_px4_mag.print_status();
}

void
ADIS16448::modify_reg16(unsigned reg, uint16_t clearbits, uint16_t setbits)
{
	uint16_t val = read_reg16(reg);
	val &= ~clearbits;
	val |= setbits;
	write_reg16(reg, val);
}

uint16_t
ADIS16448::read_reg16(unsigned reg)
{
	uint16_t cmd[1];

	cmd[0] = ((reg | DIR_READ) << 8) & 0xff00;

	transferhword(cmd, nullptr, 1);
	usleep(T_STALL);
	transferhword(nullptr, cmd, 1);
	usleep(T_STALL);

	return cmd[0];
}

void
ADIS16448::write_reg16(unsigned reg, uint16_t value)
{
	uint16_t cmd[2];

	cmd[0] = ((reg | DIR_WRITE) << 8) | (0x00ff & value);
	cmd[1] = (((reg + 0x1) | DIR_WRITE) << 8) | ((0xff00 & value) >> 8);

	transferhword(cmd, nullptr, 1);
	usleep(T_STALL);
	transferhword(cmd + 1, nullptr, 1);
	usleep(T_STALL);
}

int16_t
ADIS16448::convert_12bit_to_int16(int16_t word)
{
	int16_t outputbuffer = 0;

	if ((word >> 11) & 0x1) {
		outputbuffer = (word & 0xfff) | 0xf000;

	} else {
		outputbuffer = (word & 0x0fff);
	}

	return (outputbuffer);
}

void
ADIS16448::start()
{
	// Make sure we are stopped first.
	uint32_t last_call_interval = _call_interval;

	stop();

	_call_interval = last_call_interval;

	// Start polling at the specified rate.
	ScheduleOnInterval(_call_interval, 10000);
}

void
ADIS16448::stop()
{
	ScheduleClear();
}

int
ADIS16448::measure()
{
	// Start measuring.
	perf_begin(_sample_perf);

	//Fetch the full set of measurements from the ADIS16448 in one pass (burst read).
	ADISReport report{};
	report.cmd = ((ADIS16448_GLOB_CMD | DIR_READ) << 8) & 0xff00;

	const hrt_abstime timestamp_sample = hrt_absolute_time();

	if (OK != transferhword((uint16_t *)&report, ((uint16_t *)&report), sizeof(report) / sizeof(uint16_t))) {
		return -EIO;
	}

	if (report.accel_x == 0 && report.accel_y == 0 && report.accel_z == 0 &&
	    report.gyro_x == 0 && report.gyro_y == 0 && report.gyro_z == 0 &&
	    report.mag_x == 0 && report.mag_y == 0 && report.mag_z == 0 &&
	    report.baro == 0 && report.temp == 0) {

		perf_count(_bad_transfers);
		perf_end(_sample_perf);
		return -EIO;
	}

	// error count
	const uint64_t error_count = perf_event_count(_bad_transfers);
	_px4_accel.set_error_count(error_count);
	_px4_baro.set_error_count(error_count);
	_px4_gyro.set_error_count(error_count);
	_px4_mag.set_error_count(error_count);

	// temperature
	const float temperature = convert_12bit_to_int16(report.temp);
	_px4_accel.set_temperature(temperature);
	_px4_baro.set_temperature(temperature);
	_px4_gyro.set_temperature(temperature);
	_px4_mag.set_temperature(temperature);

	_px4_accel.update(timestamp_sample, report.accel_x, report.accel_y, report.accel_z);
	_px4_gyro.update(timestamp_sample, report.gyro_x, report.gyro_y, report.gyro_z);
	_px4_mag.update(timestamp_sample, report.mag_x, report.mag_y, report.mag_z);
	_px4_baro.update(timestamp_sample, report.baro);

	// Stop measuring.
	perf_end(_sample_perf);
	return OK;
}

void
ADIS16448::Run()
{
	// Make another measurement.
	measure();
}


/**
 * Local functions in support of the shell command.
 */
namespace adis16448
{

ADIS16448 *g_dev;

void info();
void info_cal();
void reset();
void start(enum Rotation rotation);
void usage();

/**
 * Start the driver.
 */
void
start(enum Rotation rotation)
{
	if (g_dev != nullptr) {
		// If already started, the still command succeeded.
		PX4_INFO("already started");
	}

	// Create the driver.
#if defined(PX4_SPI_BUS_EXT)
	g_dev = new ADIS16448(PX4_SPI_BUS_EXT, PX4_SPIDEV_EXT_MPU, rotation);
#else
	PX4_ERR("External SPI not available");
#endif

	if (g_dev != nullptr) {
		if (g_dev->init() == OK) {

		}

		//delete g_dev;
		//g_dev = nullptr;
	}

	PX4_ERR("driver start failed");
}

/**
 * Reset the driver.
 */
void
reset()
{

}

/**
 * Print a little info about the driver.
 */
void
info()
{
	if (g_dev == nullptr) {
		PX4_INFO("driver not running");
	}

	printf("state @ %p\n", g_dev);
	g_dev->print_info();

	exit(0);
}

/**
 * Print a sensor calibration info.
 */
void
info_cal()
{
	if (g_dev == nullptr) {
		PX4_INFO("driver not running");
	}

	g_dev->print_calibration_data();

	exit(0);
}

void
usage()
{
	PX4_INFO("missing command: try 'start', 'test', 'info', 'info_cal', 'reset',\n");
	PX4_INFO("options:");
	PX4_INFO("    -R rotation");
}

} // namespace


/**
 * Driver 'main' command.
 */
extern "C" int adis16448_main(int argc, char *argv[])
{
	enum Rotation rotation = ROTATION_NONE;
	int ch;

	/* start options */
	while ((ch = getopt(argc, argv, "R:")) != EOF) {
		switch (ch) {
		case 'R':
			rotation = (enum Rotation)atoi(optarg);
			break;

		default:
			adis16448::usage();
			exit(0);
		}
	}

	const char *verb = argv[optind];

	// Start/load the driver.
	if (!strcmp(verb, "start")) {
		adis16448::start(rotation);
	}

	// Reset the driver.
	if (!strcmp(verb, "reset")) {
		adis16448::reset();
	}

	// Print driver information.
	if (!strcmp(verb, "info")) {
		adis16448::info();
	}

	adis16448::usage();

	exit(1);
}
