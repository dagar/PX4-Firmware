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

/**
 * @file bmm150.cpp
 * Driver for the Bosch BMM 150 MEMS magnetometer connected via I2C.
 */

#include "bmm150.hpp"
#include <px4_getopt.h>

/** driver 'main' command */
extern "C" { __EXPORT int bmm150_main(int argc, char *argv[]); }


namespace bmm150
{

BMM150 *g_dev_ext = nullptr;
BMM150 *g_dev_int = nullptr;;

void start(bool, enum Rotation);
void info(bool);
void regdump(bool external_bus);
void usage();

/**
 * Start the driver.
 *
 * This function only returns if the driver is up and running
 * or failed to detect the sensor.
 */
void start(bool external_bus, enum Rotation rotation)
{
	BMM150 **g_dev_ptr = (external_bus ? &g_dev_ext : &g_dev_int);

	if (*g_dev_ptr != nullptr)
		/* if already started, the still command succeeded */
	{
		PX4_ERR("already started");
		exit(0);
	}

	/* create the driver */
	if (external_bus) {
#if defined(PX4_I2C_BUS_EXPANSION)
		*g_dev_ptr = new BMM150(PX4_I2C_BUS_EXPANSION, rotation);
#else
		PX4_ERR("External I2C not available");
		exit(0);
#endif

	} else {
#if defined(PX4_I2C_BUS_ONBOARD)
		*g_dev_ptr = new BMM150(PX4_I2C_BUS_ONBOARD, rotation);
#else
		PX4_ERR("Internal I2C not available");
		exit(0);
#endif
	}

	if (*g_dev_ptr == nullptr) {
		goto fail;
	}


	if (OK != (*g_dev_ptr)->init()) {
		goto fail;
	}

	exit(0);
fail:

	if (*g_dev_ptr != nullptr) {
		delete (*g_dev_ptr);
		*g_dev_ptr = nullptr;
	}

	PX4_ERR("driver start failed");
	exit(1);

}

void
info(bool external_bus)
{
	BMM150 **g_dev_ptr = external_bus ? &g_dev_ext : &g_dev_int;

	if (*g_dev_ptr == nullptr) {
		PX4_ERR("driver not running");
		exit(1);
	}

	printf("state @ %p\n", *g_dev_ptr);
	(*g_dev_ptr)->print_info();

	exit(0);

}

/**
 * Dump the register information
 */
void
regdump(bool external_bus)
{
	BMM150 **g_dev_ptr = external_bus ? &g_dev_ext : &g_dev_int;

	if (*g_dev_ptr == nullptr) {
		PX4_ERR("driver not running");
		exit(1);
	}

	printf("regdump @ %p\n", *g_dev_ptr);
	(*g_dev_ptr)->print_registers();

	exit(0);
}

void
usage()
{
	PX4_WARN("missing command: try 'start', 'info', 'test', 'stop',\n'reset', 'regdump'");
	PX4_WARN("options:");
	PX4_WARN("    -X    (external bus)");
}

} // namespace bmm150

BMM150::BMM150(int bus, enum Rotation rotation) :
	I2C("BMM150", nullptr, bus, BMM150_SLAVE_ADDRESS, BMM150_BUS_SPEED),
	ScheduledWorkItem(px4::device_bus_to_wq(get_device_id())),
	_px4_mag(get_device_id(), ORB_PRIO_DEFAULT, rotation),
	_sample_perf(perf_alloc(PC_ELAPSED, "bmm150: read")),
	_bad_transfers(perf_alloc(PC_COUNT, "bmm150: bad_transfers")),
	_good_transfers(perf_alloc(PC_COUNT, "bmm150: good_transfers")),
	_measure_perf(perf_alloc(PC_ELAPSED, "bmp280: measure")),
	_comms_errors(perf_alloc(PC_COUNT, "bmp280: comms errors")),
	_duplicates(perf_alloc(PC_COUNT, "bmm150: duplicates"))
{
	_px4_mag.set_device_type(DRV_MAG_DEVTYPE_BMM150);
	_px4_mag.set_scale(0.01f);
}

BMM150::~BMM150()
{
	/* make sure we are truly inactive */
	stop();

	/* delete the perf counter */
	perf_free(_sample_perf);
	perf_free(_bad_transfers);
	perf_free(_good_transfers);
	perf_free(_measure_perf);
	perf_free(_comms_errors);
	perf_free(_duplicates);
}

int
BMM150::init()
{
	/* do I2C init (and probe) first */
	int ret = I2C::init();

	/* if probe/setup failed, bail now */
	if (ret != OK) {
		DEVICE_DEBUG("I2C setup failed");
		return ret;
	}

	/* Bring the device to sleep mode */
	modify_reg(BMM150_POWER_CTRL_REG, 1, 1);
	up_udelay(10000);

	/* check  id*/
	if (read_reg(BMM150_CHIP_ID_REG) != BMM150_CHIP_ID) {
		PX4_WARN("id of magnetometer is not: 0x%02x", BMM150_CHIP_ID);
		return -EIO;
	}

	if (reset() != OK) {
		goto out;
	}

	init_trim_registers();

out:
	return ret;
}

int
BMM150::probe()
{
	/* During I2C Initialization, sensor is in suspend mode
	 * and chip Id will return 0x00, hence returning OK. After
	 * I2C initialization, sensor is brought to sleep mode
	 * and Chip Id is verified. In sleep mode, we can read
	 * chip id. */

	/* @Note: Please read BMM150 Datasheet for more details */
	return OK;
}

void
BMM150::start()
{
	/* reset the report ring and state machine */
	_collect_phase = false;
	_running = true;

	/* schedule a cycle to start things */
	ScheduleNow();
}

void
BMM150::stop()
{
	_running = false;
	ScheduleClear();
}

void
BMM150::Run()
{
	if (_collect_phase) {
		collect();
		unsigned wait_gap = _call_interval - BMM150_CONVERSION_INTERVAL;

		if ((wait_gap != 0) && (_running)) {
			// need to wait some time before new measurement
			ScheduleDelayed(wait_gap);

			return;
		}
	}

	measure();

	if (_running) {
		/* schedule a fresh cycle call when the measurement is done */
		ScheduleDelayed(BMM150_CONVERSION_INTERVAL);
	}
}

int
BMM150::measure()
{
	_collect_phase = true;

	perf_begin(_measure_perf);

	/* start measure */
	int ret = set_power_mode(BMM150_FORCED_MODE);

	if (ret != OK) {
		perf_count(_comms_errors);
		perf_cancel(_measure_perf);
		return -EIO;
	}

	perf_end(_measure_perf);

	return OK;
}

int
BMM150::collect()
{
	_collect_phase = false;

	uint8_t mag_data[8] {};
	uint8_t status = 0;
	uint16_t resistance, lsb, msb, msblsb;

	/* start collecting data */
	perf_begin(_sample_perf);

	status = read_reg(BMM150_R_LSB);

	/* Read Magnetometer data*/
	const hrt_abstime timestamp_sample = hrt_absolute_time();

	if (OK != get_data(BMM150_DATA_X_LSB_REG, mag_data, sizeof(mag_data))) {
		return -EIO;
	}

	/* Array holding the mag XYZ and R data
	mag_data[0] - X LSB
	mag_data[1] - X MSB
	mag_data[2] - Y LSB
	mag_data[3] - Y MSB
	mag_data[4] - Z LSB
	mag_data[5] - Z MSB
	mag_data[6] - R LSB
	mag_data[7] - R MSB
	*/

	/* Extract X axis data */
	lsb = ((mag_data[0] & 0xF8) >> 3);
	msb = (((int8_t)mag_data[1]) << 5);
	msblsb = (msb | lsb);
	int16_t x_raw = (int16_t)msblsb;

	/* Extract Y axis data */
	lsb = ((mag_data[2] & 0xF8) >> 3);
	msb = (((int8_t)mag_data[3]) << 5);
	msblsb = (msb | lsb);
	int16_t y_raw = (int16_t)msblsb;

	/* Extract Z axis data */
	lsb = ((mag_data[4] & 0xFE) >> 1);
	msb = (((int8_t)mag_data[5]) << 7);
	msblsb = (msb | lsb);
	int16_t z_raw = (int16_t)msblsb;

	/* Extract Resistance data */
	lsb = ((mag_data[6] & 0xFC) >> 2);
	msb = (mag_data[7] << 6);
	msblsb = (msb | lsb);
	resistance = (uint16_t)msblsb;

	/* Check whether data is new or old */
	if (!(status & 0x01)) {
		perf_end(_sample_perf);
		perf_count(_duplicates);
		_got_duplicate = true;
		return -EIO;
	}

	_got_duplicate = false;

	if (x_raw == 0 &&
	    y_raw == 0 &&
	    z_raw == 0 &&
	    resistance == 0) {

		// all zero data - probably a I2C bus error
		perf_count(_comms_errors);
		perf_count(_bad_transfers);
		perf_end(_sample_perf);
		return -EIO;
	}

	perf_count(_good_transfers);

	float x = 0.0f;
	float y = 0.0f;
	float z = 0.0f;

	/* Compensation for X axis */
	if (x_raw != BMM150_FLIP_OVERFLOW_ADCVAL) {
		/* no overflow */
		if ((resistance != 0) && (dig_xyz1 != 0)) {
			x = ((dig_xyz1 * 16384.0 / resistance) - 16384.0);
			x = (((x_raw * ((((dig_xy2 * ((float)(x * x / (float)268435456.0)) + x * dig_xy1 /
					   (float)16384.0)) + (float)256.0) * (dig_x2 + (float)160.0))) / (float)8192.0) + (dig_x1 * (float)8.0)) / (float)16.0;

		} else {
			x = BMM150_OVERFLOW_OUTPUT_FLOAT;
		}

	} else {
		x = BMM150_OVERFLOW_OUTPUT_FLOAT;
	}

	/* Compensation for Y axis */
	if (y_raw != BMM150_FLIP_OVERFLOW_ADCVAL) {
		/* no overflow */
		if ((resistance != 0) && (dig_xyz1 != 0)) {

			y = ((((float)dig_xyz1) * (float)16384.0 / resistance) - (float)16384.0);
			y = (((y_raw * ((((dig_xy2 * (y * y / (float)268435456.0) + y * dig_xy1 / (float)16384.0)) +
					 (float)256.0) * (dig_y2 + (float)160.0))) / (float)8192.0) + (dig_y1 * (float)8.0)) / (float)16.0;


		} else {
			y = BMM150_OVERFLOW_OUTPUT_FLOAT;
		}

	} else {
		/* overflow, set output to 0.0f */
		y = BMM150_OVERFLOW_OUTPUT_FLOAT;
	}

	/* Compensation for Z axis */
	if (z_raw != BMM150_HALL_OVERFLOW_ADCVAL) {
		/* no overflow */
		if ((dig_z2 != 0) && (dig_z1 != 0) && (dig_xyz1 != 0) && (resistance != 0)) {
			z = ((((z_raw - dig_z4) * (float)131072.0) - (dig_z3 * (resistance - dig_xyz1))) / ((
						dig_z2 + dig_z1 * resistance / (float)32768.0) * (float)4.0)) / (float)16.0;
		}

	} else {
		/* overflow, set output to 0.0f */
		z = BMM150_OVERFLOW_OUTPUT_FLOAT;
	}

	// report the error count as the number of bad transfers.
	// This allows the higher level code to decide if it
	// should use this sensor based on whether it has had failures
	_px4_mag.set_error_count(perf_event_count(_bad_transfers));

	_px4_mag.update(timestamp_sample, x, y, z);

	perf_end(_sample_perf);

	return OK;
}

int BMM150::reset()
{
	/* Soft-reset */
	modify_reg(BMM150_POWER_CTRL_REG, BMM150_SOFT_RESET_MASK, BMM150_SOFT_RESET_VALUE);
	usleep(5000);

	/* Enable Magnetometer in normal mode */
	int ret = set_power_mode(BMM150_DEFAULT_POWER_MODE);
	usleep(1000);

	/* Set the data rate to default */
	ret += set_data_rate(BMM150_DEFAULT_ODR);

	/* set the preset mode as regular*/
	ret += set_presetmode(BMM150_PRESETMODE_REGULAR);

	return OK;
}

uint8_t
BMM150::read_reg(uint8_t reg)
{
	const uint8_t cmd = reg;
	uint8_t ret;
	transfer(&cmd, 1, &ret, 1);
	return ret;
}

int
BMM150::write_reg(uint8_t reg, uint8_t value)
{
	const uint8_t cmd[2] = { reg, value};
	return transfer(cmd, 2, nullptr, 0);
}

int
BMM150::get_data(uint8_t reg, uint8_t *data, unsigned len)
{
	const uint8_t cmd = reg;
	return transfer(&cmd, 1, data, len);
}

void
BMM150::modify_reg(unsigned reg, uint8_t clearbits, uint8_t setbits)
{
	uint8_t val = read_reg(reg);
	val &= ~clearbits;
	val |= setbits;
	write_reg(reg, val);
}

int BMM150 :: set_power_mode(uint8_t power_mode)
{
	uint8_t setbits = 0;
	uint8_t clearbits = BMM150_POWER_MASK;

	switch (power_mode) {
	case BMM150_NORMAL_MODE:
		_power = 0;
		break;

	case BMM150_FORCED_MODE:
		_power = 1;
		break;

	case BMM150_SLEEP_MODE:
		_power = 3;
		break;

	default:
		return -EINVAL;
	}

	setbits |= power_mode;
	modify_reg(BMM150_CTRL_REG, clearbits, setbits);

	return OK;

}

int BMM150 :: set_data_rate(uint8_t data_rate)
{
	uint8_t setbits = 0;
	uint8_t clearbits = BMM150_OUTPUT_DATA_RATE_MASK;

	switch (data_rate) {
	case BMM150_DATA_RATE_10HZ:
		_output_data_rate = 10;
		break;

	case BMM150_DATA_RATE_02HZ:
		_output_data_rate = 2;
		break;

	case BMM150_DATA_RATE_06HZ:
		_output_data_rate = 6;
		break;

	case BMM150_DATA_RATE_08HZ:
		_output_data_rate = 8;
		break;

	case BMM150_DATA_RATE_15HZ:
		_output_data_rate = 15;
		break;

	case BMM150_DATA_RATE_20HZ:
		_output_data_rate = 20;
		break;

	case BMM150_DATA_RATE_25HZ:
		_output_data_rate = 25;
		break;

	case BMM150_DATA_RATE_30HZ:
		_output_data_rate = 30;
		break;

	default:
		return -EINVAL;
	}

	setbits |= data_rate;
	modify_reg(BMM150_CTRL_REG, clearbits, setbits);

	return OK;

}


int BMM150 :: init_trim_registers(void)
{
	int ret = OK;
	uint8_t data[2] = {0};
	uint16_t msb, lsb, msblsb;

	dig_x1 = read_reg(BMM150_DIG_X1);
	dig_y1 = read_reg(BMM150_DIG_Y1);
	dig_x2 = read_reg(BMM150_DIG_X2);
	dig_y2 = read_reg(BMM150_DIG_Y2);
	dig_xy1 = read_reg(BMM150_DIG_XY1);
	dig_xy2 = read_reg(BMM150_DIG_XY2);

	ret += get_data(BMM150_DIG_Z1_LSB, data, 2);
	lsb = data[0];
	msb = (data[1] << 8);
	msblsb = (msb | lsb);
	dig_z1 = (uint16_t)msblsb;

	ret += get_data(BMM150_DIG_Z2_LSB, data, 2);
	lsb = data[0];
	msb = ((int8_t)data[1] << 8);
	msblsb = (msb | lsb);
	dig_z2 = (int16_t)msblsb;

	ret += get_data(BMM150_DIG_Z3_LSB, data, 2);
	lsb = data[0];
	msb = ((int8_t)data[1] << 8);
	msblsb = (msb | lsb);
	dig_z3 = (int16_t)msblsb;

	ret += get_data(BMM150_DIG_Z4_LSB, data, 2);
	lsb = data[0];
	msb = ((int8_t)data[1] << 8);
	msblsb = (msb | lsb);
	dig_z4 = (int16_t)msblsb;

	ret += get_data(BMM150_DIG_XYZ1_LSB, data, 2);
	lsb = data[0];
	msb = ((data[1] & 0x7F) << 8);
	msblsb = (msb | lsb);
	dig_xyz1 = (uint16_t)msblsb;

	return ret;

}

int BMM150::set_rep_xy(uint8_t rep_xy)
{
	uint8_t cmd[2] = {BMM150_XY_REP_CTRL_REG, rep_xy};

	return transfer((const uint8_t *)cmd, sizeof(cmd), nullptr, 0);

}

int BMM150::set_rep_z(uint8_t rep_z)
{
	uint8_t cmd[2] = {BMM150_Z_REP_CTRL_REG, rep_z};

	return transfer((const uint8_t *)cmd, sizeof(cmd), nullptr, 0);

}


int BMM150::set_presetmode(uint8_t presetmode)
{
	int ret = OK;
	uint8_t data_rate, rep_xy, rep_z;

	if (presetmode == 1) {
		data_rate = BMM150_LOWPOWER_DR;
		rep_xy = BMM150_LOWPOWER_REPXY;
		rep_z = BMM150_LOWPOWER_REPZ;

	} else if (presetmode == 2) {
		data_rate = BMM150_REGULAR_DR;
		rep_xy = BMM150_REGULAR_REPXY;
		rep_z = BMM150_REGULAR_REPZ;

	} else if (presetmode == 3) {
		data_rate = BMM150_HIGHACCURACY_DR;
		rep_xy = BMM150_HIGHACCURACY_REPXY;
		rep_z = BMM150_HIGHACCURACY_REPZ;

	} else if (presetmode == 4) {
		data_rate = BMM150_ENHANCED_DR;
		rep_xy = BMM150_ENHANCED_REPXY;
		rep_z = BMM150_ENHANCED_REPZ;

	} else {
		return -EINVAL;
	}

	ret = set_data_rate(data_rate);

	ret += set_rep_xy(rep_xy);
	ret += set_rep_z(rep_z);

	return ret;
}

void
BMM150::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_bad_transfers);
	perf_print_counter(_good_transfers);

	_px4_mag.print_status();
}

void
BMM150::print_registers()
{
	printf("BMM150 registers\n");

	uint8_t reg = BMM150_CHIP_ID_REG;
	uint8_t v = read_reg(reg);
	printf("Chip Id: %02x:%02x ", (unsigned)reg, (unsigned)v);
	printf("\n");

	reg = BMM150_INT_SETT_CTRL_REG;
	v = read_reg(reg);
	printf("Int sett Ctrl reg: %02x:%02x ", (unsigned)reg, (unsigned)v);
	printf("\n");

	reg = BMM150_AXES_EN_CTRL_REG;
	v = read_reg(reg);
	printf("Axes En Ctrl reg: %02x:%02x ", (unsigned)reg, (unsigned)v);
	printf("\n");
}

int
bmm150_main(int argc, char *argv[])
{
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;
	bool external_bus = false;
	enum Rotation rotation = ROTATION_NONE;

	while ((ch = px4_getopt(argc, argv, "XR:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'X':
			external_bus = true;
			break;

		case 'R':
			rotation = (enum Rotation)atoi(myoptarg);
			break;

		default:
			bmm150::usage();
			return 0;
		}
	}

	if (myoptind >= argc) {
		bmm150::usage();
		return -1;
	}

	const char *verb = argv[myoptind];

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(verb, "start")) {
		bmm150::start(external_bus, rotation);
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(verb, "info")) {
		bmm150::info(external_bus);
	}

	/*
	 * Print register information.
	 */
	if (!strcmp(verb, "regdump")) {
		bmm150::regdump(external_bus);
	}

	bmm150::usage();
	return -1;
}
