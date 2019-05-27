/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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
 * @file hmc5883.cpp
 *
 * Driver for the HMC5883 / HMC5983 magnetometer connected via I2C or SPI.
 */
#include <drivers/device/Device.hpp>
#include <drivers/drv_hrt.h>
#include <lib/conversion/rotation.h>
#include <lib/drivers/magnetometer/PX4Magnetometer.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_config.h>
#include <px4_defines.h>
#include <px4_getopt.h>
#include <px4_time.h>
#include <px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/uORB.h>

#include <float.h>

#include "hmc5883.h"

/*
 * HMC5883 internal constants and data structures.
 */

/* Max measurement rate is 160Hz, however with 160 it will be set to 166 Hz, therefore workaround using 150 */
#define HMC5883_CONVERSION_INTERVAL	(1000000 / 150)	/* microseconds */

#define ADDR_CONF_A			0x00
#define ADDR_CONF_B			0x01
#define ADDR_MODE			0x02
#define ADDR_DATA_OUT_X_MSB		0x03
#define ADDR_DATA_OUT_X_LSB		0x04
#define ADDR_DATA_OUT_Z_MSB		0x05
#define ADDR_DATA_OUT_Z_LSB		0x06
#define ADDR_DATA_OUT_Y_MSB		0x07
#define ADDR_DATA_OUT_Y_LSB		0x08

/* temperature on hmc5983 only */
#define ADDR_TEMP_OUT_MSB		0x31
#define ADDR_TEMP_OUT_LSB		0x32

#define MODE_REG_SINGLE_MODE		(1 << 0) /* default */

#define HMC5983_TEMP_SENSOR_ENABLE	(1 << 7)

enum HMC5883_BUS {
	HMC5883_BUS_ALL = 0,
	HMC5883_BUS_I2C_INTERNAL,
	HMC5883_BUS_I2C_EXTERNAL,
	HMC5883_BUS_SPI
};

class HMC5883 : public px4::ScheduledWorkItem
{
public:
	HMC5883(device::Device *interface, enum Rotation rotation);
	virtual ~HMC5883();

	int			init();

	/**
	 * Stop the automatic measurement state machine.
	 */
	void			stop();

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void			print_info();

protected:
	device::Device		*_interface;

private:

	PX4Magnetometer		_px4_mag;

	unsigned		_measure_interval{HMC5883_CONVERSION_INTERVAL};

	bool			_collect_phase{false};

	float 			_range_ga;

	perf_counter_t		_sample_perf;
	perf_counter_t		_comms_errors;
	perf_counter_t		_range_errors;
	perf_counter_t		_conf_errors;

	/* status reporting */
	bool			_sensor_ok{false};		/**< sensor was found and reports ok */
	bool			_calibrated{false};		/**< the calibration is valid */

	uint8_t			_range_bits{0};
	uint8_t			_conf_reg{0};
	uint8_t			_temperature_counter{0};
	uint8_t			_temperature_error_count{0};

	/**
	 * Initialise the automatic measurement state machine and start it.
	 *
	 * @note This function is called at open and error time.  It might make sense
	 *       to make it more aggressive about resetting the bus in case of errors.
	 */
	void			start();

	/**
	 * Reset the device
	 */
	int			reset();

	/**
	 * Perform the on-sensor scale calibration routine.
	 *
	 * @note The sensor will continue to provide measurements, these
	 *	 will however reflect the uncalibrated sensor state until
	 *	 the calibration routine has been completed.
	 *
	 * @param enable set to 1 to enable self-test strap, 0 to disable
	 */
	int			calibrate(unsigned enable);

	/**
	 * Perform the on-sensor scale calibration routine.
	 *
	 * @note The sensor will continue to provide measurements, these
	 *	 will however reflect the uncalibrated sensor state until
	 *	 the calibration routine has been completed.
	 *
	 * @param enable set to 1 to enable self-test positive strap, -1 to enable
	 *        negative strap, 0 to set to normal mode
	 */
	int			set_excitement(unsigned enable);

	/**
	 * enable hmc5983 temperature compensation
	 */
	int			set_temperature_compensation(unsigned enable);

	/**
	 * Set the sensor range.
	 *
	 * Sets the internal range to handle at least the argument in Gauss.
	 */
	int 			set_range(unsigned range);

	/**
	 * check the sensor range.
	 *
	 * checks that the range of the sensor is correctly set, to
	 * cope with communication errors causing the range to change
	 */
	void 			check_range(void);

	/**
	 * check the sensor configuration.
	 *
	 * checks that the config of the sensor is correctly set, to
	 * cope with communication errors causing the configuration to
	 * change
	 */
	void 			check_conf(void);

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
	 * Write a register.
	 *
	 * @param reg		The register to write.
	 * @param val		The value to write.
	 * @return		OK on write success.
	 */
	int			write_reg(uint8_t reg, uint8_t val);

	/**
	 * Read a register.
	 *
	 * @param reg		The register to read.
	 * @param val		The value read.
	 * @return		OK on read success.
	 */
	int			read_reg(uint8_t reg, uint8_t &val);

	/**
	 * Issue a measurement command.
	 *
	 * @return		OK if the measurement command was successful.
	 */
	int			measure();

	/**
	 * Collect the result of the most recent measurement.
	 */
	int			collect();

	/**
	 * Convert a big-endian signed 16-bit value to a float.
	 *
	 * @param in		A signed 16-bit big-endian value.
	 * @return		The floating-point representation of the value.
	 */
	float			meas_to_float(uint8_t in[2]);

	/* this class has pointer data members, do not allow copying it */
	HMC5883(const HMC5883 &);
	HMC5883 operator=(const HMC5883 &);
};

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int hmc5883_main(int argc, char *argv[]);


HMC5883::HMC5883(device::Device *interface, enum Rotation rotation) :
	ScheduledWorkItem(px4::device_bus_to_wq(interface->get_device_id())),
	_interface(interface),
	_px4_mag(interface->get_device_id(), interface->external() ? ORB_PRIO_MAX : ORB_PRIO_HIGH, rotation),
	_sample_perf(perf_alloc(PC_ELAPSED, "hmc5883_read")),
	_comms_errors(perf_alloc(PC_COUNT, "hmc5883_com_err")),
	_range_errors(perf_alloc(PC_COUNT, "hmc5883_rng_err")),
	_conf_errors(perf_alloc(PC_COUNT, "hmc5883_conf_err"))
{
	_px4_mag.set_device_type(DRV_MAG_DEVTYPE_HMC5883);
}

HMC5883::~HMC5883()
{
	/* make sure we are truly inactive */
	stop();

	// free perf counters
	perf_free(_sample_perf);
	perf_free(_comms_errors);
	perf_free(_range_errors);
	perf_free(_conf_errors);
}

int
HMC5883::init()
{
	/* reset the device configuration */
	int ret = reset();

	if (ret != PX4_OK) {
		return PX4_ERROR;
	}

	start();

	return PX4_OK;
}

int HMC5883::set_range(unsigned range)
{
	float range_scale = 0.0f;

	if (range < 0.88f) {
		_range_bits = 0x00;
		range_scale = 1.0f / 1370.0f;
		_range_ga = 0.88f;

	} else if (range <= 1.3f) {
		_range_bits = 0x01;
		range_scale = 1.0f / 1090.0f;
		_range_ga = 1.3f;

	} else if (range <= 2) {
		_range_bits = 0x02;
		range_scale = 1.0f / 820.0f;
		_range_ga = 1.9f;

	} else if (range <= 3) {
		_range_bits = 0x03;
		range_scale = 1.0f / 660.0f;
		_range_ga = 2.5f;

	} else if (range <= 4) {
		_range_bits = 0x04;
		range_scale = 1.0f / 440.0f;
		_range_ga = 4.0f;

	} else if (range <= 4.7f) {
		_range_bits = 0x05;
		range_scale = 1.0f / 390.0f;
		_range_ga = 4.7f;

	} else if (range <= 5.6f) {
		_range_bits = 0x06;
		range_scale = 1.0f / 330.0f;
		_range_ga = 5.6f;

	} else {
		_range_bits = 0x07;
		range_scale = 1.0f / 230.0f;
		_range_ga = 8.1f;
	}

	_px4_mag.set_scale(range_scale);

	/*
	 * Send the command to set the range
	 */
	int ret = write_reg(ADDR_CONF_B, (_range_bits << 5));

	if (OK != ret) {
		perf_count(_comms_errors);
	}

	uint8_t range_bits_in = 0;
	ret = read_reg(ADDR_CONF_B, range_bits_in);

	if (OK != ret) {
		perf_count(_comms_errors);
	}

	return !(range_bits_in == (_range_bits << 5));
}

/**
   check that the range register has the right value. This is done
   periodically to cope with I2C bus noise causing the range of the
   compass changing.
 */
void HMC5883::check_range(void)
{
	uint8_t range_bits_in = 0;
	int ret = read_reg(ADDR_CONF_B, range_bits_in);

	if (OK != ret) {
		perf_count(_comms_errors);
		return;
	}

	if (range_bits_in != (_range_bits << 5)) {
		perf_count(_range_errors);
		ret = write_reg(ADDR_CONF_B, (_range_bits << 5));

		if (OK != ret) {
			perf_count(_comms_errors);
		}
	}
}

/**
   check that the configuration register has the right value. This is
   done periodically to cope with I2C bus noise causing the
   configuration of the compass to change.
 */
void HMC5883::check_conf(void)
{
	uint8_t conf_reg_in = 0;
	int ret = read_reg(ADDR_CONF_A, conf_reg_in);

	if (OK != ret) {
		perf_count(_comms_errors);
		return;
	}

	if (conf_reg_in != _conf_reg) {
		perf_count(_conf_errors);
		ret = write_reg(ADDR_CONF_A, _conf_reg);

		if (OK != ret) {
			perf_count(_comms_errors);
		}
	}
}

void
HMC5883::start()
{
	/* reset the report ring and state machine */
	_collect_phase = false;

	/* schedule a cycle to start things */
	ScheduleNow();
}

void
HMC5883::stop()
{
	/* ensure no new items are queued while we cancel this one */
	_measure_interval = 0;
	ScheduleClear();
}

int
HMC5883::reset()
{
	/* set range, ceil floating point number */
	return set_range(_range_ga + 0.5f);
}

void
HMC5883::Run()
{
	if (_measure_interval == 0) {
		return;
	}

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
		if (_measure_interval > HMC5883_CONVERSION_INTERVAL) {

			/* schedule a fresh cycle call when we are ready to measure again */
			ScheduleDelayed(_measure_interval - HMC5883_CONVERSION_INTERVAL);

			return;
		}
	}

	/* measurement phase */
	if (OK != measure()) {
		PX4_DEBUG("measure error");
	}

	/* next phase is collection */
	_collect_phase = true;

	if (_measure_interval > 0) {
		/* schedule a fresh cycle call when the measurement is done */
		ScheduleDelayed(HMC5883_CONVERSION_INTERVAL);
	}
}

int
HMC5883::measure()
{
	int ret;

	/*
	 * Send the command to begin a measurement.
	 */
	ret = write_reg(ADDR_MODE, MODE_REG_SINGLE_MODE);

	if (OK != ret) {
		perf_count(_comms_errors);
	}

	return ret;
}

int
HMC5883::collect()
{
#pragma pack(push, 1)
	struct { /* status register and data as read back from the device */
		uint8_t		x[2];
		uint8_t		z[2];
		uint8_t		y[2];
	}	hmc_report;
#pragma pack(pop)

	struct {
		int16_t		x, y, z;
	} report;

	perf_begin(_sample_perf);

	/* this should be fairly close to the end of the measurement, so the best approximation of the time */
	const uint64_t error_count = perf_event_count(_comms_errors);
	_px4_mag.set_error_count(error_count);

	/*
	 * @note  We could read the status register here, which could tell us that
	 *        we were too early and that the output registers are still being
	 *        written.  In the common case that would just slow us down, and
	 *        we're better off just never being early.
	 */

	/* get measurements from the device */
	const hrt_abstime timestamp_sample = hrt_absolute_time();
	int ret = _interface->read(ADDR_DATA_OUT_X_MSB, (uint8_t *)&hmc_report, sizeof(hmc_report));

	if (ret != OK) {
		perf_count(_comms_errors);
		PX4_DEBUG("data/status read error");
		return PX4_ERROR;
	}

	/* swap the data we just received */
	report.x = (((int16_t)hmc_report.x[0]) << 8) + hmc_report.x[1];
	report.y = (((int16_t)hmc_report.y[0]) << 8) + hmc_report.y[1];
	report.z = (((int16_t)hmc_report.z[0]) << 8) + hmc_report.z[1];

	/*
	 * If any of the values are -4096, there was an internal math error in the sensor.
	 * Generalise this to a simple range check that will also catch some bit errors.
	 */
	if ((abs(report.x) > 2048) ||
	    (abs(report.y) > 2048) ||
	    (abs(report.z) > 2048)) {

		perf_count(_comms_errors);
		return PX4_ERROR;
	}

	/* get measurements from the device */
	if (_conf_reg & HMC5983_TEMP_SENSOR_ENABLE) {
		/*
		  if temperature compensation is enabled read the
		  temperature too.

		  We read the temperature every 10 samples to avoid
		  excessive I2C traffic
		 */
		if (_temperature_counter++ == 10) {
			uint8_t raw_temperature[2] {};

			_temperature_counter = 0;

			ret = _interface->read(ADDR_TEMP_OUT_MSB, raw_temperature, sizeof(raw_temperature));

			if (ret == OK) {
				int16_t temp16 = (((int16_t)raw_temperature[0]) << 8) + raw_temperature[1];
				float temperature = 25 + (temp16 / (16 * 8.0f));

				_px4_mag.set_temperature(temperature);

				_temperature_error_count = 0;

			} else {
				_temperature_error_count++;

				if (_temperature_error_count == 10) {
					/*
					  it probably really is an old HMC5883,
					  and can't do temperature. Disable it
					*/
					_temperature_error_count = 0;
					PX4_DEBUG("disabling temperature compensation");
					set_temperature_compensation(0);
				}
			}
		}
	}

	/*
	 * RAW outputs
	 *
	 * to align the sensor axes with the board, x and y need to be flipped
	 * and y needs to be negated
	 */
	if (!_interface->external()) {
		// convert onboard so it matches offboard for the
		// scaling below
		report.y = -report.y;
		report.x = -report.x;
	}

	/* the standard external mag by 3DR has x pointing to the
	 * right, y pointing backwards, and z down, therefore switch x
	 * and y and invert y */
	_px4_mag.update(timestamp_sample, -report.y, report.x, report.z);

	/*
	  periodically check the range register and configuration
	  registers. With a bad I2C cable it is possible for the
	  registers to become corrupt, leading to bad readings. It
	  doesn't happen often, but given the poor cables some
	  vehicles have it is worth checking for.
	 */
	uint8_t check_counter = perf_event_count(_sample_perf) % 256;

	if (check_counter == 0) {
		check_range();
	}

	if (check_counter == 128) {
		check_conf();
	}

	ret = OK;

	perf_end(_sample_perf);
	return ret;
}

int HMC5883::calibrate(unsigned enable)
{
	//struct mag_report report;
	//ssize_t sz;
	int ret = 1;
	uint8_t good_count = 0;

	// struct mag_calibration_s mscale_previous;
	// mscale_previous.x_offset = 0.0f;
	// mscale_previous.x_scale = 1.0f;
	// mscale_previous.y_offset = 0.0f;
	// mscale_previous.y_scale = 1.0f;
	// mscale_previous.z_offset = 0.0f;
	// mscale_previous.z_scale = 1.0f;

	// struct mag_calibration_s mscale_null;
	// mscale_null.x_offset = 0.0f;
	// mscale_null.x_scale = 1.0f;
	// mscale_null.y_offset = 0.0f;
	// mscale_null.y_scale = 1.0f;
	// mscale_null.z_offset = 0.0f;
	// mscale_null.z_scale = 1.0f;

	//float sum_excited[3] = {0.0f, 0.0f, 0.0f};

	/* expected axis scaling. The datasheet says that 766 will
	 * be places in the X and Y axes and 713 in the Z
	 * axis. Experiments show that in fact 766 is placed in X,
	 * and 713 in Y and Z. This is relative to a base of 660
	 * LSM/Ga, giving 1.16 and 1.08 */
	//float expected_cal[3] = { 1.16f, 1.08f, 1.08f };

	/* start the sensor polling at 50 Hz */
	// TODO: dagar

	/* Set to 2.5 Gauss. We ask for 3 to get the right part of
	 * the chained if statement above. */
	// TODO: dagar

	// TODO: dagar MAGIOCEXSTRAP

	// TODO: dagar MAGIOCGSCALE

	// TODO: dagar MAGIOCSSCALE

	// discard 10 samples to let the sensor settle
	for (uint8_t i = 0; i < 10; i++) {
		// TODO: read?
	}

	/* read the sensor up to 150x, stopping when we have 50 good values */
	for (uint8_t i = 0; i < 150 && good_count < 50; i++) {

		/* wait for data to be ready */

		// TODO: read?

		// float cal[3] = {fabsf(expected_cal[0] / report.x),
		// 		fabsf(expected_cal[1] / report.y),
		// 		fabsf(expected_cal[2] / report.z)
		// 	       };

		// if (cal[0] > 0.3f && cal[0] < 1.7f &&
		//     cal[1] > 0.3f && cal[1] < 1.7f &&
		//     cal[2] > 0.3f && cal[2] < 1.7f) {
		// 	good_count++;
		// 	sum_excited[0] += cal[0];
		// 	sum_excited[1] += cal[1];
		// 	sum_excited[2] += cal[2];
		// }
	}

	if (good_count < 5) {
		ret = -EIO;
		goto out;
	}

	// float scaling[3];

	// scaling[0] = sum_excited[0] / good_count;
	// scaling[1] = sum_excited[1] / good_count;
	// scaling[2] = sum_excited[2] / good_count;

	/* set scaling in device */
	// mscale_previous.x_scale = 1.0f / scaling[0];
	// mscale_previous.y_scale = 1.0f / scaling[1];
	// mscale_previous.z_scale = 1.0f / scaling[2];

	ret = OK;


	_px4_mag.set_sensitivity(1.0f, 1.0f, 1.0f);

out:
	// TODO: dagar MAGIOCSSCALE

	/* set back to normal mode */
	/* Set to 1.9 Gauss */

	// TODO: dagar MAGIOCSRANGE

	// TODO: dagar MAGIOCEXSTRAP

	return ret;
}

int HMC5883::set_excitement(unsigned enable)
{
	int ret;
	/* arm the excitement strap */
	ret = read_reg(ADDR_CONF_A, _conf_reg);

	if (OK != ret) {
		perf_count(_comms_errors);
	}

	_conf_reg &= ~0x03; // reset previous excitement mode

	if (((int)enable) < 0) {
		_conf_reg |= 0x01;

	} else if (enable > 0) {
		_conf_reg |= 0x02;

	}

	// ::printf("set_excitement enable=%d regA=0x%x\n", (int)enable, (unsigned)_conf_reg);

	ret = write_reg(ADDR_CONF_A, _conf_reg);

	if (OK != ret) {
		perf_count(_comms_errors);
	}

	uint8_t conf_reg_ret = 0;
	read_reg(ADDR_CONF_A, conf_reg_ret);

	//print_info();

	return !(_conf_reg == conf_reg_ret);
}


/*
  enable/disable temperature compensation on the HMC5983

  Unfortunately we don't yet know of a way to auto-detect the
  difference between the HMC5883 and HMC5983. Both of them do
  temperature sensing, but only the 5983 does temperature
  compensation. We have noy yet found a behaviour that can be reliably
  distinguished by reading registers to know which type a particular
  sensor is

  update: Current best guess is that many sensors marked HMC5883L on
  the package are actually 5983 but without temperature compensation
  tables. Reading the temperature works, but the mag field is not
  automatically adjusted for temperature. We suspect that there may be
  some early 5883L parts that don't have the temperature sensor at
  all, although we haven't found one yet. The code that reads the
  temperature looks for 10 failed transfers in a row and disables the
  temperature sensor if that happens. It is hoped that this copes with
  the genuine 5883L parts.
 */
int HMC5883::set_temperature_compensation(unsigned enable)
{
	int ret;
	/* get current config */
	ret = read_reg(ADDR_CONF_A, _conf_reg);

	if (OK != ret) {
		perf_count(_comms_errors);
		return -EIO;
	}

	if (enable != 0) {
		_conf_reg |= HMC5983_TEMP_SENSOR_ENABLE;

	} else {
		_conf_reg &= ~HMC5983_TEMP_SENSOR_ENABLE;
	}

	ret = write_reg(ADDR_CONF_A, _conf_reg);

	if (OK != ret) {
		perf_count(_comms_errors);
		return -EIO;
	}

	uint8_t conf_reg_ret = 0;

	if (read_reg(ADDR_CONF_A, conf_reg_ret) != OK) {
		perf_count(_comms_errors);
		return -EIO;
	}

	return conf_reg_ret == _conf_reg;
}

int
HMC5883::write_reg(uint8_t reg, uint8_t val)
{
	uint8_t buf = val;
	return _interface->write(reg, &buf, 1);
}

int
HMC5883::read_reg(uint8_t reg, uint8_t &val)
{
	uint8_t buf = val;
	int ret = _interface->read(reg, &buf, 1);
	val = buf;
	return ret;
}

float
HMC5883::meas_to_float(uint8_t in[2])
{
	union {
		uint8_t	b[2];
		int16_t	w;
	} u;

	u.b[0] = in[1];
	u.b[1] = in[0];

	return (float) u.w;
}

void
HMC5883::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	printf("interval:  %u us\n", _measure_interval);

	_px4_mag.print_status();
}

/**
 * Local functions in support of the shell command.
 */
namespace hmc5883
{

/*
  list of supported bus configurations
 */
struct hmc5883_bus_option {
	enum HMC5883_BUS busid;
	HMC5883_constructor interface_constructor;
	uint8_t busnum;
	HMC5883	*dev;
} bus_options[] = {
	{ HMC5883_BUS_I2C_EXTERNAL, &HMC5883_I2C_interface, PX4_I2C_BUS_EXPANSION, NULL },
#ifdef PX4_I2C_BUS_EXPANSION1
	{ HMC5883_BUS_I2C_EXTERNAL, &HMC5883_I2C_interface, PX4_I2C_BUS_EXPANSION1, NULL },
#endif
#ifdef PX4_I2C_BUS_EXPANSION2
	{ HMC5883_BUS_I2C_EXTERNAL, &HMC5883_I2C_interface, PX4_I2C_BUS_EXPANSION2, NULL },
#endif
#ifdef PX4_I2C_BUS_ONBOARD
	{ HMC5883_BUS_I2C_INTERNAL, &HMC5883_I2C_interface, PX4_I2C_BUS_ONBOARD, NULL },
#endif
#ifdef PX4_SPIDEV_HMC
	{ HMC5883_BUS_SPI, &HMC5883_SPI_interface, PX4_SPI_BUS_SENSORS, NULL },
#endif
};
#define NUM_BUS_OPTIONS (sizeof(bus_options)/sizeof(bus_options[0]))

void	start(enum HMC5883_BUS busid, enum Rotation rotation);
int	stop();
bool	start_bus(struct hmc5883_bus_option &bus, enum Rotation rotation);
struct hmc5883_bus_option &find_bus(enum HMC5883_BUS busid);
int	info(enum HMC5883_BUS busid);
int	calibrate(enum HMC5883_BUS busid);
int	temp_enable(HMC5883_BUS busid, bool enable);
void	usage();

/**
 * start driver for a specific bus option
 */
bool
start_bus(struct hmc5883_bus_option &bus, enum Rotation rotation)
{
	if (bus.dev != nullptr) {
		errx(1, "bus option already started");
	}

	device::Device *interface = bus.interface_constructor(bus.busnum);

	if (interface->init() != OK) {
		delete interface;
		warnx("no device on bus %u (type: %u)", (unsigned)bus.busnum, (unsigned)bus.busid);
		return false;
	}

	bus.dev = new HMC5883(interface, rotation);

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
start(enum HMC5883_BUS busid, enum Rotation rotation)
{
	bool started = false;

	for (unsigned i = 0; i < NUM_BUS_OPTIONS; i++) {
		if (busid == HMC5883_BUS_ALL && bus_options[i].dev != NULL) {
			// this device is already started
			continue;
		}

		if (busid != HMC5883_BUS_ALL && bus_options[i].busid != busid) {
			// not the one that is asked for
			continue;
		}

		started |= start_bus(bus_options[i], rotation);
	}

	if (!started) {
		exit(1);
	}
}

int
stop()
{
	bool stopped = false;

	for (unsigned i = 0; i < NUM_BUS_OPTIONS; i++) {
		if (bus_options[i].dev != nullptr) {
			bus_options[i].dev->stop();
			delete bus_options[i].dev;
			bus_options[i].dev = nullptr;
			stopped = true;
		}
	}

	return !stopped;
}

/**
 * find a bus structure for a busid
 */
struct hmc5883_bus_option &find_bus(enum HMC5883_BUS busid)
{
	for (unsigned i = 0; i < NUM_BUS_OPTIONS; i++) {
		if ((busid == HMC5883_BUS_ALL ||
		     busid == bus_options[i].busid) && bus_options[i].dev != NULL) {
			return bus_options[i];
		}
	}

	errx(1, "bus %u not started", (unsigned)busid);
}

/**
 * Automatic scale calibration.
 *
 * Basic idea:
 *
 *   output = (ext field +- 1.1 Ga self-test) * scale factor
 *
 * and consequently:
 *
 *   1.1 Ga = (excited - normal) * scale factor
 *   scale factor = (excited - normal) / 1.1 Ga
 *
 *   sxy = (excited - normal) / 766	| for conf reg. B set to 0x60 / Gain = 3
 *   sz  = (excited - normal) / 713	| for conf reg. B set to 0x60 / Gain = 3
 *
 * By subtracting the non-excited measurement the pure 1.1 Ga reading
 * can be extracted and the sensitivity of all axes can be matched.
 *
 * SELF TEST OPERATION
 * To check the HMC5883L for proper operation, a self test feature in incorporated
 * in which the sensor offset straps are excited to create a nominal field strength
 * (bias field) to be measured. To implement self test, the least significant bits
 * (MS1 and MS0) of configuration register A are changed from 00 to 01 (positive bias)
 * or 10 (negetive bias), e.g. 0x11 or 0x12.
 * Then, by placing the mode register into single-measurement mode (0x01),
 * two data acquisition cycles will be made on each magnetic vector.
 * The first acquisition will be a set pulse followed shortly by measurement
 * data of the external field. The second acquisition will have the offset strap
 * excited (about 10 mA) in the positive bias mode for X, Y, and Z axes to create
 * about a ±1.1 gauss self test field plus the external field. The first acquisition
 * values will be subtracted from the second acquisition, and the net measurement
 * will be placed into the data output registers.
 * Since self test adds ~1.1 Gauss additional field to the existing field strength,
 * using a reduced gain setting prevents sensor from being saturated and data registers
 * overflowed. For example, if the configuration register B is set to 0x60 (Gain=3),
 * values around +766 LSB (1.16 Ga * 660 LSB/Ga) will be placed in the X and Y data
 * output registers and around +713 (1.08 Ga * 660 LSB/Ga) will be placed in Z data
 * output register. To leave the self test mode, change MS1 and MS0 bit of the
 * configuration register A back to 00 (Normal Measurement Mode), e.g. 0x10.
 * Using the self test method described above, the user can scale sensor
 */
int calibrate(enum HMC5883_BUS busid)
{
	int ret = PX4_OK;
	struct hmc5883_bus_option &bus = find_bus(busid);

	if (bus.dev) {
		//bus.dev->calibrate();
	}

	return ret;
}

/**
 * enable/disable temperature compensation
 */
int
temp_enable(enum HMC5883_BUS busid, bool enable)
{
	//struct hmc5883_bus_option &bus = find_bus(busid);
	//const char *path = bus.devpath;

	// call MAGIOCSTEMPCOMP

	return 0;
}

/**
 * Print a little info about the driver.
 */
int
info(enum HMC5883_BUS busid)
{
	struct hmc5883_bus_option &bus = find_bus(busid);

	PX4_INFO("running on bus: %u\n", (unsigned)bus.busid);
	bus.dev->print_info();
	exit(0);
}

void
usage()
{
	warnx("missing command: try 'start', 'info', info', 'calibrate'");
	warnx("options:");
	warnx("    -R rotation");
	warnx("    -C calibrate on start");
	warnx("    -X only external bus");
#if (PX4_I2C_BUS_ONBOARD || PX4_SPIDEV_HMC)
	warnx("    -I only internal bus");
#endif
}

} // namespace

int
hmc5883_main(int argc, char *argv[])
{
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;


	enum HMC5883_BUS busid = HMC5883_BUS_ALL;
	enum Rotation rotation = ROTATION_NONE;
	bool calibrate = false;
	bool temp_compensation = false;

	if (argc < 2) {
		hmc5883::usage();
		exit(0);
	}

	while ((ch = px4_getopt(argc, argv, "XISR:CT", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'R':
			rotation = (enum Rotation)atoi(myoptarg);
			break;
#if (PX4_I2C_BUS_ONBOARD || PX4_SPIDEV_HMC)

		case 'I':
			busid = HMC5883_BUS_I2C_INTERNAL;
			break;
#endif

		case 'X':
			busid = HMC5883_BUS_I2C_EXTERNAL;
			break;

		case 'S':
			busid = HMC5883_BUS_SPI;
			break;

		case 'C':
			calibrate = true;
			break;

		case 'T':
			temp_compensation = true;
			break;

		default:
			hmc5883::usage();
			exit(0);
		}
	}

	if (myoptind >= argc) {
		hmc5883::usage();
		exit(0);
	}

	const char *verb = argv[myoptind];


	/*
	 * Start/load the driver.
	 */
	if (!strcmp(verb, "start")) {
		hmc5883::start(busid, rotation);

		if (calibrate && hmc5883::calibrate(busid) != 0) {
			errx(1, "calibration failed");
		}

		if (temp_compensation) {
			// we consider failing to setup temperature
			// compensation as non-fatal
			hmc5883::temp_enable(busid, true);
		}

		exit(0);
	}

	/*
	 * Stop the driver
	 */
	if (!strcmp(verb, "stop")) {
		return hmc5883::stop();
	}

	/*
	 * enable/disable temperature compensation
	 */
	if (!strcmp(verb, "tempoff")) {
		hmc5883::temp_enable(busid, false);
	}

	if (!strcmp(verb, "tempon")) {
		hmc5883::temp_enable(busid, true);
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(verb, "info") || !strcmp(verb, "status")) {
		hmc5883::info(busid);
	}

	/*
	 * Autocalibrate the scaling
	 */
	if (!strcmp(verb, "calibrate")) {
		if (hmc5883::calibrate(busid) == 0) {
			errx(0, "calibration successful");

		} else {
			errx(1, "calibration failed");
		}
	}

	errx(1, "unrecognized command, try 'start', 'test', 'reset' 'calibrate', 'tempoff', 'tempon' or 'info'");
}
