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
 * @file gyrosim.cpp
 *
 * Driver for the simulated gyro
 *
 * @author Andrew Tridgell
 * @author Pat Hickey
 * @author Mark Charlebois
 */

#include <inttypes.h>

#include <px4_config.h>
#include <px4_getopt.h>
#include <simulator/simulator.h>

#include <perf/perf_counter.h>
#include <systemlib/conversions.h>
#include <drivers/drv_hrt.h>
#include <drivers/device/device.h>
#include <lib/conversion/rotation.h>
#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>

#include "VirtDevObj.hpp"

using namespace DriverFramework;

#define DIR_READ			0x80
#define DIR_WRITE			0x00

#define MPU_DEVICE_PATH_ACCEL		"/dev/gyrosim_accel"
#define MPU_DEVICE_PATH_GYRO		"/dev/gyrosim_gyro"

// MPU 6000 registers
#define MPUREG_SMPLRT_DIV		0x19
#define MPUREG_CONFIG			0x1A
#define MPUREG_GYRO_CONFIG		0x1B
#define MPUREG_ACCEL_CONFIG		0x1C
#define MPUREG_INT_STATUS		0x3A
#define MPUREG_PRODUCT_ID		0x0C

// Product ID Description for GYROSIM
// high 4 bits 	low 4 bits
// Product Name	Product Revision
#define GYROSIMES_REV_C4		0x14

#define GYROSIM_ACCEL_DEFAULT_RATE	400

#define GYROSIM_GYRO_DEFAULT_RATE	400

#ifdef PX4_SPI_BUS_EXT
#define EXTERNAL_BUS PX4_SPI_BUS_EXT
#else
#define EXTERNAL_BUS 0
#endif

/*
  the GYROSIM can only handle high SPI bus speeds on the sensor and
  interrupt status registers. All other registers have a maximum 1MHz
  SPI speed
 */
class GYROSIM_gyro;

class GYROSIM : public VirtDevObj
{
public:
	GYROSIM(const char *path_accel, const char *path_gyro, enum Rotation rotation);
	virtual ~GYROSIM();

	int             	init();
	virtual int		start();

	int			transfer(uint8_t *send, uint8_t *recv, unsigned len);

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void			print_info();

	void			print_registers();

protected:
	friend class GYROSIM_gyro;

private:
	GYROSIM_gyro		*_gyro;

	PX4Accelerometer	_px4_accel;
	PX4Gyroscope		_px4_gyro;

	uint8_t			_product{GYROSIMES_REV_C4};	/** product code */

	perf_counter_t		_accel_reads;
	perf_counter_t		_gyro_reads;
	perf_counter_t		_sample_perf;
	perf_counter_t		_good_transfers;
	perf_counter_t		_reset_retries;

	/**
	 * Reset chip.
	 *
	 * Resets the chip and measurements ranges, but not scale and offset.
	 */
	int			reset();

	/**
	 * Fetch measurements from the sensor and update the report buffers.
	 */
	virtual void		_measure();

	/**
	 * Read a register from the GYROSIM
	 *
	 * @param		The register to read.
	 * @return		The value that was read.
	 */
	uint8_t			read_reg(unsigned reg);

	/**
	 * Write a register in the GYROSIM
	 *
	 * @param reg		The register to write.
	 * @param value		The new value to write.
	 */
	void			write_reg(unsigned reg, uint8_t value);

	/**
	 * Set the GYROSIM measurement range.
	 *
	 * @param max_g		The maximum G value the range must support.
	 * @return		OK if the value can be supported, -ERANGE otherwise.
	 */
	int			set_accel_range(unsigned max_g);

	/**
	 * Swap a 16-bit value read from the GYROSIM to native byte order.
	 */
	uint16_t		swap16(uint16_t val) { return (val >> 8) | (val << 8);	}

	/* do not allow to copy this class due to pointer data members */
	GYROSIM(const GYROSIM &) = delete;
	GYROSIM operator=(const GYROSIM &) = delete;

#pragma pack(push, 1)
	/**
	 * Report conversation within the GYROSIM, including command byte and
	 * interrupt status.
	 */
	struct MPUReport {
		uint8_t		cmd;
		uint8_t		status;
		float		accel_x;
		float		accel_y;
		float		accel_z;
		float		temp;
		float		gyro_x;
		float		gyro_y;
		float		gyro_z;
	};
#pragma pack(pop)

	uint8_t _regdata[108];

};

/**
 * Helper class implementing the gyro driver node.
 */
class GYROSIM_gyro  : public VirtDevObj
{
public:
	GYROSIM_gyro(GYROSIM *parent, const char *path);
	virtual ~GYROSIM_gyro() = default;

	virtual int		init();

protected:
	friend class GYROSIM;

	virtual void 		_measure() {}
private:
	GYROSIM			*_parent;

	/* do not allow to copy this class due to pointer data members */
	GYROSIM_gyro(const GYROSIM_gyro &) = delete;
	GYROSIM_gyro operator=(const GYROSIM_gyro &) = delete;
};

/** driver 'main' command */
extern "C" { __EXPORT int gyrosim_main(int argc, char *argv[]); }

GYROSIM::GYROSIM(const char *path_accel, const char *path_gyro, enum Rotation rotation) :
	VirtDevObj("GYROSIM", path_accel, ACCEL_BASE_DEVICE_PATH, 1e6 / 400),
	_gyro(new GYROSIM_gyro(this, path_gyro)),
	_px4_accel(m_id.dev_id, ORB_PRIO_DEFAULT, rotation),
	_px4_gyro(m_id.dev_id, ORB_PRIO_DEFAULT, rotation),
	_accel_reads(perf_alloc(PC_COUNT, "gyrosim_accel_read")),
	_gyro_reads(perf_alloc(PC_COUNT, "gyrosim_gyro_read")),
	_sample_perf(perf_alloc(PC_ELAPSED, "gyrosim_read")),
	_good_transfers(perf_alloc(PC_COUNT, "gyrosim_good_transfers")),
	_reset_retries(perf_alloc(PC_COUNT, "gyrosim_reset_retries"))
{
	_px4_accel.set_device_type(DRV_ACC_DEVTYPE_GYROSIM);
	_px4_gyro.set_device_type(DRV_GYR_DEVTYPE_GYROSIM);
}

GYROSIM::~GYROSIM()
{
	/* make sure we are truly inactive */
	stop();

	/* delete the perf counter */
	perf_free(_sample_perf);
	perf_free(_accel_reads);
	perf_free(_gyro_reads);
	perf_free(_good_transfers);
}

int
GYROSIM::init()
{
	int ret = VirtDevObj::init();

	if (ret != 0) {
		PX4_WARN("Base class init failed");
		return ret;
	}

	if (reset() != OK) {
		PX4_WARN("reset failed");
		return PX4_ERROR;
	}

	/* do init for the gyro device node, keep it optional */
	ret = _gyro->init();

	/* if probe/setup failed, bail now */
	if (ret != OK) {
		PX4_ERR("gyro init failed");
		return ret;
	}

	ret = start();

	if (ret != OK) {
		PX4_ERR("gyro accel start failed (%d)", ret);
		return ret;
	}

	return PX4_OK;
}

int GYROSIM::reset()
{
	return OK;
}

int
GYROSIM::transfer(uint8_t *send, uint8_t *recv, unsigned len)
{
	uint8_t cmd = send[0];
	uint8_t reg = cmd & 0x7F;
	const uint8_t MPUREAD = MPUREG_INT_STATUS | DIR_READ;

	if (cmd == MPUREAD) {
		// Get data from the simulator
		Simulator *sim = Simulator::getInstance();

		if (sim == nullptr) {
			PX4_WARN("failed accessing simulator");
			return ENODEV;
		}

		// FIXME - not sure what interrupt status should be
		recv[1] = 0;

		// skip cmd and status bytes
		if (len > 2) {
			sim->getMPUReport(&recv[2], len - 2);
		}

	} else if (cmd & DIR_READ) {
		PX4_DEBUG("Reading %u bytes from register %u", len - 1, reg);
		memcpy(&_regdata[reg - MPUREG_PRODUCT_ID], &send[1], len - 1);

	} else {
		PX4_DEBUG("Writing %u bytes to register %u", len - 1, reg);

		if (recv) {
			memcpy(&recv[1], &_regdata[reg - MPUREG_PRODUCT_ID], len - 1);
		}
	}

	return PX4_OK;
}

uint8_t
GYROSIM::read_reg(unsigned reg)
{
	uint8_t cmd[2] = { (uint8_t)(reg | DIR_READ), 0};

	// general register transfer at low clock speed
	transfer(cmd, cmd, sizeof(cmd));

	return cmd[1];
}

void
GYROSIM::write_reg(unsigned reg, uint8_t value)
{
	uint8_t	cmd[2];

	cmd[0] = reg | DIR_WRITE;
	cmd[1] = value;

	// general register transfer at low clock speed
	transfer(cmd, nullptr, sizeof(cmd));
}

int
GYROSIM::start()
{
	/* make sure we are stopped first */
	stop();

	/* start polling at the specified rate */
	return DevObj::start();
}

void
GYROSIM::_measure()
{
	MPUReport mpu_report{};

	/* start measuring */
	perf_begin(_sample_perf);

	/*
	 * Fetch the full set of measurements from the GYROSIM in one pass.
	 */
	mpu_report.cmd = DIR_READ | MPUREG_INT_STATUS;

	// sensor transfer at high clock speed
	const hrt_abstime timestamp_sample = hrt_absolute_time();

	if (OK != transfer((uint8_t *)&mpu_report, ((uint8_t *)&mpu_report), sizeof(mpu_report))) {
		return;
	}

	_px4_accel.set_temperature(mpu_report.temp);
	_px4_gyro.set_temperature(mpu_report.temp);

	_px4_accel.update(timestamp_sample, mpu_report.accel_x, mpu_report.accel_y, mpu_report.accel_z);
	_px4_gyro.update(timestamp_sample, mpu_report.gyro_x, mpu_report.gyro_y, mpu_report.gyro_z);

	/* stop measuring */
	perf_end(_sample_perf);
}

void
GYROSIM::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_accel_reads);
	perf_print_counter(_gyro_reads);
	perf_print_counter(_good_transfers);
	perf_print_counter(_reset_retries);
}

GYROSIM_gyro::GYROSIM_gyro(GYROSIM *parent, const char *path) :
	// Set sample interval to 0 since device is read by parent
	VirtDevObj("GYROSIM_gyro", path, GYRO_BASE_DEVICE_PATH, 0),
	_parent(parent)
{
}

int
GYROSIM_gyro::init()
{
	int ret = VirtDevObj::init();
	PX4_DEBUG("GYROSIM_gyro::init base class ret: %d", ret);
	return ret;
}

/**
 * Local functions in support of the shell command.
 */
namespace gyrosim
{

GYROSIM	*g_dev_sim; // on simulated bus

int	start(enum Rotation /*rotation*/);
int	stop();
int	info();
void	usage();

/**
 * Start the driver.
 *
 * This function only returns if the driver is up and running
 * or failed to detect the sensor.
 */
int
start(enum Rotation rotation)
{
	GYROSIM **g_dev_ptr = &g_dev_sim;
	const char *path_accel = MPU_DEVICE_PATH_ACCEL;
	const char *path_gyro  = MPU_DEVICE_PATH_GYRO;
	DevHandle h;

	if (*g_dev_ptr != nullptr) {
		/* if already started, the still command succeeded */
		PX4_WARN("already started");
		return 0;
	}

	/* create the driver */
	*g_dev_ptr = new GYROSIM(path_accel, path_gyro, rotation);

	if (*g_dev_ptr == nullptr) {
		goto fail;
	}

	if (OK != (*g_dev_ptr)->init()) {
		goto fail;
	}

	return 0;
fail:

	if (*g_dev_ptr != nullptr) {
		delete *g_dev_ptr;
		*g_dev_ptr = nullptr;
	}

	PX4_WARN("driver start failed");
	return 1;
}

int
stop()
{
	GYROSIM **g_dev_ptr = &g_dev_sim;

	if (*g_dev_ptr != nullptr) {
		delete *g_dev_ptr;
		*g_dev_ptr = nullptr;

	} else {
		/* warn, but not an error */
		PX4_WARN("already stopped.");
	}

	return 0;
}

/**
 * Print a little info about the driver.
 */
int
info()
{
	GYROSIM **g_dev_ptr = &g_dev_sim;

	if (*g_dev_ptr == nullptr) {
		PX4_ERR("driver not running");
		return 1;
	}

	PX4_INFO("state @ %p", *g_dev_ptr);
	(*g_dev_ptr)->print_info();

	return 0;
}

void
usage()
{
	PX4_INFO("missing command: try 'start', 'info', stop'");
	PX4_INFO("options:");
	PX4_INFO("    -R rotation");
}

} // namespace

int
gyrosim_main(int argc, char *argv[])
{
	int ch;
	enum Rotation rotation = ROTATION_NONE;
	int ret;

	/* jump over start/off/etc and look at options first */
	int myoptind = 1;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "R:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'R':
			rotation = (enum Rotation)atoi(myoptarg);
			break;

		default:
			gyrosim::usage();
			return 0;
		}
	}

	if (myoptind >= argc) {
		gyrosim::usage();
		return 1;
	}

	const char *verb = argv[myoptind];

	/*
	 * Start/load the driver.

	 */
	if (!strcmp(verb, "start")) {
		ret = gyrosim::start(rotation);
	}

	else if (!strcmp(verb, "stop")) {
		ret = gyrosim::stop();
	}

	/*
	 * Print driver information.
	 */
	else if (!strcmp(verb, "info")) {
		ret = gyrosim::info();

	} else  {
		gyrosim::usage();
		ret = 1;
	}

	return ret;
}
