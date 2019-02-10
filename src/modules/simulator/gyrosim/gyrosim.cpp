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
 * @author Mark Charlebois
 */

#include <drivers/drv_hrt.h>
#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>
#include <perf/perf_counter.h>
#include <px4_config.h>
#include <px4_getopt.h>
#include <simulator/simulator.h>

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

#define GYROSIM_ACCEL_DEFAULT_RATE	250

#define GYROSIM_GYRO_DEFAULT_RATE	250

/*
  the GYROSIM can only handle high SPI bus speeds on the sensor and
  interrupt status registers. All other registers have a maximum 1MHz
  SPI speed
 */

class GYROSIM : public VirtDevObj
{
public:
	GYROSIM(const char *path_accel, const char *path_gyro, enum Rotation rotation);
	virtual ~GYROSIM();

	int             	init();
	virtual int		start();

	int			transfer(uint8_t *send, uint8_t *recv, unsigned len);

private:

	PX4Accelerometer	_px4_accel;
	PX4Gyroscope		_px4_gyro;

	uint8_t			_product;	/** product code */

	unsigned		_call_interval;

	perf_counter_t		_sample_perf;

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

	/**
	 * Measurement self test
	 *
	 * @return 0 on success, 1 on failure
	 */
	int 			self_test();

	/*
	  set sample rate (approximate) - 1kHz to 5Hz
	*/
	void _set_sample_rate(unsigned desired_sample_rate_hz);

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

/** driver 'main' command */
extern "C" { __EXPORT int gyrosim_main(int argc, char *argv[]); }

GYROSIM::GYROSIM(const char *path_accel, const char *path_gyro, enum Rotation rotation) :
	VirtDevObj("GYROSIM", path_accel, ACCEL_BASE_DEVICE_PATH, 1e6 / GYROSIM_ACCEL_DEFAULT_RATE),
	_px4_accel(m_id.dev_id, ORB_PRIO_HIGH, rotation),
	_px4_gyro(m_id.dev_id, ORB_PRIO_HIGH, rotation),
	_product(GYROSIMES_REV_C4),
	_sample_perf(perf_alloc(PC_ELAPSED, "gyrosim_read"))
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
}

int
GYROSIM::init()
{
	PX4_DEBUG("init");
	int ret = 1;

	ret = VirtDevObj::init();

	if (ret != 0) {
		PX4_WARN("Base class init failed");
		ret = 1;
		goto out;
	}

	ret = start();

	if (ret != OK) {
		PX4_ERR("gyro accel start failed (%d)", ret);
		return ret;
	}

out:
	return ret;
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

/*
  set sample rate (approximate) - 1kHz to 5Hz, for both accel and gyro
*/
void
GYROSIM::_set_sample_rate(unsigned desired_sample_rate_hz)
{
	PX4_DEBUG("_set_sample_rate %u Hz", desired_sample_rate_hz);

	if (desired_sample_rate_hz == 0) {
		desired_sample_rate_hz = GYROSIM_GYRO_DEFAULT_RATE;
	}

	uint8_t div = 1000 / desired_sample_rate_hz;

	if (div > 200) { div = 200; }

	if (div < 1) { div = 1; }

	// This does nothing in the simulator but writes the value in the "register" so
	// register dumps look correct
	write_reg(MPUREG_SMPLRT_DIV, div - 1);

	unsigned sample_rate = 1000 / div;
	PX4_DEBUG("Changed sample rate to %uHz", sample_rate);
	setSampleInterval(1000000 / sample_rate);

	_px4_accel.set_sample_rate(sample_rate);
	_px4_gyro.set_sample_rate(sample_rate);
}

int
GYROSIM::self_test()
{
	if (perf_event_count(_sample_perf) == 0) {
		_measure();
	}

	/* return 0 on success, 1 else */
	return (perf_event_count(_sample_perf) > 0) ? 0 : 1;
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
GYROSIM::set_accel_range(unsigned max_g_in)
{
	int _accel_range_scale = 1.0f;

	// workaround for bugged versions of MPU6k (rev C)
	switch (_product) {
	case GYROSIMES_REV_C4:
		write_reg(MPUREG_ACCEL_CONFIG, 1 << 3);
		_accel_range_scale = (CONSTANTS_ONE_G / 4096.0f);
		return OK;
	}

	uint8_t afs_sel;
	float lsb_per_g;
	//float max_accel_g;

	if (max_g_in > 8) { // 16g - AFS_SEL = 3
		afs_sel = 3;
		lsb_per_g = 2048;
		//max_accel_g = 16;

	} else if (max_g_in > 4) { //  8g - AFS_SEL = 2
		afs_sel = 2;
		lsb_per_g = 4096;
		//max_accel_g = 8;

	} else if (max_g_in > 2) { //  4g - AFS_SEL = 1
		afs_sel = 1;
		lsb_per_g = 8192;
		//max_accel_g = 4;

	} else {                //  2g - AFS_SEL = 0
		afs_sel = 0;
		lsb_per_g = 16384;
		//max_accel_g = 2;
	}

	write_reg(MPUREG_ACCEL_CONFIG, afs_sel << 3);
	_accel_range_scale = (CONSTANTS_ONE_G / lsb_per_g);

	_px4_accel.set_scale(_accel_range_scale);

	return OK;
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
	struct MPUReport mpu_report = {};

	/* start measuring */
	perf_begin(_sample_perf);

	/*
	 * Fetch the full set of measurements from the GYROSIM in one pass.
	 */
	mpu_report.cmd = DIR_READ | MPUREG_INT_STATUS;

	// for now use local time but this should be the timestamp of the simulator
	const uint64_t timestamp_sample = hrt_absolute_time();

	if (OK != transfer((uint8_t *)&mpu_report, ((uint8_t *)&mpu_report), sizeof(mpu_report))) {
		return;
	}

	/* fake device ID */
	// arb.device_id = 1376264;

	/* fake device ID */
	//grb.device_id = 2293768;

	_px4_accel.set_temperature(mpu_report.temp);
	_px4_gyro.set_temperature(mpu_report.temp);

	_px4_accel.update(timestamp_sample, mpu_report.accel_x, mpu_report.accel_y, mpu_report.accel_z);
	_px4_gyro.update(timestamp_sample, mpu_report.gyro_x, mpu_report.gyro_y, mpu_report.gyro_z);

	/* stop measuring */
	perf_end(_sample_perf);
}

/**
 * Local functions in support of the shell command.
 */
namespace gyrosim
{

GYROSIM	*g_dev_sim; // on simulated bus

int	start(enum Rotation /*rotation*/);
int	stop();
int	test();
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

void
usage()
{
	PX4_INFO("missing command: try 'start', 'stop'");
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

	if (!strcmp(verb, "start")) {
		ret = gyrosim::start(rotation);

	} else if (!strcmp(verb, "stop")) {
		ret = gyrosim::stop();

	} else  {
		gyrosim::usage();
		ret = 1;
	}

	return ret;
}
