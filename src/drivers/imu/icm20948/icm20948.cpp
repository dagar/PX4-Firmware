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
 * @file icm20948.cpp
 *
 * Driver for the Invensense ICM20948 connected via I2C or SPI.
 *
 *
 * based on the icm20948 driver
 */

#include "icm20948.h"

static constexpr uint8_t Bit0 = (1 << 0);
static constexpr uint8_t Bit1 = (1 << 1);
static constexpr uint8_t Bit2 = (1 << 2);
static constexpr uint8_t Bit3 = (1 << 3);
static constexpr uint8_t Bit4 = (1 << 4);
static constexpr uint8_t Bit5 = (1 << 5);
static constexpr uint8_t Bit6 = (1 << 6);
static constexpr uint8_t Bit7 = (1 << 7);

/*
  we set the timer interrupt to run a bit faster than the desired
  sample rate and then throw away duplicates by comparing
  accelerometer values. This time reduction is enough to cope with
  worst case timing jitter due to other timers
 */
#define ICM20948_TIMER_REDUCTION				200


constexpr uint16_t ICM20948::_checked_registers[];


static constexpr int16_t combine(uint8_t msb, uint8_t lsb) { return (msb << 8u) | lsb; }

ICM20948::ICM20948(device::Device *interface, device::Device *mag_interface, enum Rotation rotation) :
	ScheduledWorkItem(MODULE_NAME, px4::device_bus_to_wq(interface->get_device_id())),
	_interface(interface),
	_px4_accel(_interface->get_device_id(), (_interface->external() ? ORB_PRIO_DEFAULT : ORB_PRIO_HIGH), rotation),
	_px4_gyro(_interface->get_device_id(), (_interface->external() ? ORB_PRIO_DEFAULT : ORB_PRIO_HIGH), rotation),
	_mag(this, mag_interface, rotation),
	_selected_bank(0xFF),	// invalid/improbable bank value, will be set on first read/write
	_sample_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": read")),
	_bad_transfers(perf_alloc(PC_COUNT, MODULE_NAME": bad_trans")),
	_bad_registers(perf_alloc(PC_COUNT, MODULE_NAME": bad_reg")),
	_good_transfers(perf_alloc(PC_COUNT, MODULE_NAME": good_trans")),
	_duplicates(perf_alloc(PC_COUNT, MODULE_NAME": dupe"))
{
	_px4_accel.set_device_type(DRV_DEVTYPE_ICM20948);
	_px4_gyro.set_device_type(DRV_DEVTYPE_ICM20948);
}

ICM20948::~ICM20948()
{
	// make sure we are truly inactive
	stop();

	// delete the perf counter
	perf_free(_sample_perf);
	perf_free(_bad_transfers);
	perf_free(_bad_registers);
	perf_free(_good_transfers);
	perf_free(_duplicates);
}

int
ICM20948::init()
{
	/*
	 * If the MPU is using I2C we should reduce the sample rate to 200Hz and
	 * make the integration autoreset faster so that we integrate just one
	 * sample since the sampling rate is already low.
	*/
	if (_interface->get_device_bus_type() == device::Device::DeviceBusType_I2C) {
		_sample_rate = 200;
	}

	int ret = probe();

	if (ret != OK) {
		PX4_DEBUG("probe failed");
		return ret;
	}

	_reset_wait = hrt_absolute_time() + 100000;

	if (reset_mpu() != OK) {
		PX4_ERR("Exiting! Device failed to take initialization");
		return ret;
	}

	/* Magnetometer setup */

#ifdef USE_I2C
	px4_usleep(100);

	if (!_mag.is_passthrough() && _mag._interface->init() != PX4_OK) {
		PX4_ERR("failed to setup ak09916 interface");
	}

#endif /* USE_I2C */

	ret = _mag.ak09916_reset();

	if (ret != OK) {
		PX4_DEBUG("mag reset failed");
		return ret;
	}

	start();

	return ret;
}

int ICM20948::reset()
{
	/* When the icm20948 starts from 0V the internal power on circuit
	 * per the data sheet will require:
	 *
	 * Start-up time for register read/write From power-up Typ:11 max:100 ms
	 *
	 */
	px4_usleep(110000);

	// Hold off sampling until done (100 MS will be shortened)
	_reset_wait = hrt_absolute_time() + 100000;

	int ret = reset_mpu();

	if (ret == OK) {
		ret = _mag.ak09916_reset();
	}

	_reset_wait = hrt_absolute_time() + 10;

	return ret;
}

int ICM20948::reset_mpu()
{
	write_reg(ICMREG_20948_PWR_MGMT_1, BIT_H_RESET);
	px4_usleep(2000);

	modify_reg(ICMREG_20948_PWR_MGMT_1, MPU_SLEEP, MPU_CLK_SEL_AUTO);
	px4_usleep(1500);

	write_reg(ICMREG_20948_PWR_MGMT_2, 0);

	// Enable I2C bus or Disable I2C bus (recommended on data sheet)
	if (_interface->get_device_bus_type() == device::Device::DeviceBusType_I2C) {
		modify_reg(ICMREG_20948_USER_CTRL, BIT_I2C_IF_DIS, 0);

	} else {
		modify_reg(ICMREG_20948_USER_CTRL, 0, BIT_I2C_IF_DIS);
	}

	// Gyro scale 2000 deg/s with DLPF
	modify_checked_reg(ICMREG_20948_GYRO_CONFIG_1, 0, ICM_BITS_GYRO_DLPF_CFG_119HZ | ICM_BITS_GYRO_FS_SEL_2000DPS | 1);

	// correct gyro scale factors
	// scale to rad/s in SI units
	// 2000 deg/s = (2000/180)*PI = 34.906585 rad/s
	// scaling factor:
	// 1/(2^15)*(2000/180)*PI
	_px4_gyro.set_scale(0.0174532 / 16.4); //1.0f / (32768.0f * (2000.0f / 180.0f) * M_PI_F);

	// Accel 16G with DLPF
	modify_checked_reg(ICMREG_20948_ACCEL_CONFIG, 0, ICM_BITS_ACCEL_DLPF_CFG_111HZ | ICM_BITS_ACCEL_FS_SEL_16G | 1);
	_px4_accel.set_scale(CONSTANTS_ONE_G / 16384);

	modify_checked_reg(ICMREG_20948_ACCEL_CONFIG_2, 0, ICM_BITS_DEC3_CFG_32);

#ifdef USE_I2C
	bool bypass = !_mag.is_passthrough();
#else
	bool bypass = false;
#endif

	/* INT: Clear on any read.
	 * If this instance is for a device is on I2C bus the Mag will have an i2c interface
	 * that it will use to access the either: a) the internal mag device on the internal I2C bus
	 * or b) it could be used to access a downstream I2C devices connected to the chip on
	 * it's AUX_{ASD|SCL} pins. In either case we need to disconnect (bypass) the internal master
	 * controller that chip provides as a SPI to I2C bridge.
	 * so bypass is true if the mag has an i2c non null interfaces.
	 */

	write_reg(ICMREG_20948_INT_PIN_CFG, BIT_INT_ANYRD_2CLEAR | (bypass ? BIT_INT_BYPASS_EN : 0));


	uint8_t retries = 5;
	bool all_ok = false;

	while (!all_ok && retries--) {

		// Assume all checked values are as expected
		all_ok = true;

		for (uint8_t i = 0; i < ICM20948_NUM_CHECKED_REGISTERS; i++) {
			uint8_t value = read_reg(_checked_registers[i]);

			if (value != _checked_values[i]) {
				uint8_t bankcheck = 0;
				_interface->read(ICMREG_20948_BANK_SEL, &bankcheck, 1);

				PX4_ERR("Reg %d is: %d s/b: %d Tries: %d - bank s/b %d, is %d", REG_ADDRESS(_checked_registers[i]), value,
					_checked_values[i], retries, REG_BANK(_checked_registers[i]), bankcheck >> 4);

				all_ok = false;
			}
		}
	}

	return all_ok ? OK : -EIO;
}

int
ICM20948::probe()
{
	_whoami = read_reg(ICMREG_20948_WHOAMI);

	if (_whoami != ICM_WHOAMI_20948) {
		PX4_ERR("invalid WHOAMI: 0x%02x", _whoami);
		return PX4_ERROR;
	}

	return PX4_OK;
}

int
ICM20948::select_register_bank(uint8_t bank)
{
	const uint8_t write_bank_buf = bank << 4;

	if (_selected_bank != bank) {
		uint8_t buf = write_bank_buf;
		uint8_t ret = _interface->write(ICMREG_20948_BANK_SEL, &buf, 1);

		if (ret != OK) {
			PX4_ERR("ICMREG_20948_BANK_SEL write error");
			return ret;
		}

		up_udelay(50);

	} else {
		return PX4_OK;
	}

	/*
	 * Making sure the right register bank is selected (even if it should be). Observed some
	 * unexpected changes to this, don't risk writing to the wrong register bank.
	 */
	uint8_t buf{};
	_interface->read(ICMREG_20948_BANK_SEL, &buf, 1);
	int retries = 5;

	while (buf != write_bank_buf && retries > 0) {
		buf = write_bank_buf;
		_interface->write(ICMREG_20948_BANK_SEL, &buf, 1);
		up_udelay(50);

		buf = 0;
		_interface->read(ICMREG_20948_BANK_SEL, &buf, 1);

		retries--;
	}

	_selected_bank = (buf & 0b110000) >> 4;

	if (bank != _selected_bank) {
		PX4_DEBUG("bank select failed selected: %d requsted: %d buf: %d", _selected_bank, bank, buf);
		return PX4_ERROR;

	} else {
		return PX4_OK;
	}
}

uint8_t ICM20948::read_reg(unsigned reg)
{
	select_register_bank(REG_BANK(reg));

	uint8_t buf{};
	_interface->read(reg, &buf, 1);

	return buf;
}

uint8_t ICM20948::read_reg_range(unsigned start_reg, uint8_t *buf, uint8_t count)
{
	if (buf == nullptr) {
		return PX4_ERROR;
	}

	select_register_bank(REG_BANK(start_reg));
	return _interface->read(start_reg, buf, count);
}

void
ICM20948::write_reg(unsigned reg, uint8_t value)
{
	// general register transfer at low clock speed
	select_register_bank(REG_BANK(reg));
	_interface->write(reg, &value, 1);
}

void
ICM20948::modify_reg(unsigned reg, uint8_t clearbits, uint8_t setbits)
{
	uint8_t	val = read_reg(reg);
	val &= ~clearbits;
	val |= setbits;
	write_reg(reg, val);
}

void
ICM20948::modify_checked_reg(unsigned reg, uint8_t clearbits, uint8_t setbits)
{
	uint8_t	val = read_reg(reg);
	val &= ~clearbits;
	val |= setbits;
	write_checked_reg(reg, val);
}

void
ICM20948::write_checked_reg(unsigned reg, uint8_t value)
{
	write_reg(reg, value);

	for (uint8_t i = 0; i < ICM20948_NUM_CHECKED_REGISTERS; i++) {
		if (reg == _checked_registers[i]) {
			_checked_values[i] = value;
			return;
		}
	}
}

void ICM20948::start()
{
	/* make sure we are stopped first */
	stop();

	ScheduleOnInterval(_call_interval - ICM20948_TIMER_REDUCTION, 1000);
}

void ICM20948::stop()
{
	ScheduleClear();
}

void ICM20948::Run()
{
	/* make another measurement */
	measure();
}

void ICM20948::check_registers()
{
	uint8_t v = 0;

	if ((v = read_reg(_checked_registers[_checked_next])) != _checked_values[_checked_next]) {

		PX4_DEBUG("reg: %d bad value %d (%d)", REG_ADDRESS(_checked_registers[_checked_next]), v, _checked_values[_checked_next]);
		/*
		  if we get the wrong value then we know the SPI bus
		  or sensor is very sick. We set _register_wait to 20
		  and wait until we have seen 20 good values in a row
		  before we consider the sensor to be OK again.
		 */
		perf_count(_bad_registers);

		/*
		  try to fix the bad register value. We only try to
		  fix one per loop to prevent a bad sensor hogging the
		  bus.
		 */
		if (_register_wait == 0 || _checked_next == 0) {
			// if the product_id is wrong then reset the
			// sensor completely
			reset_mpu();

			// after doing a reset we need to wait a long
			// time before we do any other register writes
			// or we will end up with the icm20948 in a
			// bizarre state where it has all correct
			// register values but large offsets on the
			// accel axes
			_reset_wait = hrt_absolute_time() + 10000;
			_checked_next = 0;

		} else {
			write_reg(_checked_registers[_checked_next], _checked_values[_checked_next]);

			// waiting 3ms between register writes seems
			// to raise the chance of the sensor
			// recovering considerably
			_reset_wait = hrt_absolute_time() + 3000;
		}

		_register_wait = 20;
	}

	_checked_next = (_checked_next + 1) % ICM20948_NUM_CHECKED_REGISTERS;
}

bool ICM20948::check_duplicate(uint8_t *accel_data)
{
	/*
	   see if this is duplicate accelerometer data. Note that we
	   can't use the data ready interrupt status bit in the status
	   register as that also goes high on new gyro data, and when
	   we run with BITS_DLPF_CFG_256HZ_NOLPF2 the gyro is being
	   sampled at 8kHz, so we would incorrectly think we have new
	   data when we are in fact getting duplicate accelerometer data.
	*/
	if (!_got_duplicate && memcmp(accel_data, &_last_accel_data, sizeof(_last_accel_data)) == 0) {
		// it isn't new data - wait for next timer
		perf_count(_duplicates);
		_got_duplicate = true;

	} else {
		memcpy(&_last_accel_data, accel_data, sizeof(_last_accel_data));
		_got_duplicate = false;
	}

	return _got_duplicate;
}

void ICM20948::measure()
{
	perf_begin(_sample_perf);

	if (hrt_absolute_time() < _reset_wait) {
		// we're waiting for a reset to complete
		perf_end(_sample_perf);
		return;
	}

	ICMReport report{};

	const hrt_abstime timestamp_sample = hrt_absolute_time();

	// Fetch the full set of measurements from the ICM20948 in one pass
	if (_mag.is_passthrough() && _register_wait == 0) {

		select_register_bank(REG_BANK(ICMREG_20948_ACCEL_XOUT_H));

		if (OK != read_reg_range(ICMREG_20948_ACCEL_XOUT_H, (uint8_t *)&report, sizeof(report))) {
			perf_end(_sample_perf);
			return;
		}

		check_registers();

		if (check_duplicate((uint8_t *)&report)) {
			perf_end(_sample_perf);
			return;
		}
	}

	/*
	 * In case of a mag passthrough read, hand the magnetometer data over to _mag. Else,
	 * try to read a magnetometer report.
	 */

#   ifdef USE_I2C

	if (_mag.is_passthrough()) {
#   endif

/* 		select_register_bank(REG_BANK(ICMREG_20948_EXT_SLV_SENS_DATA_00));
		ak09916_regs mag{};
		if (OK != read_reg_range(ICMREG_20948_EXT_SLV_SENS_DATA_00, (uint8_t *)&mag, sizeof(mag))) {
			perf_end(_sample_perf);
			return;
		}

		_mag._measure(timestamp_sample, mag); */

#   ifdef USE_I2C

	} else {
		_mag.measure();
	}

#   endif

	// Continue evaluating gyro and accelerometer results
	if (_register_wait == 0) {
		// Convert from big to little endian
		int16_t accel_x = combine(report.ACCEL_XOUT_H, report.ACCEL_XOUT_L);
		int16_t accel_y = combine(report.ACCEL_YOUT_H, report.ACCEL_YOUT_L);
		int16_t accel_z = combine(report.ACCEL_ZOUT_H, report.ACCEL_ZOUT_L);

		int16_t gyro_x = combine(report.GYRO_XOUT_H, report.GYRO_XOUT_L);
		int16_t gyro_y = combine(report.GYRO_YOUT_H, report.GYRO_YOUT_L);
		int16_t gyro_z = combine(report.GYRO_ZOUT_H, report.GYRO_ZOUT_L);

		//int16_t temp = combine(report.TEMP_OUT_H, report.TEMP_OUT_L);

		// Get sensor temperature
		//_last_temperature = temp / 333.87f + 21.0f;

		//_px4_accel.set_temperature(_last_temperature);
		//_px4_gyro.set_temperature(_last_temperature);

		/* NOTE: Axes have been swapped to match the board a few lines above. */
		_px4_accel.update(timestamp_sample, accel_x, accel_y, accel_z);
		_px4_gyro.update(timestamp_sample, gyro_x, gyro_y, gyro_z);
	} else {
		/*
		 * We are waiting for some good transfers before using the sensor again.
		 * We still increment _good_transfers, but don't return any data yet.
		*/
		_register_wait--;
		perf_end(_sample_perf);
		return;
	}

	// report the error count as the sum of the number of bad
	// transfers and bad register reads. This allows the higher
	// level code to decide if it should use this sensor based on
	// whether it has had failures
	const uint64_t error_count = perf_event_count(_bad_transfers) + perf_event_count(_bad_registers);
	_px4_accel.set_error_count(error_count);
	_px4_gyro.set_error_count(error_count);

	/* stop measuring */
	perf_end(_sample_perf);
}

void
ICM20948::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_bad_transfers);
	perf_print_counter(_bad_registers);
	perf_print_counter(_good_transfers);
	perf_print_counter(_duplicates);

	_px4_accel.print_status();
	_px4_gyro.print_status();
	_mag.print_status();
}
