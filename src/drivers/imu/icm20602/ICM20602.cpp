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

#include "ICM20602.hpp"

#include <drivers/boards/common/board_dma_alloc.h>

using namespace time_literals;
using namespace InvenSense_ICM20602;

ICM20602::ICM20602(int bus, uint32_t device, enum Rotation rotation) :
	SPI("ICM20602", nullptr, bus, device, SPIDEV_MODE3, 10000000),
	ScheduledWorkItem(px4::device_bus_to_wq(get_device_id())),
	_px4_accel(get_device_id(), (external() ? ORB_PRIO_MAX : ORB_PRIO_HIGH), rotation),
	_px4_gyro(get_device_id(), (external() ? ORB_PRIO_MAX : ORB_PRIO_HIGH), rotation),
	_interval_perf(perf_alloc(PC_INTERVAL, "icm20602: run interval")),
	_transfer_perf(perf_alloc(PC_ELAPSED, "icm20602: transfer")),
	_cycle_perf(perf_alloc(PC_ELAPSED, "icm20602: run")),
	_bad_transfers_perf(perf_alloc(PC_COUNT, "icm20602: bad transfers")),
	_fifo_empty_perf(perf_alloc(PC_COUNT, "icm20602: fifo empty")),
	_fifo_overflow_perf(perf_alloc(PC_COUNT, "icm20602: fifo overflow")),
	_fifo_reset_perf(perf_alloc(PC_COUNT, "icm20602: fifo reset")),
	_reset_perf(perf_alloc(PC_COUNT, "icm20602: reset")),
	_accel_process_perf(perf_alloc(PC_ELAPSED, "icm20602: accel process")),
	_gyro_process_perf(perf_alloc(PC_ELAPSED, "icm20602: gyro process"))
{
	_px4_accel.set_device_type(DRV_ACC_DEVTYPE_ICM20602);
	_px4_accel.set_sample_rate(4000);

	_px4_gyro.set_device_type(DRV_GYR_DEVTYPE_ICM20602);
	_px4_gyro.set_sample_rate(32000);
}

ICM20602::~ICM20602()
{
	// make sure we are truly inactive
	stop();

	if (_dma_data_buffer != nullptr) {
		board_dma_free(_dma_data_buffer, FIFO::SIZE);
	}

	// delete the perf counters
	perf_free(_interval_perf);
	perf_free(_transfer_perf);
	perf_free(_cycle_perf);
	perf_free(_bad_transfers_perf);
	perf_free(_fifo_empty_perf);
	perf_free(_fifo_overflow_perf);
	perf_free(_fifo_reset_perf);
	perf_free(_reset_perf);
	perf_free(_gyro_process_perf);
	perf_free(_accel_process_perf);
}

int
ICM20602::init()
{
	// probe again to get our settings that are based on the device type
	int ret = SPI::init();

	// if probe failed, bail now
	if (ret != PX4_OK) {
		return ret;
	}

	// allocate DMA capable buffer
	_dma_data_buffer = (uint8_t *)board_dma_alloc(FIFO::SIZE);

	if (_dma_data_buffer == nullptr) {
		PX4_ERR("DMA alloc failed");
		return PX4_ERROR;
	}

	if (reset() != PX4_OK) {
		PX4_ERR("reset failed");
		return ret;
	}

	start();

	return ret;
}

int
ICM20602::probe()
{
	uint8_t whoami = registerRead(Register::WHO_AM_I);

	if (whoami != ICM_WHOAMI_20602) {
		PX4_ERR("unexpected WHO_AM_I 0x%02x", whoami);
		return PX4_ERROR;
	}

	return OK;
}

int ICM20602::reset()
{
	PX4_ERR("reset");

	perf_count(_reset_perf);

	int ret = PX4_ERROR;

	for (int i = 0; i < 5; i++) {

		// Device Reset (via PWR_MGMT_1)
		// CLKSEL[2:0] must be set to 001 to achieve full gyroscope performance.
		registerWrite(Register::PWR_MGMT_1, PWR_MGMT_1_BIT::DEVICE_RESET);
		usleep(100);

		// CLKSEL[2:0] must be set to 001 to achieve full gyroscope performance.
		registerWrite(Register::PWR_MGMT_1, PWR_MGMT_1_BIT::CLKSEL_0);
		usleep(100);

		configureAccel();
		configureGyro();

		const bool reset_done = !(registerRead(Register::PWR_MGMT_1) & PWR_MGMT_1_BIT::DEVICE_RESET);
		const bool clksel_done = (registerRead(Register::PWR_MGMT_1) & PWR_MGMT_1_BIT::CLKSEL_0);
		const bool data_ready = (registerRead(Register::INT_STATUS) & INT_STATUS_BIT::DATA_RDY_INT);

		// reset done once data is ready
		if (reset_done && clksel_done && data_ready) {
			ret = PX4_OK;
			break;
		}
	}

	return ret;
}

bool
ICM20602::configureAccel()
{
	// 16 G range
	registerSetBits(Register::ACCEL_CONFIG, ACCEL_CONFIG_BIT::ACCEL_FS_SEL_16G);

	_px4_accel.set_scale(CONSTANTS_ONE_G / 2048);

	return true;
}

bool
ICM20602::configureGyro()
{
	// 2000 degrees/second
	registerSetBits(Register::GYRO_CONFIG, GYRO_CONFIG_BIT::FS_SEL_2000_DPS);

	_px4_gyro.set_scale(math::radians(1 / 16.4f));

	return true;
}

bool
ICM20602::resetFIFO()
{
	perf_count(_fifo_reset_perf);

	// Accel DLPF disabled for full rate (4 kHz)
	registerSetBits(Register::ACCEL_CONFIG2, ACCEL_CONFIG2_BIT::ACCEL_FCHOICE_B_BYPASS_DLPF);

	// Gyro DLPF disabled for full rate (32 kHz)
	registerSetBits(Register::GYRO_CONFIG, GYRO_CONFIG_BIT::FCHOICE_B_BYPASS_DLPF);

	// disable FIFO
	registerWrite(Register::FIFO_EN, 0);
	registerClearBits(Register::USER_CTRL, USER_CTRL_BIT::FIFO_EN | USER_CTRL_BIT::FIFO_RST);

	// reset FIFO (via USER_CTRL)
	registerSetBits(Register::USER_CTRL, USER_CTRL_BIT::FIFO_RST);
	up_udelay(1); // bit auto clears after one clock cycle of the internal 20 MHz clock

	// enable FIFO (via USER_CTRL)
	registerSetBits(Register::USER_CTRL, USER_CTRL_BIT::FIFO_EN);

	// CONFIG
	// should ensure that bit 7 of register 0x1A is set to 0 before using FIFO watermark feature
	registerSetBits(Register::CONFIG, CONFIG_BIT::FIFO_MODE);
	registerClearBits(Register::CONFIG, CONFIG_BIT::FIFO_WM);

	// FIFO watermark
	// uint16_t fifo_watermark = sizeof(FIFO::DATA) * 32;
	// uint8_t fifo_wm_th1 = (fifo_watermark >> 8) & 0xFF;	// FIFO_WM_TH1 (FIFO_WM_TH[9:8])
	// uint8_t fifo_wm_th2 = fifo_watermark & 0xFF;		// FIFO_WM_TH2 (FIFO_WM_TH[7:0])
	// registerWrite(Register::FIFO_WM_TH1, fifo_wm_th1);
	// registerWrite(Register::FIFO_WM_TH2, fifo_wm_th2);

	// FIFO enable both gyro and accel
	registerWrite(Register::FIFO_EN, FIFO_EN_BIT::GYRO_FIFO_EN | FIFO_EN_BIT::ACCEL_FIFO_EN);
	up_udelay(10);

#ifdef GPIO_DRDY_ICM20608
	// Disable data ready callback
	//px4_arch_gpiosetevent(GPIO_DRDY_ICM20608, true, false, true, &ICM20602::data_ready_interrupt, this);
#endif // GPIO_DRDY_ICM20608

	return true;
}

uint8_t
ICM20602::registerRead(Register reg)
{
	uint8_t cmd[2] {};
	cmd[0] = static_cast<uint8_t>(reg) | DIR_READ;

	transfer(cmd, cmd, sizeof(cmd));

	return cmd[1];
}

void
ICM20602::registerWrite(Register reg, uint8_t value)
{
	uint8_t cmd[2] {};
	cmd[0] = static_cast<uint8_t>(reg) | DIR_WRITE;
	cmd[1] = value;
	transfer(cmd, cmd, sizeof(cmd));
}

void
ICM20602::registerSetBits(Register reg, uint8_t setbits)
{
	uint8_t val = registerRead(reg);

	// only write if necessary
	if (!(val & setbits)) {
		val |= setbits;
		registerWrite(reg, val);
	}
}

void
ICM20602::registerClearBits(Register reg, uint8_t clearbits)
{
	uint8_t val = registerRead(reg);

	// only write if necessary
	if (val & clearbits) {
		val &= !clearbits;
		registerWrite(reg, val);
	}
}

bool
ICM20602::selfTest()
{
	PX4_WARN("selfTest");

	return true;
}

void
ICM20602::start()
{
	stop();

	resetFIFO();

	// start polling at the specified rate
	ScheduleOnInterval(1000);
}

void
ICM20602::stop()
{
#ifdef GPIO_DRDY_ICM20608
	// Disable data ready callback
	//px4_arch_gpiosetevent(GPIO_DRDY_ICM20608, false, false, false, nullptr, nullptr);
#endif

	ScheduleClear();
}

int
ICM20602::data_ready_interrupt(int irq, void *context, void *arg)
{
	ICM20602 *dev = reinterpret_cast<ICM20602 *>(arg);

	PX4_INFO("data_ready_interrupt");

	// make another measurement
	dev->ScheduleNow();

	return PX4_OK;
}

void
ICM20602::Run()
{
	// make another measurement
	measure();
}

static constexpr int16_t combine(uint8_t msb, uint8_t lsb)
{
	return (msb << 8u) | lsb;
}

int
ICM20602::measure()
{
	perf_count(_interval_perf);
	perf_begin(_cycle_perf);


	// check FIFO count
	uint8_t fifo_count_buf[3] {};
	fifo_count_buf[0] = static_cast<uint8_t>(Register::FIFO_COUNTH) | DIR_READ;

	if (transfer(fifo_count_buf, fifo_count_buf, sizeof(fifo_count_buf)) != PX4_OK) {
		perf_count(_bad_transfers_perf);
		perf_end(_cycle_perf);
		return -EIO;
	}

	size_t fifo_count = combine(fifo_count_buf[1], fifo_count_buf[2]);

	const int samples = fifo_count / sizeof(FIFO::DATA);
	const int samples_constrained = math::constrain(samples, 0, 32);


	// FIFO overflow
	const bool fifo_overflow = (registerRead(Register::INT_STATUS) & INT_STATUS_BIT::FIFO_OFLOW_INT);

	if (fifo_overflow) {
		perf_count(_fifo_overflow_perf);
		//PX4_ERR("FIFO overflow");

		resetFIFO();

		perf_count(_bad_transfers_perf);
		perf_end(_cycle_perf);
		return -EIO;
	}


	// 1 (cmd) + sizeof(FIFO_TRANSFER) * 32 = 449 bytes
	struct ICM_Report {
		uint8_t cmd;
		FIFO::DATA f[32];
	};
	static_assert(sizeof(ICM_Report) == (sizeof(FIFO::DATA) * 32 + 1), "sizeof(ICM_Report) == 449");
	static_assert(sizeof(ICM_Report) == 449, "sizeof(ICM_Report) == 449");

	const size_t transfer_size = samples_constrained * sizeof(FIFO::DATA) + 1;

	// zero buffer
	memset(_dma_data_buffer, 0, transfer_size);

	ICM_Report *report = (ICM_Report *)_dma_data_buffer;
	report->cmd = static_cast<uint8_t>(Register::FIFO_R_W) | DIR_READ;

	const hrt_abstime timestamp_sample = hrt_absolute_time();

	// TODO: use sample rate to calculate timestamp?
	// read from FIFO
	perf_begin(_transfer_perf);

	if (transfer(_dma_data_buffer, _dma_data_buffer, transfer_size) != PX4_OK) {
		perf_end(_transfer_perf);
		perf_count(_bad_transfers_perf);
		perf_end(_cycle_perf);
		return -EIO;
	}

	perf_end(_transfer_perf);

	// TODO: check for 0xFF
	// If the FIFO buffer is empty, reading register FIFO_DATA will return a unique value of 0xFF until new data is available.

	const uint64_t error_count = perf_event_count(_bad_transfers_perf);

	const int16_t temperature_0 = combine(report->f[0].TEMP_OUT_H, report->f[0].TEMP_OUT_L);
	const float temperature = temperature_0 / 326.8f + 25.0f;	// 326.8 LSB/oC


	// TODO: check all temperature samples for sanity


	// gyro
	perf_begin(_gyro_process_perf);
	_px4_gyro.set_error_count(error_count);
	_px4_gyro.set_temperature(temperature);
	sensor_gyro_fifo_s gyro_fifo{};
	gyro_fifo.timestamp_sample = timestamp_sample;
	gyro_fifo.device_id = get_device_id();
	gyro_fifo.samples = samples_constrained;
	gyro_fifo.dt = 1000000.0f / 32000.0f;

	for (size_t i = 0; i < gyro_fifo.samples; i++) {
		const FIFO::DATA &fifo_sample = report->f[i];

		gyro_fifo.x[i] = combine(fifo_sample.GYRO_XOUT_H, fifo_sample.GYRO_XOUT_L);
		gyro_fifo.y[i] = combine(fifo_sample.GYRO_YOUT_H, fifo_sample.GYRO_YOUT_L);
		gyro_fifo.z[i] = combine(fifo_sample.GYRO_ZOUT_H, fifo_sample.GYRO_ZOUT_L);
	}

	gyro_fifo.timestamp = hrt_absolute_time();
	_sensor_gyro_fifo_pub.publish(gyro_fifo);
	_px4_gyro.updateFIFO(gyro_fifo);
	perf_end(_gyro_process_perf);


	// accel
	perf_begin(_accel_process_perf);
	_px4_accel.set_error_count(error_count);
	_px4_accel.set_temperature(temperature);
	sensor_accel_fifo_s accel_fifo{};
	accel_fifo.timestamp_sample = timestamp_sample;
	accel_fifo.device_id = get_device_id();
	accel_fifo.samples = samples_constrained / 8;
	accel_fifo.dt = 1000000.0f / 4000.0f;

	for (size_t i = 0; i < accel_fifo.samples; i++) {
		const FIFO::DATA &fifo_sample = report->f[i * 8];

		accel_fifo.x[i] = combine(fifo_sample.ACCEL_XOUT_H, fifo_sample.ACCEL_XOUT_L);
		accel_fifo.y[i] = combine(fifo_sample.ACCEL_YOUT_H, fifo_sample.ACCEL_YOUT_L);
		accel_fifo.z[i] = combine(fifo_sample.ACCEL_ZOUT_H, fifo_sample.ACCEL_ZOUT_L);
	}

	accel_fifo.timestamp = hrt_absolute_time();
	_sensor_accel_fifo_pub.publish(accel_fifo);
	_px4_accel.updateFIFO(accel_fifo);
	perf_end(_accel_process_perf);

	perf_end(_cycle_perf);

	return OK;
}

void
ICM20602::print_info()
{
	perf_print_counter(_interval_perf);
	perf_print_counter(_transfer_perf);
	perf_print_counter(_cycle_perf);
	perf_print_counter(_bad_transfers_perf);
	perf_print_counter(_fifo_empty_perf);
	perf_print_counter(_fifo_overflow_perf);
	perf_print_counter(_fifo_reset_perf);
	perf_print_counter(_reset_perf);

	_px4_accel.print_status();
	_px4_gyro.print_status();
}

void
ICM20602::print_registers()
{
	PX4_INFO("");

	PX4_INFO("SELF_TEST_X_ACCEL: %X", registerRead(Register::SELF_TEST_X_ACCEL));
	PX4_INFO("SELF_TEST_Y_ACCEL: %X", registerRead(Register::SELF_TEST_Y_ACCEL));
	PX4_INFO("SELF_TEST_Z_ACCEL: %X", registerRead(Register::SELF_TEST_Z_ACCEL));

	PX4_INFO("CONFIG: %X", registerRead(Register::CONFIG));
	PX4_INFO("GYRO_CONFIG: %X", registerRead(Register::GYRO_CONFIG));
	PX4_INFO("ACCEL_CONFIG: %X", registerRead(Register::ACCEL_CONFIG));
	PX4_INFO("ACCEL_CONFIG2: %X", registerRead(Register::ACCEL_CONFIG2));

	PX4_INFO("SMPLRT_DIV: %X", registerRead(Register::SMPLRT_DIV));

	PX4_INFO("FIFO_EN: %X", registerRead(Register::FIFO_EN));

	PX4_INFO("INT_PIN_CFG: %X", registerRead(Register::INT_PIN_CFG));

	PX4_INFO("FIFO_WM_INT_STATUS: %X", registerRead(Register::FIFO_WM_INT_STATUS));
	PX4_INFO("INT_STATUS: %X", registerRead(Register::INT_STATUS));

	PX4_INFO("SELF_TEST_X_GYRO: %X", registerRead(Register::SELF_TEST_X_GYRO));
	PX4_INFO("SELF_TEST_Y_GYRO: %X", registerRead(Register::SELF_TEST_Y_GYRO));
	PX4_INFO("SELF_TEST_Z_GYRO: %X", registerRead(Register::SELF_TEST_Z_GYRO));

	PX4_INFO("FIFO_WM_TH1: %X", registerRead(Register::FIFO_WM_TH1));
	PX4_INFO("FIFO_WM_TH2: %X", registerRead(Register::FIFO_WM_TH2));

	PX4_INFO("USER_CTRL: %X", registerRead(Register::USER_CTRL));
	PX4_INFO("PWR_MGMT_1: %X", registerRead(Register::PWR_MGMT_1));
	PX4_INFO("PWR_MGMT_2: %X", registerRead(Register::PWR_MGMT_2));

	PX4_INFO("FIFO_COUNTH: %X", registerRead(Register::FIFO_COUNTH));
	PX4_INFO("FIFO_COUNTL: %X", registerRead(Register::FIFO_COUNTL));
	PX4_INFO("WHO_AM_I: %X", registerRead(Register::WHO_AM_I));

	PX4_INFO("");
}
