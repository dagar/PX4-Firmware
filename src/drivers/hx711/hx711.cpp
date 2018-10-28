/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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

#include <px4_config.h>
#include <px4_defines.h>

#include <cstring>

#include <arch/board/board.h>

#include <nuttx/arch.h>
#include <nuttx/wqueue.h>

#include <board_config.h>

#include <drivers/device/i2c.h>
#include <drivers/drv_adc.h>

#include <systemlib/err.h>

#define SLAVE_ADDR 0x50
#define ADC_ENABLE_REG 0x00
#define ADC_CHANNEL_REG 0x05
#define MAX_CHANNEL 5

// 10Hz
#define CYCLE_TICKS_DELAY MSEC2TICK(100)

enum HX711_BUS {
	HX711_BUS_ALL = 0,
	HX711_BUS_I2C_INTERNAL,
	HX711_BUS_I2C_EXTERNAL
};

static constexpr struct HX711_bus_option {
	enum HX711_BUS busid;
	uint8_t busnum;
} bus_options[] = {
#ifdef PX4_I2C_BUS_EXPANSION
	{ HX711_BUS_I2C_EXTERNAL, PX4_I2C_BUS_EXPANSION },
#endif
#ifdef PX4_I2C_BUS_EXPANSION1
	{ HX711_BUS_I2C_EXTERNAL, PX4_I2C_BUS_EXPANSION1 },
#endif
#ifdef PX4_I2C_BUS_ONBOARD
	{ HX711_BUS_I2C_INTERNAL, PX4_I2C_BUS_ONBOARD },
#endif
};

extern "C" { __EXPORT int hx711_main(int argc, char *argv[]); }

class HX711 : public device::I2C
{
public:
	HX711(uint8_t bus);
	virtual ~HX711();

	virtual int init();
	virtual int ioctl(file *filp, int cmd, unsigned long arg);
	virtual ssize_t read(file *filp, char *buffer, size_t len);

private:
	virtual int probe();

	void cycle();
	static void cycle_trampoline(void *arg);

	struct work_s _work;
	px4_adc_msg_t _sample;
	pthread_mutex_t _sample_mutex;
};

static HX711 *instance = nullptr;

static void test()
{
	int fd = open(ADC0_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		err(1, "can't open ADC device");
	}

	px4_adc_msg_t data[MAX_CHANNEL];
	ssize_t count = read(fd, data, sizeof(data));

	if (count < 0) {
		errx(1, "read error");
	}

	unsigned channels = count / sizeof(data[0]);

	for (unsigned j = 0; j < channels; j++) {
		printf("%d: %u  ", data[j].am_channel, data[j].am_data);
	}

	printf("\n");

	exit(0);
}

static void help()
{
	printf("missing command: try 'start' or 'test'\n");
	printf("options:\n");
	printf("    -I only internal I2C bus\n");
	printf("    -X only external I2C bus\n");
}

int hx711_main(int argc, char *argv[])
{
	int ch;
	enum HX711_BUS busid = HX711_BUS_ALL;

	while ((ch = getopt(argc, argv, "XI")) != EOF) {
		switch (ch) {
		case 'X':
			busid = HX711_BUS_I2C_EXTERNAL;
			break;

		case 'I':
			busid = HX711_BUS_I2C_INTERNAL;
			break;

		default:
			help();
			return -1;
		}
	}

	if (optind >= argc) {
		help();
		return PX4_ERROR;
	}

	const char *verb = argv[optind];

	if (!strcmp(verb, "start")) {
		if (instance) {
			warn("HX711 was already started");
			return PX4_OK;
		}

		for (uint8_t i = 0; i < (sizeof(bus_options) / sizeof(bus_options[0])); i++) {
			if (busid != HX711_BUS_ALL && busid != bus_options[i].busid) {
				continue;
			}

			instance = new HX711(bus_options[i].busnum);

			if (!instance) {
				warn("No memory to instance HX711");
				return PX4_ERROR;
			}

			if (instance->init() == PX4_OK) {
				break;
			}

			warn("HX711 not found on busnum=%u", bus_options[i].busnum);
			delete instance;
			instance = nullptr;
		}

		if (!instance) {
			warn("HX711 not found");
			return PX4_ERROR;
		}

	} else if (!strcmp(verb, "test")) {
		test();

	} else {
		warn("Action not supported");
		help();
		return PX4_ERROR;
	}

	return PX4_OK;
}

HX711::HX711(uint8_t bus) :
	I2C("HX711", ADC0_DEVICE_PATH, bus, SLAVE_ADDR, 400000),
	_sample{}
{
	_sample.am_channel = 1;
	pthread_mutex_init(&_sample_mutex, NULL);
}

HX711::~HX711()
{
	work_cancel(HPWORK, &_work);
}

int HX711::init()
{
	int ret = I2C::init();

	if (ret != PX4_OK) {
		return ret;
	}

	work_queue(HPWORK, &_work, (worker_t)&HX711::cycle_trampoline, this,
		   CYCLE_TICKS_DELAY);

	return PX4_OK;
}

int HX711::probe()
{
	uint8_t buffer[2];
	int ret;

	_retries = 3;

	/* Enable ADC */
	buffer[0] = ADC_ENABLE_REG;
	buffer[1] = 0x01;
	ret = transfer(buffer, 2, NULL, 0);

	if (ret != PX4_OK) {
		goto error;
	}

	usleep(10000);

	/* Read ADC value */
	buffer[0] = ADC_CHANNEL_REG;
	ret = transfer(buffer, 1, buffer, 2);

	if (ret != PX4_OK) {
		goto error;
	}

	return PX4_OK;

error:
	return -EIO;
}

int HX711::ioctl(file *filp, int cmd, unsigned long arg)
{
	return -ENOTTY;
}

ssize_t HX711::read(file *filp, char *buffer, size_t len)
{
	if (len < sizeof(_sample)) {
		return -ENOSPC;
	}

	if (len > sizeof(_sample)) {
		len = sizeof(_sample);
	}

	pthread_mutex_lock(&_sample_mutex);
	memcpy(buffer, &_sample, len);
	pthread_mutex_unlock(&_sample_mutex);

	return len;
}

void HX711::cycle_trampoline(void *arg)
{
	HX711 *dev = reinterpret_cast<HX711 * >(arg);
	dev->cycle();
}

void HX711::cycle()
{
	uint8_t buffer[2];
	int ret;

	buffer[0] = ADC_CHANNEL_REG;
	ret = transfer(buffer, 1, buffer, sizeof(buffer));

	if (ret != PX4_OK) {
		warn("Error reading sample");
		return;
	}

	pthread_mutex_lock(&_sample_mutex);

	_sample.am_data = (buffer[0] | (buffer[1] << 8));

	pthread_mutex_unlock(&_sample_mutex);

	work_queue(HPWORK, &_work, (worker_t)&HX711::cycle_trampoline, this, CYCLE_TICKS_DELAY);
}
