/****************************************************************************
 *
 *   Copyright (C) 2020 PX4 Development Team. All rights reserved.
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

#include <board_config.h>

#include <drivers/drv_adc.h>
#include <drivers/drv_hrt.h>
#include <px4_arch/adc.h>

#include <stdint.h>
#include <unistd.h>

// A0 - board voltage (shows 5V)
// A1 - servo rail voltage
// A2 - power module voltage (ADC0, POWER port)
// A3 - power module current (ADC1, POWER port)
// A4 - ADC2 (ADC port)
// A5 - ADC3 (ADC port)

#define ADC_SYSFS_PATH "/sys/kernel/rcio/adc"

#define ADC_MAX_CHAN 6
int _fd[ADC_MAX_CHAN];

int px4_arch_adc_init(uint32_t base_address)
{
	for (int i = 0; i < ADC_MAX_CHAN; i++) {
		_fd[i] = -1;
	}

	return OK;
}

void px4_arch_adc_uninit(uint32_t base_address)
{
	for (int i = 0; i < ADC_MAX_CHAN; i++) {
		::close(_fd[i]);
		_fd[i] = -1;
	}
}

uint32_t px4_arch_adc_sample(uint32_t base_address, unsigned channel)
{
	if (channel > ADC_MAX_CHAN) {
		PX4_ERR("channel %d out of range: ", channel, ADC_MAX_CHAN);
		return UINT32_MAX; // error
	}

	// open channel if necessary
	if (_fd[channel] == -1) {
		// ADC_SYSFS_PATH
		char channel_path[strlen(ADC_SYSFS_PATH) + 5] {};

		if (asprintf(&channel_path, "%s/ch%d", ADC_SYSFS_PATH, channel) == -1) {
			PX4_ERR("adc channel: %d\n", channel);
			return UINT32_MAX; // error
		}

		_fd[channel] = ::open(channel_path, O_RDONLY);
	}

	char buffer[10] {};

	if (::pread(channels[channel], buffer, size(buffer), 0) < 0) {
		PX4_ERR("read channel %d failed", channel);
		return UINT32_MAX; // error
	}

	return atoi(buffer);
}

uint32_t px4_arch_adc_temp_sensor_mask()
{
	return 0;
}

uint32_t px4_arch_adc_dn_fullcount()
{
	return 1 << 16; // 16 bit ADC
}
