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

#include "ICM20608G_DRDY.hpp"

ICM20608G_DRDY::ICM20608G_DRDY(int bus, uint32_t device, enum Rotation rotation, uint32_t drdy_gpio) :
	ICM20608G(bus, device, rotation),
	WorkItem(MODULE_NAME, px4::device_bus_to_wq(get_device_id())),
	_drdy_gpio(drdy_gpio)
{
}

ICM20608G_DRDY::~ICM20608G_DRDY()
{
	Stop();

	perf_free(_drdy_count_perf);
	perf_free(_drdy_interval_perf);
}

int
ICM20608G_DRDY::DataReadyInterruptCallback(int irq, void *context, void *arg)
{
	ICM20608G_DRDY *dev = reinterpret_cast<ICM20608G_DRDY *>(arg);
	dev->DataReady();
	return 0;
}

void
ICM20608G_DRDY::DataReady()
{
	perf_count(_drdy_count_perf);
	perf_count(_drdy_interval_perf);

	_data_ready_count++;

	if (_data_ready_count >= 8) {
		_time_data_ready = hrt_absolute_time();

		_data_ready_count = 0;

		// make another measurement
		ScheduleNow();
	}
}

void
ICM20608G_DRDY::Start()
{
	Stop();

	ResetFIFO();

	// Setup data ready on rising edge
	px4_arch_gpiosetevent(_drdy_gpio, true, false, true, &ICM20608G::DataReadyInterruptCallback, this);
}

void
ICM20608G_DRDY::Stop()
{
	// Disable data ready callback
	px4_arch_gpiosetevent(_drdy_gpio, false, false, false, nullptr, nullptr);
	RegisterClearBits(Register::INT_ENABLE, INT_ENABLE_BIT::DATA_RDY_INT_EN);
}

void
ICM20608G_DRDY::PrintInfo()
{
	ICM20608G::PrintInfo();

	perf_print_counter(_drdy_count_perf);
	perf_print_counter(_drdy_interval_perf);
}
