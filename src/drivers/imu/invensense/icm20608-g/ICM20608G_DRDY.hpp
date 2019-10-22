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
 * @file ICM20608g.hpp
 *
 * Driver for the Invensense ICM20608G connected via SPI.
 *
 */

#pragma once

#include "ICM20608G.hpp"

class ICM20608G_DRDY : public px4::WorkItem
{
public:
	ICM20608G_DRDY(int bus, uint32_t device, enum Rotation rotation, uint32_t drdy_gpio);
	virtual ~ICM20608G_DRDY();

	void			Start() override;
	void			Stop() override;

private:

	static int		DataReadyInterruptCallback(int irq, void *context, void *arg);
	void			DataReady();

	perf_counter_t		_drdy_count_perf{perf_alloc(PC_COUNT, MODULE_NAME": drdy count")};
	perf_counter_t		_drdy_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": drdy interval")};

	hrt_abstime		_time_data_ready{0};
	int			_data_ready_count{0};

	const uint32_t		_drdy_gpio{0};

};
