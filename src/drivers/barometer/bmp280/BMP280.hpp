/****************************************************************************
 *
 *   Copyright (c) 2012-2019 PX4 Development Team. All rights reserved.
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
 * @file bmp280.cpp
 * Driver for the BMP280 barometric pressure sensor connected via I2C TODO or SPI.
 */

#pragma once

#include "IBMP280.hpp"

#include <lib/drivers/barometer/PX4Barometer.hpp>

/*
 * BMP280 internal constants and data structures.
 */
class BMP280 : public px4::ScheduledWorkItem
{
public:
	BMP280(bmp280::IBMP280 *interface);
	virtual ~BMP280();

	virtual int		init();

	void			print_info();

private:
	bmp280::IBMP280	*_interface;

	PX4Barometer		_px4_baro;

	bool                	_running;

	uint8_t			_curr_ctrl;

	unsigned		_report_interval; // 0 - no cycling, otherwise period of sending a report
	unsigned		_max_measure_interval; // interval in microseconds needed to measure

	bool			_collect_phase;

	perf_counter_t		_sample_perf;
	perf_counter_t		_measure_perf;
	perf_counter_t		_comms_errors;

	struct bmp280::calibration_s *_cal; //stored calibration constants
	struct bmp280::fcalibration_s _fcal; //pre processed calibration constants

	float			_P; /* in Pa */
	float			_T; /* in K */

	void			start();
	void			stop();

	void			Run() override;

	int			measure(); //start measure
	int			collect(); //get results and publish
};
