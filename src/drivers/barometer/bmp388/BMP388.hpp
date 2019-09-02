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
 * @file bmp388.cpp
 * Driver for the BMP388 barometric pressure sensor connected via I2C
 * (SPI still TODO/test).
 */

#pragma once

#include "IBMP388.hpp"

#include <drivers/drv_hrt.h>
#include <drivers/device/i2c.h>
#include <drivers/device/spi.h>
#include <px4_config.h>
#include <lib/drivers/barometer/PX4Barometer.hpp>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <lib/perf/perf_counter.h>

enum BMP388_BUS {
	BMP388_BUS_ALL = 0,
	BMP388_BUS_I2C_INTERNAL,
	BMP388_BUS_I2C_INTERNAL1,
	BMP388_BUS_I2C_EXTERNAL,
	BMP388_BUS_SPI_INTERNAL,
	BMP388_BUS_SPI_EXTERNAL
};

/*
 * BMP388 internal constants and data structures.
 */

class BMP388 : public px4::ScheduledWorkItem
{
public:
	BMP388(bmp388::IBMP388 *interface);
	virtual ~BMP388();

	virtual int		init();

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void			print_info();

private:
	PX4Barometer		_px4_baro;

	bmp388::IBMP388		*_interface;

	bool               	_running{false};

	uint8_t			_curr_ctrl{0};

	unsigned		_report_interval{0}; // 0 - no cycling, otherwise period of sending a report
	unsigned		_max_measure_interval{0}; // interval in microseconds needed to measure

	bool			_collect_phase{false};

	perf_counter_t		_sample_perf;
	perf_counter_t		_measure_perf;
	perf_counter_t		_comms_errors;

	struct bmp388::calibration_s *_cal; // stored calibration constants
	struct bmp388::fcalibration_s _fcal; // pre processed calibration constants

	float			_P; /* in Pa */
	float			_T; /* in K */


	/* periodic execution helpers */
	void			start_cycle();
	void			stop_cycle();

	void			Run() override;

	int			measure(); //start measure
	int			collect(); //get results and publish

	// from BMP3 library...
	bool			soft_reset();
	bool			get_calib_data();
	bool			validate_trimming_param();
	bool 			set_sensor_settings();
	bool			set_op_mode(uint8_t op_mode);

	bool 			get_sensor_data(uint8_t sensor_comp, struct bmp3_data *comp_data);
	bool			compensate_data(uint8_t sensor_comp, const struct bmp3_uncomp_data *uncomp_data, struct bmp3_data *comp_data);
};
