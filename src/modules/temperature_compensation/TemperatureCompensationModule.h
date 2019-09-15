/****************************************************************************
 *
 *   Copyright (c) 2016-2019 PX4 Development Team. All rights reserved.
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

#pragma once

#include <drivers/drv_hrt.h>

#include <mathlib/mathlib.h>

#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/sensor_baro.h>
#include <uORB/topics/sensor_correction.h>
#include <px4_config.h>
#include <px4_module.h>
#include <px4_module_params.h>
#include <px4_getopt.h>
#include <px4_posix.h>
#include <px4_tasks.h>
#include <px4_time.h>
#include <lib/mathlib/mathlib.h>

#include <drivers/drv_hrt.h>

#include <lib/parameters/param.h>
#include <lib/perf/perf_counter.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/parameter_update.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include "TemperatureCompensation.h"

namespace temperature_compensation
{

/**
 ** class TemperatureCompensationModule
 */
class TemperatureCompensationModule : public ModuleBase<TemperatureCompensationModule>, public ModuleParams,
	public px4::ScheduledWorkItem
{
public:
	TemperatureCompensationModule();
	~TemperatureCompensationModule();

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static TemperatureCompensationModule *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	void Run() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;

	/**
	 * Initializes scheduling on work queue.
	 */
	bool init();

private:

	/**
	 * Poll the accelerometer for updated data.
	 */
	void		accel_poll();

	/**
	 * Poll the gyro for updated data.
	 */
	void		gyro_poll();

	/**
	 * Poll the barometer for updated data.
	 */
	void		baro_poll();

	/**
	 * call this whenever parameters got updated. Make sure to have initialize_sensors() called at least
	 * once before calling this.
	 */
	void		parameters_update();

	uORB::Subscription _accel_subs[ACCEL_COUNT_MAX] {
		{ORB_ID(sensor_accel), 0},
		{ORB_ID(sensor_accel), 1},
		{ORB_ID(sensor_accel), 2}
	};

	uORB::Subscription _gyro_subs[GYRO_COUNT_MAX] {
		{ORB_ID(sensor_gyro), 0},
		{ORB_ID(sensor_gyro), 1},
		{ORB_ID(sensor_gyro), 2}
	};

	uORB::Subscription _baro_subs[BARO_COUNT_MAX] {
		{ORB_ID(sensor_baro), 0},
		{ORB_ID(sensor_baro), 1},
		{ORB_ID(sensor_baro), 2}
	};

	uORB::Subscription	_params_sub{ORB_ID(parameter_update)};				/**< notification of parameter updates */

	perf_counter_t		_loop_perf;			/**< loop performance counter */

	orb_advert_t		_mavlink_log_pub{nullptr};

	/* sensor thermal compensation */
	TemperatureCompensation _temperature_compensation;

	sensor_correction_s _corrections{}; /**< struct containing the sensor corrections to be published to the uORB*/
	uORB::Publication<sensor_correction_s>	_sensor_correction_pub{ORB_ID(sensor_correction)};	/**< handle to the sensor correction uORB topic */

	bool _corrections_changed{true};

protected:
	// Volatile because threads
	static volatile bool _start_calibration;
	static volatile bool _is_accel_calibration;
	static volatile bool _is_baro_calibration;
	static volatile bool _is_gyro_calibration;
	volatile bool _is_calibrating{false};
};

volatile bool TemperatureCompensationModule::_start_calibration = false;
volatile bool TemperatureCompensationModule::_is_accel_calibration = false;
volatile bool TemperatureCompensationModule::_is_baro_calibration = false;
volatile bool TemperatureCompensationModule::_is_gyro_calibration = false;


} // namespace temperature_compensation
