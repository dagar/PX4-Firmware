/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
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

#include "MulticopterRateControl.hpp"

MulticopterRateControl::MulticopterRateControl() :
	ORBActor(px4::wq_configurations::rate_ctrl, ORB_ID(sensor_gyro), 0),
	ModuleParams(nullptr),
	_loop_perf(perf_alloc(PC_ELAPSED, "mc_rate_control")),
	_loop_interval_perf(perf_alloc(PC_INTERVAL, "mc_rate_control: interval")),
	_gyro_latency_perf(perf_alloc(PC_ELAPSED, "mc_rate_control: gyro latency"))
{
}

MulticopterRateControl::~MulticopterRateControl()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
	perf_free(_gyro_latency_perf);
}

void
MulticopterRateControl::OnUpdate(const sensor_gyro_s &gyro)
{
	perf_count(_loop_interval_perf);
	perf_begin(_loop_perf);

	const hrt_abstime now = hrt_absolute_time();

	perf_set_elapsed(_gyro_latency_perf, now - gyro.timestamp);

	// control_attitude_rates();

	perf_end(_loop_perf);
}

int
MulticopterRateControl::print_status()
{
	PX4_INFO("running");

	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	perf_print_counter(_gyro_latency_perf);

	return PX4_OK;
}

void
MulticopterRateControl::request_stop()
{
	Unregister();

	ModuleBase::request_stop();

	exit_and_cleanup();
}

int
MulticopterRateControl::task_spawn(int argc, char *argv[])
{
	MulticopterRateControl *instance = new MulticopterRateControl();

	if (instance != nullptr) {
		if (instance->Register()) {
			_object.store(instance);
			_task_id = task_id_is_work_queue;
			return PX4_OK;
		}

	} else {
		delete instance;
	}

	return PX4_ERROR;
}

int
MulticopterRateControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int
MulticopterRateControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This implements the multicopter rate controller. It takes rate setpoints (in acro mode
via `manual_control_setpoint` topic) as inputs and outputs actuator control messages.

The controller has a PID loop for angular rate error.

Publication documenting the implemented Quaternion Attitude Control:
Nonlinear Quadrocopter Attitude Control (2013)
by Dario Brescianini, Markus Hehn and Raffaello D'Andrea
Institute for Dynamic Systems and Control (IDSC), ETH Zurich

https://www.research-collection.ethz.ch/bitstream/handle/20.500.11850/154099/eth-7387-01.pdf

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("mc_rate_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

/**
 * Multicopter rate control app start / stop handling function
 */
extern "C" __EXPORT int mc_rate_control_main(int argc, char *argv[]);

int mc_rate_control_main(int argc, char *argv[])
{
	return MulticopterRateControl::main(argc, argv);
}
