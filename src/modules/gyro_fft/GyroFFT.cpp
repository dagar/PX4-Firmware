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

#include "GyroFFT.hpp"

#include <drivers/drv_hrt.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>

using namespace matrix;
using namespace time_literals;
using math::radians;

GyroFFT::GyroFFT() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
	for (int axis = 0; axis < 3; axis++) {
		arm_rfft_init_q15(&_rfft_q15[axis], FFT_LENGTH, 0, 1);
	}
}

GyroFFT::~GyroFFT()
{
	perf_free(_cycle_perf);
	perf_free(_cycle_interval_perf);
	perf_free(_fft_perf);
}

bool GyroFFT::init()
{
	if (!_sensor_gyro_fifo_sub.registerCallback()) {
		PX4_ERR("sensor_gyro_fifo callback registration failed!");
		return false;
	}

	return true;
}

void GyroFFT::Run()
{
	if (should_exit()) {
		_sensor_gyro_fifo_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_cycle_perf);
	perf_count(_cycle_interval_perf);

	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		updateParams();
		//parameters_updated();
	}

	bool updated = false;

	/* run controller on gyro changes */
	sensor_gyro_fifo_s sensor_gyro_fifo;

	while (_sensor_gyro_fifo_sub.update(&sensor_gyro_fifo)) {

		if (_sensor_gyro_fifo_sub.get_last_generation() != _gyro_last_generation + 1) {
			// force reset if we've missed a sample
			_fft_buffer_index[0] = 0;
			_fft_buffer_index[1] = 0;
			_fft_buffer_index[2] = 0;
		}

		_gyro_last_generation = _sensor_gyro_fifo_sub.get_last_generation();

		const int N = sensor_gyro_fifo.samples;

		for (int axis = 0; axis < 3; axis++) {
			int16_t *input = nullptr;

			switch (axis) {
			case 0:
				input = sensor_gyro_fifo.x;
				break;

			case 1:
				input = sensor_gyro_fifo.y;
				break;

			case 2:
				input = sensor_gyro_fifo.z;
				break;
			}

			for (int n = 0; n < N; n++) {
				int &buffer_index = _fft_buffer_index[axis];

				_fft_input_buffer[axis][buffer_index] = input[n] / 2;
				buffer_index++;

				// if we have enough samples, begin processing
				if (buffer_index >= FFT_LENGTH) {
					perf_begin(_fft_perf);
					arm_rfft_q15(&_rfft_q15[axis], _fft_input_buffer[axis], _fft_output_buffer[axis]);
					perf_end(_fft_perf);

					// reset
					buffer_index = 0;

					updated = true;
				}
			}
		}
	}


	if (updated) {

		const int N = math::min((unsigned)FFT_LENGTH, sizeof(dynamic_notch_s::x) / sizeof(dynamic_notch_s::x[0]));

		// publish
		for (int axis = 0; axis < 3; axis++) {
			int16_t *output = nullptr;

			switch (axis) {
			case 0:
				output = _dynamic_notch.x;
				break;

			case 1:
				output = _dynamic_notch.y;
				break;

			case 2:
				output = _dynamic_notch.z;
				break;
			}

			int16_t max = 0;

			for (unsigned n = 0; n < N; n++) {
				output[n] = _fft_output_buffer[axis][n];

				if (abs(output[n]) >= max) {
					_dynamic_notch.max_index[axis] = n;
					max = abs(output[n]);
				}
			}
		}

		_dynamic_notch.samples = FFT_LENGTH;
		_dynamic_notch.timestamp = hrt_absolute_time();
		_dynamic_notch_pub.publish(_dynamic_notch);
	}

	perf_end(_cycle_perf);
}

int GyroFFT::task_spawn(int argc, char *argv[])
{
	GyroFFT *instance = new GyroFFT();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int GyroFFT::print_status()
{
	perf_print_counter(_cycle_perf);
	perf_print_counter(_cycle_interval_perf);
	perf_print_counter(_fft_perf);
	return 0;
}

int GyroFFT::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int GyroFFT::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("gyro_fft", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int gyro_fft_main(int argc, char *argv[])
{
	return GyroFFT::main(argc, argv);
}
