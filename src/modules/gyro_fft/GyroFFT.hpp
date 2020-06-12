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

#pragma once

#include <lib/matrix/matrix/math.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/dynamic_notch.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_gyro_fifo.h>

#include "arm_math.h"
#include "arm_const_structs.h"

class GyroFFT : public ModuleBase<GyroFFT>, public ModuleParams, public px4::WorkItem
{
public:
	GyroFFT();
	~GyroFFT() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::print_status() */
	int print_status() override;

	bool init();

private:
	void Run() override;

	uORB::Publication<dynamic_notch_s> _dynamic_notch_pub{ORB_ID(dynamic_notch)};

	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};

	uORB::SubscriptionCallbackWorkItem _sensor_gyro_fifo_sub{this, ORB_ID(sensor_gyro_fifo)};

	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t _cycle_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": cycle interval")};
	perf_counter_t _fft_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": FFT")};

	static constexpr uint16_t FFT_LENGTH = 512;
	arm_rfft_instance_q15 _rfft_q15[3];

	q15_t _fft_input_buffer[3][FFT_LENGTH] {};
	q15_t _fft_output_buffer[3][FFT_LENGTH * 2] {}; // arm_rfft_q15: the output buffer must have length 2*N

	int _fft_buffer_index[3] {};

	unsigned _gyro_last_generation{0};

	dynamic_notch_s _dynamic_notch{};


	DEFINE_PARAMETERS(

		(ParamInt<px4::params::CBRK_RATE_CTRL>) _param_cbrk_rate_ctrl
	)
};
