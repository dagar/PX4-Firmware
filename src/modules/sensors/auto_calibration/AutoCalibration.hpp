/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#include <lib/mathlib/math/Limits.hpp>
#include <lib/matrix/matrix/math.hpp>
#include <lib/perf/perf_counter.h>
#include <lib/systemlib/mavlink_log.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/sensor_correction.h>

namespace sensors
{
class AutoCalibration : public ModuleParams, public px4::ScheduledWorkItem
{
public:

	AutoCalibration();
	~AutoCalibration() override;

	bool Start();
	void Stop();

	void PrintStatus();

private:
	void Run() override;

	void ParametersUpdate();
	void SensorCorrectionsUpdate(bool force = false);

	static constexpr int MAX_SENSOR_COUNT = 3;

	uORB::Subscription _params_sub{ORB_ID(parameter_update)};
	uORB::Subscription _sensor_correction_sub{ORB_ID(sensor_correction)};

	uORB::Subscription _sensor_accel_sub[MAX_SENSOR_COUNT] {{ORB_ID(sensor_accel), 0}, {ORB_ID(sensor_accel), 1}, {ORB_ID(sensor_accel), 2}};
	uORB::Subscription _sensor_gyro_sub[MAX_SENSOR_COUNT] {{ORB_ID(sensor_gyro), 0}, {ORB_ID(sensor_gyro), 1}, {ORB_ID(sensor_gyro), 2}};

	int _accel_sum_count[3] {};
	int _gyro_sum_count[3] {};

	matrix::Vector3f _accel_sum[3] {};
	matrix::Vector3f _gyro_sum[3] {};

	matrix::Vector3f _accel_thermal_offset[3] {};
	matrix::Vector3f _gyro_thermal_offset[3] {};

	float _accel_temperature[3] {};
	float _gyro_temperature[3] {};

	struct calibration {
		matrix::Vector3f offset;
		float temperature{0.f};
	};


	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};

};
}; // namespace sensors
