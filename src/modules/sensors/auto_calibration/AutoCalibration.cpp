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

#include "AutoCalibration.hpp"

#include <px4_platform_common/log.h>
#include <lib/ecl/geo/geo.h>

namespace sensors
{

using namespace matrix;
using namespace time_literals;

AutoCalibration::AutoCalibration() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
}

AutoCalibration::~AutoCalibration()
{
	Stop();
	perf_free(_cycle_perf);
}

bool AutoCalibration::Start()
{
	ScheduleNow();
	return true;
}

void AutoCalibration::Stop()
{
	Deinit();
}

void AutoCalibration::SensorCorrectionsUpdate(bool force)
{
	if (_sensor_correction_sub.updated() || force) {
		sensor_correction_s corrections;

		if (_sensor_correction_sub.copy(&corrections)) {

			_accel_thermal_offset[0] = Vector3f{corrections.accel_offset_0};
			_accel_thermal_offset[1] = Vector3f{corrections.accel_offset_1};
			_accel_thermal_offset[2] = Vector3f{corrections.accel_offset_2};

			_gyro_thermal_offset[0] = Vector3f{corrections.gyro_offset_0};
			_gyro_thermal_offset[1] = Vector3f{corrections.gyro_offset_1};
			_gyro_thermal_offset[2] = Vector3f{corrections.gyro_offset_2};

		}
	}
}

void AutoCalibration::ParametersUpdate()
{
	// Check if parameters have changed
	if (_params_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_params_sub.copy(&param_update);

		updateParams();
	}
}

void AutoCalibration::Run()
{
	perf_begin(_cycle_perf);

	ParametersUpdate();

	SensorCorrectionsUpdate();

	for (int i = 0; i < 3; i++) {

		sensor_accel_s accel;

		while (_sensor_accel_sub[i].update(&accel)) {
			_accel_sum_count[i]++;
			_accel_sum[i] += Vector3f{accel.x, accel.y, accel.z} - _accel_thermal_offset[i];

			_accel_temperature[i] += accel.temperature;
		}

		sensor_gyro_s gyro;

		while (_sensor_gyro_sub[i].update(&gyro)) {
			_gyro_sum_count[i]++;
			_gyro_sum[i] += Vector3f{gyro.x, gyro.y, gyro.z} - _gyro_thermal_offset[i];

			_gyro_temperature[i] += gyro.temperature;
		}

		// detect still (gyro)

		// detect still (accel)
		//   magnitude constant and close to 1 G
	}


	// reschedule timeout
	ScheduleDelayed(10_ms);

	perf_end(_cycle_perf);
}

void AutoCalibration::PrintStatus()
{

}

}; // namespace sensors
