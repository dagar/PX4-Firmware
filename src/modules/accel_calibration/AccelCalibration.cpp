/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

#include "AccelCalibration.hpp"

#include <lib/ecl/geo/geo.h>

using namespace time_literals;
using matrix::Vector3f;

AccelCalibration::AccelCalibration() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
}

AccelCalibration::~AccelCalibration()
{
	perf_free(_loop_interval_perf);
	perf_free(_calibration_updated_perf);
}

bool AccelCalibration::init()
{
	ScheduleOnInterval(INTERVAL_US);
	return true;
}

void AccelCalibration::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_count(_loop_interval_perf);

	if (_vehicle_status_sub.updated()) {
		vehicle_status_s vehicle_status;

		if (_vehicle_status_sub.copy(&vehicle_status)) {
			const bool armed = (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);

			if (armed != _armed) {
				if (!_armed && armed) {
					// run at minimal rate unless disarmed
					ScheduleOnInterval(3_s);

				} else if (_armed && !armed) {
					ScheduleOnInterval(INTERVAL_US);
				}

				_armed = armed;
				//Reset();
			}
		}
	}

	if (_armed) {
		// do nothing if armed
		return;
	}

	if (_vehicle_status_flags_sub.updated()) {
		vehicle_status_flags_s vehicle_status_flags;

		if (_vehicle_status_flags_sub.copy(&vehicle_status_flags)) {
			if (_system_calibrating != vehicle_status_flags.condition_calibration_enabled) {
				//Reset();
				_system_calibrating = vehicle_status_flags.condition_calibration_enabled;
			}
		}
	}

	if (_system_calibrating) {
		// do nothing if system is calibrating
		return;
	}


	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		for (auto &cal : _accel_calibration) {
			cal.ParametersUpdate();
		}
	}

	// check all accelerometers for possible movement
	for (int accel = 0; accel < _sensor_accel_subs.size(); accel++) {
		sensor_accel_s sensor_accel;

		if (_sensor_accel_subs[accel].update(&sensor_accel)) {
			//const Vector3f acceleration{sensor_accel.x, sensor_accel.y, sensor_accel.z};
		}
	}
}

void AccelCalibration::error_function(M_mat, b_vec, y)
{
	// Optimisation error function for a point.
	//  param numpy.ndarray M_mat: The scale factor matrix of this iteration.
	//  param numpy.ndarray b_vec: The zero-g offset vector of this iteration.
	//  param numpy.ndarray y: The point ot estimate error for.
	// return: The square sum of the error of this point.

	return float(np.sum((M_mat.dot((y - b_vec)) **2)) - 1)
}

void AccelCalibration::calculate_jacobian(const matrix::SquareMatrix<float, 3>& M, const matrix::Vector3f& b, const matrix::Vector3f& p)
{
	// Calculate the Jacobian for a point.
	//  M: The scale factor matrix of this iteration.
	//  b: The zero-g offset vector of this iteration.
	//  p: The point to estimate error for.

	matrix::Vector<float, 9> jac;

	jac(0) = 2 * (b(0) - p(0)) * (M(0, 0) * (b(0) - p(0)) + M(0, 1) * (b(1) - p(1)) + M(0, 2) * (b(2) - p(2)));

	jac(1) = 2 * (b(1) - p(1)) * (M(0, 0) * (b(0) - p(0)) + M(0, 1) * (b(1) - p(1)) + M(0, 2) * (b(2) - p(2))) + 2 * (b(0) - p(0)) * (M(0,1) * (b(0) - p(0)) + M(1, 1) * (b(1) - p(1)) + M(1, 2) * (b(2) - p(2)));

	jac(2) = 2 * (b(0) - p(0)) * (M(0, 2) * (b(0) - p(0)) + M(1, 2) * (b(1) - p(1)) + M(2, 2) * (b(2) - p(2))) + 2 * (b(2) - p(2)) * (M(0, 0) * (b(0) - p(0)) + M(0, 1) * (b(1) - p(1)) + M(0, 2) * (b(2) - p(2)));

	jac(3) = 2 * (b(1) - p(1)) * (M(0, 1) * (b(0) - p(0)) + M(1, 1) * (b(1) - p(1)) + M(1, 2) * (b(2) - p(2)));

	jac(4) = 2 * (b(1) - p(1)) * (M(0, 2) * (b(0) - p(0)) + M(1, 2) * (b(1) - p(1)) + M(2, 2) * (b(2) - p(2))) + 2 * (b(2) - p(2)) * (M(0, 1) * (b(0) - p(0)) + M(1, 1) * (b(1) - p(1)) + M(1, 2) * (b(2) - p(2)));

	jac(5) = 2 * (b(2) - p(2)) * (M(0, 2) * (b(0) - p(0)) + M(1, 2) * (b(1) - p(1)) + M(2, 2) * (b(2) - p(2)));

	jac(6) = 2 * M(0, 0) * (M(0, 0) * (b(0) - p(0)) + M(0, 1) * (b(1) - p(1)) + M(0, 2) * (b(2) - p(2))) + 2 * M(0, 1) * (M(0, 1) * (b(0) - p(0)) + M(1, 1) * (b(1) - p(1)) + M(1, 2) * (b(2) - p(2))) + 2 * M(0, 2) * (M(0, 2) * (b(0) - p(0)) + M(1, 2) * (b(1) - p(1)) + M(2, 2) * (b(2) - p(2)));

	jac(7) = 2 * M(0, 1) * (M(0, 0) * (b(0) - p(0)) + M(0, 1) * (b(1) - p(1)) + M(0, 2) * (b(2) - p(2))) + 2 * M(1, 1) * (M(0, 1) * (b(0) - p(0)) + M(1, 1) * (b(1) - p(1)) + M(1, 2) * (b(2) - p(2))) + 2 * M(1, 2) * (M(0, 2) * (b(0) - p(0)) + M(1, 2) * (b(1) - p(1)) + M(2, 2) * (b(2) - p(2)));

	jac(8) = 2 * M(0, 2) * (M(0, 0) * (b(0) - p(0)) + M(0, 1) * (b(1) - p(1)) + M(0, 2) * (b(2) - p(2))) + 2 * M(1, 2) * (M(0, 1) * (b(0) - p(0)) + M(1, 1) * (b(1) - p(1)) + M(1, 2) * (b(2) - p(2))) + 2 * M(2, 2) * (M(0, 2) * (b(0) - p(0)) + M(1, 2) * (b(1) - p(1)) + M(2, 2) * (b(2) - p(2)));

	return jac;
}

int AccelCalibration::task_spawn(int argc, char *argv[])
{
	AccelCalibration *instance = new AccelCalibration();

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

int AccelCalibration::print_status()
{
	// for (int gyro = 0; gyro < _sensor_gyro_subs.size(); gyro++) {
	// 	if (_gyro_calibration[gyro].device_id() != 0) {
	// 		PX4_INFO_RAW("gyro %d (%d), [%.5f, %.5f, %.5f] var: [%.9f, %.9f, %.9f] %.1f°C (count %d)\n",
	// 			     gyro, _accel_calibration[gyro].device_id(),
	// 			     (double)_gyro_mean[gyro].mean()(0), (double)_gyro_mean[gyro].mean()(1), (double)_gyro_mean[gyro].mean()(2),
	// 			     (double)_gyro_mean[gyro].variance()(0), (double)_gyro_mean[gyro].variance()(1), (double)_gyro_mean[gyro].variance()(2),
	// 			     (double)_temperature[gyro], _gyro_mean[gyro].count());
	// 	}
	// }

	perf_print_counter(_loop_interval_perf);
	perf_print_counter(_calibration_updated_perf);
	return 0;
}

int AccelCalibration::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int AccelCalibration::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Online accelerometer calibration.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("accel_calibration", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int accel_calibration_main(int argc, char *argv[])
{
	return AccelCalibration::main(argc, argv);
}
