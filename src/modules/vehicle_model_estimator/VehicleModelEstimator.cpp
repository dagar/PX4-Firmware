/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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
 * @file VehicleModelEstimator.cpp
 *
 * Vehicle model estimator.
 *
 * To estimate the vehicle inertia, the estimator low-pass filters and
 * computes the time derivative of the vehicle angular acceleration
 * and of the vehicle torque setpoint, then applies a least-mean-square.
 * Similarly, to estimate the vehicle mass, the estimator low-pass
 * filters and computes the time derivative of the vehicle acceleration
 * and of the vehicle thrust setpoint, then applies a least-mean-square.
 *
 * @author Julien Lecoeur <julien.lecoeur@gmail.com>
 */

#include "VehicleModelEstimator.hpp"

#include <drivers/drv_hrt.h>
#include <matrix/matrix/math.hpp>

using namespace matrix;
using namespace time_literals;

VehicleModelEstimator::VehicleModelEstimator() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
	parameters_updated();
	_I_est_inv = inv(_I);
	_mass_est_inv = 1.f / _mass;
}

VehicleModelEstimator::~VehicleModelEstimator()
{
	perf_free(_loop_perf);
}

bool VehicleModelEstimator::init()
{
	if (!_vehicle_angular_acceleration_sub.registerCallback()) {
		PX4_ERR("callback registration failed!");
		return false;
	}

	if (!_vehicle_acceleration_sub.registerCallback()) {
		PX4_ERR("callback registration failed!");
		return false;
	}

	if (!_vehicle_torque_setpoint_sub.registerCallback()) {
		PX4_ERR("callback registration failed!");
		return false;
	}

	if (!_vehicle_thrust_setpoint_sub.registerCallback()) {
		PX4_ERR("callback registration failed!");
		return false;
	}

	return true;
}

void VehicleModelEstimator::parameters_updated()
{
	// vehicle inertia matrix
	const float inertia[3][3] = {
		{_param_vm_inertia_xx.get(), _param_vm_inertia_xy.get(), _param_vm_inertia_xz.get()},
		{_param_vm_inertia_xy.get(), _param_vm_inertia_yy.get(), _param_vm_inertia_yz.get()},
		{_param_vm_inertia_xz.get(), _param_vm_inertia_yz.get(), _param_vm_inertia_zz.get()}
	};
	_I = matrix::Matrix3f(inertia);

	// vehicle mass
	_mass = _param_vm_mass.get();

	// low pass filters
	if (fabsf(_lpf_ang_accel.get_cutoff_freq() - _param_vm_est_cutoff.get()) > 0.01f) {
		_lpf_ang_accel.set_cutoff_frequency(_loop_update_rate_hz_ang_accel, _param_vm_est_cutoff.get());
		_lpf_ang_accel.reset(_ang_accel_filtered);
	}

	if (fabsf(_lpf_torque_sp.get_cutoff_freq() - _param_vm_est_cutoff.get()) > 0.01f) {
		_lpf_torque_sp.set_cutoff_frequency(_loop_update_rate_hz_torque_sp, _param_vm_est_cutoff.get());
		_lpf_torque_sp.reset(_torque_sp_filtered);
	}

	if (fabsf(_lpf_lin_accel.get_cutoff_freq() - _param_vm_est_cutoff.get()) > 0.01f) {
		_lpf_lin_accel.set_cutoff_frequency(_loop_update_rate_hz_lin_accel, _param_vm_est_cutoff.get());
		_lpf_lin_accel.reset(_lin_accel_filtered);
	}

	if (fabsf(_lpf_thrust_sp.get_cutoff_freq() - _param_vm_est_cutoff.get()) > 0.01f) {
		_lpf_thrust_sp.set_cutoff_frequency(_loop_update_rate_hz_thrust_sp, _param_vm_est_cutoff.get());
		_lpf_thrust_sp.reset(_thrust_sp_filtered);
	}
}

void VehicleModelEstimator::Run()
{
	if (should_exit()) {
		_vehicle_angular_acceleration_sub.unregisterCallback();
		_vehicle_acceleration_sub.unregisterCallback();
		_vehicle_torque_setpoint_sub.unregisterCallback();
		_vehicle_thrust_setpoint_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		updateParams();
		parameters_updated();
	}

	// check for updates in other topics
	if (_vehicle_land_detected_sub.updated()) {
		vehicle_land_detected_s vehicle_land_detected;

		if (_vehicle_land_detected_sub.copy(&vehicle_land_detected)) {
			_landed = vehicle_land_detected.landed;
			_maybe_landed = vehicle_land_detected.maybe_landed;
		}
	}

	if (_vehicle_control_mode_sub.updated()) {
		vehicle_control_mode_s vehicle_control_mode;

		if (_vehicle_control_mode_sub.copy(&vehicle_control_mode)) {
			_armed = vehicle_control_mode.flag_armed;
		}
	}

	const hrt_abstime now = hrt_absolute_time();

	/* run inertia estimator on angular acceleration and torque setpoint changes */
	bool inertia_update = false;

	const unsigned vehicle_angular_acceleration_last_generation = _vehicle_angular_acceleration_sub.get_last_generation();
	vehicle_angular_acceleration_s angular_acceleration;

	if (_vehicle_angular_acceleration_sub.update(&angular_acceleration)) {
		if (_vehicle_angular_acceleration_sub.get_last_generation() != vehicle_angular_acceleration_last_generation + 1) {
			PX4_ERR("vehicle_angular_acceleration update missed");
		}

		const float dt_ang_accel = math::constrain(((angular_acceleration.timestamp_sample - _timestamp_ang_accel) / 1e6f),
					   0.0002f, 0.02f);
		_timestamp_ang_accel = angular_acceleration.timestamp_sample;

		// Calculate loop update rate
		_dt_accumulator_ang_accel += dt_ang_accel;
		++_loop_counter_ang_accel;

		if (_dt_accumulator_ang_accel > 2.f) {
			const float loop_update_rate = (float)_loop_counter_ang_accel / _dt_accumulator_ang_accel;
			_loop_update_rate_hz_ang_accel = _loop_update_rate_hz_ang_accel * 0.5f + loop_update_rate * 0.5f;

			if ((fabsf(_lpf_ang_accel.get_sample_freq() - _loop_update_rate_hz_ang_accel) / _loop_update_rate_hz_ang_accel) >
			    0.05f) {

				PX4_DEBUG("lpf_ang_accel: sample freq %.3f -> %.3f", (double)_lpf_ang_accel.get_sample_freq(),
					  (double)_loop_update_rate_hz_ang_accel);
				_lpf_ang_accel.set_cutoff_frequency(_loop_update_rate_hz_ang_accel, _param_vm_est_cutoff.get());
			}

			_dt_accumulator_ang_accel = 0.f;
			_loop_counter_ang_accel = 0;
		}

		// Apply low pass filter
		const Vector3f ang_accel_filtered_new(_lpf_ang_accel.apply(Vector3f(angular_acceleration.xyz)));

		// Time derivative
		_ang_accel_filtered_diff = (ang_accel_filtered_new - _ang_accel_filtered) / dt_ang_accel;
		_ang_accel_filtered = ang_accel_filtered_new;

		// Indicate that new data is available to estimate the vehicle inertia
		inertia_update = true;
	}


	const unsigned vehicle_torque_setpoint_last_generation = _vehicle_torque_setpoint_sub.get_last_generation();
	vehicle_torque_setpoint_s vehicle_torque_setpoint;

	if (_vehicle_torque_setpoint_sub.update(&vehicle_torque_setpoint)) {
		if (_vehicle_torque_setpoint_sub.get_last_generation() != vehicle_torque_setpoint_last_generation + 1) {
			PX4_ERR("vehicle_torque_setpoint update missed");

		} else {
			// Buffer incoming data
			_torque_sp_buffer.push(TimestampedVector3f{vehicle_torque_setpoint.timestamp_sample, Vector3f(vehicle_torque_setpoint.xyz)});
		}

		const hrt_abstime timestamp_delay = vehicle_torque_setpoint.timestamp_sample
						    - static_cast<uint64_t>(_param_vm_est_delay.get() * 1_s);
		TimestampedVector3f torque_setpoint;

		if (_torque_sp_buffer.pop_first_older_than(timestamp_delay, &torque_setpoint)) {

			const float dt_torque_sp = math::constrain(((torque_setpoint.time_us - _timestamp_torque_sp) / 1e6f), 0.0002f, 0.02f);
			_timestamp_torque_sp = torque_setpoint.time_us;

			// Calculate loop update rate
			_dt_accumulator_torque_sp += dt_torque_sp;
			_loop_counter_torque_sp++;

			if (_dt_accumulator_torque_sp > 2.f) {
				const float loop_update_rate = (float)_loop_counter_torque_sp / _dt_accumulator_torque_sp;
				_loop_update_rate_hz_torque_sp = _loop_update_rate_hz_torque_sp * 0.5f + loop_update_rate * 0.5f;

				if ((fabsf(_lpf_torque_sp.get_sample_freq() - loop_update_rate) / loop_update_rate) > 0.01f) {

					PX4_DEBUG("lpf_torque_sp: sample freq %.3f -> %.3f",
						  (double)_lpf_torque_sp.get_sample_freq(), (double)loop_update_rate);
					_lpf_torque_sp.set_cutoff_frequency(loop_update_rate, _param_vm_est_cutoff.get());

					// Compute required buffer size
					if (!_armed) {
						int _torque_sp_buffer_length = (_param_vm_est_delay.get() * loop_update_rate) + 1;

						if (_torque_sp_buffer.get_length() < _torque_sp_buffer_length) {
							PX4_ERR("Torque setpoint buffer too small (%d < %d)", _torque_sp_buffer.get_length(), _torque_sp_buffer_length);
						}
					}
				}

				_dt_accumulator_torque_sp = 0.f;
				_loop_counter_torque_sp = 0;
			}

			// Apply low pass filter
			const Vector3f torque_sp_filtered_new(_lpf_torque_sp.apply(torque_setpoint.data));

			// Time derivative
			_torque_sp_filtered_diff = (torque_sp_filtered_new - _torque_sp_filtered) / dt_torque_sp;
			_torque_sp_filtered = torque_sp_filtered_new;

			// Indicate that new data is available to estimate the vehicle inertia
			inertia_update = true;
		}
	}


	/* run mass estimator on linear acceleration and thrust setpoint changes */
	bool mass_update = false;

	const unsigned vehicle_acceleration_sub_setpoint_last_generation = _vehicle_acceleration_sub.get_last_generation();
	vehicle_acceleration_s acceleration;

	if (_vehicle_acceleration_sub.update(&acceleration)) {
		if (_vehicle_acceleration_sub.get_last_generation() != vehicle_acceleration_sub_setpoint_last_generation + 1) {
			PX4_ERR("vehicle_acceleration update missed");
		}

		const float dt_lin_accel = math::constrain(((acceleration.timestamp_sample - _timestamp_lin_accel) / 1e6f), 0.0002f,
					   0.02f);
		_timestamp_lin_accel = acceleration.timestamp_sample;

		// Calculate loop update rate
		_dt_accumulator_lin_accel += dt_lin_accel;
		++_loop_counter_lin_accel;

		if (_dt_accumulator_lin_accel > 2.f) {
			const float loop_update_rate = (float)_loop_counter_lin_accel / _dt_accumulator_lin_accel;
			_loop_update_rate_hz_lin_accel = _loop_update_rate_hz_lin_accel * 0.5f + loop_update_rate * 0.5f;

			if ((fabsf(_lpf_lin_accel.get_sample_freq() - _loop_update_rate_hz_lin_accel) / _loop_update_rate_hz_lin_accel) >
			    0.05f) {

				PX4_DEBUG("lpf_lin_accel: sample freq %.3f -> %.3f",
					  (double)_lpf_lin_accel.get_sample_freq(), (double)_loop_update_rate_hz_lin_accel);
				_lpf_lin_accel.set_cutoff_frequency(_loop_update_rate_hz_lin_accel, _param_vm_est_cutoff.get());
			}

			_dt_accumulator_lin_accel = 0.f;
			_loop_counter_lin_accel = 0;
		}

		// Apply low pass filter
		const Vector3f lin_accel_filtered_new(_lpf_lin_accel.apply(Vector3f(acceleration.xyz)));

		// Time derivative
		_lin_accel_filtered_diff = (lin_accel_filtered_new - _lin_accel_filtered) / dt_lin_accel;
		_lin_accel_filtered = lin_accel_filtered_new;

		// Indicate that new data is available to estimate the vehicle mass
		mass_update = true;
	}


	const unsigned vehicle_thrust_setpoint_last_generation = _vehicle_thrust_setpoint_sub.get_last_generation();
	vehicle_thrust_setpoint_s vehicle_thrust_setpoint;

	if (_vehicle_thrust_setpoint_sub.update(&vehicle_thrust_setpoint)) {
		if (_vehicle_thrust_setpoint_sub.get_last_generation() != vehicle_thrust_setpoint_last_generation + 1) {
			PX4_ERR("vehicle_thrust_setpoint update missed");

		} else {
			// Buffer incoming data
			_thrust_sp_buffer.push(TimestampedVector3f{vehicle_thrust_setpoint.timestamp_sample, Vector3f(vehicle_thrust_setpoint.xyz)});
		}

		const hrt_abstime timestamp_delay = vehicle_thrust_setpoint.timestamp_sample - static_cast<uint64_t>
						    (_param_vm_est_delay.get() * 1_s);

		TimestampedVector3f thrust_setpoint;

		if (_thrust_sp_buffer.pop_first_older_than(timestamp_delay, &thrust_setpoint)) {
			const float dt_thrust_sp = math::constrain(((thrust_setpoint.time_us - _timestamp_thrust_sp) / 1e6f), 0.0002f,
						   0.02f);
			_timestamp_thrust_sp = thrust_setpoint.time_us;

			// Calculate loop update rate
			_dt_accumulator_thrust_sp += dt_thrust_sp;
			++_loop_counter_thrust_sp;

			if (_dt_accumulator_thrust_sp > 2.f) {
				const float loop_update_rate = (float)_loop_counter_thrust_sp / _dt_accumulator_thrust_sp;
				_loop_update_rate_hz_thrust_sp = _loop_update_rate_hz_thrust_sp * 0.5f + loop_update_rate * 0.5f;

				if ((fabsf(_lpf_thrust_sp.get_sample_freq() - _loop_update_rate_hz_thrust_sp) / _loop_update_rate_hz_thrust_sp) >
				    0.05f) {
					PX4_DEBUG("lpf_thrust_sp: sample freq %.3f -> %.3f",
						  (double)_lpf_thrust_sp.get_sample_freq(), (double)_loop_update_rate_hz_thrust_sp);
					_lpf_thrust_sp.set_cutoff_frequency(_loop_update_rate_hz_thrust_sp, _param_vm_est_cutoff.get());

					// Compute required buffer size
					if (!_armed) {
						int thrust_sp_buffer_length = (_param_vm_est_delay.get() * _loop_update_rate_hz_thrust_sp) + 1;

						if (_thrust_sp_buffer.get_length() < thrust_sp_buffer_length) {
							PX4_ERR("Thrust setpoint buffer too small %d < %d", _thrust_sp_buffer.get_length(), thrust_sp_buffer_length);
						}
					}
				}

				_dt_accumulator_thrust_sp = 0.f;
				_loop_counter_thrust_sp = 0;
			}

			// Apply low pass filter
			const Vector3f thrust_sp_filtered_new(_lpf_thrust_sp.apply(thrust_setpoint.data));

			// Time derivative
			_thrust_sp_filtered_diff = (thrust_sp_filtered_new - _thrust_sp_filtered) / dt_thrust_sp;
			_thrust_sp_filtered = thrust_sp_filtered_new;

			// Indicate that new data is available to estimate the vehicle mass
			mass_update = true;
		}
	}

	// Update inertia estimate only if we have new data and we are flying
	if (inertia_update && _armed && !_landed && !_maybe_landed) {
		// Predict angular jerk from time derivative of torque setpoint and estimated inertia
		const Matrix<float, 3, 1> jerk_pred = _I_est_inv * _torque_sp_filtered_diff;

		// Compute prediction error
		const matrix::Matrix<float, 3, 1> error = _ang_accel_filtered_diff - jerk_pred;

		// Update estimate
		_I_est_inv += error * _torque_sp_filtered_diff.T() * _param_vm_est_gain.get();

		// Sanity check
		bool reset_needed = false;

		for (size_t i = 0; i < 3; i++) {
			for (size_t j = 0; j < 3; j++) {
				if (!PX4_ISFINITE(_I_est_inv(i, j))) {
					reset_needed = true;
				}
			}
		}

		if (reset_needed) {
			_I_est_inv = inv(_I);
		}

		// Enforce positive values on diagonal
		for (size_t i = 0; i < 3; i++) {
			if (_I_est_inv(i, i) < FLT_EPSILON) {
				_I_est_inv(i, i) = FLT_EPSILON;
			}
		}

		// Enforce symmetry
		_I_est_inv(1, 0) = 0.5f * (_I_est_inv(0, 1) + _I_est_inv(1, 0));
		_I_est_inv(0, 1) = _I_est_inv(1, 0);
		_I_est_inv(2, 0) = 0.5f * (_I_est_inv(0, 2) + _I_est_inv(2, 0));
		_I_est_inv(0, 2) = _I_est_inv(2, 0);
		_I_est_inv(2, 1) = 0.5f * (_I_est_inv(1, 2) + _I_est_inv(2, 1));
		_I_est_inv(1, 2) = _I_est_inv(2, 1);
	}

	// Update mass estimate only if we have new data and we are flying
	if (mass_update && _armed && !_landed && !_maybe_landed) {
		// Predict jerk from time derivative of thrust setpoint and estimated mass
		Vector3f jerk_pred = _mass_est_inv * _thrust_sp_filtered_diff;

		// Compute prediction error
		Vector3f error = _lin_accel_filtered_diff - jerk_pred;

		// Update estimate
		_mass_est_inv += error.dot(_torque_sp_filtered_diff) * _param_vm_est_gain.get();

		// Sanity check
		if (!PX4_ISFINITE(_mass_est_inv)) {
			_mass_est_inv = 1.f / _mass;
		}

		// Enforce positive value
		if (_mass_est_inv < FLT_EPSILON) {
			_mass_est_inv = FLT_EPSILON;
		}

		if ((1.f / _mass_est_inv) < FLT_EPSILON) {
			_mass_est_inv = 1.f / _mass;
		}
	}

	// Publish estimated model
	if ((now - _prev_publish_time) > (1e6f / _param_vm_est_pubrate.get())) {
		_prev_publish_time = now;

		// Compute inertia matrix and mass
		Matrix3f I = _I;
		float mass = _mass;

		if (_param_vm_est_en.get()) {
			// Invert mass estimation
			mass = 1.f / _mass_est_inv;

			// Try to invert inertia estimation
			bool invertible = inv(_I_est_inv, I);

			if (!invertible) {
				// Fallback to default value
				I = _I;
				_I_est_inv = inv(_I);
			}

		} else {
			// reset estimation
			_I_est_inv = inv(_I);
			_mass_est_inv = 1.f / _mass;
		}

		// Publish
		vehicle_model_s vehicle_model;
		vehicle_model.mass = mass;
		I.copyTo(vehicle_model.inertia);
		vehicle_model.timestamp = hrt_absolute_time();
		_vehicle_model_pub.publish(vehicle_model);
	}

	perf_end(_loop_perf);
}

int VehicleModelEstimator::task_spawn(int argc, char *argv[])
{
	VehicleModelEstimator *instance = new VehicleModelEstimator();

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

int VehicleModelEstimator::print_status()
{
	PX4_INFO("Running");

	perf_print_counter(_loop_perf);

	return 0;
}

int VehicleModelEstimator::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int VehicleModelEstimator::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This implements the estimation of vehicle model (mass & inertia).
To estimate the vehicle inertia, the estimator low-pass filters and
computes the time derivative of the vehicle angular acceleration
and of the vehicle torque setpoint, then applies a least-mean-square.
Similarly, to estimate the vehicle mass, the estimator low-pass
filters and computes the time derivative of the vehicle acceleration
and of the vehicle thrust setpoint, then applies a least-mean-square.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME(MODULE_NAME, "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int vehicle_model_estimator_main(int argc, char *argv[])
{
	return VehicleModelEstimator::main(argc, argv);
}
