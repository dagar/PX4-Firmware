/****************************************************************************
 *
 *   Copyright (c) 2013-2018 PX4 Development Team. All rights reserved.
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
 * @file mc_att_control_main.cpp
 * Multicopter attitude controller.
 *
 * @author Lorenz Meier		<lorenz@px4.io>
 * @author Anton Babushkin	<anton.babushkin@me.com>
 * @author Sander Smeets	<sander@droneslab.com>
 * @author Matthias Grob	<maetugr@gmail.com>
 * @author Beat Küng		<beat-kueng@gmx.net>
 *
 */

#include "mc_att_control.hpp"

#include <drivers/drv_hrt.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>

using namespace matrix;

MulticopterAttitudeControl::MulticopterAttitudeControl() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::att_pos_ctrl),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	_vehicle_status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;

	parameters_updated();
}

MulticopterAttitudeControl::~MulticopterAttitudeControl()
{
	perf_free(_loop_perf);
}

bool
MulticopterAttitudeControl::init()
{
	if (!_vehicle_attitude_sub.registerCallback()) {
		PX4_ERR("vehicle_attitude callback registration failed!");
		return false;
	}

	return true;
}

void
MulticopterAttitudeControl::parameters_updated()
{
	// Store some of the parameters in a more convenient way & precompute often-used values
	_attitude_control.setProportionalGain(Vector3f(_param_mc_roll_p.get(), _param_mc_pitch_p.get(), _param_mc_yaw_p.get()));

	// angular rate limits
	using math::radians;
	_attitude_control.setRateLimit(Vector3f(radians(_param_mc_rollrate_max.get()), radians(_param_mc_pitchrate_max.get()),
						radians(_param_mc_yawrate_max.get())));
}

void
MulticopterAttitudeControl::vehicle_status_poll()
{
	/* check if there is new status information */
	if (_vehicle_status_sub.update(&_vehicle_status)) {
		/* set correct uORB ID, depending on if vehicle is VTOL or not */
		if (_attitude_sp_id == nullptr) {
			if (_vehicle_status.is_vtol) {
				_attitude_sp_id = ORB_ID(mc_virtual_attitude_setpoint);

				int32_t vt_type = -1;

				if (param_get(param_find("VT_TYPE"), &vt_type) == PX4_OK) {
					_is_tailsitter = (static_cast<vtol_type>(vt_type) == vtol_type::TAILSITTER);
				}

			} else {
				_attitude_sp_id = ORB_ID(vehicle_attitude_setpoint);
			}
		}
	}
}

void
MulticopterAttitudeControl::Run()
{
	if (should_exit()) {
		_vehicle_attitude_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	// Check if parameters have changed
	if (_params_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_params_sub.copy(&param_update);

		updateParams();
		parameters_updated();
	}


	_v_control_mode_sub.update(&_v_control_mode);

	if (_v_control_mode.flag_control_rattitude_enabled) {
		// Check if we are in rattitude mode and the pilot is above the threshold on pitch
		// or roll (yaw can rotate 360 in normal att control). If both are true don't
		// even bother running the attitude controllers
		manual_control_setpoint_s manual_control_sp{};	/**< manual control setpoint */

		if (_manual_control_sp_sub.copy(&manual_control_sp)) {
			_v_control_mode.flag_control_attitude_enabled =
				(fabsf(manual_control_sp.y) <= _param_mc_ratt_th.get()) &&
				(fabsf(manual_control_sp.x) <= _param_mc_ratt_th.get());
		}
	}

	// only run in attitude control mode
	if (_v_control_mode.flag_control_attitude_enabled) {

		vehicle_status_poll();

		if (_vehicle_status.is_vtol) {
			if (_vehicle_status.in_transition_mode && !_is_tailsitter) {
				// don't run during VTOL transition (except for tailsitter)
				return;
			}

			if (_vehicle_status.vehicle_type != vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
				// only run if currently in rotary wing mode
				return;
			}
		}

		// run controller on attitude updates
		vehicle_attitude_s v_att;

		if (_vehicle_attitude_sub.update(&v_att)) {
			if (_v_control_mode.flag_armed) {
				// Check for a heading reset
				if (_quat_reset_counter_prev != v_att.quat_reset_counter) {
					_q_d += Quatf{v_att.delta_q_reset};
					_quat_reset_counter_prev = v_att.quat_reset_counter;

				} else {
					vehicle_attitude_setpoint_s v_att_sp;

					if (_v_att_sp_sub.update(&v_att_sp)) {
						_q_d = Quatf(v_att_sp.q_d);

						_v_rates_sp.thrust_body[0] = v_att_sp.thrust_body[0];
						_v_rates_sp.thrust_body[1] = v_att_sp.thrust_body[1];
						_v_rates_sp.thrust_body[2] = v_att_sp.thrust_body[2];

						_yaw_sp_move_rate = v_att_sp.yaw_sp_move_rate;
					}
				}

			} else {
				// reinitialize the setpoint while not armed to make sure no value from the last mode or flight is still kept
				_q_d = Quatf(v_att.q);

				_v_rates_sp.thrust_body[0] = 0.0f;
				_v_rates_sp.thrust_body[1] = 0.0f;
				_v_rates_sp.thrust_body[2] = 0.0f;

				_yaw_sp_move_rate = 0.0f;
			}

			const Vector3f rates_sp = _attitude_control.update(Quatf(v_att.q), _q_d, _yaw_sp_move_rate);

			// publish rates setpoint
			_v_rates_sp.roll = rates_sp(0);
			_v_rates_sp.pitch = rates_sp(1);
			_v_rates_sp.yaw = rates_sp(2);
			_v_rates_sp.timestamp = hrt_absolute_time();
			_v_rates_sp_pub.publish(_v_rates_sp);
		}
	}

	perf_end(_loop_perf);
}

int MulticopterAttitudeControl::task_spawn(int argc, char *argv[])
{
	MulticopterAttitudeControl *instance = new MulticopterAttitudeControl();

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

int MulticopterAttitudeControl::print_status()
{
	PX4_INFO("Running");

	perf_print_counter(_loop_perf);

	return 0;
}

int MulticopterAttitudeControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int MulticopterAttitudeControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This implements the multicopter attitude controller. It takes attitude
setpoints (`vehicle_attitude_setpoint`) as inputs and outputs a rate setpoint.

The controller has a P loop for angular error

Publication documenting the implemented Quaternion Attitude Control:
Nonlinear Quadrocopter Attitude Control (2013)
by Dario Brescianini, Markus Hehn and Raffaello D'Andrea
Institute for Dynamic Systems and Control (IDSC), ETH Zurich

https://www.research-collection.ethz.ch/bitstream/handle/20.500.11850/154099/eth-7387-01.pdf

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("mc_att_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

/**
 * Multicopter attitude control app start / stop handling function
 */
extern "C" __EXPORT int mc_att_control_main(int argc, char *argv[])
{
	return MulticopterAttitudeControl::main(argc, argv);
}
