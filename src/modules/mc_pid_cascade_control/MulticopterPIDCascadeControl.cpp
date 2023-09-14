/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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

#include "MulticopterPIDCascadeControl.hpp"

#include <drivers/drv_hrt.h>

using namespace matrix;
using namespace time_literals;
using math::radians;

MulticopterPIDCascadeControl::MulticopterPIDCascadeControl() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
{

}

MulticopterPIDCascadeControl::~MulticopterPIDCascadeControl()
{
	perf_free(_loop_perf);
}

bool MulticopterPIDCascadeControl::init()
{
	if (!_vehicle_angular_velocity_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

void MulticopterPIDCascadeControl::Run()
{
	if (should_exit()) {
		_vehicle_angular_velocity_sub.unregisterCallback();
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
	}

	// const hrt_abstime now = angular_velocity.timestamp_sample;

	// // Guard against too small (< 0.125ms) and too large (> 20ms) dt's.
	// const float dt = math::constrain(((now - _last_run) * 1e-6f), 0.000125f, 0.02f);
	// _last_run = now;

	/* check for updates in other topics */

	if (_vehicle_land_detected_sub.updated()) {
		vehicle_land_detected_s vehicle_land_detected;

		if (_vehicle_land_detected_sub.copy(&vehicle_land_detected)) {
			_landed = vehicle_land_detected.landed;
			_maybe_landed = vehicle_land_detected.maybe_landed;
		}
	}

	_vehicle_status_sub.update(&_vehicle_status);
	_vehicle_control_mode_sub.update(&_vehicle_control_mode);

	// position/velocity control
	if (_vehicle_control_mode.flag_control_velocity_enabled || _vehicle_control_mode.flag_control_altitude_enabled) {
		//_position_control.update();
	}

	// attitude control
	if (_vehicle_control_mode.flag_control_attitude_enabled) {
		//_attitude_control.update();


		// _attitude_control.setAttitudeSetpoint(Quatf(vehicle_attitude_setpoint.q_d), vehicle_attitude_setpoint.yaw_sp_move_rate);

		// Check for a heading reset
		// if (_quat_reset_counter != v_att.quat_reset_counter) {

		// }



	}

	// rate control
	vehicle_angular_velocity_s angular_velocity;

	if (_vehicle_angular_velocity_sub.update(&angular_velocity) && _vehicle_control_mode.flag_control_rates_enabled) {

		// attitude control
		if (_vehicle_control_mode.flag_control_attitude_enabled) {

			// position/velocity control
			if (_vehicle_control_mode.flag_control_velocity_enabled || _vehicle_control_mode.flag_control_altitude_enabled) {
				//_position_control.update(time_now_us, position);
				//_velocity_control.update(time_now_us, velocity);

				// thrust to attitude always
				//ControlMath::thrustToAttitude(_thr_sp, _yaw_sp, attitude_setpoint);
				//attitude_setpoint.yaw_sp_move_rate = _yawspeed_sp;



			} else {
				// otherwise get attitude setpoint and pass into attitude controller?
			}

			// run attitude controller
			// Vector3f rates_sp = _attitude_control.update(q);

			// pass rate setpoint into rate controller

		} else {
			// otherwise get rate setpoint and pass into rate controller
		}






		//_rate_control.update();


		// TODO: send the unallocated value directly for better anti-windup
		//_rate_control.setSaturationStatus(saturation_positive, saturation_negative);

		// // run rate controller
		// const Vector3f att_control = _rate_control.update(rates, _rates_setpoint, angular_accel, dt, _maybe_landed || _landed);

		// // publish rate controller status
		// rate_ctrl_status_s rate_ctrl_status{};
		// _rate_control.getRateControlStatus(rate_ctrl_status);
		// rate_ctrl_status.timestamp = hrt_absolute_time();
		// _controller_status_pub.publish(rate_ctrl_status);

		// // publish thrust and torque setpoints
		// vehicle_thrust_setpoint_s vehicle_thrust_setpoint{};
		// vehicle_torque_setpoint_s vehicle_torque_setpoint{};

		// _thrust_setpoint.copyTo(vehicle_thrust_setpoint.xyz);
		// vehicle_torque_setpoint.xyz[0] = PX4_ISFINITE(att_control(0)) ? att_control(0) : 0.f;
		// vehicle_torque_setpoint.xyz[1] = PX4_ISFINITE(att_control(1)) ? att_control(1) : 0.f;
		// vehicle_torque_setpoint.xyz[2] = PX4_ISFINITE(att_control(2)) ? att_control(2) : 0.f;


		// TODO: control allocator directly


		// vehicle_torque_setpoint_s vehicle_torque_setpoint;
		// vehicle_thrust_setpoint_s vehicle_thrust_setpoint;

		// _control_allocation[i]->setControlSetpoint(c[i]);

		// // Do allocation
		// _control_allocation[i]->allocate();


		// Publish actuator setpoint and allocator status
		//publish_actuator_controls();
	}



	perf_end(_loop_perf);
}

int MulticopterPIDCascadeControl::task_spawn(int argc, char *argv[])
{
	MulticopterPIDCascadeControl *instance = new MulticopterPIDCascadeControl();

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

int MulticopterPIDCascadeControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int MulticopterPIDCascadeControl::print_usage(const char *reason)
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

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("mc_pid_cascade_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int mc_pid_cascade_control_main(int argc, char *argv[])
{
	return MulticopterPIDCascadeControl::main(argc, argv);
}
