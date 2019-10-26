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

#include <drivers/drv_hrt.h>
#include <circuit_breaker/circuit_breaker.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>

using namespace matrix;
using namespace time_literals;
using math::radians;

MulticopterRateControl::MulticopterRateControl() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	_vehicle_status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;

	parameters_updated();
}

MulticopterRateControl::~MulticopterRateControl()
{
	perf_free(_loop_perf);
}

bool
MulticopterRateControl::init()
{
	if (!_vehicle_angular_velocity_sub.registerCallback()) {
		PX4_ERR("vehicle_angular_velocity callback registration failed!");
		return false;
	}

	return true;
}

void
MulticopterRateControl::parameters_updated()
{
	// rate control parameters
	// The controller gain K is used to convert the parallel (P + I/s + sD) form
	// to the ideal (K * [1 + 1/sTi + sTd]) form
	const Vector3f rate_k = Vector3f(_param_mc_rollrate_k.get(), _param_mc_pitchrate_k.get(), _param_mc_yawrate_k.get());

	_rate_control.setGains(
		rate_k.emult(Vector3f(_param_mc_rollrate_p.get(), _param_mc_pitchrate_p.get(), _param_mc_yawrate_p.get())),
		rate_k.emult(Vector3f(_param_mc_rollrate_i.get(), _param_mc_pitchrate_i.get(), _param_mc_yawrate_i.get())),
		rate_k.emult(Vector3f(_param_mc_rollrate_d.get(), _param_mc_pitchrate_d.get(), _param_mc_yawrate_d.get())));

	_rate_control.setIntegratorLimit(
		Vector3f(_param_mc_rr_int_lim.get(), _param_mc_pr_int_lim.get(), _param_mc_yr_int_lim.get()));

	_rate_control.setDTermCutoff(_loop_update_rate_hz, _param_mc_dterm_cutoff.get(), false);

	_rate_control.setFeedForwardGain(
		Vector3f(_param_mc_rollrate_ff.get(), _param_mc_pitchrate_ff.get(), _param_mc_yawrate_ff.get()));

	_actuators_0_circuit_breaker_enabled = circuit_breaker_enabled_by_val(_param_cbrk_rate_ctrl.get(), CBRK_RATE_CTRL_KEY);
}

void
MulticopterRateControl::vehicle_status_poll()
{
	/* check if there is new status information */
	if (_vehicle_status_sub.update(&_vehicle_status)) {
		/* set correct uORB ID, depending on if vehicle is VTOL or not */
		if (_actuators_id == nullptr) {
			if (_vehicle_status.is_vtol) {
				_actuators_id = ORB_ID(actuator_controls_virtual_mc);

			} else {
				_actuators_id = ORB_ID(actuator_controls_0);
			}
		}
	}
}

void
MulticopterRateControl::Run()
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
		parameters_updated();
	}

	_v_control_mode_sub.update(&_v_control_mode);

	if (_v_control_mode.flag_control_rates_enabled && !_actuators_0_circuit_breaker_enabled) {
		// run controller on gyro changes
		vehicle_angular_velocity_s angular_velocity;

		if (_vehicle_angular_velocity_sub.update(&angular_velocity)) {

			vehicle_status_poll();

			// update land detected
			if (_vehicle_land_detected_sub.updated()) {
				vehicle_land_detected_s vehicle_land_detected;

				if (_vehicle_land_detected_sub.copy(&vehicle_land_detected)) {
					_landed = vehicle_land_detected.landed;
					_maybe_landed = vehicle_land_detected.maybe_landed;
				}
			}

			// update landing gear
			if (_landing_gear_sub.updated()) {
				landing_gear_s landing_gear{};
				_landing_gear_sub.copy(&landing_gear);
				_landing_gear_actuator = (float)landing_gear.landing_gear;
			}

			// update saturation status from mixer feedback
			if (_motor_limits_sub.updated()) {
				multirotor_motor_limits_s motor_limits;

				if (_motor_limits_sub.copy(&motor_limits)) {
					MultirotorMixer::saturation_status saturation_status;
					saturation_status.value = motor_limits.saturation_status;

					_rate_control.setSaturationStatus(saturation_status);
				}
			}

			// update rate setpoint
			if (_v_rates_sp_sub.updated()) {
				vehicle_rates_setpoint_s v_rates_sp;
				_v_rates_sp_sub.copy(&v_rates_sp);
				_rates_sp(0) = v_rates_sp.roll;
				_rates_sp(1) = v_rates_sp.pitch;
				_rates_sp(2) = v_rates_sp.yaw;
				_thrust_sp = -v_rates_sp.thrust_body[2];
			}

			// Guard against too small (< 0.2ms) and too large (> 20ms) dt's.
			const hrt_abstime now = hrt_absolute_time();
			const float dt = math::constrain(((now - _last_run) / 1e6f), 0.0002f, 0.02f);
			_last_run = now;

			// calculate loop update rate while disarmed or at least a few times (updating the filter is expensive)
			if (!_v_control_mode.flag_armed || (now - _task_start) < 3300000) {
				_dt_accumulator += dt;
				++_loop_counter;

				if (_dt_accumulator > 1.0f) {
					const float loop_update_rate = (float)_loop_counter / _dt_accumulator;
					_loop_update_rate_hz = _loop_update_rate_hz * 0.5f + loop_update_rate * 0.5f;
					_dt_accumulator = 0;
					_loop_counter = 0;
					_rate_control.setDTermCutoff(_loop_update_rate_hz, _param_mc_dterm_cutoff.get(), true);
				}
			}

			actuator_controls_s actuators{};

			if (!_v_control_mode.flag_control_termination_enabled) {

				// reset integral if disarmed
				if (!_v_control_mode.flag_armed || (_vehicle_status.vehicle_type != vehicle_status_s::VEHICLE_TYPE_ROTARY_WING)) {
					_rate_control.resetIntegral();
				}

				// run rate controller and fill actuator controls
				Vector3f att_control = _rate_control.update(Vector3f{angular_velocity.xyz}, _rates_sp, dt, _maybe_landed || _landed);
				float throttle = _thrust_sp;

				// scale effort by battery status if enabled
				if (_param_mc_bat_scale_en.get()) {
					if (_battery_status_sub.updated()) {
						battery_status_s battery_status;

						if (_battery_status_sub.copy(&battery_status)) {
							_battery_status_scale = battery_status.scale;
						}
					}

					if (_battery_status_scale > 0.0f) {
						att_control *= _battery_status_scale;
						throttle *= _battery_status_scale;
					}
				}

				actuators.control[actuator_controls_s::INDEX_ROLL] = PX4_ISFINITE(att_control(0)) ? att_control(0) : 0.0f;
				actuators.control[actuator_controls_s::INDEX_PITCH] = PX4_ISFINITE(att_control(1)) ? att_control(1) : 0.0f;
				actuators.control[actuator_controls_s::INDEX_YAW] = PX4_ISFINITE(att_control(2)) ? att_control(2) : 0.0f;
				actuators.control[actuator_controls_s::INDEX_THROTTLE] = PX4_ISFINITE(throttle) ? throttle : 0.0f;
			}

			actuators.control[actuator_controls_s::INDEX_LANDING_GEAR] = _landing_gear_actuator;
			actuators.timestamp_sample = angular_velocity.timestamp_sample;
			actuators.timestamp = hrt_absolute_time();
			orb_publish_auto(_actuators_id, &_actuators_0_pub, &actuators, nullptr, ORB_PRIO_DEFAULT);

			// publish rate controller status
			rate_ctrl_status_s rate_ctrl_status{};
			_rate_control.getRateControlStatus(rate_ctrl_status);
			rate_ctrl_status.timestamp = hrt_absolute_time();
			_controller_status_pub.publish(rate_ctrl_status);
		}
	}

	perf_end(_loop_perf);
}

int MulticopterRateControl::task_spawn(int argc, char *argv[])
{
	MulticopterRateControl *instance = new MulticopterRateControl();

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

int MulticopterRateControl::print_status()
{
	PX4_INFO("Running");

	perf_print_counter(_loop_perf);

	return 0;
}

int MulticopterRateControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int MulticopterRateControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This implements the multicopter rate controller. It takes rate setpoints as inputs and outputs actuator control messages.

The controller has a PID loop for angular rate error.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME(MODULE_NAME, "controller");
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
