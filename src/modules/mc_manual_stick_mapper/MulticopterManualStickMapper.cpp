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

#include "MulticopterManualStickMapper.hpp"

#include <lib/conversion/rotation.h>
#include <drivers/drv_hrt.h>
#include <lib/ecl/geo/geo.h>
#include <circuit_breaker/circuit_breaker.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>

using namespace matrix;
using math::radians;

MulticopterManualStickMapper::MulticopterManualStickMapper() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::hp_default),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME))
{
	_vehicle_status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;

	parameters_updated();
}

MulticopterManualStickMapper::~MulticopterManualStickMapper()
{
	perf_free(_loop_perf);
}

bool
MulticopterManualStickMapper::init()
{
	if (!_manual_control_setpoint_sub.registerCallback()) {
		PX4_ERR("manual_control_setpoint callback registration failed!");
		return false;
	}

	return true;
}

void
MulticopterManualStickMapper::parameters_updated()
{
	// manual rate control acro mode rate limits
	_acro_rate_max = Vector3f(radians(_param_mc_acro_r_max.get()), radians(_param_mc_acro_p_max.get()),
				  radians(_param_mc_acro_y_max.get()));

	_man_tilt_max = math::radians(_param_mpc_man_tilt_max.get());
}

void
MulticopterManualStickMapper::vehicle_status_poll()
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

float
MulticopterManualStickMapper::throttle_curve(float throttle_stick_input)
{
	float throttle_min = _vehicle_land_detected.landed ? 0.0f : _param_mpc_manthr_min.get();

	// throttle_stick_input is in range [0, 1]
	switch (_param_mpc_thr_curve.get()) {
	case 1: // no rescaling to hover throttle
		return throttle_min + throttle_stick_input * (_param_mpc_thr_max.get() - throttle_min);

	default: // 0 or other: rescale to hover throttle at 0.5 stick
		if (throttle_stick_input < 0.5f) {
			return (_param_mpc_thr_hover.get() - throttle_min) / 0.5f * throttle_stick_input +
			       throttle_min;

		} else {
			return (_param_mpc_thr_max.get() - _param_mpc_thr_hover.get()) / 0.5f * (throttle_stick_input - 1.0f) +
			       _param_mpc_thr_max.get();
		}
	}
}

void
MulticopterManualStickMapper::update_landing_gear_state(const manual_control_setpoint_s &manual_control_sp)
{
	// Only switch the landing gear up if we are not landed and if
	// the user switched from gear down to gear up.
	// If the user had the switch in the gear up position and took off ignore it
	// until he toggles the switch to avoid retracting the gear immediately on takeoff.
	if (_vehicle_land_detected.landed) {
		_gear_state_initialized = false;
	}

	int8_t landing_gear_position = landing_gear_s::GEAR_DOWN; // default to down

	if (manual_control_sp.gear_switch == manual_control_setpoint_s::SWITCH_POS_ON && _gear_state_initialized) {
		landing_gear_position = landing_gear_s::GEAR_UP;

	} else if (manual_control_sp.gear_switch == manual_control_setpoint_s::SWITCH_POS_OFF) {
		// Switching the gear off does put it into a safe defined state
		_gear_state_initialized = true;
	}

	landing_gear_s landing_gear{};
	landing_gear.landing_gear = landing_gear_position;
	landing_gear.timestamp = hrt_absolute_time();
	_landing_gear_pub.publish(landing_gear);
}

void
MulticopterManualStickMapper::Run()
{
	if (should_exit()) {
		_manual_control_setpoint_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	// check for parameter updates
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		updateParams();
		parameters_updated();
	}

	// only run in manual mode (acro, stabilized, rattitude)
	_v_control_mode_sub.update(&_v_control_mode);

	if (_v_control_mode.flag_control_manual_enabled &&
	    !_v_control_mode.flag_control_altitude_enabled &&
	    !_v_control_mode.flag_control_velocity_enabled &&
	    !_v_control_mode.flag_control_position_enabled) {

		vehicle_status_poll();

		if (_vehicle_status.is_vtol) {
			if (_vehicle_status.in_transition_mode && !_is_tailsitter) {
				// don't run during VTOL transition (except for tailsitter)
				return;
			}
		}

		manual_control_setpoint_s manual_control_sp;

		if (_manual_control_setpoint_sub.update(&manual_control_sp)) {
			// Guard against too small (< 0.2ms) and too large (> 200ms) dt's.
			const hrt_abstime now = hrt_absolute_time();
			const float dt = math::constrain(((now - _last_run) / 1e6f), 0.0002f, 0.2f);
			_last_run = now;

			/* check for updates in other topics */
			_vehicle_land_detected_sub.update(&_vehicle_land_detected);

			// Check if we are in rattitude mode and the pilot is above the threshold on pitch
			// or roll (yaw can rotate 360 in normal att control). If both are true don't
			// even bother running the attitude controllers
			if (_v_control_mode.flag_control_rattitude_enabled) {
				_v_control_mode.flag_control_attitude_enabled =
					fabsf(manual_control_sp.y) <= _param_mc_ratt_th.get() &&
					fabsf(manual_control_sp.x) <= _param_mc_ratt_th.get();
			}

			if (_v_control_mode.flag_control_attitude_enabled) {
				// manual attitude control - STABILIZED mode

				vehicle_attitude_setpoint_s attitude_setpoint{};

				vehicle_attitude_s v_att{};
				_v_att_sub.copy(&v_att);
				const float yaw = Eulerf(Quatf(v_att.q)).psi();

				// reset yaw setpoint during transitions, tailsitter.cpp generates
				// attitude setpoint for the transition
				if (_vehicle_land_detected.landed || (_vehicle_status.is_vtol && _vehicle_status.in_transition_mode)) {
					_man_yaw_sp = yaw;

				} else if (manual_control_sp.z > 0.05f || _param_mc_airmode.get() == (int32_t)Mixer::Airmode::roll_pitch_yaw) {

					// Check for a heading reset
					if (_quat_reset_counter_prev != v_att.quat_reset_counter) {
						// we only extract the heading change from the delta quaternion
						_man_yaw_sp += Eulerf(Quatf(v_att.delta_q_reset)).psi();
						_quat_reset_counter_prev = v_att.quat_reset_counter;
					}

					attitude_setpoint.yaw_sp_move_rate = manual_control_sp.r * radians(_param_mpc_man_y_max.get());
					_man_yaw_sp = wrap_pi(_man_yaw_sp + attitude_setpoint.yaw_sp_move_rate * dt);
				}

				/*
				 * Input mapping for roll & pitch setpoints
				 * ----------------------------------------
				 * We control the following 2 angles:
				 * - tilt angle, given by sqrt(x*x + y*y)
				 * - the direction of the maximum tilt in the XY-plane, which also defines the direction of the motion
				 *
				 * This allows a simple limitation of the tilt angle, the vehicle flies towards the direction that the stick
				 * points to, and changes of the stick input are linear.
				 */
				const float x = manual_control_sp.x * _man_tilt_max;
				const float y = manual_control_sp.y * _man_tilt_max;

				// we want to fly towards the direction of (x, y), so we use a perpendicular axis angle vector in the XY-plane
				Vector2f v = Vector2f(y, -x);
				float v_norm = v.norm(); // the norm of v defines the tilt angle

				if (v_norm > _man_tilt_max) { // limit to the configured maximum tilt angle
					v *= _man_tilt_max / v_norm;
				}

				const Eulerf euler_sp{AxisAnglef{v(0), v(1), 0.0f}};
				attitude_setpoint.roll_body = euler_sp(0);
				attitude_setpoint.pitch_body = euler_sp(1);
				// The axis angle can change the yaw as well (noticeable at higher tilt angles).
				// This is the formula by how much the yaw changes:
				//   let a := tilt angle, b := atan(y/x) (direction of maximum tilt)
				//   yaw = atan(-2 * sin(b) * cos(b) * sin^2(a/2) / (1 - 2 * cos^2(b) * sin^2(a/2))).
				attitude_setpoint.yaw_body = _man_yaw_sp + euler_sp(2);

				/* modify roll/pitch only if we're a VTOL */
				if (_vehicle_status.is_vtol) {
					// Construct attitude setpoint rotation matrix. Modify the setpoints for roll
					// and pitch such that they reflect the user's intention even if a large yaw error
					// (yaw_sp - yaw) is present. In the presence of a yaw error constructing a rotation matrix
					// from the pure euler angle setpoints will lead to unexpected attitude behaviour from
					// the user's view as the euler angle sequence uses the  yaw setpoint and not the current
					// heading of the vehicle.
					// However there's also a coupling effect that causes oscillations for fast roll/pitch changes
					// at higher tilt angles, so we want to avoid using this on multicopters.
					// The effect of that can be seen with:
					// - roll/pitch into one direction, keep it fixed (at high angle)
					// - apply a fast yaw rotation
					// - look at the roll and pitch angles: they should stay pretty much the same as when not yawing

					// calculate our current yaw error
					float yaw_error = wrap_pi(attitude_setpoint.yaw_body - yaw);

					// compute the vector obtained by rotating a z unit vector by the rotation
					// given by the roll and pitch commands of the user
					const Vector3f zB = {0.0f, 0.0f, 1.0f};
					const Dcmf R_sp_roll_pitch = Eulerf(attitude_setpoint.roll_body, attitude_setpoint.pitch_body, 0.0f);
					Vector3f z_roll_pitch_sp = R_sp_roll_pitch * zB;

					// transform the vector into a new frame which is rotated around the z axis
					// by the current yaw error. this vector defines the desired tilt when we look
					// into the direction of the desired heading
					const Dcmf R_yaw_correction = Eulerf(0.0f, 0.0f, -yaw_error);
					z_roll_pitch_sp = R_yaw_correction * z_roll_pitch_sp;

					// use the formula z_roll_pitch_sp = R_tilt * [0;0;1]
					// R_tilt is computed from_euler; only true if cos(roll) not equal zero
					// -> valid if roll is not +-pi/2;
					attitude_setpoint.roll_body = -asinf(z_roll_pitch_sp(1));
					attitude_setpoint.pitch_body = atan2f(z_roll_pitch_sp(0), z_roll_pitch_sp(2));
				}

				// copy quaternion setpoint to attitude setpoint topic
				const Quatf q_sp = Eulerf(attitude_setpoint.roll_body, attitude_setpoint.pitch_body, attitude_setpoint.yaw_body);
				q_sp.copyTo(attitude_setpoint.q_d);
				attitude_setpoint.q_d_valid = true;
				attitude_setpoint.thrust_body[2] = -throttle_curve(manual_control_sp.z);
				attitude_setpoint.timestamp = hrt_absolute_time();

				if (_attitude_sp_id != nullptr) {
					orb_publish_auto(_attitude_sp_id, &_vehicle_attitude_setpoint_pub, &attitude_setpoint, nullptr, ORB_PRIO_DEFAULT);
				}

			} else if (_v_control_mode.flag_control_rates_enabled) {
				// manual rates control - ACRO mode
				const Vector3f man_rate_sp{
					math::superexpo(manual_control_sp.y, _param_mc_acro_expo.get(), _param_mc_acro_supexpo.get()),
					math::superexpo(-manual_control_sp.x, _param_mc_acro_expo.get(), _param_mc_acro_supexpo.get()),
					math::superexpo(manual_control_sp.r, _param_mc_acro_expo_y.get(), _param_mc_acro_supexpoy.get())};

				const Vector3f rates_sp = man_rate_sp.emult(_acro_rate_max);

				// publish vehicle rates setpoint
				vehicle_rates_setpoint_s v_rates_sp{};
				v_rates_sp.roll = rates_sp(0);
				v_rates_sp.pitch = rates_sp(1);
				v_rates_sp.yaw = rates_sp(2);
				v_rates_sp.thrust_body[2] = -manual_control_sp.z;
				v_rates_sp.timestamp = hrt_absolute_time();
				_v_rates_sp_pub.publish(v_rates_sp);
			}

			update_landing_gear_state(manual_control_sp);
		}
	}

	perf_end(_loop_perf);
}

int MulticopterManualStickMapper::task_spawn(int argc, char *argv[])
{
	MulticopterManualStickMapper *instance = new MulticopterManualStickMapper();

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

int MulticopterManualStickMapper::print_status()
{
	PX4_INFO("Running");

	perf_print_counter(_loop_perf);

	return 0;
}

int MulticopterManualStickMapper::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int MulticopterManualStickMapper::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This implements the multicopter attitude and rate controller. It takes attitude
setpoints (`vehicle_attitude_setpoint`) or rate setpoints (in acro mode
via `manual_control_setpoint` topic) as inputs and outputs actuator control messages.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("mc_manual_stick_mapper", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int mc_manual_stick_mapper_main(int argc, char *argv[])
{
	return MulticopterManualStickMapper::main(argc, argv);
}
