/****************************************************************************
 *
 *   Copyright (c) 2013-2020 PX4 Development Team. All rights reserved.
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

#include "MulticopterPositionControl.hpp"

#include <float.h>
#include <lib/mathlib/mathlib.h>
#include <lib/matrix/matrix/math.hpp>
#include "PositionControl/ControlMath.hpp"

using namespace matrix;

MulticopterPositionControl::MulticopterPositionControl(bool vtol) :
	SuperBlock(nullptr, "MPC"),
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	_vehicle_attitude_setpoint_pub(vtol ? ORB_ID(mc_virtual_attitude_setpoint) : ORB_ID(vehicle_attitude_setpoint)),
	_vel_x_deriv(this, "VELD"),
	_vel_y_deriv(this, "VELD"),
	_vel_z_deriv(this, "VELD"),
	_cycle_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle time"))
{
	// fetch initial parameter values
	parameters_update(true);

	// set failsafe hysteresis
	_failsafe_land_hysteresis.set_hysteresis_time_from(false, LOITER_TIME_BEFORE_DESCEND);

	reset_setpoint_to_nan(_setpoint);
}

MulticopterPositionControl::~MulticopterPositionControl()
{
	perf_free(_cycle_perf);
}

bool MulticopterPositionControl::init()
{
	if (!_local_pos_sub.registerCallback()) {
		PX4_ERR("vehicle_local_position callback registration failed!");
		return false;
	}

	_time_stamp_last_loop = hrt_absolute_time();
	ScheduleNow();

	return true;
}

int MulticopterPositionControl::parameters_update(bool force)
{
	// check for parameter updates
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		ModuleParams::updateParams();
		SuperBlock::updateParams();

		if (_param_mpc_tiltmax_air.get() > MAX_SAFE_TILT_DEG) {
			_param_mpc_tiltmax_air.set(MAX_SAFE_TILT_DEG);
			_param_mpc_tiltmax_air.commit();
			mavlink_log_critical(&_mavlink_log_pub, "Tilt constrained to safe value");
		}

		if (_param_mpc_tiltmax_lnd.get() > _param_mpc_tiltmax_air.get()) {
			_param_mpc_tiltmax_lnd.set(_param_mpc_tiltmax_air.get());
			_param_mpc_tiltmax_lnd.commit();
			mavlink_log_critical(&_mavlink_log_pub, "Land tilt has been constrained by max tilt");
		}

		_control.setPositionGains(Vector3f(_param_mpc_xy_p.get(), _param_mpc_xy_p.get(), _param_mpc_z_p.get()));
		_control.setVelocityGains(
			Vector3f(_param_mpc_xy_vel_p_acc.get(), _param_mpc_xy_vel_p_acc.get(), _param_mpc_z_vel_p_acc.get()),
			Vector3f(_param_mpc_xy_vel_i_acc.get(), _param_mpc_xy_vel_i_acc.get(), _param_mpc_z_vel_i_acc.get()),
			Vector3f(_param_mpc_xy_vel_d_acc.get(), _param_mpc_xy_vel_d_acc.get(), _param_mpc_z_vel_d_acc.get()));

		// Check that the design parameters are inside the absolute maximum constraints
		if (_param_mpc_xy_cruise.get() > _param_mpc_xy_vel_max.get()) {
			_param_mpc_xy_cruise.set(_param_mpc_xy_vel_max.get());
			_param_mpc_xy_cruise.commit();
			mavlink_log_critical(&_mavlink_log_pub, "Cruise speed has been constrained by max speed");
		}

		if (_param_mpc_vel_manual.get() > _param_mpc_xy_vel_max.get()) {
			_param_mpc_vel_manual.set(_param_mpc_xy_vel_max.get());
			_param_mpc_vel_manual.commit();
			mavlink_log_critical(&_mavlink_log_pub, "Manual speed has been constrained by max speed");
		}

		if (_param_mpc_thr_hover.get() > _param_mpc_thr_max.get() ||
		    _param_mpc_thr_hover.get() < _param_mpc_thr_min.get()) {
			_param_mpc_thr_hover.set(math::constrain(_param_mpc_thr_hover.get(), _param_mpc_thr_min.get(),
						 _param_mpc_thr_max.get()));
			_param_mpc_thr_hover.commit();
			mavlink_log_critical(&_mavlink_log_pub, "Hover thrust has been constrained by min/max");
		}

		if (!_param_mpc_use_hte.get() || !_hover_thrust_initialized) {
			_control.setHoverThrust(_param_mpc_thr_hover.get());
			_hover_thrust_initialized = true;
		}

		// initialize vectors from params and enforce constraints
		_param_mpc_tko_speed.set(math::min(_param_mpc_tko_speed.get(), _param_mpc_z_vel_max_up.get()));
		_param_mpc_land_speed.set(math::min(_param_mpc_land_speed.get(), _param_mpc_z_vel_max_dn.get()));

		_takeoff.setSpoolupTime(_param_mpc_spoolup_time.get());
		_takeoff.setTakeoffRampTime(_param_mpc_tko_ramp_t.get());
		_takeoff.generateInitialRampValue(_param_mpc_z_vel_p_acc.get());
	}

	return OK;
}

void MulticopterPositionControl::set_vehicle_states(const vehicle_local_position_s &local_pos)
{
	// only set position states if valid and finite
	if (PX4_ISFINITE(local_pos.x) && PX4_ISFINITE(local_pos.y) && local_pos.xy_valid) {
		_states.position(0) = local_pos.x;
		_states.position(1) = local_pos.y;

	} else {
		_states.position(0) = _states.position(1) = NAN;
	}

	if (PX4_ISFINITE(local_pos.z) && local_pos.z_valid) {
		_states.position(2) = local_pos.z;

	} else {
		_states.position(2) = NAN;
	}

	if (PX4_ISFINITE(local_pos.vx) && PX4_ISFINITE(local_pos.vy) && local_pos.v_xy_valid) {
		_states.velocity(0) = local_pos.vx;
		_states.velocity(1) = local_pos.vy;
		_states.acceleration(0) = _vel_x_deriv.update(_states.velocity(0));
		_states.acceleration(1) = _vel_y_deriv.update(_states.velocity(1));

	} else {
		_states.velocity(0) = _states.velocity(1) = NAN;
		_states.acceleration(0) = _states.acceleration(1) = NAN;

		// reset derivatives to prevent acceleration spikes when regaining velocity
		_vel_x_deriv.reset();
		_vel_y_deriv.reset();
	}

	if (PX4_ISFINITE(local_pos.vz) && local_pos.v_z_valid) {
		_states.velocity(2) = local_pos.vz;
		_states.acceleration(2) = _vel_z_deriv.update(_states.velocity(2));

	} else {
		_states.velocity(2) = _states.acceleration(2) = NAN;

		// reset derivative to prevent acceleration spikes when regaining velocity
		_vel_z_deriv.reset();
	}

	if (PX4_ISFINITE(local_pos.heading)) {
		_states.yaw = local_pos.heading;
	}
}

void MulticopterPositionControl::Run()
{
	if (should_exit()) {
		_local_pos_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	// reschedule backup
	ScheduleDelayed(100_ms);

	parameters_update(false);

	perf_begin(_cycle_perf);
	vehicle_local_position_s local_pos;

	if (_local_pos_sub.update(&local_pos)) {
		const hrt_abstime time_stamp_now = local_pos.timestamp;
		const float dt = math::constrain(((time_stamp_now - _time_stamp_last_loop) * 1e-6f), 0.002f, 0.04f);
		_time_stamp_last_loop = time_stamp_now;

		// set _dt in controllib Block for BlockDerivative
		setDt(dt);
		set_vehicle_states(local_pos);

		const bool was_in_failsafe = _in_failsafe;
		_in_failsafe = false;

		_control_mode_sub.update(&_control_mode);
		_vehicle_land_detected_sub.update(&_vehicle_land_detected);

		if (_param_mpc_use_hte.get()) {
			hover_thrust_estimate_s hte;

			if (_hover_thrust_estimate_sub.update(&hte)) {
				if (hte.valid) {
					_control.updateHoverThrust(hte.hover_thrust);
				}
			}
		}

		if (_control_mode.flag_control_climb_rate_enabled) {

			_trajectory_setpoint_sub.update(&_setpoint);

			// adjust existing (or older) setpoint with any EKF reset deltas
			if ((_setpoint.timestamp != 0) && (_setpoint.timestamp < local_pos.timestamp)) {
				if (local_pos.vxy_reset_counter != _vxy_reset_counter) {
					if (PX4_ISFINITE(_setpoint.vx)) {
						_setpoint.vx += local_pos.delta_vxy[0];
					}

					if (PX4_ISFINITE(_setpoint.vy)) {
						_setpoint.vy += local_pos.delta_vxy[1];
					}
				}

				if (local_pos.vz_reset_counter != _vz_reset_counter) {
					if (PX4_ISFINITE(_setpoint.vz)) {
						_setpoint.vz += local_pos.delta_vz;
					}
				}

				if (local_pos.xy_reset_counter != _xy_reset_counter) {
					if (PX4_ISFINITE(_setpoint.x)) {
						_setpoint.x += local_pos.delta_xy[0];
					}

					if (PX4_ISFINITE(_setpoint.y)) {
						_setpoint.y += local_pos.delta_xy[1];
					}
				}

				if (local_pos.z_reset_counter != _z_reset_counter) {
					if (PX4_ISFINITE(_setpoint.z)) {
						_setpoint.z += local_pos.delta_z;
					}
				}
			}

			// update vehicle constraints and handle smooth takeoff
			_vehicle_constraints_sub.update(&_vehicle_constraints);

			// fix to prevent the takeoff ramp to ramp to a too high value or get stuck because of NAN
			// TODO: this should get obsolete once the takeoff limiting moves into the flight tasks
			if (!PX4_ISFINITE(_vehicle_constraints.speed_up) || (_vehicle_constraints.speed_up > _param_mpc_z_vel_max_up.get())) {
				_vehicle_constraints.speed_up = _param_mpc_z_vel_max_up.get();
			}

			// handle smooth takeoff
			_takeoff.updateTakeoffState(_control_mode.flag_armed, _vehicle_land_detected.landed, _vehicle_constraints.want_takeoff,
						    _vehicle_constraints.speed_up, false, time_stamp_now);

			const bool not_taken_off = (_takeoff.getTakeoffState() < TakeoffState::rampup);
			const bool flying = (_takeoff.getTakeoffState() >= TakeoffState::flight);
			const bool flying_but_ground_contact = (flying && _vehicle_land_detected.ground_contact);

			if (not_taken_off || flying_but_ground_contact) {
				// we are not flying yet and need to avoid any corrections
				reset_setpoint_to_nan(_setpoint);
				Vector3f(0.f, 0.f, 100.f).copyTo(_setpoint.acceleration); // High downwards acceleration to make sure there's no thrust

				// prevent any integrator windup
				_control.resetIntegral();
			}

			// limit tilt during takeoff ramupup
			if (_takeoff.getTakeoffState() < TakeoffState::flight) {
				_control.setTiltLimit(math::radians(_param_mpc_tiltmax_lnd.get()));

			} else {
				_control.setTiltLimit(math::radians(_param_mpc_tiltmax_air.get()));
			}

			const float speed_up = _takeoff.updateRamp(dt, _vehicle_constraints.speed_up);
			const float speed_down = PX4_ISFINITE(_vehicle_constraints.speed_down) ? _vehicle_constraints.speed_down :
						 _param_mpc_z_vel_max_dn.get();
			const float speed_horizontal = PX4_ISFINITE(_vehicle_constraints.speed_xy) ? _vehicle_constraints.speed_xy :
						       _param_mpc_xy_vel_max.get();

			// Allow ramping from zero thrust on takeoff
			const float minimum_thrust = flying ? _param_mpc_thr_min.get() : 0.f;

			// update states
			if (PX4_ISFINITE(_setpoint.vz) && (fabsf(_setpoint.vz) > FLT_EPSILON) && PX4_ISFINITE(local_pos.z_deriv)
			    && local_pos.v_xy_valid) {

				// A change in velocity is demanded. Set velocity to the derivative of position
				// because it has less bias but blend it in across the landing speed range
				float weighting = fminf(fabsf(_setpoint.vz) / _param_mpc_land_speed.get(), 1.f);
				_states.velocity(2) = local_pos.z_deriv * weighting + local_pos.vz * (1.f - weighting);
			}

			// Run position control
			_control.setThrustLimits(minimum_thrust, _param_mpc_thr_max.get());
			_control.setVelocityLimits(
				math::constrain(speed_horizontal, 0.f, _param_mpc_xy_vel_max.get()),
				math::constrain(speed_up, 0.f, _param_mpc_z_vel_max_up.get()),
				math::constrain(speed_down, 0.f, _param_mpc_z_vel_max_dn.get()));
			_control.setState(_states);
			_control.setInputSetpoint(_setpoint);

			if (_control.update(dt)) {
				_failsafe_land_hysteresis.set_state_and_update(false, time_stamp_now);

			} else {
				// Failsafe
				if ((time_stamp_now - _last_warn) > 2_s) {
					PX4_WARN("invalid setpoints");
					_last_warn = time_stamp_now;
				}

				failsafe(time_stamp_now, _setpoint, _states, !was_in_failsafe);

				// reset constraints
				_vehicle_constraints = {0, NAN, NAN, NAN, NAN, NAN, false, {}};

				_control.setInputSetpoint(_setpoint);
				_control.setVelocityLimits(_param_mpc_xy_vel_max.get(), _param_mpc_z_vel_max_up.get(), _param_mpc_z_vel_max_dn.get());
				_control.update(dt);
			}

			// Publish internal position control setpoints
			// on top of the input/feed-forward setpoints these containt the PID corrections
			// This message is used by other modules (such as Landdetector) to determine vehicle intention.
			vehicle_local_position_setpoint_s local_pos_sp{};
			_control.getLocalPositionSetpoint(local_pos_sp);
			local_pos_sp.timestamp = hrt_absolute_time();
			_local_pos_sp_pub.publish(local_pos_sp);

			// Publish attitude setpoint output
			vehicle_attitude_setpoint_s attitude_setpoint{};
			_control.getAttitudeSetpoint(attitude_setpoint);
			attitude_setpoint.timestamp = hrt_absolute_time();
			_vehicle_attitude_setpoint_pub.publish(attitude_setpoint);

		} else {
			// an update is necessary here because otherwise the takeoff state doesn't get skiped with non-altitude-controlled modes
			_takeoff.updateTakeoffState(_control_mode.flag_armed, _vehicle_land_detected.landed, false, 10.f, true, time_stamp_now);
		}

		// Publish takeoff status
		const uint8_t takeoff_state = static_cast<uint8_t>(_takeoff.getTakeoffState());

		if (takeoff_state != _old_takeoff_state) {
			takeoff_status_s takeoff_status{};
			takeoff_status.takeoff_state = takeoff_state;
			takeoff_status.timestamp = hrt_absolute_time();
			_takeoff_status_pub.publish(takeoff_status);

			_old_takeoff_state = takeoff_state;
		}

		// save latest reset counters
		_vxy_reset_counter = local_pos.vxy_reset_counter;
		_vz_reset_counter = local_pos.vz_reset_counter;
		_xy_reset_counter = local_pos.xy_reset_counter;
		_z_reset_counter = local_pos.z_reset_counter;
	}

	perf_end(_cycle_perf);
}

void MulticopterPositionControl::failsafe(const hrt_abstime &now, vehicle_local_position_setpoint_s &setpoint,
		const PositionControlStates &states, bool warn)
{
	// do not warn while we are disarmed, as we might not have valid setpoints yet
	if (!_control_mode.flag_armed) {
		warn = false;
	}

	// Only react after a short delay
	_failsafe_land_hysteresis.set_state_and_update(true, now);

	if (_failsafe_land_hysteresis.get_state()) {
		reset_setpoint_to_nan(setpoint);

		if (PX4_ISFINITE(_states.velocity(0)) && PX4_ISFINITE(_states.velocity(1))) {
			// don't move along xy
			setpoint.vx = setpoint.vy = 0.f;

			if (warn) {
				PX4_WARN("Failsafe: stop and wait");
			}

		} else {
			// descend with land speed since we can't stop
			setpoint.acceleration[0] = setpoint.acceleration[1] = 0.f;
			setpoint.vz = _param_mpc_land_speed.get();

			if (warn) {
				PX4_WARN("Failsafe: blind land");
			}
		}

		if (PX4_ISFINITE(_states.velocity(2))) {
			// don't move along z if we can stop in all dimensions
			if (!PX4_ISFINITE(setpoint.vz)) {
				setpoint.vz = 0.f;
			}

		} else {
			// emergency descend with a bit below hover thrust
			setpoint.vz = NAN;
			setpoint.acceleration[2] = .3f;

			if (warn) {
				PX4_WARN("Failsafe: blind descend");
			}
		}

		_in_failsafe = true;
	}
}

void MulticopterPositionControl::reset_setpoint_to_nan(vehicle_local_position_setpoint_s &setpoint)
{
	setpoint.x = setpoint.y = setpoint.z = NAN;
	setpoint.vx = setpoint.vy = setpoint.vz = NAN;
	setpoint.yaw = setpoint.yawspeed = NAN;
	setpoint.acceleration[0] = setpoint.acceleration[1] = setpoint.acceleration[2] = NAN;
	setpoint.thrust[0] = setpoint.thrust[1] = setpoint.thrust[2] = NAN;
}

int MulticopterPositionControl::task_spawn(int argc, char *argv[])
{
	bool vtol = false;

	if (argc > 1) {
		if (strcmp(argv[1], "vtol") == 0) {
			vtol = true;
		}
	}

	MulticopterPositionControl *instance = new MulticopterPositionControl(vtol);

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

int MulticopterPositionControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int MulticopterPositionControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
The controller has two loops: a P loop for position error and a PID loop for velocity error.
Output of the velocity controller is thrust vector that is split to thrust direction
(i.e. rotation matrix for multicopter orientation) and thrust scalar (i.e. multicopter thrust itself).

The controller doesn't use Euler angles for its work, they are generated only for more human-friendly control and
logging.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("mc_pos_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_ARG("vtol", "VTOL mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int mc_pos_control_main(int argc, char *argv[])
{
	return MulticopterPositionControl::main(argc, argv);
}
