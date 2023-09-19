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
/**
 * @file FlightTaskDrop.cpp
 */

#include "FlightTaskDrop.hpp"
#include <mathlib/mathlib.h>
#include <float.h>

#include <lib/systemlib/mavlink_log.h>

using namespace matrix;

using namespace time_literals;

bool FlightTaskDrop::activate(const trajectory_setpoint_s &last_setpoint)
{
	//PX4_WARN("FlightTaskDrop::activate()");

	bool ret = FlightTask::activate(last_setpoint);

	_position_setpoint = _position;
	_velocity_setpoint = _velocity;
	_yaw_setpoint = _yaw;
	_yawspeed_setpoint = 0.0f;

	// Set setpoints equal current state.
	_velocity_setpoint = _velocity;
	_position_setpoint = _position;

	Vector3f vel_prev{last_setpoint.velocity};
	Vector3f pos_prev{last_setpoint.position};
	Vector3f accel_prev{last_setpoint.acceleration};

	for (int i = 0; i < 3; i++) {
		// If the position setpoint is unknown, set to the current position
		if (!PX4_ISFINITE(pos_prev(i))) {
			pos_prev(i) = _position(i);
		}

		// If the velocity setpoint is unknown, set to the current velocity
		if (!PX4_ISFINITE(vel_prev(i))) {
			vel_prev(i) = _velocity(i);
		}

		// No acceleration estimate available, set to zero if the setpoint is NAN
		if (!PX4_ISFINITE(accel_prev(i))) {
			accel_prev(i) = 0.f;
		}
	}

	//_position_smoothing.reset(accel_prev, vel_prev, pos_prev);

	//_yaw_sp_prev = PX4_ISFINITE(last_setpoint.yaw) ? last_setpoint.yaw : _yaw;
	//_updateTrajConstraints();
	//_is_emergency_braking_active = false;

	return ret;
}

void FlightTaskDrop::reActivate()
{
	//PX4_WARN("FlightTaskDrop::reActivate()");

	//FlightTask::reActivate();

	_actuator_armed_sub.update();

	if (!_actuator_armed_sub.get().armed) {
		_state = DropState::DISARMED;
	}

#if 0
	// TODO: reactivate called during rampup
	_actuator_armed_sub.update();

	if (!_actuator_armed_sub.get().armed) {
		_state = DropState::DISARMED;

	} else {
		_state = DropState::ARMING;
	}

#endif

	// On ground, reset acceleration and velocity to zero
	//_position_smoothing.reset({0.f, 0.f, 0.f}, {0.f, 0.f, 0.7f}, _position);
}

bool FlightTaskDrop::updateInitialize()
{
	bool ret = FlightTask::updateInitialize();

	_actuator_armed_sub.update();
	_failsafe_flags_sub.update();
	_home_position_sub.update();
	_vehicle_local_position_sub.update();
	_vehicle_status_sub.update();

	// require valid position
	//ret = ret && _position.isAllFinite() && _velocity.isAllFinite();

	return ret;
}

bool FlightTaskDrop::update()
{
	//PX4_WARN("FlightTaskDrop::update()");

	bool ret = FlightTask::update();

	_actuator_armed_sub.update();
	const actuator_armed_s &actuator_armed = _actuator_armed_sub.get();

	_failsafe_flags_sub.update();
	const failsafe_flags_s &failsafe_flags = _failsafe_flags_sub.get();

	_vehicle_local_position_sub.update();
	const vehicle_local_position_s &vehicle_local_position = _vehicle_local_position_sub.get();

	const Vector3f acceleration{vehicle_local_position.ax, vehicle_local_position.ay, vehicle_local_position.az};
	const Vector3f velocity{vehicle_local_position.vx, vehicle_local_position.vy, vehicle_local_position.vz};
	const Vector3f position{vehicle_local_position.x, vehicle_local_position.y, vehicle_local_position.z};

	const bool acceleration_valid = acceleration.isAllFinite()
					&& (hrt_elapsed_time(&vehicle_local_position.timestamp) < 1_s);
	const bool velocity_valid = velocity.isAllFinite() && (hrt_elapsed_time(&vehicle_local_position.timestamp) < 1_s)
				    && vehicle_local_position.v_xy_valid && vehicle_local_position.v_z_valid;
	const bool position_valid = position.isAllFinite() && (hrt_elapsed_time(&vehicle_local_position.timestamp) < 1_s)
				    && vehicle_local_position.xy_valid && vehicle_local_position.z_valid;

	const DropState state_prev = _state;

	switch (_state) {
	case DropState::UNKNOWN: {
			//PX4_WARN("Drop: uninitialized");

			if (actuator_armed.armed) {
				_state = DropState::ARMING;

			} else {
				_state = DropState::DISARMED;
			}

			_state_last_transition_time = hrt_absolute_time();
		}
		break;

	case DropState::DISARMED: {

			if (_state_prev != _state) {
				mavlink_log_info(&_mavlink_log_pub, "Drop: disarmed");
			}

			// DISARMED
			_constraints.want_takeoff = false;
			_constraints.speed_up = 0.f;
			_constraints.speed_down = 0.f;

			_position_setpoint.setNaN();
			_velocity_setpoint.setNaN();
			_acceleration_setpoint.setNaN();
			_jerk_setpoint.setNaN();

			if (actuator_armed.armed) {
				_state = DropState::ARMING;
				_state_last_transition_time = hrt_absolute_time();
			}
		}
		break;

	case DropState::ARMING: {

			if (_state_prev != _state) {
				mavlink_log_info(&_mavlink_log_pub, "Drop: arming");
			}

			// ARMING
			if (actuator_armed.armed) {

				_constraints.want_takeoff = false;
				_constraints.speed_up = 0.f;
				_constraints.speed_down = 0.f;

				_position_setpoint.setNaN();
				_velocity_setpoint.setNaN();
				_acceleration_setpoint.setNaN();
				_jerk_setpoint.setNaN();

				const hrt_abstime arm_timeout_us = math::constrain(_param_mpc_drop_laun_t.get(), 0.f, 3600.f) * 1e6f;

				const hrt_abstime elapsed_us = hrt_elapsed_time(&_state_last_transition_time);

				int64_t time_remaining_us = (int64_t)arm_timeout_us - elapsed_us;

				if (elapsed_us > arm_timeout_us) {
					mavlink_log_info(&_mavlink_log_pub, "Drop: drop launch armed. Throw the vehicle now");

					_state = DropState::WAITING_FOR_DROP_DETECT;
					_state_last_transition_time = hrt_absolute_time();

				} else if (time_remaining_us > 0 && time_remaining_us < 100'000) {
					PX4_INFO("elapsed: %.3f, remaining: %.3f", (double)elapsed_us * 1e-6, (double)time_remaining_us * 1e-6);
					// TODO: countdown
					// 3..2..1
				}

				// mavlink log info countdown
				//  beeps, lights
				// TODO: dshot tunes?

			} else {
				_state = DropState::DISARMED;
			}
		}
		break;

	case DropState::WAITING_FOR_DROP_DETECT: {

			if (_state_prev != _state) {
				mavlink_log_info(&_mavlink_log_pub, "Drop: waiting for drop detect");
			}

			if (actuator_armed.armed) {

				if (_state_prev != _state) {
					_constraints.want_takeoff = false;
					_constraints.speed_up = 0.f;
					_constraints.speed_down = 0.f;

					_position_setpoint.setNaN();
					_velocity_setpoint.setNaN();
					_acceleration_setpoint.setNaN();
					_jerk_setpoint.setNaN();
				}

				bool drop_detected = false;

				drop_detected = velocity_valid && (velocity(2) > _param_mpc_drop_vz_thr.get())
						&& acceleration_valid && (acceleration(2) > _param_mpc_drop_az_thr.get());

				// configurable velocity
				// configurable acceleration
				// filter both

				if (drop_detected && !failsafe_flags.angular_velocity_invalid) {
					mavlink_log_info(&_mavlink_log_pub, "Drop: Drop detected! Vz:%.1f, Az:%.1f. Activating rate control",
							 (double)velocity(2), (double)acceleration(2));

					_state = DropState::RATE_CONTROL_ENABLED;
					_state_last_transition_time = hrt_absolute_time();
				}

			} else {
				_state = DropState::DISARMED;
			}

		}
		break;

	case DropState::RATE_CONTROL_ENABLED: {

			if (_state_prev != _state) {
				mavlink_log_info(&_mavlink_log_pub, "Drop: rate control enabled");
			}


			if (actuator_armed.armed) {

				if (_state_prev != _state) {
					// reset constraints
					_constraints.speed_up = _param_mpc_z_vel_max_up.get();
					_constraints.speed_down = _param_mpc_z_vel_max_dn.get();
					_constraints.want_takeoff = true;

					_position_setpoint.setNaN();
					_velocity_setpoint.setNaN();
					_acceleration_setpoint.setNaN();
					_jerk_setpoint.setNaN();
				}

				if (!failsafe_flags.attitude_invalid) {
					mavlink_log_info(&_mavlink_log_pub, "Drop: Activating attitude control");
					_state = DropState::ATTITUDE_CONTROL_ENABLED;
					_state_last_transition_time = hrt_absolute_time();

					_position_setpoint.setNaN();
					_velocity_setpoint.setNaN();
					_acceleration_setpoint.zero();
					_jerk_setpoint.zero();
				}

			} else {
				_state = DropState::DISARMED;
			}

		}
		break;

	case DropState::ATTITUDE_CONTROL_ENABLED: {

			if (_state_prev != _state) {
				mavlink_log_info(&_mavlink_log_pub, "Drop: attitude control enabled");
			}

			if (actuator_armed.armed) {

				if (_state_prev != _state) {
					// reset constraints
					_constraints.speed_up = _param_mpc_z_vel_max_up.get();
					_constraints.speed_down = _param_mpc_z_vel_max_dn.get();
					_constraints.want_takeoff = true;

					_position_setpoint.setNaN();
					_velocity_setpoint.setNaN();
					_acceleration_setpoint.zero();
					_jerk_setpoint.zero();
				}

				if (!failsafe_flags.local_altitude_invalid) {
					mavlink_log_info(&_mavlink_log_pub, "Drop: Activating height rate control");
					_state = DropState::HEIGHT_RATE_CONTROL_ENABLED;
					_state_last_transition_time = hrt_absolute_time();

					_position_setpoint.setNaN();
					_velocity_setpoint.setNaN();
					_velocity_setpoint(2) = 0.f;
					_acceleration_setpoint.zero();
					_jerk_setpoint.zero();
				}

			} else {
				_state = DropState::DISARMED;
			}

		}
		break;

	case DropState::HEIGHT_RATE_CONTROL_ENABLED: {

			if (_state_prev != _state) {
				mavlink_log_info(&_mavlink_log_pub, "Drop: height rate control enabled");
			}

			if (actuator_armed.armed) {

				if (_state_prev != _state) {
					// reset constraints
					_constraints.speed_up = _param_mpc_z_vel_max_up.get();
					_constraints.speed_down = _param_mpc_z_vel_max_dn.get();
					_constraints.want_takeoff = true;

					_position_setpoint.setNaN();
					_velocity_setpoint.setNaN();

					_velocity_setpoint(2) = 0.f;

					_acceleration_setpoint.zero();
					_jerk_setpoint.zero();
				}

				// wait for full
				if (!failsafe_flags.local_altitude_invalid && (fabsf(_velocity(2)) < 1.f)) {
					mavlink_log_info(&_mavlink_log_pub, "Drop: Activating altitude control");
					_state = DropState::ALTITUDE_CONTROL_ENABLED;
					_state_last_transition_time = hrt_absolute_time();

					_position_setpoint.setNaN();
					_velocity_setpoint.setNaN();

					// latch onto current height
					_position_setpoint(2) = _position(2);

					_acceleration_setpoint.zero();
					_jerk_setpoint.zero();
				}

			} else {
				_state = DropState::DISARMED;
			}
		}
		break;

	case DropState::ALTITUDE_CONTROL_ENABLED: {

			if (_state_prev != _state) {
				mavlink_log_info(&_mavlink_log_pub, "Drop: altitude control enabled");
			}

			if (actuator_armed.armed) {

				if (_state_prev != _state) {
					// reset constraints
					_constraints.speed_up = _param_mpc_z_vel_max_up.get();
					_constraints.speed_down = _param_mpc_z_vel_max_dn.get();
					_constraints.want_takeoff = true;

					_position_setpoint.setNaN();
					_velocity_setpoint.setNaN();

					// latch onto current height
					_position_setpoint(2) = _position(2);

					// otherwise all zero
					_acceleration_setpoint.zero();
					_jerk_setpoint.zero();
				}

				if (!failsafe_flags.local_velocity_invalid) {
					mavlink_log_info(&_mavlink_log_pub, "Drop: Activating velocity control");
					_state = DropState::HORIZONTAL_VELOCITY_CONTROL_ENABLED;
					_state_last_transition_time = hrt_absolute_time();

					_velocity_setpoint.zero();
					_acceleration_setpoint.zero();
					_jerk_setpoint.zero();
				}

			} else {
				_state = DropState::DISARMED;
			}
		}
		break;

	case DropState::HORIZONTAL_VELOCITY_CONTROL_ENABLED: {

			if (_state_prev != _state) {
				mavlink_log_info(&_mavlink_log_pub, "Drop: full velocity control enabled");
			}

			if (actuator_armed.armed) {

				if (_state_prev != _state) {
					// reset constraints
					_constraints.speed_up = _param_mpc_z_vel_max_up.get();
					_constraints.speed_down = _param_mpc_z_vel_max_dn.get();
					_constraints.want_takeoff = true;

					// position still not active
					//_position_setpoint.setNaN();

					// otherwise all zero
					_velocity_setpoint.zero();
					_acceleration_setpoint.zero();
					_jerk_setpoint.zero();
				}

				if (position_valid && !failsafe_flags.local_position_invalid && (_velocity.length() < 5.f)) {
					_state = DropState::POSITION_CONTROL_ENABLED;
					_state_last_transition_time = hrt_absolute_time();
				}

			} else {
				_state = DropState::DISARMED;
			}
		}
		break;

	case DropState::POSITION_CONTROL_ENABLED: {

			if (_state_prev != _state) {
				mavlink_log_info(&_mavlink_log_pub, "Drop: full position control enabled");
			}

			if (actuator_armed.armed) {

				if (_state_prev != _state) {
					// reset constraints
					_constraints.speed_up = _param_mpc_z_vel_max_up.get();
					_constraints.speed_down = _param_mpc_z_vel_max_dn.get();
					_constraints.want_takeoff = true;

					// latch position setpoint
					_position_setpoint = _position;

					// otherwise all zero
					_velocity_setpoint.zero();
					_acceleration_setpoint.zero();
					_jerk_setpoint.zero();
				}

				const hrt_abstime hold_timeout_us = math::constrain(_param_mpc_drop_hold_t.get(), 0.f, 3600.f) * 1e6f;

				if (hrt_elapsed_time(&_state_last_transition_time) > hold_timeout_us) {
					//mavlink_log_info(&_mavlink_log_pub, "Drop: complete, full flying");
					_state = DropState::FLYING;
					_state_last_transition_time = hrt_absolute_time();

					// require offboard position setpoint
					_position_smoothing.forceSetPosition(_position);
					_position_smoothing.forceSetVelocity(_velocity);
				}

			} else {
				_state = DropState::DISARMED;
			}
		}
		break;

	case DropState::FLYING: {

			if (_state_prev != _state) {
				mavlink_log_info(&_mavlink_log_pub, "Drop: flying!");
			}

			if (actuator_armed.armed) {
				if (_state_prev != _state) {
					mavlink_log_info(&_mavlink_log_pub, "Drop: ready for OFFBOARD");

					// latch position setpoint
					_position_setpoint = _position;

					// default all zero
					_velocity_setpoint.zero();
					_acceleration_setpoint.zero();
					_jerk_setpoint.zero();


					// update params of the position smoothing
					_position_smoothing.setMaxAllowedHorizontalError(_param_mpc_xy_err_max.get());
					_position_smoothing.setVerticalAcceptanceRadius(_param_nav_mc_alt_rad.get());
					_position_smoothing.setCruiseSpeed(_param_mpc_xy_cruise.get());
					_position_smoothing.setHorizontalTrajectoryGain(_param_mpc_xy_traj_p.get());
					_position_smoothing.setTargetAcceptanceRadius(2.f); // TODO:

					// Update the constraints of the trajectories
					_position_smoothing.setMaxAccelerationXY(_param_mpc_acc_hor.get()); // TODO : Should be computed using heading
					_position_smoothing.setMaxVelocityXY(_param_mpc_xy_vel_max.get());
					float max_jerk = _param_mpc_jerk_auto.get();
					_position_smoothing.setMaxJerk({max_jerk, max_jerk, max_jerk}); // TODO : Should be computed using heading

					if (_velocity_setpoint(2) < 0.f) { // up
						_position_smoothing.setMaxVelocityZ(_param_mpc_z_v_auto_up.get());
						_position_smoothing.setMaxAccelerationZ(_param_mpc_acc_up_max.get());

					} else { // down
						_position_smoothing.setMaxAccelerationZ(_param_mpc_acc_down_max.get());
						_position_smoothing.setMaxVelocityZ(_param_mpc_z_v_auto_dn.get());
					}

					_position_smoothing.reset(Vector3f{}, _velocity, _position);
				}

				if (_offboard_trajectory_setpoint_sub.update()) {

					const trajectory_setpoint_s &trajectory_setpoint = _offboard_trajectory_setpoint_sub.get();

					if (hrt_elapsed_time(&trajectory_setpoint.timestamp) < 1_s) {

						// update params of the position smoothing
						_position_smoothing.setMaxAllowedHorizontalError(_param_mpc_xy_err_max.get());
						_position_smoothing.setVerticalAcceptanceRadius(_param_nav_mc_alt_rad.get());
						_position_smoothing.setCruiseSpeed(_param_mpc_xy_cruise.get());
						_position_smoothing.setHorizontalTrajectoryGain(_param_mpc_xy_traj_p.get());
						_position_smoothing.setTargetAcceptanceRadius(2.f);

						// Update the constraints of the trajectories
						_position_smoothing.setMaxAccelerationXY(_param_mpc_acc_hor.get()); // TODO : Should be computed using heading
						_position_smoothing.setMaxVelocityXY(_param_mpc_xy_vel_max.get());
						float max_jerk = _param_mpc_jerk_auto.get();
						_position_smoothing.setMaxJerk({max_jerk, max_jerk, max_jerk}); // TODO : Should be computed using heading

						bool is_emergency_braking_active = false; // TODO

						if (is_emergency_braking_active) {
							// When initializing with large velocity, allow 1g of
							// acceleration in 1s on all axes for fast braking
							_position_smoothing.setMaxAcceleration({9.81f, 9.81f, 9.81f});
							_position_smoothing.setMaxJerk({9.81f, 9.81f, 9.81f});

							// If the current velocity is beyond the usual constraints, tell
							// the controller to exceptionally increase its saturations to avoid
							// cutting out the feedforward
							_constraints.speed_down = math::max(fabsf(_position_smoothing.getCurrentVelocityZ()), _constraints.speed_down);
							_constraints.speed_up = math::max(fabsf(_position_smoothing.getCurrentVelocityZ()), _constraints.speed_up);

						} else if (_unsmoothed_velocity_setpoint(2) < 0.f) { // up
							float z_accel_constraint = _param_mpc_acc_up_max.get();
							float z_vel_constraint = _param_mpc_z_v_auto_up.get();

							_position_smoothing.setMaxVelocityZ(z_vel_constraint);
							_position_smoothing.setMaxAccelerationZ(z_accel_constraint);

						} else { // down
							_position_smoothing.setMaxAccelerationZ(_param_mpc_acc_down_max.get());
							_position_smoothing.setMaxVelocityZ(_param_mpc_z_v_auto_dn.get());
						}

						// Stretch the constraints of the velocity controller to leave some room for an additional
						// correction required by the altitude/vertical position controller
						_constraints.speed_down = math::max(_constraints.speed_down, 1.2f * _param_mpc_z_v_auto_dn.get());
						_constraints.speed_up = math::max(_constraints.speed_up, 1.2f * _param_mpc_z_v_auto_up.get());
						_constraints.want_takeoff = true;

						_offboard_pos_sp_last = Vector3f{trajectory_setpoint.position};
						_offboard_vel_sp_last = Vector3f{trajectory_setpoint.velocity};

						_yaw = trajectory_setpoint.yaw;
						//_yawspeed = trajectory_setpoint.yawspeed;
					}
				}

				if (_offboard_pos_sp_last.isAllFinite()) {

					bool force_zero_velocity_setpoint = false; // TODO

					PositionSmoothing::PositionSmoothingSetpoints smoothed_setpoints{};
					//float dt = math::constrain((_time_stamp_current - _time_stamp_last) * 1e-6f, 0.001f, 1.f);
					_position_smoothing.generateSetpoints(
						_position,
						_offboard_pos_sp_last,
						_offboard_vel_sp_last,
						_deltatime,
						force_zero_velocity_setpoint,
						smoothed_setpoints
					);
					_offboard_time_stamp_last = _time_stamp_current;

					_jerk_setpoint = smoothed_setpoints.jerk;
					_acceleration_setpoint = smoothed_setpoints.acceleration;
					_velocity_setpoint = smoothed_setpoints.velocity;
					_position_setpoint = smoothed_setpoints.position;

					_unsmoothed_velocity_setpoint = smoothed_setpoints.unsmoothed_velocity;
					//_want_takeoff = true;
				}

#if 0

				if (hrt_elapsed_time(&trajectory_setpoint.timestamp) > 10_s) {
					mavlink_log_critical(&_mavlink_log_pub, "Drop: OFFBOARD timeout, HOLDING");

					// latch position setpoint
					_position_setpoint = _position;

					// default all zero
					_velocity_setpoint.zero();
					_acceleration_setpoint.zero();
					_jerk_setpoint.zero();
				}

#endif

			} else {
				_state = DropState::DISARMED;
			}

		}
		break;
	}

	if (_state != state_prev) {
		_state_last_transition_time = hrt_absolute_time();
	}

	_state_prev = state_prev;



	// FALLTHROUGH
	// case WaypointType::takeoff:
	// case WaypointType::position:
	// default:
	// 	// Simple waypoint navigation: go to xyz target, with standard limitations
	// 	_position_setpoint = _target;
	// 	_velocity_setpoint.setNaN();
	// 	break;
	// }

	//const bool should_wait_for_yaw_align = _param_mpc_yaw_mode.get() == 4 && !_yaw_sp_aligned;
	//const bool force_zero_velocity_setpoint = should_wait_for_yaw_align || _is_emergency_braking_active;

	// _jerk_setpoint = smoothed_setpoints.jerk;
	// _acceleration_setpoint = smoothed_setpoints.acceleration;
	// _velocity_setpoint = smoothed_setpoints.velocity;
	// _position_setpoint = smoothed_setpoints.position;

	//_unsmoothed_velocity_setpoint = smoothed_setpoints.unsmoothed_velocity;
	//_want_takeoff = smoothed_setpoints.unsmoothed_velocity(2) < -0.3f;

	if (!PX4_ISFINITE(_yaw_setpoint) && !PX4_ISFINITE(_yawspeed_setpoint)) {
		// no valid heading -> generate heading in this flight task
		// Generate heading along trajectory if possible, otherwise hold the previous yaw setpoint

	}

	// If the FlightTask generates a yaw or a yawrate setpoint that exceeds this value
	// it will see its setpoint constrained here
	//_limitYawRate();

	//_constraints.want_takeoff = _checkTakeoff();

	return ret;
}

void FlightTaskDrop::_ekfResetHandlerPositionXY(const matrix::Vector2f &delta_xy)
{
	//_position_smoothing.forceSetPosition({_position(0), _position(1), NAN});
}

void FlightTaskDrop::_ekfResetHandlerVelocityXY(const matrix::Vector2f &delta_vxy)
{
	//_position_smoothing.forceSetVelocity({_velocity(0), _velocity(1), NAN});
}

void FlightTaskDrop::_ekfResetHandlerPositionZ(float delta_z)
{
	//_position_smoothing.forceSetPosition({NAN, NAN, _position(2)});
}

void FlightTaskDrop::_ekfResetHandlerVelocityZ(float delta_vz)
{
	//_position_smoothing.forceSetVelocity({NAN, NAN, _velocity(2)});
}

void FlightTaskDrop::_ekfResetHandlerHeading(float delta_psi)
{
	//_yaw_sp_prev += delta_psi;
}

void FlightTaskDrop::updateParams()
{
	FlightTask::updateParams();

}
