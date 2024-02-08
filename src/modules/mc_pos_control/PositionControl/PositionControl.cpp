/****************************************************************************
 *
 *   Copyright (c) 2018 - 2019 PX4 Development Team. All rights reserved.
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
 * @file PositionControl.cpp
 */

#include "PositionControl.hpp"
#include "ControlMath.hpp"
#include <float.h>
#include <mathlib/mathlib.h>
#include <px4_platform_common/defines.h>
#include <geo/geo.h>

using namespace matrix;

const trajectory_setpoint_s PositionControl::empty_trajectory_setpoint = {0, {NAN, NAN, NAN}, {NAN, NAN, NAN}, {NAN, NAN, NAN}, {NAN, NAN, NAN}, NAN, NAN};

void PositionControl::setVelocityGains(const Vector3f &P, const Vector3f &I, const Vector3f &D)
{
	_gain_vel_p = P;
	_gain_vel_i = I;
	_gain_vel_d = D;
}

void PositionControl::setVelocityLimits(const float vel_horizontal, const float vel_up, const float vel_down)
{
	_lim_vel_horizontal = vel_horizontal;
	_lim_vel_up = vel_up;
	_lim_vel_down = vel_down;
}

void PositionControl::setThrustLimits(const float min, const float max)
{
	// make sure there's always enough thrust vector length to infer the attitude
	_lim_thr_min = math::max(min, 10e-4f);
	_lim_thr_max = max;
}

void PositionControl::setHorizontalThrustMargin(const float margin)
{
	_lim_thr_xy_margin = margin;
}

void PositionControl::updateHoverThrust(const float hover_thrust_new)
{
	// Given that the equation for thrust is T = a_sp * Th / g - Th
	// with a_sp = desired acceleration, Th = hover thrust and g = gravity constant,
	// we want to find the acceleration that needs to be added to the integrator in order obtain
	// the same thrust after replacing the current hover thrust by the new one.
	// T' = T => a_sp' * Th' / g - Th' = a_sp * Th / g - Th
	// so a_sp' = (a_sp - g) * Th / Th' + g
	// we can then add a_sp' - a_sp to the current integrator to absorb the effect of changing Th by Th'
	const float previous_hover_thrust = _hover_thrust;
	setHoverThrust(hover_thrust_new);

	_vel_int(2) += (_acc_sp(2) - CONSTANTS_ONE_G) * previous_hover_thrust / _hover_thrust
		       + CONSTANTS_ONE_G - _acc_sp(2);
}

bool PositionControl::setInputSetpoint(const trajectory_setpoint_s &setpoint)
{
	// Note: NAN value means no feed forward/leave state uncontrolled if there's no higher order setpoint.
	const Vector3f pos_sp{setpoint.position};
	const Vector3f vel_sp{setpoint.velocity};
	const Vector3f acc_sp{setpoint.acceleration};

	// xy
	bool pos_sp_xy_valid = PX4_ISFINITE(pos_sp(0)) && PX4_ISFINITE(pos_sp(1));
	bool vel_sp_xy_valid = PX4_ISFINITE(vel_sp(0)) && PX4_ISFINITE(vel_sp(1));
	bool acc_sp_xy_valid = PX4_ISFINITE(acc_sp(0)) && PX4_ISFINITE(acc_sp(1));

	if (pos_sp_xy_valid) {
		// XY position: position enabled, velocity FF, acc FF
		_pos_xy_active = true;
		_pos_sp.xy() = pos_sp.xy();

		_vel_xy_active = true;
		_vel_ff.xy() = vel_sp_xy_valid ? vel_sp.xy() : Vector2f{0.f, 0.f};

		_acc_ff.xy() = acc_sp_xy_valid ? acc_sp.xy() : Vector2f{0.f, 0.f};

	} else if (vel_sp_xy_valid) {
		// XY velocity: position disabled, velocity enabled, acc is feed-forward
		_pos_xy_active = false;
		_pos_sp.xy() = Vector2f{NAN, NAN}; // disabled

		_vel_xy_active = true;
		_vel_sp.xy() = vel_sp.xy();
		_vel_ff.xy() = Vector2f{0.f, 0.f}; // no FF

		_acc_ff.xy() = acc_sp_xy_valid ? acc_sp.xy() : Vector2f{0.f, 0.f};

	} else if (acc_sp_xy_valid) {
		// XY acceleration: position disabled, velocity disabled, acc enabled
		_pos_xy_active = false;
		_pos_sp.xy() = Vector2f{NAN, NAN}; // disabled

		_vel_xy_active = false;
		_vel_sp.xy() = Vector2f{NAN, NAN}; // disabled
		_vel_ff.xy() = Vector2f{0.f, 0.f};

		_acc_sp.xy() = acc_sp.xy();
		_acc_ff.xy() = Vector2f{0.f, 0.f}; // no FF

	} else {
		// invalid setpoint
		return false;
	}


	// z
	bool pos_sp_z_valid = PX4_ISFINITE(pos_sp(2));
	bool vel_sp_z_valid = PX4_ISFINITE(vel_sp(2));
	bool acc_sp_z_valid = PX4_ISFINITE(acc_sp(2));

	if (pos_sp_z_valid) {
		// Z position: position enabled, velocity FF, acc FF
		_pos_z_active = true;
		_pos_sp(2) = pos_sp(2);

		_vel_z_active = true;
		_vel_ff(2) = vel_sp_z_valid ? vel_sp(2) : 0.f;

		_acc_ff(2) = acc_sp_z_valid ? acc_sp(2) : 0.f;

	} else if (vel_sp_z_valid) {
		// Z velocity: position disabled, velocity enabled, acc is feed-forward
		_pos_z_active = false;
		_pos_sp(2) = NAN; // disabled

		_vel_z_active = true;
		_vel_sp(2) = vel_sp(2);
		_vel_ff(2) = 0.f;

		_acc_ff(2) = acc_sp_z_valid ? acc_sp(2) : 0.f;

	} else if (acc_sp_z_valid) {
		// Z acceleration: position disabled, velocity disabled, acc enabled
		_pos_z_active = false;
		_pos_sp(2) = NAN; // disabled

		_vel_z_active = true;
		_vel_sp(2) = NAN; // disabled
		_vel_ff(2) = 0.f;

		_acc_sp(2) = acc_sp(2);
		_acc_ff(2) = 0.f; // no FF

	} else {
		// invalid setpoint
		return false;
	}

	return true;
}

bool PositionControl::update(const PositionControlStates &states, const float dt)
{
	bool valid = true;

	// For each controlled state the estimate has to be valid
	for (int i = 0; i <= 2; i++) {
		if (PX4_ISFINITE(_pos_sp(i))) {
			valid = valid && PX4_ISFINITE(states.position(i));
		}

		if (PX4_ISFINITE(_vel_sp(i))) {
			valid = valid && PX4_ISFINITE(states.velocity(i)) && PX4_ISFINITE(states.acceleration(i));
		}
	}

	if (valid) {

		_positionControl(states.position);



		// if ((PX4_ISFINITE(pos_sp(0)) == PX4_ISFINITE(pos_sp(1)))
		// && (PX4_ISFINITE(states.position(0)) == PX4_ISFINITE(states.position(1)))
		// && (PX4_ISFINITE(pos_sp(2)))
		// ) {

		// }

		_velocityControl(states.velocity, states.acceleration, dt);

		// There has to be a valid output acceleration and thrust setpoint otherwise something went wrong
		return _acc_sp.isAllFinite() && _thr_sp.isAllFinite();
	}

	return false;
}

void PositionControl::_positionControl(const Vector3f &pos)
{
	// P-position controller
	Vector3f vel_sp = (_pos_sp - pos).emult(_gain_pos_p);

	if (_pos_xy_active) {
		_vel_sp_out.xy() = vel_sp.xy();
	}

	if (_pos_z_active) {
		_vel_sp_out(2) = vel_sp(2);
	}
}

void PositionControl::_velocityControl(const Vector3f &vel, const Vector3f &vel_dot, const float dt)
{
	Vector3f vel_sp;

	if (_vel_xy_active) {
		// Constrain horizontal velocity by prioritizing the velocity component along the
		// the desired position setpoint over the feed-forward term.
		vel_sp.xy() = ControlMath::constrainXY(_vel_sp.xy(), _vel_ff.xy(), _lim_vel_horizontal);
	}

	if (_vel_z_active) {
		// Constrain velocity in z-direction.
		vel_sp(2) = math::constrain(_vel_sp(2) + _vel_ff(2), -_lim_vel_up, _lim_vel_down);

		// Constrain vertical velocity integral
		_vel_int(2) = math::constrain(_vel_int(2), -CONSTANTS_ONE_G, CONSTANTS_ONE_G);
	}


	// PID velocity control
	Vector3f vel_error = vel_sp - vel;
	Vector3f acc_sp = vel_error.emult(_gain_vel_p) + _vel_int - vel_dot.emult(_gain_vel_d);

	//if (_vel_xy_active) {
	// 	_acc_sp_out.xy() = acc_sp.xy() + _acc_ff.xy();

	// } else {
	// 	_acc_sp_out.xy() = _acc_sp.xy();
	// }

	// if (_vel_z_active) {
	// 	_acc_sp_out(2) = acc_sp(2) + _acc_ff(2);

	// } else {
	// 	_acc_sp_out(2) = _acc_sp(2);
	// }

	_acc_sp_out = acc_sp;

	/**< desired thrust */
	_accelerationControl(_acc_sp_out);

	if (_vel_xy_active) {
		// Use tracking Anti-Windup for horizontal direction: during saturation, the integrator is used to unsaturate the output
		// see Anti-Reset Windup for PID controllers, L.Rundqwist, 1990
		const Vector2f acc_sp_xy_produced = Vector2f(_thr_sp) * (CONSTANTS_ONE_G / _hover_thrust);
		const float arw_gain = 2.f / _gain_vel_p(0);

		// The produced acceleration can be greater or smaller than the desired acceleration due to the saturations and the actual vertical thrust (computed independently).
		// The ARW loop needs to run if the signal is saturated only.
		const Vector2f acc_sp_xy = _acc_sp_out.xy();
		const Vector2f acc_limited_xy = (acc_sp_xy.norm_squared() > acc_sp_xy_produced.norm_squared())
						? acc_sp_xy_produced
						: acc_sp_xy;
		Vector2f vel_xy_error = Vector2f(vel_error) - arw_gain * (acc_sp_xy - acc_limited_xy);

		// Update integral part of velocity control
		_vel_int(0) += vel_xy_error(0) * _gain_vel_i(0) * dt;
		_vel_int(1) += vel_xy_error(1) * _gain_vel_i(1) * dt;
	}

	if (_vel_z_active) {
		// Integrator anti-windup in vertical direction
		if ((_thr_sp(2) >= -_lim_thr_min && vel_error(2) >= 0.f) ||
		    (_thr_sp(2) <= -_lim_thr_max && vel_error(2) <= 0.f)) {

			vel_error(2) = 0.f;

		} else {
			_vel_int(2) += vel_error(2) * _gain_vel_i(2) * dt;
		}
	}
}

void PositionControl::_accelerationControl(const Vector3f &acc_sp)
{
	// Assume standard acceleration due to gravity in vertical direction for attitude generation
	Vector3f body_z = Vector3f(-acc_sp(0), -acc_sp(1), CONSTANTS_ONE_G).normalized();
	ControlMath::limitTilt(body_z, Vector3f(0, 0, 1), _lim_tilt);
	// Scale thrust assuming hover thrust produces standard gravity
	float collective_thrust = acc_sp(2) * (_hover_thrust / CONSTANTS_ONE_G) - _hover_thrust;
	// Project thrust to planned body attitude
	collective_thrust /= (Vector3f(0, 0, 1).dot(body_z));
	collective_thrust = math::min(collective_thrust, -_lim_thr_min);

	_thr_sp = body_z * collective_thrust;

	// Prioritize vertical control while keeping a horizontal margin
	const Vector2f thrust_sp_xy(_thr_sp);
	const float thrust_sp_xy_norm = thrust_sp_xy.norm();
	const float thrust_max_squared = math::sq(_lim_thr_max);

	// Determine how much vertical thrust is left keeping horizontal margin
	const float allocated_horizontal_thrust = math::min(thrust_sp_xy_norm, _lim_thr_xy_margin);
	const float thrust_z_max_squared = thrust_max_squared - math::sq(allocated_horizontal_thrust);

	// Saturate maximal vertical thrust
	_thr_sp(2) = math::max(_thr_sp(2), -sqrtf(thrust_z_max_squared));

	// Determine how much horizontal thrust is left after prioritizing vertical control
	const float thrust_max_xy_squared = thrust_max_squared - math::sq(_thr_sp(2));
	float thrust_max_xy = 0.f;

	if (thrust_max_xy_squared > 0.f) {
		thrust_max_xy = sqrtf(thrust_max_xy_squared);
	}

	// Saturate thrust in horizontal direction
	if (thrust_sp_xy_norm > thrust_max_xy) {
		_thr_sp.xy() = thrust_sp_xy / thrust_sp_xy_norm * thrust_max_xy;
	}
}

void PositionControl::getLocalPositionSetpoint(vehicle_local_position_setpoint_s &local_position_setpoint) const
{
	local_position_setpoint.x = _pos_sp(0);
	local_position_setpoint.y = _pos_sp(1);
	local_position_setpoint.z = _pos_sp(2);
	local_position_setpoint.vx = _vel_sp(0);
	local_position_setpoint.vy = _vel_sp(1);
	local_position_setpoint.vz = _vel_sp(2);
	_acc_sp.copyTo(local_position_setpoint.acceleration);
	_thr_sp.copyTo(local_position_setpoint.thrust);
}
