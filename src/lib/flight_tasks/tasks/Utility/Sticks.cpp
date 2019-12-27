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

/**
 * @file Sticks.cpp
 */

#include "Sticks.hpp"

using namespace time_literals;

Sticks::Sticks(ModuleParams *parent) :
	ModuleParams(parent)
{
	manual_control_switches_s manual_control_switches{};
	_manual_control_switches_sub.copy(&manual_control_switches);

	if (manual_control_switches.timestamp != 0) {
		_gear_switch_old = manual_control_switches.gear_switch;
	}
};

bool Sticks::checkAndSetStickInputs(hrt_abstime now)
{
	_vehicle_status_sub.update();

	if (!_vehicle_status_sub.get().rc_signal_lost) {

		manual_control_setpoint_s manual_control_setpoint;

		if (_manual_control_setpoint_sub.update(&manual_control_setpoint)) {
			// Linear scale
			_positions(0) = manual_control_setpoint.x; // NED x, pitch [-1,1]
			_positions(1) = manual_control_setpoint.y; // NED y, roll [-1,1]
			_positions(2) = -(manual_control_setpoint.z - 0.5f) * 2.f; // NED z, thrust rescaled from [0,1] to [-1,1]
			_positions(3) = manual_control_setpoint.r; // yaw [-1,1]

			// Exponential scale
			_positions_expo(0) = math::expo_deadzone(_positions(0), _param_mpc_xy_man_expo.get(), _param_mpc_hold_dz.get());
			_positions_expo(1) = math::expo_deadzone(_positions(1), _param_mpc_xy_man_expo.get(), _param_mpc_hold_dz.get());
			_positions_expo(2) = math::expo_deadzone(_positions(2), _param_mpc_z_man_expo.get(), _param_mpc_hold_dz.get());
			_positions_expo(3) = math::expo_deadzone(_positions(3), _param_mpc_yaw_expo.get(), _param_mpc_hold_dz.get());

			// valid stick inputs are required
			const bool valid_sticks = PX4_ISFINITE(_positions(0))
						  && PX4_ISFINITE(_positions(1))
						  && PX4_ISFINITE(_positions(2))
						  && PX4_ISFINITE(_positions(3));

			_input_available = valid_sticks;
		}

	} else {
		// Timeout: set all sticks to zero
		_positions.zero();
		_positions_expo.zero();
		_input_available = false;
	}

	return _input_available;
}

void Sticks::setGearAccordingToSwitch(landing_gear_s &gear)
{
	// Only switch the landing gear up if the user switched from gear down to gear up.
	// If the user had the switch in the gear up position and took off ignore it
	// until he toggles the switch to avoid retracting the gear immediately on takeoff.
	manual_control_switches_s manual_control_switches;

	if (_manual_control_switches_sub.update(&manual_control_switches)) {
		if (_gear_switch_old != manual_control_switches.gear_switch) {
			if (manual_control_switches.gear_switch == manual_control_switches_s::SWITCH_POS_OFF) {
				gear.landing_gear = landing_gear_s::GEAR_DOWN;

			} else if (manual_control_switches.gear_switch == manual_control_switches_s::SWITCH_POS_ON) {
				gear.landing_gear = landing_gear_s::GEAR_UP;
			}
		}

		_gear_switch_old = manual_control_switches.gear_switch;
	}
}
