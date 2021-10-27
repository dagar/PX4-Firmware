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

/**
 * @file ActuatorEffectivenessPlane.cpp
 *
 * Actuator effectiveness for a plane.
 *
 */

#include "ActuatorEffectivenessPlane.hpp"

void ActuatorEffectivenessPlane::updateAirspeedScaling(bool force)
{
	if (_airspeed_sub.updated() || force) {
		airspeed_validated_s airspeed;

		if (_airspeed_sub.copy(&airspeed)) {
			if (PX4_ISFINITE(airspeed.indicated_airspeed_m_s)) {
				const float airspeed_scaling = _param_airspeed_ias_trim.get() / math::max(airspeed.indicated_airspeed_m_s, 3.0f);

				if (fabsf(airspeed_scaling - _airspeed_scaling_prev) > 0.1f) {
					const float SC = 1.f / (airspeed_scaling * airspeed_scaling);

					const float B_plane[NUM_AXES][NUM_ACTUATORS] = {
						//  0,   1,   2,         3,   4,          5,         6,   7,         8,   9,  10,  11,  12,  13,  14,  15
						{ 0.f, 0.f, 0.f,       0.f, 0.f, -0.5f * SC, 0.5f * SC, 0.f,       0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f}, // ROLL
						{ 0.f, 0.f, 0.f,       0.f, 0.f,  0.f,       0.f,       0.5f * SC, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f}, // PITCH
						{ 0.f, 0.f, 0.5f * SC, 0.f, 0.f,  0.f,       0.f,       0.f,       0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f}, // YAW
						{ 0.f, 0.f, 0.f,       0.f, 1.f,  0.f,       0.f,       0.f,       0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f}, // THRUST X
						{ 0.f, 0.f, 0.f,       0.f, 0.f,  0.f,       0.f,       0.f,       0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f}, // THRUST Y
						{ 0.f, 0.f, 0.f,       0.f, 0.f,  0.f,       0.f,       0.f,       0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f}  // THRUST Z
					};

					_effectiveness = matrix::Matrix<float, NUM_AXES, NUM_ACTUATORS>(B_plane);

					_airspeed_scaling_prev = airspeed_scaling;

					_updated = true;
				}
			}
		}
	}
}

bool ActuatorEffectivenessPlane::getEffectivenessMatrix(matrix::Matrix<float, NUM_AXES, NUM_ACTUATORS> &matrix,
		bool force)
{
	updateAirspeedScaling(force);

	if (_updated) {
		matrix = _effectiveness;
		return true;
	}

	return false;
}
