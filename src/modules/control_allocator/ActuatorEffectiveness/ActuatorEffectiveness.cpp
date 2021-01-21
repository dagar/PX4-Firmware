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

#include "ActuatorEffectiveness.hpp"

#include <lib/parameters/param.h>

// void ActuatorEffectiveness::updateAirspeedScaling(const float airspeed_scaling)
// {
// 	_updated = true;

// 	const float SC = 1.0f / (airspeed_scaling * airspeed_scaling);

// 	const float B_plane[NUM_AXES][NUM_ACTUATORS] = {
// 		{ 0.f, 0.f, 0.f,       0.f, 0.f, -0.5f * SC, 0.5f * SC, 0.f,       0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
// 		{ 0.f, 0.f, 0.f,       0.f, 0.f,  0.f,       0.f,       0.5f * SC, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
// 		{ 0.f, 0.f, 0.5f * SC, 0.f, 0.f,  0.f,       0.f,       0.f,       0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
// 		{ 0.f, 0.f, 0.f,       0.f, 1.f,  0.f,       0.f,       0.f,       0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
// 		{ 0.f, 0.f, 0.f,       0.f, 0.f,  0.f,       0.f,       0.f,       0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
// 		{ 0.f, 0.f, 0.f,       0.f, 0.f,  0.f,       0.f,       0.f,       0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f}
// 	};

// 	_effectiveness = matrix::Matrix<float, NUM_AXES, NUM_ACTUATORS>(B_plane);
// }

bool ActuatorEffectiveness::getEffectivenessMatrix(Matrix<float, NUM_AXES, NUM_ACTUATORS> &matrix)
{
	// Check if parameters have changed
	if (_updated || _parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		_updated = false;

		// CA_ACT${i}_TRQ_R
		// CA_ACT${i}_TRQ_P
		// CA_ACT${i}_TRQ_Y
		// CA_ACT${i}_THR_X
		// CA_ACT${i}_THR_Y
		// CA_ACT${i}_THR_Z

		for (int n = 0; n < 16; n++) {
			char torque_str[3][16];
			sprintf(torque_str[0], "CA_ACT%u_TRQ_R", n);
			sprintf(torque_str[1], "CA_ACT%u_TRQ_P", n);
			sprintf(torque_str[2], "CA_ACT%u_TRQ_Y", n);
			param_get(param_find(torque_str[0]), &_effectiveness(0, n));
			param_get(param_find(torque_str[1]), &_effectiveness(1, n));
			param_get(param_find(torque_str[2]), &_effectiveness(2, n));

			for (int axis = 0; axis < 3; axis++) {
				char str[16];
				char axis_char = 'X' + axis;
				sprintf(str, "CA_ACT%u_THR_%c", n, axis_char);
				param_get(param_find(str), &_effectiveness(3 + axis, n));
			}
		}

		_num_actuators = 16; // TODO

		return true;
	}

	return false;
}
