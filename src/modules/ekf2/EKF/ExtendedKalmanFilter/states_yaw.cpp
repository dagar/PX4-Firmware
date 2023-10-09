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

#include "ExtendedKalmanFilter.hpp"

#include "derivation/generated/compute_yaw_321_innov_var_and_h.h"
#include "derivation/generated/compute_yaw_312_innov_var_and_h.h"

#include "derivation/generated/quat_var_to_rot_var.h"

#include <lib/mathlib/math/Utilities.hpp>

using math::Utilities::shouldUse321RotationSequence;

void ExtendedKalmanFilter::computeYawInnovVarAndH(float variance, float &innovation_variance, VectorState &H_YAW) const
{
	if (shouldUse321RotationSequence(Dcmf(_state.quat_nominal))) {
		sym::ComputeYaw321InnovVarAndH(_state.vector(), P, variance, FLT_EPSILON, &innovation_variance, &H_YAW);

	} else {
		sym::ComputeYaw312InnovVarAndH(_state.vector(), P, variance, FLT_EPSILON, &innovation_variance, &H_YAW);
	}
}

bool ExtendedKalmanFilter::fuseYaw(const float obs_var, const float innov)
{
	float innov_var;
	VectorState H_YAW;
	computeYawInnovVarAndH(obs_var, innov_var, H_YAW);

	return fuseYaw(innov, innov_var, H_YAW);
}

bool ExtendedKalmanFilter::fuseYaw(const float innov, const float innov_var, const VectorState &H_YAW)
{
	// calculate the Kalman gains
	// only calculate gains for states we are using
	VectorState Kfusion;
	const float heading_innov_var_inv = 1.f / innov_var;

	for (uint8_t row = 0; row < State::size; row++) {
		for (uint8_t col = 0; col <= 3; col++) {
			Kfusion(row) += P(row, col) * H_YAW(col);
		}

		Kfusion(row) *= heading_innov_var_inv;
	}

	if (measurementUpdate(Kfusion, innov_var, innov)) {
		_time_last_yaw_fuse = _time_delayed_us;
		return true;
	}

	// otherwise
	return false;
}
