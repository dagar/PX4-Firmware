/****************************************************************************
 *
 *   Copyright (c) 2015-2024 PX4 Development Team. All rights reserved.
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

#include "../ExtendedKalmanFilter.hpp"

using matrix::Vector2f;
using matrix::Vector3f;

bool ExtendedKalmanFilter::fuseHorizontalVelocity(const Vector2f &innovation, const Vector2f &innovation_variance,
		const Vector2f &observation_variance)
{
	bool fused = false;

	// vx, vy
	if (fuseDirectStateMeasurement(innovation(0), innovation_variance(0), observation_variance(0), State::vel.idx + 0)
	    && fuseDirectStateMeasurement(innovation(1), innovation_variance(1), observation_variance(1), State::vel.idx + 1)
	   ) {
		fused = true;
	}

	return fused;
}

bool ExtendedKalmanFilter::fuseVelocity(const Vector3f &innovation, const Vector3f &innovation_variance,
					const Vector3f &observation_variance)
{
	bool fused = false;

	// vx, vy, vz
	if (fuseDirectStateMeasurement(innovation(0), innovation_variance(0), observation_variance(0), State::vel.idx + 0)
	    && fuseDirectStateMeasurement(innovation(1), innovation_variance(1), observation_variance(1), State::vel.idx + 1)
	    && fuseDirectStateMeasurement(innovation(2), innovation_variance(2), observation_variance(2), State::vel.idx + 2)
	   ) {
		fused = true;
	}

	return fused;
}

void ExtendedKalmanFilter::resetHorizontalVelocityTo(const Vector2f &new_horz_vel, const Vector2f &new_horz_vel_var)
{
	const Vector2f delta_horz_vel = new_horz_vel - Vector2f(_state.vel);
	_state.vel.xy() = new_horz_vel;

	if (PX4_ISFINITE(new_horz_vel_var(0))) {
		P.uncorrelateCovarianceSetVariance<1>(State::vel.idx, math::max(sq(0.01f), new_horz_vel_var(0)));
	}

	if (PX4_ISFINITE(new_horz_vel_var(1))) {
		P.uncorrelateCovarianceSetVariance<1>(State::vel.idx + 1, math::max(sq(0.01f), new_horz_vel_var(1)));
	}

	// record the state change
	if (_state_reset_status.reset_count.velNE == _state_reset_count_prev.velNE) {
		_state_reset_status.velNE_change = delta_horz_vel;

	} else {
		// there's already a reset this update, accumulate total delta
		_state_reset_status.velNE_change += delta_horz_vel;
	}

	_state_reset_status.reset_count.velNE++;
}

void ExtendedKalmanFilter::resetVerticalVelocityTo(float new_vert_vel, float new_vert_vel_var)
{
	const float delta_vert_vel = new_vert_vel - _state.vel(2);
	_state.vel(2) = new_vert_vel;

	if (PX4_ISFINITE(new_vert_vel_var)) {
		P.uncorrelateCovarianceSetVariance<1>(State::vel.idx + 2, math::max(sq(0.01f), new_vert_vel_var));
	}

	// record the state change
	if (_state_reset_status.reset_count.velD == _state_reset_count_prev.velD) {
		_state_reset_status.velD_change = delta_vert_vel;

	} else {
		// there's already a reset this update, accumulate total delta
		_state_reset_status.velD_change += delta_vert_vel;
	}

	_state_reset_status.reset_count.velD++;
}

void ExtendedKalmanFilter::resetVelocityTo(const Vector3f &new_vel, const Vector3f &new_vel_var)
{
	resetHorizontalVelocityTo(Vector2f(new_vel), Vector2f(new_vel_var(0), new_vel_var(1)));
	resetVerticalVelocityTo(new_vel(2), new_vel_var(2));
}
