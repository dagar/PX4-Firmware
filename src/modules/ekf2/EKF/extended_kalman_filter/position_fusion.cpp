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

#include "ExtendedKalmanFilter.hpp"

using matrix::Vector2f;

bool ExtendedKalmanFilter::fuseHorizontalPosition(const Vector2f &innovation, const Vector2f &innovation_variance, const Vector2f &observation_variance)
{
	bool fused = false;

	// x & y
	if (fuseDirectStateMeasurement(innovation(0), innovation_variance(0), observation_variance(0), State::pos.idx + 0)
	    && fuseDirectStateMeasurement(innovation(1), innovation_variance(1), observation_variance(1), State::pos.idx + 1)
	   ) {
		_time_last_hor_pos_fuse = _time_us;

		fused = true;
	}

	return fused;
}

bool ExtendedKalmanFilter::fuseVerticalPosition(const float innovation, const float innovation_variance, const float observation_variance)
{
	bool fused = false;

	// z
	if (fuseDirectStateMeasurement(innovation, innovation_variance, observation_variance, State::pos.idx + 2)
	   ) {
		_time_last_hgt_fuse = _time_us;

		fused = true;
	}

	return fused;
}

void ExtendedKalmanFilter::resetHorizontalPositionTo(const Vector2f &new_horz_pos, const Vector2f &new_horz_pos_var)
{
	const Vector2f delta_horz_pos{new_horz_pos - Vector2f{_state.pos}};
	_state.pos.xy() = new_horz_pos;

	if (PX4_ISFINITE(new_horz_pos_var(0))) {
		P.uncorrelateCovarianceSetVariance<1>(State::pos.idx, math::max(sq(0.01f), new_horz_pos_var(0)));
	}

	if (PX4_ISFINITE(new_horz_pos_var(1))) {
		P.uncorrelateCovarianceSetVariance<1>(State::pos.idx + 1, math::max(sq(0.01f), new_horz_pos_var(1)));
	}

	// record the state change
	if (_state_reset_status.reset_count.posNE == _state_reset_count_prev.posNE) {
		_state_reset_status.posNE_change = delta_horz_pos;

	} else {
		// there's already a reset this update, accumulate total delta
		_state_reset_status.posNE_change += delta_horz_pos;
	}

	_state_reset_status.reset_count.posNE++;

	// Reset the timout timer
	_time_last_hor_pos_fuse = _time_us;
}

void ExtendedKalmanFilter::resetVerticalPositionTo(const float new_vert_pos, float new_vert_pos_var)
{
	const float old_vert_pos = _state.pos(2);
	_state.pos(2) = new_vert_pos;

	if (PX4_ISFINITE(new_vert_pos_var)) {
		// the state variance is the same as the observation
		P.uncorrelateCovarianceSetVariance<1>(State::pos.idx + 2, math::max(sq(0.01f), new_vert_pos_var));
	}

	const float delta_z = new_vert_pos - old_vert_pos;

	// record the state change
	if (_state_reset_status.reset_count.posD == _state_reset_count_prev.posD) {
		_state_reset_status.posD_change = delta_z;

	} else {
		// there's already a reset this update, accumulate total delta
		_state_reset_status.posD_change += delta_z;
	}

	_state_reset_status.reset_count.posD++;

	// Reset the timout timer
	_time_last_hgt_fuse = _time_us;
}
