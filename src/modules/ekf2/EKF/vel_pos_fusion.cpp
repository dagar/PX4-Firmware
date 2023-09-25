/****************************************************************************
 *
 *   Copyright (c) 2015-2023 PX4 Development Team. All rights reserved.
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
 * @file vel_pos_fusion.cpp
 *
 * @author Roman Bast <bapstroman@gmail.com>
 * @author Siddharth Bharat Purohit <siddharthbharatpurohit@gmail.com>
 * @author Paul Riseborough <p_riseborough@live.com.au>
 *
 */

#include <mathlib/mathlib.h>
#include "ekf.h"

void Ekf::updateVelocityAidSrcStatus(const uint64_t &time_us, const Vector2f &obs, const Vector2f &obs_var,
				     const float innov_gate, estimator_aid_source2d_s &aid_src) const
{
	Vector2f observation_variance {
		math::max(sq(0.01f), obs_var(0)),
		math::max(sq(0.01f), obs_var(1))
	};

	Vector2f innov = Vector2f(_state.vel.xy()) - obs;
	Vector2f innov_var = P.slice<2, 2>(State::vel.idx, State::vel.idx).diag() + observation_variance;

	updateEstimatorAidStatus(aid_src,
				 time_us,              // sample timestamp
				 obs,                  // observation
				 observation_variance, // observation variance
				 innov,                // innovation
				 innov_var,            // innovation variance
				 innov_gate);          // gate sigma
}

void Ekf::updateVelocityAidSrcStatus(const uint64_t &time_us, const Vector3f &obs, const Vector3f &obs_var,
				     const float innov_gate, estimator_aid_source3d_s &aid_src) const
{
	Vector3f observation_variance {
		math::max(sq(0.01f), obs_var(0)),
		math::max(sq(0.01f), obs_var(1)),
		math::max(sq(0.01f), obs_var(2))
	};

	Vector3f innov = _state.vel - obs;
	Vector3f innov_var = P.slice<3, 3>(State::vel.idx, State::vel.idx).diag() + observation_variance;

	updateEstimatorAidStatus(aid_src,
				 time_us,              // sample timestamp
				 obs,                  // observation
				 observation_variance, // observation variance
				 innov,                // innovation
				 innov_var,            // innovation variance
				 innov_gate);          // gate sigma

	// vz special case if there is bad vertical acceleration data, then don't reject measurement,
	// but limit innovation to prevent spikes that could destabilise the filter
	if (_fault_status.flags.bad_acc_vertical && aid_src.innovation_rejected) {
		const float innov_limit = innov_gate * sqrtf(aid_src.innovation_variance[2]);
		aid_src.innovation[2] = math::constrain(aid_src.innovation[2], -innov_limit, innov_limit);
		aid_src.innovation_rejected = false;
	}
}

void Ekf::updateVerticalPositionAidSrcStatus(const uint64_t &time_us, const float obs, const float obs_var,
		const float innov_gate, estimator_aid_source1d_s &aid_src) const
{
	float observation_variance = math::max(sq(0.01f), obs_var);

	float innov = _state.pos(2) - obs;
	float innov_var = P(State::pos.idx+2, State::pos.idx+2) + observation_variance;

	updateEstimatorAidStatus(aid_src,
				 time_us,              // sample timestamp
				 obs,                  // observation
				 observation_variance, // observation variance
				 innov,                // innovation
				 innov_var,            // innovation variance
				 innov_gate);          // gate sigma

	// z special case if there is bad vertical acceleration data, then don't reject measurement,
	// but limit innovation to prevent spikes that could destabilise the filter
	if (_fault_status.flags.bad_acc_vertical && aid_src.innovation_rejected) {
		const float innov_limit = innov_gate * sqrtf(aid_src.innovation_variance);
		aid_src.innovation = math::constrain(aid_src.innovation, -innov_limit, innov_limit);
		aid_src.innovation_rejected = false;
	}
}

void Ekf::updateHorizontalPositionAidSrcStatus(const uint64_t &time_us, const Vector2f &obs, const Vector2f &obs_var,
		const float innov_gate, estimator_aid_source2d_s &aid_src) const
{
	Vector2f observation_variance {
		math::max(sq(0.01f), obs_var(0)),
		math::max(sq(0.01f), obs_var(1))
	};

	Vector2f innov = Vector2f(_state.pos.xy()) - obs;
	Vector2f innov_var = P.slice<2, 2>(State::pos.idx, State::pos.idx).diag() + observation_variance;

	updateEstimatorAidStatus(aid_src,
				 time_us,              // sample timestamp
				 obs,                  // observation
				 observation_variance, // observation variance
				 innov,                // innovation
				 innov_var,            // innovation variance
				 innov_gate);          // gate sigma
}

bool Ekf::fuseVelocity(estimator_aid_source2d_s &aid_src)
{
	// fuse vx, vy
	if (!aid_src.innovation_rejected) {
		if (fuseVelPosHeight(aid_src.innovation[0], aid_src.innovation_variance[0], State::vel.idx)
		    && fuseVelPosHeight(aid_src.innovation[1], aid_src.innovation_variance[1], State::vel.idx + 1)
		   ) {
			aid_src.fused = true;
			aid_src.time_last_fuse = _time_delayed_us;
			return true;
		}
	}

	aid_src.fused = false;
	return false;
}

bool Ekf::resetVelocity(estimator_aid_source2d_s &aid_src)
{
	// reset vx, vy
	if (PX4_ISFINITE(aid_src.observation[0]) && PX4_ISFINITE(aid_src.observation_variance[0])
	    && PX4_ISFINITE(aid_src.observation[1]) && PX4_ISFINITE(aid_src.observation_variance[1])
	   ) {
		resetHorizontalVelocityTo(Vector2f(aid_src.observation), Vector2f(aid_src.observation_variance));

		aid_src.innovation[0] = 0.f;
		aid_src.innovation[1] = 0.f;

		aid_src.innovation_filtered[0] = 0.f;
		aid_src.innovation_filtered[1] = 0.f;

		aid_src.test_ratio[0] = 0.f;
		aid_src.test_ratio[1] = 0.f;

		aid_src.test_ratio_filtered[0] = 0.f;
		aid_src.test_ratio_filtered[1] = 0.f;

		aid_src.time_last_fuse = _time_delayed_us;

		aid_src.innovation_rejected = false;
		aid_src.fused = true;

		setVelPosStatus(State::vel.idx + 0, true);
		setVelPosStatus(State::vel.idx + 1, true);

		return true;
	}

	return false;
}

bool Ekf::fuseVelocity(estimator_aid_source3d_s &aid_src)
{
	// fuse vx, vy, vz
	if (!aid_src.innovation_rejected) {
		if (fuseVelPosHeight(aid_src.innovation[0], aid_src.innovation_variance[0], State::vel.idx + 0)
		    && fuseVelPosHeight(aid_src.innovation[1], aid_src.innovation_variance[1], State::vel.idx + 1)
		    && fuseVelPosHeight(aid_src.innovation[2], aid_src.innovation_variance[2], State::vel.idx + 2)
		   ) {
			aid_src.fused = true;
			aid_src.time_last_fuse = _time_delayed_us;
			return true;
		}
	}

	aid_src.fused = false;
	return false;
}

bool Ekf::resetVelocity(estimator_aid_source3d_s &aid_src)
{
	// reset vx, vy, vz
	if (PX4_ISFINITE(aid_src.observation[0]) && PX4_ISFINITE(aid_src.observation_variance[0])
	    && PX4_ISFINITE(aid_src.observation[1]) && PX4_ISFINITE(aid_src.observation_variance[1])
	    && PX4_ISFINITE(aid_src.observation[2]) && PX4_ISFINITE(aid_src.observation_variance[2])
	   ) {
		resetVelocityTo(Vector3f(aid_src.observation), Vector3f(aid_src.observation_variance));

		aid_src.innovation[0] = 0.f;
		aid_src.innovation[1] = 0.f;
		aid_src.innovation[2] = 0.f;

		aid_src.innovation_filtered[0] = 0.f;
		aid_src.innovation_filtered[1] = 0.f;
		aid_src.innovation_filtered[2] = 0.f;

		aid_src.test_ratio[0] = 0.f;
		aid_src.test_ratio[1] = 0.f;
		aid_src.test_ratio[2] = 0.f;

		aid_src.test_ratio_filtered[0] = 0.f;
		aid_src.test_ratio_filtered[1] = 0.f;
		aid_src.test_ratio_filtered[2] = 0.f;

		aid_src.time_last_fuse = _time_delayed_us;

		aid_src.innovation_rejected = false;
		aid_src.fused = true;

		setVelPosStatus(State::vel.idx + 0, true);
		setVelPosStatus(State::vel.idx + 1, true);
		setVelPosStatus(State::vel.idx + 2, true);

		return true;
	}

	return false;
}

bool Ekf::fuseHorizontalPosition(estimator_aid_source2d_s &aid_src)
{
	// fuse x & y
	if (!aid_src.innovation_rejected) {
		if (fuseVelPosHeight(aid_src.innovation[0], aid_src.innovation_variance[0], State::pos.idx)
		    && fuseVelPosHeight(aid_src.innovation[1], aid_src.innovation_variance[1], State::pos.idx + 1)
		   ) {
			aid_src.fused = true;
			aid_src.time_last_fuse = _time_delayed_us;
			return true;
		}
	}

	aid_src.fused = false;
	return aid_src.fused;
}

bool Ekf::resetHorizontalPosition(estimator_aid_source2d_s &aid_src)
{
	// reset x, y
	if (PX4_ISFINITE(aid_src.observation[0]) && PX4_ISFINITE(aid_src.observation_variance[0])
	    && PX4_ISFINITE(aid_src.observation[1]) && PX4_ISFINITE(aid_src.observation_variance[1])
	   ) {
		resetHorizontalPositionTo(Vector2f(aid_src.observation), Vector2f(aid_src.observation_variance));

		aid_src.time_last_fuse = _time_delayed_us;

		aid_src.innovation[0] = 0.f;
		aid_src.innovation[1] = 0.f;

		aid_src.innovation_filtered[0] = 0.f;
		aid_src.innovation_filtered[1] = 0.f;

		aid_src.test_ratio[0] = 0.f;
		aid_src.test_ratio[1] = 0.f;

		aid_src.test_ratio_filtered[0] = 0.f;
		aid_src.test_ratio_filtered[1] = 0.f;

		aid_src.innovation_rejected = false;
		aid_src.fused = true;

		setVelPosStatus(State::pos.idx + 0, true);
		setVelPosStatus(State::pos.idx + 1, true);

		return true;
	}

	return false;
}

bool Ekf::fuseVerticalPosition(estimator_aid_source1d_s &aid_src)
{
	// fuse z
	if (!aid_src.innovation_rejected) {
		if (fuseVelPosHeight(aid_src.innovation, aid_src.innovation_variance, State::pos.idx + 2)) {
			aid_src.fused = true;
			aid_src.time_last_fuse = _time_delayed_us;
			return true;
		}
	}

	aid_src.fused = false;
	return false;
}

bool Ekf::resetVerticalPosition(estimator_aid_source1d_s &aid_src)
{
	// reset z
	if (PX4_ISFINITE(aid_src.observation) && PX4_ISFINITE(aid_src.observation_variance)) {

		resetVerticalPositionTo(aid_src.observation, aid_src.observation_variance);

		aid_src.innovation = 0.f;
		aid_src.innovation_filtered = 0.f;

		aid_src.test_ratio = 0.f;
		aid_src.test_ratio_filtered = 0.f;

		aid_src.time_last_fuse = _time_delayed_us;

		aid_src.innovation_rejected = false;
		aid_src.fused = true;

		setVelPosStatus(State::pos.idx + 2, true);

		return true;
	}

	return false;
}

// Helper function that fuses a single velocity or position measurement
bool Ekf::fuseVelPosHeight(const float innov, const float innov_var, const int state_index)
{
	VectorState Kfusion;  // Kalman gain vector for any single observation - sequential fusion is used.

	// calculate kalman gain K = PHS, where S = 1/innovation variance
	for (int row = 0; row < State::size; row++) {
		Kfusion(row) = P(row, state_index) / innov_var;
	}

	clearInhibitedStateKalmanGains(Kfusion);

	SquareMatrixState KHP;

	for (unsigned row = 0; row < State::size; row++) {
		for (unsigned column = 0; column < State::size; column++) {
			KHP(row, column) = Kfusion(row) * P(state_index, column);
		}
	}

	const bool healthy = checkAndFixCovarianceUpdate(KHP);

	setVelPosStatus(state_index, healthy);

	if (healthy) {
		// apply the covariance corrections
		P -= KHP;

		fixCovarianceErrors(true);

		// apply the state corrections
		fuse(Kfusion, innov);

		return true;
	}

	return false;
}

void Ekf::setVelPosStatus(const int state_index, const bool healthy)
{
	switch (state_index) {
	case State::vel.idx:
		if (healthy) {
			_fault_status.flags.bad_vel_N = false;
			_time_last_hor_vel_fuse = _time_delayed_us;

		} else {
			_fault_status.flags.bad_vel_N = true;
		}

		break;

	case State::vel.idx + 1:
		if (healthy) {
			_fault_status.flags.bad_vel_E = false;
			_time_last_hor_vel_fuse = _time_delayed_us;

		} else {
			_fault_status.flags.bad_vel_E = true;
		}

		break;

	case State::vel.idx + 2:
		if (healthy) {
			_fault_status.flags.bad_vel_D = false;
			_time_last_ver_vel_fuse = _time_delayed_us;

		} else {
			_fault_status.flags.bad_vel_D = true;
		}

		break;

	case State::pos.idx:
		if (healthy) {
			_fault_status.flags.bad_pos_N = false;
			_time_last_hor_pos_fuse = _time_delayed_us;

		} else {
			_fault_status.flags.bad_pos_N = true;
		}

		break;

	case State::pos.idx + 1:
		if (healthy) {
			_fault_status.flags.bad_pos_E = false;
			_time_last_hor_pos_fuse = _time_delayed_us;

		} else {
			_fault_status.flags.bad_pos_E = true;
		}

		break;

	case State::pos.idx + 2:
		if (healthy) {
			_fault_status.flags.bad_pos_D = false;
			_time_last_hgt_fuse = _time_delayed_us;

		} else {
			_fault_status.flags.bad_pos_D = true;
		}

		break;
	}
}
