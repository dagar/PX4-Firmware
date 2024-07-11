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

#include "ekf.h"

#include <ekf_derivation/generated/compute_body_vel_xyz_innov_var.h>
#include <ekf_derivation/generated/compute_body_vel_hx.h>
#include <ekf_derivation/generated/compute_body_vel_hy.h>
#include <ekf_derivation/generated/compute_body_vel_hz.h>

void Ekf::updateBodyVelocityAidStatus(estimator_aid_source3d_s &aid_src, const uint64_t &time_us,
				      const Vector3f &measurement, const Vector3f &measurement_variance, const float innovation_gate) const
{
	const Vector3f innovation = (_R_to_earth.transpose() * _state.vel) - measurement;

	Vector3f innovation_variance;
	sym::ComputeBodyVelXyzInnovVar(_state.vector(), P, measurement_variance,
				       &innovation_variance);

	updateAidSourceStatus(aid_src,
			      time_us,              // sample timestamp
			      measurement,          // measurement
			      measurement_variance, // measurement variance
			      innovation,           // innovation
			      innovation_variance,  // innovation variance
			      innovation_gate);     // innovation gate
}

bool Ekf::fuseHorizontalVelocity(estimator_aid_source2d_s &aid_src)
{
	// vx, vy
	if (!aid_src.innovation_rejected
	    && fuseDirectStateMeasurement(aid_src.innovation[0], aid_src.innovation_variance[0], aid_src.observation_variance[0],
					  State::vel.idx + 0)
	    && fuseDirectStateMeasurement(aid_src.innovation[1], aid_src.innovation_variance[1], aid_src.observation_variance[1],
					  State::vel.idx + 1)
	   ) {
		aid_src.fused = true;
		aid_src.time_last_fuse = _time_delayed_us;

		_time_last_hor_vel_fuse = _time_delayed_us;

	} else {
		aid_src.fused = false;
	}

	return aid_src.fused;
}

bool Ekf::fuseVelocity(estimator_aid_source3d_s &aid_src)
{
	// vx, vy, vz
	if (!aid_src.innovation_rejected
	    && fuseDirectStateMeasurement(aid_src.innovation[0], aid_src.innovation_variance[0], aid_src.observation_variance[0],
					  State::vel.idx + 0)
	    && fuseDirectStateMeasurement(aid_src.innovation[1], aid_src.innovation_variance[1], aid_src.observation_variance[1],
					  State::vel.idx + 1)
	    && fuseDirectStateMeasurement(aid_src.innovation[2], aid_src.innovation_variance[2], aid_src.observation_variance[2],
					  State::vel.idx + 2)
	   ) {
		aid_src.fused = true;
		aid_src.time_last_fuse = _time_delayed_us;

		_time_last_hor_vel_fuse = _time_delayed_us;
		_time_last_ver_vel_fuse = _time_delayed_us;

	} else {
		aid_src.fused = false;
	}

	return aid_src.fused;
}

bool Ekf::fuseBodyVelocity(estimator_aid_source3d_s &aid_src)
{
	if (aid_src.innovation_rejected) {
		aid_src.fused = false;
		return false;
	}

	bool fused[3] {false, false, false};
	const auto state_vector = _state.vector();

	for (uint8_t index = 0; index <= 2; index++) {
		VectorState H;

		if (index == 0) {
			sym::ComputeBodyVelHx(state_vector, &H);

		} else if (index == 1) {
			sym::ComputeBodyVelHy(state_vector, &H);

			// recalculate innovation using the updated state
			aid_src.innovation[1] = (_R_to_earth.transpose() * _state.vel - Vector3f(aid_src.observation))(1, 0);

			// recalculate innovation variance because state covariances have changed due to previous fusion (linearise using the same initial state for all axes)
			aid_src.innovation_variance[1] = (H.T() * P * H)(0, 0) + aid_src.observation_variance[1];

		} else if (index == 2) {
			sym::ComputeBodyVelHz(state_vector, &H);

			// recalculate innovation using the updated state
			aid_src.innovation[2] = (_R_to_earth.transpose() * _state.vel - Vector3f(aid_src.observation))(2, 0);

			// recalculate innovation variance because state covariances have changed due to previous fusion (linearise using the same initial state for all axes)
			aid_src.innovation_variance[2] = (H.T() * P * H)(0, 0) + aid_src.observation_variance[2];
		}

		if (aid_src.innovation_variance[index] < aid_src.observation_variance[index]) {
			ECL_ERR("body velocity numerical error covariance reset");
			resetQuatCov();
			P.uncorrelateCovarianceSetVariance<State::vel.dof>(State::vel.idx, Vector3f(aid_src.observation_variance));
			return false;
		}

		VectorState Kfusion = P * H / aid_src.innovation_variance[index];
		fused[index] = measurementUpdate(Kfusion, H, aid_src.observation_variance[index], aid_src.innovation[index]);
	}

	if (fused[0] && fused[1] && fused[2]) {
		aid_src.fused = true;
		aid_src.time_last_fuse = _time_delayed_us;

		_time_last_hor_vel_fuse = _time_delayed_us;
		_time_last_ver_vel_fuse = _time_delayed_us;
	}

	return aid_src.fused;
}

void Ekf::resetHorizontalVelocityTo(const Vector2f &new_horz_vel, const Vector2f &new_horz_vel_var)
{
	const Vector2f delta_horz_vel = new_horz_vel - Vector2f(_state.vel);
	_state.vel.xy() = new_horz_vel;

	if (PX4_ISFINITE(new_horz_vel_var(0))) {
		P.uncorrelateCovarianceSetVariance<1>(State::vel.idx, math::max(sq(0.01f), new_horz_vel_var(0)));
	}

	if (PX4_ISFINITE(new_horz_vel_var(1))) {
		P.uncorrelateCovarianceSetVariance<1>(State::vel.idx + 1, math::max(sq(0.01f), new_horz_vel_var(1)));
	}

	_output_predictor.resetHorizontalVelocityTo(delta_horz_vel);

	// record the state change
	if (_state_reset_status.reset_count.velNE == _state_reset_count_prev.velNE) {
		_state_reset_status.velNE_change = delta_horz_vel;

	} else {
		// there's already a reset this update, accumulate total delta
		_state_reset_status.velNE_change += delta_horz_vel;
	}

	_state_reset_status.reset_count.velNE++;

	// Reset the timout timer
	_time_last_hor_vel_fuse = _time_delayed_us;
}

void Ekf::resetVerticalVelocityTo(float new_vert_vel, float new_vert_vel_var)
{
	const float delta_vert_vel = new_vert_vel - _state.vel(2);
	_state.vel(2) = new_vert_vel;

	if (PX4_ISFINITE(new_vert_vel_var)) {
		P.uncorrelateCovarianceSetVariance<1>(State::vel.idx + 2, math::max(sq(0.01f), new_vert_vel_var));
	}

	_output_predictor.resetVerticalVelocityTo(delta_vert_vel);

	// record the state change
	if (_state_reset_status.reset_count.velD == _state_reset_count_prev.velD) {
		_state_reset_status.velD_change = delta_vert_vel;

	} else {
		// there's already a reset this update, accumulate total delta
		_state_reset_status.velD_change += delta_vert_vel;
	}

	_state_reset_status.reset_count.velD++;

	// Reset the timout timer
	_time_last_ver_vel_fuse = _time_delayed_us;
}

void Ekf::resetHorizontalVelocityToZero()
{
	ECL_INFO("reset velocity to zero");
	_information_events.flags.reset_vel_to_zero = true;

	// Used when falling back to non-aiding mode of operation
	resetHorizontalVelocityTo(Vector2f{0.f, 0.f}, 25.f);
}

void Ekf::resetVerticalVelocityToZero()
{
	// we don't know what the vertical velocity is, so set it to zero
	// Set the variance to a value large enough to allow the state to converge quickly
	// that does not destabilise the filter
	resetVerticalVelocityTo(0.0f, 10.f);
}

void Ekf::resetVelocityTo(const Vector3f &new_vel, const Vector3f &new_vel_var)
{
	resetHorizontalVelocityTo(Vector2f(new_vel), Vector2f(new_vel_var(0), new_vel_var(1)));
	resetVerticalVelocityTo(new_vel(2), new_vel_var(2));
}

void Ekf::resetBodyVelocityTo(const Vector3f &new_vel, const Vector3f &new_vel_var)
{
	// rotate body velocity into EKF frame
	const Vector3f velocity = _R_to_earth * new_vel;

	const matrix::SquareMatrix<float, 3> R_cov = _R_to_earth * matrix::diag(new_vel_var) * _R_to_earth.transpose();
	const Vector3f velocity_var = R_cov.diag();

	resetVelocityTo(velocity, velocity_var);
}
