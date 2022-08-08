/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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
 * @file fake_height_control.cpp
 * Control functions for ekf fake height fusion
 */

#include "ekf.h"

void Ekf::controlFakeHgtFusion()
{
	auto &fake_pos = _aid_src_fake_pos;

	// If we aren't doing any aiding, fake position measurements at the last known vertical position to constrain drift
	const bool fake_hgt_data_ready = isTimedOut(fake_pos.time_last_fuse[2], (uint64_t)2e5); // Fuse fake height at a limited rate

	if (fake_hgt_data_ready) {
		const bool continuing_conditions_passing = !isVerticalAidingActive();
		const bool starting_conditions_passing = continuing_conditions_passing;

		if (_using_synthetic_hgt) {
			if (continuing_conditions_passing) {
				fuseFakeHgt();

				const bool is_fusion_failing = isTimedOut(fake_pos.time_last_fuse[2], (uint64_t)4e5);

				if (is_fusion_failing) {
					resetFakeHgtFusion();
				}

			} else {
				stopFakeHgtFusion();
			}

		} else {
			if (starting_conditions_passing) {
				startFakeHgtFusion();
			}
		}
	}
}

void Ekf::startFakeHgtFusion()
{
	if (!_using_synthetic_hgt) {
		ECL_INFO("start fake position fusion");
		_using_synthetic_hgt = true;
		resetFakeHgtFusion();
	}
}

void Ekf::resetFakeHgtFusion()
{
	ECL_INFO("reset fake height fusion");
	_last_known_pos(2) = _state.pos(2);

	resetVerticalVelocityToZero();
	resetHeightToLastKnown();

	_aid_src_fake_pos.time_last_fuse[2] = _time_last_imu;
}

void Ekf::resetHeightToLastKnown()
{
	_information_events.flags.reset_pos_to_last_known = true;
	ECL_INFO("reset height to last known");
	resetVerticalPositionTo(_last_known_pos(2));
	P.uncorrelateCovarianceSetVariance<1>(9, sq(_params.pos_noaid_noise));
}

void Ekf::stopFakeHgtFusion()
{
	if (_using_synthetic_hgt) {
		ECL_INFO("stop fake height fusion");
		_using_synthetic_hgt = false;

		resetEstimatorAidStatus(_aid_src_fake_pos);
	}
}

void Ekf::fuseFakeHgt()
{
	const float obs_var = sq(0.5f);

	const float innov_gate = 3.f;

	auto &fake_pos = _aid_src_fake_pos;

	fake_pos.observation[2] = _last_known_pos(2);
	fake_pos.observation_variance[2] = obs_var;

	fake_pos.innovation[2] = _state.pos(2) - _last_known_pos(2);
	fake_pos.innovation_variance[2] = P(9, 9) + obs_var;

	setEstimatorAidStatusTestRatio(fake_pos, innov_gate);

	// always protect against extreme values that could result in a NaN
	fake_pos.fusion_enabled[2] = fake_pos.test_ratio[2] < sq(100.0f / innov_gate);

	// fuse
	if (fake_pos.fusion_enabled[2] && !fake_pos.innovation_rejected[2]) {
		if (fuseVelPosHeight(fake_pos.innovation[2], fake_pos.innovation_variance[2], 5)) {
			fake_pos.fused[2] = true;
			fake_pos.time_last_fuse[2] = _time_last_imu;
		}
	}

	fake_pos.timestamp_sample = _time_last_imu;
}
