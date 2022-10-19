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
 * @file ev_vel_control.cpp
 * Control functions for ekf external vision velocity fusion
 */

#include "ekf.h"

void Ekf::controlEvVelFusion(const extVisionSample &ev_sample, bool starting_conditions_passing, bool ev_reset,
			     bool quality_sufficient, estimator_aid_source3d_s &aid_src)
{
	static constexpr const char *AID_SRC_NAME = "EV";

	// determine if we should use EV velocity aiding
	bool continuing_conditions_passing = (_params.ev_ctrl & static_cast<int32_t>(EvCtrl::VEL))
					     && _control_status.flags.tilt_align
					     && PX4_ISFINITE(ev_sample.vel(0))
					     && PX4_ISFINITE(ev_sample.vel(1))
					     && PX4_ISFINITE(ev_sample.vel(2));

	// correct velocity for offset relative to IMU
	const Vector3f pos_offset_body = _params.ev_pos_body - _params.imu_pos_body;
	const Vector3f vel_offset_body = _ang_rate_delayed_raw % pos_offset_body;
	const Vector3f vel_offset_earth = _R_to_earth * vel_offset_body;

	// rotate measurement into correct earth frame if required
	Vector3f vel{NAN, NAN, NAN};
	Matrix3f vel_cov{};

	switch (ev_sample.vel_frame) {
	case VelocityFrame::LOCAL_FRAME_NED:
		if (_control_status.flags.yaw_align) {
			vel = ev_sample.vel - vel_offset_earth;
			vel_cov = matrix::diag(ev_sample.velVar);

		} else {
			continuing_conditions_passing = false;
		}

		break;

	case VelocityFrame::LOCAL_FRAME_FRD:
		if (_control_status.flags.ev_yaw) {
			// using EV frame
			vel = ev_sample.vel - vel_offset_earth;
			vel_cov = matrix::diag(ev_sample.velVar);

		} else {
			// rotate EV to the EKF reference frame
			const Quatf q_error((_state.quat_nominal * ev_sample.quat.inversed()).normalized());
			const Dcmf R_ev_to_ekf = Dcmf(q_error);

			vel = R_ev_to_ekf * ev_sample.vel - vel_offset_earth;
			vel_cov = R_ev_to_ekf * matrix::diag(ev_sample.velVar) * R_ev_to_ekf.transpose();
		}

		break;

	case VelocityFrame::BODY_FRAME_FRD:
		vel = _R_to_earth * (ev_sample.vel - vel_offset_body);
		vel_cov = _R_to_earth * matrix::diag(ev_sample.velVar) * _R_to_earth.transpose();
		break;

	default:
		continuing_conditions_passing = false;
		break;
	}

	const Vector3f obs_var {
		math::max(vel_cov(0, 0), sq(0.01f)),
		math::max(vel_cov(1, 1), sq(0.01f)),
		math::max(vel_cov(2, 2), sq(0.01f))
	};

	updateVelocityAidSrcStatus(ev_sample.time_us, vel, obs_var, fmaxf(_params.ev_vel_innov_gate, 1.f), aid_src);

	if (!vel.isAllFinite() || !vel_cov.isAllFinite()) {
		continuing_conditions_passing = false;
	}

	starting_conditions_passing &= continuing_conditions_passing
				       && ((Vector3f(aid_src.test_ratio).max() < 0.1f) || !isHorizontalAidingActive());

	if (_control_status.flags.ev_vel) {
		aid_src.fusion_enabled = true;

		if (continuing_conditions_passing) {

			if (ev_reset && quality_sufficient && isOnlyActiveSourceOfHorizontalAiding(_control_status.flags.ev_vel)) {
				_information_events.flags.reset_vel_to_vision = true;
				ECL_INFO("reset to vision velocity");
				resetVelocityTo(vel, obs_var);
				aid_src.time_last_fuse = _imu_sample_delayed.time_us;

			} else if (quality_sufficient) {
				fuseVelocity(aid_src);

			} else {
				aid_src.innovation_rejected = true;
			}

			const bool is_fusion_failing = isTimedOut(aid_src.time_last_fuse, _params.no_aid_timeout_max); // 1 second

			if (is_fusion_failing) {

				if ((_nb_ev_vel_reset_available > 0) && quality_sufficient) {
					// Data seems good, attempt a reset
					_information_events.flags.reset_vel_to_vision = true;
					ECL_INFO("reset to vision velocity");
					resetVelocityTo(vel, obs_var);
					aid_src.time_last_fuse = _imu_sample_delayed.time_us;

					if (_control_status.flags.in_air) {
						_nb_ev_vel_reset_available--;
					}

				} else if (starting_conditions_passing) {
					// Data seems good, but previous reset did not fix the issue
					// something else must be wrong, declare the sensor faulty and stop the fusion
					//_control_status.flags.ev_vel_fault = true;
					ECL_WARN("stopping %s velocity fusion, starting conditions failing", AID_SRC_NAME);
					stopEvVelFusion();

				} else {
					// A reset did not fix the issue but all the starting checks are not passing
					// This could be a temporary issue, stop the fusion without declaring the sensor faulty
					ECL_WARN("stopping %s velocity fusion, fusion failing", AID_SRC_NAME);
					stopEvVelFusion();
				}
			}

		} else {
			// Stop fusion but do not declare it faulty
			ECL_WARN("stopping %s velocity fusion, continuing conditions failing", AID_SRC_NAME);
			stopEvVelFusion();
		}

	} else {
		if (starting_conditions_passing) {
			// activate EV velocity fusion
			if (!isHorizontalAidingActive()) {
				// reset if necessary
				_information_events.flags.reset_vel_to_vision = true;
				ECL_INFO("reset to vision velocity");
				resetVelocityTo(vel, obs_var);
			}

			aid_src.time_last_fuse = _imu_sample_delayed.time_us;

			_nb_ev_vel_reset_available = 5;
			_information_events.flags.starting_vision_vel_fusion = true;
			ECL_INFO("starting vision velocity fusion");
			_control_status.flags.ev_vel = true;
		}
	}
}

void Ekf::stopEvVelFusion()
{
	if (_control_status.flags.ev_vel) {
		resetEstimatorAidStatus(_aid_src_ev_vel);
		_control_status.flags.ev_vel = false;
	}
}
