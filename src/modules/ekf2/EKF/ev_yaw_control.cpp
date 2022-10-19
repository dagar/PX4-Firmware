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
 * @file ev_yaw_control.cpp
 * Control functions for ekf external vision yaw fusion
 */

#include "ekf.h"

void Ekf::controlEvYawFusion(const extVisionSample &ev_sample, bool starting_conditions_passing, bool ev_reset,
			     bool quality_sufficient, estimator_aid_source1d_s &aid_src)
{
	static constexpr const char *AID_SRC_NAME = "EV";

	resetEstimatorAidStatus(aid_src);
	aid_src.timestamp_sample = ev_sample.time_us;
	aid_src.observation =  getEulerYaw(ev_sample.quat);
	aid_src.observation_variance = fmaxf(ev_sample.angVar, 1.e-4f);
	aid_src.innovation = wrap_pi(getEulerYaw(_R_to_earth) - aid_src.observation);

	// determine if we should use EV yaw aiding
	bool continuing_conditions_passing = (_params.ev_ctrl & static_cast<int32_t>(EvCtrl::YAW))
					     && _control_status.flags.tilt_align
					     && !_inhibit_ev_yaw_use
					     && PX4_ISFINITE(aid_src.observation)
					     && PX4_ISFINITE(aid_src.observation_variance);

	// if GPS enabled only allow EV yaw if EV is NED
	if (_control_status.flags.gps && _control_status.flags.yaw_align
	    && (ev_sample.pos_frame != PositionFrame::LOCAL_FRAME_NED)
	   ) {
		continuing_conditions_passing = false;
	}

	starting_conditions_passing &= continuing_conditions_passing;

	if (_control_status.flags.ev_yaw) {
		aid_src.fusion_enabled = true;

		if (continuing_conditions_passing) {

			if (ev_reset && quality_sufficient) {
				// reset count changed in EV sample
				//_information_events.flags.reset_yaw_to_vision = true; // TODO
				ECL_INFO("reset yaw to EV");
				resetQuatStateYaw(aid_src.observation, aid_src.observation_variance);
				aid_src.time_last_fuse = _imu_sample_delayed.time_us;

				// TODO: wait for acceptable quality to trigger reset?

			} else if (quality_sufficient) {
				fuseYaw(aid_src.innovation, aid_src.observation_variance, aid_src);

			} else {
				aid_src.innovation_rejected = true;
			}

			const bool is_fusion_failing = isTimedOut(aid_src.time_last_fuse, _params.no_aid_timeout_max);

			if (is_fusion_failing) {
				if ((_nb_ev_yaw_reset_available > 0) && quality_sufficient) {
					// Data seems good, attempt a reset
					//_information_events.flags.reset_yaw_to_vision = true; // TODO
					ECL_INFO("reset yaw to EV");
					resetQuatStateYaw(aid_src.innovation, aid_src.observation_variance);
					aid_src.time_last_fuse = _imu_sample_delayed.time_us;

					if (_control_status.flags.in_air) {
						_nb_ev_yaw_reset_available--;
					}

				} else if (starting_conditions_passing) {
					// Data seems good, but previous reset did not fix the issue
					// something else must be wrong, declare the sensor faulty and stop the fusion
					//_control_status.flags.ev_yaw_fault = true;
					ECL_WARN("stopping %s yaw fusion, starting conditions failing", AID_SRC_NAME);
					stopEvYawFusion();

				} else {
					// A reset did not fix the issue but all the starting checks are not passing
					// This could be a temporary issue, stop the fusion without declaring the sensor faulty
					ECL_WARN("stopping %s yaw fusion, fusion failing", AID_SRC_NAME);
					stopEvYawFusion();
				}
			}

		} else {
			// Stop fusion but do not declare it faulty
			ECL_WARN("stopping %s yaw fusion, continuing conditions failing", AID_SRC_NAME);
			stopEvYawFusion();
		}

	} else {
		if (starting_conditions_passing) {
			// activate EV yaw fusion
			if (ev_sample.pos_frame == PositionFrame::LOCAL_FRAME_NED) {

				if (!_control_status.flags.yaw_align) {
					// reset yaw to EV and set yaw_align
					ECL_INFO("reset yaw to EV");
					const float yaw_new = getEulerYaw(ev_sample.quat);
					const float yaw_new_variance = fmaxf(ev_sample.angVar, sq(1.e-2f));
					resetQuatStateYaw(yaw_new, yaw_new_variance);
					aid_src.time_last_fuse = _imu_sample_delayed.time_us;

					_control_status.flags.yaw_align = true;
				}

				_control_status.flags.ev_yaw = true;
				_information_events.flags.starting_vision_yaw_fusion = true;
				ECL_INFO("starting vision yaw fusion (aligned north)");

			} else if (ev_sample.pos_frame == PositionFrame::LOCAL_FRAME_FRD) {
				if (!_control_status.flags.gps) {
					// turn on fusion of external vision yaw measurements and disable all other heading fusion
					stopMagFusion();
					stopGpsYawFusion();

					// reset yaw to EV
					ECL_INFO("reset yaw to EV");
					const float yaw_new = getEulerYaw(ev_sample.quat);
					const float yaw_new_variance = fmaxf(ev_sample.angVar, sq(1.e-2f));
					resetQuatStateYaw(yaw_new, yaw_new_variance);
					aid_src.time_last_fuse = _imu_sample_delayed.time_us;

					_information_events.flags.starting_vision_yaw_fusion = true;
					ECL_INFO("starting vision yaw fusion");

					_control_status.flags.yaw_align = false;
					_control_status.flags.ev_yaw = true;
				}
			}

			if (_control_status.flags.ev_yaw) {
				_nb_ev_yaw_reset_available = 3;
			}
		}
	}
}

void Ekf::stopEvYawFusion()
{
	if (_control_status.flags.ev_yaw) {
		resetEstimatorAidStatus(_aid_src_ev_yaw);
		_control_status.flags.ev_yaw = false;
	}
}
