/****************************************************************************
 *
 *   Copyright (c) 2022-2024 PX4 Development Team. All rights reserved.
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

void Ekf::controlEvVelBodyFusion(const uint64_t &timestamp_sample,
				 const Vector3f &measurement_raw, const Vector3f measurement_var,
				 const bool common_starting_conditions_passing, const bool ev_reset, const bool quality_sufficient,
				 estimator_aid_source3d_s &aid_src)
{
	static constexpr const char *AID_SRC_NAME = "EV body velocity";

	// correct velocity for offset relative to IMU
	const Vector3f pos_offset_body = _params.ev_pos_body - _params.imu_pos_body;
	const Vector3f vel_offset_body = _ang_rate_delayed_raw % pos_offset_body;

	const Vector3f measurement = measurement_raw - vel_offset_body;

	bool continuing_conditions_passing = measurement.isAllFinite()
					     && measurement_var.isAllFinite();

	updateBodyVelocityAidStatus(aid_src, timestamp_sample, measurement, measurement_var,
				    math::max(_params.ev_vel_innov_gate, 1.f));

	const bool starting_conditions_passing = common_starting_conditions_passing
			&& continuing_conditions_passing
			&& ((Vector3f(aid_src.test_ratio_filtered).max() < 0.1f) || !isHorizontalAidingActive());

	if (_control_status.flags.ev_vel) {
		if (continuing_conditions_passing) {
			if (quality_sufficient) {
				fuseBodyVelocity(aid_src);

			} else {
				aid_src.innovation_rejected = true;
			}

			const bool is_fusion_failing = isTimedOut(aid_src.time_last_fuse, _params.no_aid_timeout_max); // 1 second

			if (is_fusion_failing) {
				if ((_nb_ev_vel_reset_available > 0) && quality_sufficient) {
					// Data seems good, attempt a reset
					ECL_WARN("%s fusion failing, resetting", AID_SRC_NAME);
					_information_events.flags.reset_vel_to_vision = true;

					resetBodyVelocityTo(measurement, measurement_var);
					resetAidSourceStatusZeroInnovation(aid_src);

					if (_control_status.flags.in_air) {
						_nb_ev_vel_reset_available--;
					}

				} else {
					// A reset did not fix the issue but all the starting checks are not passing
					// This could be a temporary issue, stop the fusion without declaring the sensor faulty
					ECL_WARN("stopping %s, fusion failing", AID_SRC_NAME);
					stopEvVelFusion();
				}
			}

		} else {
			// Stop fusion but do not declare it faulty
			ECL_WARN("stopping %s fusion, continuing conditions failing", AID_SRC_NAME);
			stopEvVelFusion();
		}

	} else {
		if (starting_conditions_passing) {
			// activate fusion, only reset if necessary
			if (!isHorizontalAidingActive()) {
				ECL_INFO("starting %s fusion, resetting velocity to (%.3f, %.3f, %.3f)", AID_SRC_NAME,
					 (double)measurement(0), (double)measurement(1), (double)measurement(2));
				_information_events.flags.reset_vel_to_vision = true;

				resetBodyVelocityTo(measurement, measurement_var);
				resetAidSourceStatusZeroInnovation(aid_src);

				_control_status.flags.ev_vel = true;

			} else if (fuseBodyVelocity(aid_src)) {
				ECL_INFO("starting %s fusion", AID_SRC_NAME);
				_control_status.flags.ev_vel = true;
			}

			if (_control_status.flags.ev_vel) {
				_nb_ev_vel_reset_available = 5;
				_information_events.flags.starting_vision_vel_fusion = true;
			}
		}
	}
}
