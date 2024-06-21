/****************************************************************************
 *
 *   Copyright (c) 2021-2024 PX4 Development Team. All rights reserved.
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

void Ekf::controlGNSSHorizontalVelocityFusion(const gnssSample &gnss_sample,
		const bool common_starting_conditions_passing, bool reset)
{
	static constexpr const char *AID_SRC_NAME = "GNSS horizontal velocity";

	estimator_aid_source2d_s &aid_src = _aid_src_gnss_vel_xy;

	// determine if we should use GNSS horizontal velocity aiding
	bool continuing_conditions_passing = (_params.gnss_ctrl & static_cast<int32_t>(GnssCtrl::VEL))
					     && PX4_ISFINITE(gnss_sample.vel(0)) && PX4_ISFINITE(gnss_sample.vel(1))
					     && _control_status.flags.tilt_align
					     && _control_status.flags.yaw_align;

	const bool quality_good = (gnss_sample.sacc < _params.req_sacc);

	// correct velocity for offset relative to IMU
	const Vector3f pos_offset_body = _params.gps_pos_body - _params.imu_pos_body;

	const Vector3f vel_offset_body = _ang_rate_delayed_raw % pos_offset_body;
	const Vector3f vel_offset_earth = _R_to_earth * vel_offset_body;
	const Vector2f velocity(gnss_sample.vel - vel_offset_earth);

	const float vel_var = sq(math::max(gnss_sample.sacc, _params.gps_vel_noise, 0.01f));
	const Vector2f vel_obs_var(vel_var, vel_var);

	updateAidSourceStatus(aid_src,
			      gnss_sample.time_us,                            // sample timestamp
			      velocity,                                       // observation
			      vel_obs_var,                                    // observation variance
			      Vector2f(_state.vel) - velocity,                // innovation
			      Vector2f(getVelocityVariance()) + vel_obs_var,  // innovation variance
			      math::max(_params.gps_vel_innov_gate, 1.f));    // innovation gate

	const bool starting_conditions_passing = continuing_conditions_passing
			&& common_starting_conditions_passing
			&& quality_good;

	if (_control_status.flags.gnss_vel_xy) {
		if (continuing_conditions_passing) {

			fuseHorizontalVelocity(aid_src);

			const bool is_fusion_failing = isTimedOut(aid_src.time_last_fuse, _params.reset_timeout_max);

			// TODO: reset ?

			if (is_fusion_failing) {

				if (quality_good) {
					// Data seems good, attempt a reset
					ECL_WARN("%s fusion timeout, resetting", AID_SRC_NAME);

					_information_events.flags.reset_vel_to_gps = true;
					resetHorizontalVelocityTo(velocity, vel_obs_var);
					resetAidSourceStatusZeroInnovation(aid_src);

				} else {
					// A reset did not fix the issue but all the starting checks are not passing
					// This could be a temporary issue, stop the fusion without declaring the sensor faulty
					ECL_WARN("stopping %s, fusion failing", AID_SRC_NAME);
					_control_status.flags.gnss_vel_xy = false;
				}
			}

		} else {
			ECL_INFO("continuing conditions failing, stopping %s", AID_SRC_NAME);
			_control_status.flags.gnss_vel_xy = false;
		}

	} else {
		if (starting_conditions_passing) {
			// when already using another velocity source velocity reset is not necessary
			if (!isHorizontalAidingActive()
			    || isTimedOut(_time_last_hor_vel_fuse, _params.reset_timeout_max)
			    || !_control_status_prev.flags.yaw_align
			   ) {
				// reset velocity
				_information_events.flags.reset_vel_to_gps = true;
				resetHorizontalVelocityTo(velocity, vel_obs_var);
				resetAidSourceStatusZeroInnovation(aid_src);

				_control_status.flags.gnss_vel_xy = true;
				ECL_INFO("starting %s fusion, resetting", AID_SRC_NAME);

			} else if (fuseHorizontalVelocity(aid_src)) {
				_control_status.flags.gnss_vel_xy = true;
				ECL_INFO("starting %s fusion", AID_SRC_NAME);
			}
		}
	}
}
