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

void Ekf::controlGnssHorizontalPositionFusion(const gnssSample &gnss_sample,
		const bool common_starting_conditions_passing, bool reset)
{
	static constexpr const char *AID_SRC_NAME = "GNSS horizontal position";

	estimator_aid_source2d_s &aid_src = _aid_src_gnss_pos;

	// if (!_pos_ref.isInitialized()) {
	// 	return;
	// }

	// determine if we should use GNSS position aiding
	bool continuing_conditions_passing = (_params.gnss_ctrl & static_cast<int32_t>(GnssCtrl::HPOS))
					     && PX4_ISFINITE(gnss_sample.lat) && PX4_ISFINITE(gnss_sample.lon)
					     && _control_status.flags.tilt_align
					     && _control_status.flags.yaw_align
					     && _pos_ref.isInitialized()
					     && _NED_origin_initialised; // TODO: _NED_origin_initialised on start?

	const bool quality_good = (gnss_sample.sacc < _params.req_sacc);

	// correct position and height for offset relative to IMU
	const Vector3f pos_offset_body = _params.gps_pos_body - _params.imu_pos_body;
	const Vector3f pos_offset_earth = _R_to_earth * pos_offset_body;
	const Vector2f position = _pos_ref.project(gnss_sample.lat, gnss_sample.lon) - pos_offset_earth.xy();

	// relax the upper observation noise limit which prevents bad GPS perturbing the position estimate
	float pos_noise = math::max(gnss_sample.hacc, _params.gps_pos_noise);

	if (!isOtherSourceOfHorizontalAidingThan(_control_status.flags.gps)) {
		// if we are not using another source of aiding, then we are reliant on the GPS
		// observations to constrain attitude errors and must limit the observation noise value.
		if (pos_noise > _params.pos_noaid_noise) {
			pos_noise = _params.pos_noaid_noise;
		}
	}

	const float pos_var = math::max(sq(pos_noise), sq(0.01f));
	const Vector2f pos_obs_var(pos_var, pos_var);

	updateAidSourceStatus(aid_src,
			      gnss_sample.time_us,                                    // sample timestamp
			      position,                                               // observation
			      pos_obs_var,                                            // observation variance
			      Vector2f(_state.pos) - position,                        // innovation
			      Vector2f(getStateVariance<State::pos>()) + pos_obs_var, // innovation variance
			      math::max(_params.gps_pos_innov_gate, 1.f));            // innovation gate

	const bool starting_conditions_passing = continuing_conditions_passing
			&& common_starting_conditions_passing
			&& quality_good;

	if (_control_status.flags.gps) {
		if (continuing_conditions_passing) {

			fuseHorizontalPosition(aid_src);

			const bool is_fusion_failing = isTimedOut(aid_src.time_last_fuse, _params.reset_timeout_max);

			if (is_fusion_failing) {

				if (quality_good) {
					// Data seems good, attempt a reset
					ECL_WARN("%s fusion timeout, resetting", AID_SRC_NAME);

					_information_events.flags.reset_vel_to_gps = true;
					resetHorizontalVelocityTo(position, pos_obs_var);
					resetAidSourceStatusZeroInnovation(aid_src);

				} else {
					// A reset did not fix the issue but all the starting checks are not passing
					// This could be a temporary issue, stop the fusion without declaring the sensor faulty
					ECL_WARN("stopping %s, fusion failing", AID_SRC_NAME);
					_control_status.flags.gps = false;
				}
			}

		} else {
			ECL_INFO("stopping %s fusion", AID_SRC_NAME);
			_control_status.flags.gps = false;
		}

	} else {
		if (starting_conditions_passing) {
			// when already using another position source position reset is not necessary
			if (!isHorizontalAidingActive()
			    || isTimedOut(_time_last_hor_pos_fuse, _params.reset_timeout_max)
			    || !_control_status_prev.flags.yaw_align
			   ) {
				// reset position
				_information_events.flags.reset_pos_to_gps = true;
				resetHorizontalPositionTo(Vector2f(aid_src.observation), Vector2f(aid_src.observation_variance));
				resetAidSourceStatusZeroInnovation(aid_src);

				_control_status.flags.gps = true;
				ECL_INFO("starting %s fusion, resetting", AID_SRC_NAME);

			} else if (fuseHorizontalPosition(aid_src)) {
				_control_status.flags.gps = true;
				ECL_INFO("starting %s fusion", AID_SRC_NAME);
			}

			if (_control_status.flags.gps) {
				_gpos_origin_eph = 0.f; // The uncertainty of the global origin is now contained in the local position uncertainty
			}
		}
	}
}
