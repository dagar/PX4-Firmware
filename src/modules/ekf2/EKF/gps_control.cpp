/****************************************************************************
 *
 *   Copyright (c) 2021-2022 PX4 Development Team. All rights reserved.
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
 * @file gps_control.cpp
 * Control functions for ekf GNSS fusion
 */

#include "ekf.h"
#include <mathlib/mathlib.h>

void Ekf::controlGpsFusion()
{
	// Check for new GPS data that has fallen behind the fusion time horizon
	if (_gps_data_ready) {

		const gpsSample &gps_sample{_gps_sample_delayed};

		updateGpsYaw(gps_sample);

		const bool gps_checks_passing = isTimedOut(_last_gps_fail_us, (uint64_t)5e6);
		const bool gps_checks_failing = isTimedOut(_last_gps_pass_us, (uint64_t)5e6);

		controlGpsYawFusion(gps_sample, gps_checks_passing, gps_checks_failing);

		// GNSS velocity
		const Vector3f velocity{gps_sample.vel};
		const float vel_axis_obs_var = sq(math::max(gps_sample.sacc, _params.gps_vel_noise));
		const Vector3f vel_obs_var{vel_axis_obs_var, vel_axis_obs_var, vel_axis_obs_var * sq(1.5f)};
		updateVelocityAidSrcStatus(gps_sample.time_us,
					   velocity,                                                   // observation
					   vel_obs_var,                                                // observation variance
					   math::max(_params.gps_vel_innov_gate, 1.f),                 // innovation gate
					   _aid_src_gnss_vel);
		_aid_src_gnss_vel.fusion_enabled = (_params.gnss_ctrl & GnssCtrl::VEL);

		// GNSS position
		const Vector2f position{gps_sample.pos};
		// relax the upper observation noise limit which prevents bad GPS perturbing the position estimate
		float pos_noise = math::max(gps_sample.hacc, _params.gps_pos_noise);

		if (!isOtherSourceOfHorizontalAidingThan(_control_status.flags.gps)) {
			// if we are not using another source of aiding, then we are reliant on the GPS
			// observations to constrain attitude errors and must limit the observation noise value.
			if (pos_noise > _params.pos_noaid_noise) {
				pos_noise = _params.pos_noaid_noise;
			}
		}

		const float pos_axis_obs_var = sq(pos_noise);
		const Vector2f pos_obs_var{pos_axis_obs_var, pos_axis_obs_var};
		updateHorizontalPositionAidSrcStatus(gps_sample.time_us,
						     gps_sample.pos,                             // observation
						     pos_obs_var,                                // observation variance
						     math::max(_params.gps_pos_innov_gate, 1.f), // innovation gate
						     _aid_src_gnss_pos);
		_aid_src_gnss_pos.fusion_enabled = (_params.gnss_ctrl & GnssCtrl::HPOS);

		// update GSF yaw estimator velocity (basic sanity check on GNSS velocity data)
		if ((gps_sample.sacc > 0.f) && (gps_sample.sacc <= _params.req_sacc)) {
			_yawEstimator.setVelocity(velocity.xy(), math::max(gps_sample.sacc, _params.gps_vel_noise));
		}

		// allow GPS to perform yaw align or in flight mag alignment
		if (_control_status.flags.tilt_align
		    && gps_checks_passing && !gps_checks_failing) {

			if (!_control_status.flags.yaw_align
			    || (_control_status.flags.mag_hdg && !_control_status.flags.mag_aligned_in_flight)
			   ) {
				if (resetYawToEKFGSF()) {
					_information_events.flags.yaw_aligned_to_imu_gps = true;
					ECL_INFO("Yaw aligned using IMU and GPS");

				} else if (resetYawToGps(gps_sample.yaw)) {
					ECL_INFO("Yaw aligned using GPS yaw");
				}
			}
		}

		// Determine if we should use GPS aiding for velocity and horizontal position
		// To start using GPS we need angular alignment completed, the local NED origin set and GPS data that has not failed checks recently
		const bool mandatory_conditions_passing = ((_params.gnss_ctrl & GnssCtrl::HPOS) || (_params.gnss_ctrl & GnssCtrl::VEL))
				&& _control_status.flags.tilt_align
				&& _control_status.flags.yaw_align
				&& _NED_origin_initialised;

		const bool continuing_conditions_passing = mandatory_conditions_passing && !gps_checks_failing;

		const bool starting_conditions_passing = continuing_conditions_passing
				&& isNewestSampleRecent(_time_last_gps_buffer_push, 2 * GPS_MAX_INTERVAL)
				&& _gps_checks_passed
				&& gps_checks_passing
				&& !gps_checks_failing;

		if (_control_status.flags.gps) {
			if (mandatory_conditions_passing) {
				if (continuing_conditions_passing
				    || !isOtherSourceOfHorizontalAidingThan(_control_status.flags.gps)) {

					_aid_src_gnss_vel.fusion_enabled = (_params.gnss_ctrl & GnssCtrl::VEL);
					fuseVelocity(_aid_src_gnss_vel);

					_aid_src_gnss_pos.fusion_enabled = (_params.gnss_ctrl & GnssCtrl::HPOS);
					fuseHorizontalPosition(_aid_src_gnss_pos);

					if (shouldResetGpsFusion()) {
						const bool was_gps_signal_lost = isTimedOut(_time_prev_gps_us, 1'000'000);

						/* A reset is not performed when getting GPS back after a significant period of no data
						 * because the timeout could have been caused by bad GPS.
						 * The total number of resets allowed per boot cycle is limited.
						 */
						if (isYawFailure()
						    && _control_status.flags.in_air
						    && !was_gps_signal_lost
						    && _ekfgsf_yaw_reset_count < _params.EKFGSF_reset_count_limit
						    && resetYawToEKFGSF()) {
							// The minimum time interval between resets to the EKF-GSF estimate is limited to allow the EKF-GSF time
							// to improve its estimate if the previous reset was not successful.
							ECL_WARN("GPS emergency yaw reset");
							_ekfgsf_yaw_reset_count++;

							if (_control_status.flags.mag_hdg || _control_status.flags.mag_3D) {
								// stop using the magnetometer in the main EKF otherwise it's fusion could drag the yaw around
								// and cause another navigation failure
								_control_status.flags.mag_fault = true;
								_warning_events.flags.emergency_yaw_reset_mag_stopped = true;
							}

							if (_control_status.flags.gps_yaw) {
								_control_status.flags.gps_yaw_fault = true;
								_warning_events.flags.emergency_yaw_reset_gps_yaw_stopped = true;
							}

							if (_control_status.flags.ev_yaw) {
								_control_status.flags.ev_yaw_fault = true;
								_warning_events.flags.emergency_yaw_reset_ev_yaw_stopped = true;
							}
						}

						ECL_WARN("GPS fusion timeout - resetting");

						// reset velocity to GPS
						ECL_INFO("reset velocity to GPS");
						_information_events.flags.reset_vel_to_gps = true;
						resetVelocityTo(gps_sample.vel, vel_obs_var);
						_aid_src_gnss_vel.time_last_fuse = _imu_sample_delayed.time_us;

						// reset horizontal position to GPS
						ECL_INFO("reset position to GPS");
						_information_events.flags.reset_pos_to_gps = true;
						resetHorizontalPositionTo(gps_sample.pos, pos_obs_var);
						_aid_src_gnss_pos.time_last_fuse = _imu_sample_delayed.time_us;
					}

				} else {
					stopGpsFusion();
					_warning_events.flags.gps_quality_poor = true;
					ECL_WARN("GPS quality poor - stopping use");
				}

			} else { // mandatory conditions are not passing
				stopGpsFusion();
			}

		} else {
			if (starting_conditions_passing) {

				// reset heading proactively if yaw estimate is available and there's a large error
				if (isYawEmergencyEstimateAvailable() && isYawFailure()) {
					resetYawToEKFGSF();
				}

				// reset horizontal position to GPS
				_information_events.flags.reset_pos_to_gps = true;
				ECL_INFO("reset position to GPS");
				resetHorizontalPositionTo(gps_sample.pos, pos_obs_var);
				_aid_src_gnss_pos.time_last_fuse = _imu_sample_delayed.time_us;

				// reset velocity to GPS if not currently using another velocity source
				if (!isHorizontalAidingActive()) {
					_information_events.flags.reset_vel_to_gps = true;
					ECL_INFO("reset velocity to GPS");
					resetVelocityTo(gps_sample.vel, vel_obs_var);
					_aid_src_gnss_vel.time_last_fuse = _imu_sample_delayed.time_us;
				}

				_information_events.flags.starting_gps_fusion = true;
				ECL_INFO("starting GPS fusion");
				_control_status.flags.gps = true;
			}
		}

	} else if (_control_status.flags.gps && isTimedOut(_time_prev_gps_us, (uint64_t)10e6)) {
		stopGpsFusion();
		_warning_events.flags.gps_data_stopped = true;
		ECL_WARN("GPS data stopped");

	}  else if (_control_status.flags.gps && isTimedOut(_time_prev_gps_us, (uint64_t)1e6)
		    && isOtherSourceOfHorizontalAidingThan(_control_status.flags.gps)) {

		// Handle the case where we are fusing another position source along GPS,
		// stop waiting for GPS after 1 s of lost signal
		stopGpsFusion();
		_warning_events.flags.gps_data_stopped_using_alternate = true;
		ECL_WARN("GPS data stopped, using only EV, OF or air data");
	}
}

bool Ekf::shouldResetGpsFusion() const
{
	/* We are relying on aiding to constrain drift so after a specified time
	 * with no aiding we need to do something
	 */
	const bool is_reset_required = hasHorizontalAidingTimedOut()
				       || isTimedOut(_time_last_hor_pos_fuse, 2 * _params.reset_timeout_max);

	/* Logic controlling the reset of navigation filter yaw to the EKF-GSF estimate to recover from loss of
	 * navigation casued by a bad yaw estimate.

	 * A rapid reset to the EKF-GSF estimate is performed after a recent takeoff if horizontal velocity
	 * innovation checks fail. This enables recovery from a bad yaw estimate. After 30 seconds from takeoff,
	 * different test criteria are used that take longer to trigger and reduce false positives. A reset is
	 * not performed if the fault condition was present before flight to prevent triggering due to GPS glitches
	 * or other sensor errors.
	 */
	const bool is_recent_takeoff_nav_failure = _control_status.flags.in_air
			&& isRecent(_time_last_on_ground_us, 30'000'000)
			&& isTimedOut(_time_last_hor_vel_fuse, _params.EKFGSF_reset_delay)
			&& (_time_last_hor_vel_fuse > _time_last_on_ground_us);

	const bool is_inflight_nav_failure = _control_status.flags.in_air
					     && isTimedOut(_time_last_hor_vel_fuse, _params.reset_timeout_max)
					     && isTimedOut(_time_last_hor_pos_fuse, _params.reset_timeout_max)
					     && (_time_last_hor_vel_fuse > _time_last_on_ground_us)
					     && (_time_last_hor_pos_fuse > _time_last_on_ground_us);

	return (is_reset_required || is_recent_takeoff_nav_failure || is_inflight_nav_failure);
}

bool Ekf::isYawFailure() const
{
	if (!isYawEmergencyEstimateAvailable()) {
		return false;
	}

	const float euler_yaw = getEulerYaw(_R_to_earth);
	const float yaw_error = wrap_pi(euler_yaw - _yawEstimator.getYaw());

	return fabsf(yaw_error) > math::radians(25.f);
}
