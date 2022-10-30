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

#include <lib/mathlib/mathlib.h>
#include <lib/world_magnetic_model/geo_mag_declination.h>

void Ekf::controlGpsFusion()
{
	_gps_hgt_b_est.predict(_dt_ekf_avg);

	// Check for new GPS data that has fallen behind the fusion time horizon
	if (_gps_data_ready) {

		const gpsSample &gps_sample{_gps_sample_delayed};

		const bool gps_checks_passed = gps_is_good(gps_sample);

		if (!_NED_origin_initialised && gps_checks_passed) {

			if (setGnssPositionOrigin(gps_sample) && setGnssAltitudeOrigin(gps_sample)) {
				updateWorldMagneticField(gps_sample);
				updateEarthRateNED(gps_sample);

				_NED_origin_initialised = true;
				_information_events.flags.gps_checks_passed = true;
				ECL_INFO("GPS checks passed");
			}

		} else {
			if (!PX4_ISFINITE(_mag_declination_gps) || (!_gps_checks_passed && gps_checks_passed)) {
				updateWorldMagneticField(gps_sample);
			}

			// calculate the earth rotation vector
			updateEarthRateNED(gps_sample);
		}

		_gps_checks_passed = gps_checks_passed;

		const bool gps_checks_passing = isTimedOut(_last_gps_fail_us, (uint64_t)5e6);
		const bool gps_checks_failing = isTimedOut(_last_gps_pass_us, (uint64_t)5e6);

		// GNSS velocity

		// correct velocity for offset relative to IMU
		const Vector3f pos_offset_body = _params.gps_pos_body - _params.imu_pos_body;
		const Vector3f vel_offset_body = _ang_rate_delayed_raw % pos_offset_body;
		const Vector3f vel_offset_earth = _R_to_earth * vel_offset_body;

		const Vector3f velocity = gps_sample.velocity - vel_offset_earth;
		const float vel_var = sq(math::max(gps_sample.speed_accuracy, _params.gps_vel_noise));
		const Vector3f vel_obs_var(vel_var, vel_var, vel_var * sq(1.5f));
		updateVelocityAidSrcStatus(gps_sample.time_us,
					   velocity,                                             // observation
					   vel_obs_var,                                                // observation variance
					   math::max(_params.gps_vel_innov_gate, 1.f),                 // innovation gate
					   _aid_src_gnss_vel);
		_aid_src_gnss_vel.fusion_enabled = (_params.gnss_ctrl & GnssCtrl::VEL)
						   && (gps_sample.fix_type >= 3)
						   && gps_sample.velocity.isAllFinite();

		// GNSS position
		// relax the upper observation noise limit which prevents bad GPS perturbing the position estimate
		float pos_noise = math::max(gps_sample.horizontal_accuracy, _params.gps_pos_noise);

		if (!isOtherSourceOfHorizontalAidingThan(_control_status.flags.gps)) {
			// if we are not using another source of aiding, then we are reliant on the GPS
			// observations to constrain attitude errors and must limit the observation noise value.
			if (pos_noise > _params.pos_noaid_noise) {
				pos_noise = _params.pos_noaid_noise;
			}
		}

		const float pos_var = sq(pos_noise);
		const Vector2f pos_obs_var(pos_var, pos_var);

		// correct position for offset relative to IMU
		const Vector3f pos_offset_earth = _R_to_earth * pos_offset_body;
		Vector2f position{0.f, 0.f};

		if (_pos_ref.isInitialized() && (gps_sample.fix_type >= 2)) {
			position = _pos_ref.project(gps_sample.latitude, gps_sample.longitude) - pos_offset_earth.xy();
		}

		updateHorizontalPositionAidSrcStatus(gps_sample.time_us,
						     position,                                   // observation
						     pos_obs_var,                                // observation variance
						     math::max(_params.gps_pos_innov_gate, 1.f), // innovation gate
						     _aid_src_gnss_pos);
		_aid_src_gnss_pos.fusion_enabled = (_params.gnss_ctrl & GnssCtrl::HPOS)
						   && (gps_sample.fix_type >= 2)
						   && _pos_ref.isInitialized();

		// GNSS height
		if (gps_sample.fix_type >= 3) {
			// correct height for offset relative to IMU
			const float gnss_altitude = gps_sample.altitude + pos_offset_earth(2);

			// relax the upper observation noise limit which prevents bad GPS perturbing the estimate
			const float gnss_altitude_noise = math::max(gps_sample.vertical_accuracy * 1.5f,
							  _params.gps_pos_noise); // use 1.5 as a typical ratio of vacc/hacc

			controlGnssHeightFusion(gps_sample.time_us, gnss_altitude, sq(gnss_altitude_noise), velocity(2), vel_obs_var(2));

		} else {
			stopGpsHgtFusion();
		}

		// Determine if we should use GPS aiding for velocity and horizontal position
		// To start using GPS we need angular alignment completed, the local NED origin set and GPS data that has not failed checks recently
		const bool mandatory_conditions_passing = ((_params.gnss_ctrl & GnssCtrl::HPOS) || (_params.gnss_ctrl & GnssCtrl::VEL))
				&& (gps_sample.fix_type >= 3)
				&& _control_status.flags.tilt_align
				&& _control_status.flags.yaw_align
				&& _NED_origin_initialised;

		const bool continuing_conditions_passing = mandatory_conditions_passing && !gps_checks_failing;

		const bool starting_conditions_passing = continuing_conditions_passing && gps_checks_passed;

		if (_control_status.flags.gps) {
			if (mandatory_conditions_passing) {
				if (continuing_conditions_passing
				    || !isOtherSourceOfHorizontalAidingThan(_control_status.flags.gps)) {

					fuseVelocity(_aid_src_gnss_vel);
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
						    && isTimedOut(_ekfgsf_yaw_reset_time, 5'000'000)) {
							// The minimum time interval between resets to the EKF-GSF estimate is limited to allow the EKF-GSF time
							// to improve its estimate if the previous reset was not successful.
							if (resetYawToEKFGSF()) {
								ECL_WARN("GPS emergency yaw reset");
							}
						}

						ECL_WARN("GPS fusion timeout, resetting velocity and position");
						_information_events.flags.reset_vel_to_gps = true;
						_information_events.flags.reset_pos_to_gps = true;
						resetVelocityTo(velocity, vel_obs_var);
						resetHorizontalPositionTo(position, pos_obs_var);
					}

				} else {
					stopGpsFusion();
					_warning_events.flags.gps_quality_poor = true;
					ECL_WARN("GPS quality poor - stopping use");

					// TODO: move this to EV control logic
					// Reset position state to external vision if we are going to use absolute values
					if (_control_status.flags.ev_pos && !(_params.fusion_mode & SensorFusionMask::ROTATE_EXT_VIS)) {
						resetHorizontalPositionToVision();
					}
				}

			} else { // mandatory conditions are not passing
				stopGpsFusion();
			}

		} else {
			if (starting_conditions_passing) {
				// Do not use external vision for yaw if using GPS because yaw needs to be
				// defined relative to an NED reference frame
				if (_control_status.flags.ev_yaw
				    || _mag_inhibit_yaw_reset_req
				    || _mag_yaw_reset_req) {

					_mag_yaw_reset_req = true;

					// Stop the vision for yaw fusion and do not allow it to start again
					stopEvYawFusion();
					_inhibit_ev_yaw_use = true;

				} else {
					ECL_INFO("starting GPS fusion");
					_information_events.flags.starting_gps_fusion = true;

					// reset position
					_information_events.flags.reset_pos_to_gps = true;
					resetHorizontalPositionTo(position, pos_obs_var);

					// when already using another velocity source velocity reset is not necessary
					if (!isHorizontalAidingActive()) {
						_information_events.flags.reset_vel_to_gps = true;
						resetVelocityTo(velocity, vel_obs_var);
					}

					_control_status.flags.gps = true;
				}

			} else if (gps_checks_passing && !_control_status.flags.yaw_align && (_params.mag_fusion_type == MagFuseType::NONE)) {
				// If no mag is used, align using the yaw estimator (if available)
				if (resetYawToEKFGSF()) {
					_information_events.flags.yaw_aligned_to_imu_gps = true;
					ECL_INFO("Yaw aligned using IMU and GPS");

					ECL_INFO("reset velocity and position to GPS");
					_information_events.flags.reset_vel_to_gps = true;
					_information_events.flags.reset_pos_to_gps = true;
					resetVelocityTo(velocity, vel_obs_var);
					resetHorizontalPositionTo(position, pos_obs_var);
				}
			}
		}

		_time_prev_gps_us = gps_sample.time_us;

	} else if (_control_status.flags.gps && isTimedOut(_time_prev_gps_us, (uint64_t)10e6)) {
		stopGpsFusion();
		_warning_events.flags.gps_data_stopped = true;
		ECL_WARN("GPS data stopped");

	} else if (_control_status.flags.gps && isTimedOut(_time_prev_gps_us, 2 * GPS_MAX_INTERVAL)
		   && isOtherSourceOfHorizontalAidingThan(_control_status.flags.gps)) {

		// Handle the case where we are fusing another position source along GPS,
		// stop waiting for GPS after 1 s of lost signal
		stopGpsFusion();
		_warning_events.flags.gps_data_stopped_using_alternate = true;
		ECL_WARN("GPS data stopped, using only EV, OF or air data");
	}

	if (isTimedOut(_time_prev_gps_us, 2 * GPS_MAX_INTERVAL)) {
		stopGpsHgtFusion();
	}
}

bool Ekf::setGnssPositionOrigin(const gpsSample &gps)
{
	if (gps.fix_type >= 2 && (gps.horizontal_accuracy < _params.req_hacc)) {

		// If we have good GPS data set the origin's WGS-84 position to the last gps fix
		if (_pos_ref.isInitialized()) {
			ECL_INFO("updating GNSS origin LAT, LON (%.5f, %.5f) -> (%.5f, %.5f)",
				_pos_ref.getProjectionReferenceLat(), _pos_ref.getProjectionReferenceLon(),
				gps.latitude, gps.longitude);

			_pos_ref.initReference(gps.latitude, gps.longitude, gps.time_us);

		} else {
			_pos_ref.initReference(gps.latitude, gps.longitude, gps.time_us);
			ECL_INFO("initializing GNSS origin LAT: %.6f deg, LON: %.6f deg", _pos_ref.getProjectionReferenceLat(), _pos_ref.getProjectionReferenceLon());
		}

		// if we are already doing aiding, correct for the change in position since the EKF started navigating
		if (isHorizontalAidingActive()) {
			double est_lat;
			double est_lon;
			_pos_ref.reproject(-_state.pos(0), -_state.pos(1), est_lat, est_lon);
			_pos_ref.initReference(est_lat, est_lon, gps.time_us);
		}

		// save the horizontal position uncertainty of the origin
		_gps_origin_eph = gps.horizontal_accuracy;

		return _pos_ref.isInitialized();
	}

	return false;
}

bool Ekf::setGnssAltitudeOrigin(const gpsSample &gps)
{
	if (gps.fix_type >= 3 && (gps.vertical_accuracy < _params.req_vacc)) {
		// Take the current GPS height and subtract the filter height above origin to estimate the GPS height of the origin
		const float gps_alt_ref = gps.altitude + _state.pos(2);

		if (PX4_ISFINITE(_gps_alt_ref)) {
			ECL_INFO("updating GNSS origin altitude %.3f m -> %.3f m", (double)_gps_alt_ref, (double)gps_alt_ref);

		} else {
			ECL_INFO("initializing GNSS origin altitude: %.3f m (accuracy: %.3f m)", (double)gps_alt_ref, (double)gps.vertical_accuracy);
		}

		_gps_alt_ref = gps_alt_ref;

		// save the vertical position uncertainty of the origin
		_gps_origin_epv = gps.vertical_accuracy;

		return true;
	}

	return false;
}

bool Ekf::updateWorldMagneticField(const gpsSample &gps)
{
	if ((gps.fix_type >= 2) && (gps.horizontal_accuracy < 1000)) {

		// set the magnetic field data returned by the geo library using the current GPS position
		const float mag_declination_gps = get_mag_declination_radians(gps.latitude, gps.longitude);
		const float mag_inclination_gps = get_mag_inclination_radians(gps.latitude, gps.longitude);
		const float mag_strength_gps = get_mag_strength_gauss(gps.latitude, gps.longitude);

		if ((fabsf(mag_declination_gps - _mag_declination_gps) > math::radians(1.f))
		    || (fabsf(mag_inclination_gps - _mag_inclination_gps) > math::radians(1.f))
		    || (fabsf(mag_strength_gps - _mag_strength_gps) > 0.1f)
		   ) {
			if (_control_status.flags.mag_hdg || _control_status.flags.mag_3D) {
				// request a reset of the yaw using the new declination
				_mag_yaw_reset_req = true;
			}
		}

		_mag_declination_gps = mag_declination_gps;
		_mag_inclination_gps = mag_inclination_gps;
		_mag_strength_gps = mag_strength_gps;
	}

	return false;
}

void Ekf::updateEarthRateNED(const gpsSample &gps)
{
	if ((gps.fix_type >= 2) && (gps.horizontal_accuracy < 1000)) {

		// calculate the earth rotation vector from a given latitude
		const double lat_rad = math::radians(gps.latitude);

		_earth_rate_NED = Vector3f(static_cast<float>(CONSTANTS_EARTH_SPIN_RATE * cos(lat_rad)),
					   0.f,
					   static_cast<float>(-CONSTANTS_EARTH_SPIN_RATE * sin(lat_rad)));
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
			&& isRecent(_time_last_on_ground_us, 30000000)
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
