/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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
	if (!(_params.fusion_mode & SensorFusionMask::USE_GPS)) {
		stopGpsFusion();
		return;
	}

	// Check for new GPS data that has fallen behind the fusion time horizon
	if (_gps_data_ready) {

		const gpsSample &gps_sample{_gps_sample_delayed};
		const bool gps_checks_passing = isTimedOut(_last_gps_fail_us, (uint64_t)5e6);
		const bool gps_checks_failing = isTimedOut(_last_gps_pass_us, (uint64_t)5e6);

		if (!_control_status.flags.yaw_align && _control_status.flags.tilt_align
		    && gps_checks_passing && !gps_checks_failing) {

			if (resetYawToEKFGSF()) {
				ECL_INFO("Yaw aligned using IMU and GPS");

			} else if (resetYawToGps(gps_sample)) {
				ECL_INFO("Yaw aligned using GPS yaw");

			} else if (realignYawGPS(gps_sample)) {
				ECL_INFO("Yaw aligned using GPS course");
			}
		}

		updateGpsYaw(gps_sample);
		updateGpsVel(gps_sample);
		updateGpsPos(gps_sample);

		controlGpsYawFusion(gps_checks_passing, gps_checks_failing);

		// Determine if we should use GPS aiding for velocity and horizontal position
		// To start using GPS we need angular alignment completed, the local NED origin set and GPS data that has not failed checks recently
		const bool mandatory_conditions_passing = _control_status.flags.tilt_align
				&& _control_status.flags.yaw_align
				&& _NED_origin_initialised;

		const bool continuing_conditions_passing = mandatory_conditions_passing && !gps_checks_failing;
		const bool starting_conditions_passing = continuing_conditions_passing && gps_checks_passing;

		if (_control_status.flags.gps) {
			if (mandatory_conditions_passing) {
				if (continuing_conditions_passing
				    || !isOtherSourceOfHorizontalAidingThan(_control_status.flags.gps)) {

					fuseGpsVel();
					fuseGpsPos();

					const bool fusion_timeout = (isTimedOut(_time_last_hor_pos_fuse, _params.reset_timeout_max)
								     && isTimedOut(_time_last_hor_vel_fuse, _params.reset_timeout_max))
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
							&& (_time_last_hor_vel_fuse > _time_last_on_ground_us)
							&& isRecent(_time_last_on_ground_us, 30'000'000)
							&& isTimedOut(_time_last_hor_vel_fuse, _params.EKFGSF_reset_delay);

					const bool is_inflight_nav_failure = _control_status.flags.in_air
									     && (_time_last_hor_pos_fuse > _time_last_on_ground_us)
									     && (_time_last_hor_vel_fuse > _time_last_on_ground_us)
									     && isTimedOut(_time_last_hor_vel_fuse, _params.reset_timeout_max)
									     && isTimedOut(_time_last_hor_pos_fuse, _params.reset_timeout_max);

					if (fusion_timeout || is_recent_takeoff_nav_failure || is_inflight_nav_failure) {
						const bool was_gps_signal_lost = isTimedOut(_time_prev_gps_us, 1'000'000);

						/* A reset is not performed when getting GPS back after a significant period of no data
						 * because the timeout could have been caused by bad GPS.
						 * The total number of resets allowed per boot cycle is limited.
						 */
						if (isYawFailure()
						    && _control_status.flags.in_air
						    && !was_gps_signal_lost
						    && resetYawToEKFGSF()) {

							ECL_WARN("GPS emergency yaw reset");

							if (_control_status.flags.mag_hdg || _control_status.flags.mag_3D) {
								// stop using the magnetometer in the main EKF otherwise it's fusion could drag the yaw around
								// and cause another navigation failure
								_control_status.flags.mag_fault = true;
								_warning_events.flags.emergency_yaw_reset_mag_stopped = true;
								stopMagFusion();

							} else if (_control_status.flags.gps_yaw) {
								_control_status.flags.gps_yaw_fault = true;
								_warning_events.flags.emergency_yaw_reset_gps_yaw_stopped = true;
								stopGpsYawFusion();

							} else if (_control_status.flags.ev_yaw) {
								stopEvYawFusion();
							}

						} else {
							_warning_events.flags.gps_fusion_timout = true;
							ECL_WARN("GPS fusion timeout - resetting");

							// use GPS velocity data to check and correct yaw angle if a FW vehicle
							if (_control_status.flags.fixed_wing && _control_status.flags.in_air) {
								// if flying a fixed wing aircraft, do a complete reset that includes yaw

								// calculate GPS course over ground angle
								const float gps_cog = atan2f(gps_sample.vel(1), gps_sample.vel(0));

								// calculate course yaw angle
								const float ekf_cog = atan2f(_state.vel(1), _state.vel(0));

								// Check the EKF and GPS course over ground for consistency
								const float course_yaw_error = wrap_pi(gps_cog - ekf_cog);

								// If the angles disagree and horizontal GPS velocity innovations are large or no previous yaw alignment, we declare the magnetic yaw as bad
								const bool bad_yaw_error = fabsf(course_yaw_error) > math::radians(25.f);

								if (bad_yaw_error) {
									_num_bad_flight_yaw_events++;

									_warning_events.flags.bad_yaw_using_gps_course = true;
									ECL_WARN("bad yaw, using GPS course");

									if (_control_status.flags.mag_hdg || _control_status.flags.mag_3D) {

										// declare the magnetometer as failed if a bad yaw has occurred more than once
										if (_control_status.flags.mag_aligned_in_flight
										    && (_num_bad_flight_yaw_events >= 2)) {

											_control_status.flags.mag_fault = true;

											stopMagFusion();
										}
									}

									realignYawGPS(gps_sample);
								}
							}
						}

						resetVelocityToGps(gps_sample);
						resetHorizontalPositionToGps(gps_sample);
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

				bool yaw_reset_needed = false;

				if (_control_status.flags.ev_yaw) {
					// Stop the vision for yaw fusion and do not allow it to start again
					stopEvYawFusion();

					yaw_reset_needed = true;
				}

				if (isYawEmergencyEstimateAvailable() && isYawError(math::radians(10.f))) {
					yaw_reset_needed = true;
				}

				// yaw needs to be defined relative to an NED reference frame
				if (yaw_reset_needed) {
					if (resetYawToEKFGSF()) {
						ECL_INFO("starting GPS, reset yaw using yaw estimator");

					} else if (resetYawToGps(gps_sample)) {
						ECL_INFO("starting GPS, reset yaw to GPS");

					} else if (realignYawGPS(gps_sample)) {
						ECL_INFO("starting GPS, reset yaw using GPS course");

					} else {
						// all failed
						ECL_ERR("starting GPS");
					}
				}

				if (!isHorizontalAidingActive()) {
					resetVelocityToGps(gps_sample);
				}

				resetHorizontalPositionToGps(gps_sample);

				_information_events.flags.starting_gps_fusion = true;
				ECL_INFO("starting GPS fusion");
				_control_status.flags.gps = true;

			}
		}

	} else if (_control_status.flags.gps && (_imu_sample_delayed.time_us - _gps_sample_delayed.time_us > (uint64_t)10e6)) {
		stopGpsFusion();
		_warning_events.flags.gps_data_stopped = true;
		ECL_WARN("GPS data stopped");

	}  else if (_control_status.flags.gps && (_imu_sample_delayed.time_us - _gps_sample_delayed.time_us > (uint64_t)1e6)
		    && isOtherSourceOfHorizontalAidingThan(_control_status.flags.gps)) {
		// Handle the case where we are fusing another position source along GPS,
		// stop waiting for GPS after 1 s of lost signal
		stopGpsFusion();
		_warning_events.flags.gps_data_stopped_using_alternate = true;
		ECL_WARN("GPS data stopped, using only EV, OF or air data");
	}
}

bool Ekf::isYawError(float error_threshold) const
{
	if (!isYawEmergencyEstimateAvailable()) {
		return false;
	}

	const float euler_yaw = getEulerYaw(_R_to_earth);
	const float yaw_error = wrap_pi(euler_yaw - _yawEstimator.getYaw());

	return fabsf(yaw_error) > error_threshold;
}
