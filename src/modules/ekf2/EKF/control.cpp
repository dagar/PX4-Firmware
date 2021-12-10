/****************************************************************************
 *
 *   Copyright (c) 2015-2020 Estimation and Control Library (ECL). All rights reserved.
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
 * 3. Neither the name ECL nor the names of its contributors may be
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
 * @file control.cpp
 * Control functions for ekf attitude and position estimator.
 *
 * @author Paul Riseborough <p_riseborough@live.com.au>
 *
 */


#include "ekf.h"
#include <mathlib/mathlib.h>

void Ekf::controlFusionModes()
{
	// Store the status to enable change detection
	_control_status_prev.value = _control_status.value;

	// monitor the tilt alignment
	if (!_control_status.flags.tilt_align) {
		// whilst we are aligning the tilt, monitor the variances
		const Vector3f angle_err_var_vec = calcRotVecVariances();

		// Once the tilt variances have reduced to equivalent of 3deg uncertainty
		// and declare the tilt alignment complete
		if ((angle_err_var_vec(0) + angle_err_var_vec(1)) < sq(math::radians(3.0f))) {
			_control_status.flags.tilt_align = true;

			// send alignment status message to the console
			const char *height_source = nullptr;

			if (_control_status.flags.baro_hgt) {
				height_source = "baro";

			} else if (_control_status.flags.ev_hgt) {
				height_source = "ev";

			} else if (_control_status.flags.gps_hgt) {
				height_source = "gps";

			} else if (_control_status.flags.rng_hgt) {
				height_source = "rng";

			} else {
				height_source = "unknown";

			}

			if (height_source) {
				ECL_INFO("%llu: EKF aligned, (%s hgt, IMU buf: %i, OBS buf: %i)",
					 (unsigned long long)_imu_sample_delayed.time_us, height_source, (int)_imu_buffer_length, (int)_obs_buffer_length);
			}
		}
	}

	// check for arrival of new sensor data at the fusion time horizon
	_time_prev_gps_us = _gps_sample_delayed.time_us;

	const uint64_t delta_time_prev_gps_us = _gps_sample_delayed.time_us;
	_gps_data_ready = _gps_buffer.pop_first_older_than(_imu_sample_delayed.time_us, &_gps_sample_delayed);

	// if we have a new sample save the delta time between this sample and the last sample which is used for height offset calculations
	if (_gps_data_ready && (delta_time_prev_gps_us != 0)) {
		_delta_time_gps_us = _gps_sample_delayed.time_us - delta_time_prev_gps_us;
	}


	_mag_data_ready = _mag_buffer.pop_first_older_than(_imu_sample_delayed.time_us, &_mag_sample_delayed);

	if (_mag_data_ready) {
		_mag_lpf.update(_mag_sample_delayed.mag);

		// if enabled, use knowledge of theoretical magnetic field vector to calculate a synthetic magnetomter Z component value.
		// this is useful if there is a lot of interference on the sensor measurement.
		if (_params.synthesize_mag_z && (_params.mag_declination_source & MASK_USE_GEO_DECL) && (_NED_origin_initialised
				|| PX4_ISFINITE(_mag_declination_gps))) {

			const Vector3f mag_earth_pred = Dcmf(Eulerf(0, -_mag_inclination_gps, _mag_declination_gps)) * Vector3f(_mag_strength_gps, 0, 0);
			_mag_sample_delayed.mag(2) = calculate_synthetic_mag_z_measurement(_mag_sample_delayed.mag, mag_earth_pred);
			_control_status.flags.synthetic_mag_z = true;

		} else {
			_control_status.flags.synthetic_mag_z = false;
		}
	}


	const uint64_t delta_time_prev_baro_us = _baro_sample_delayed.time_us;
	_baro_data_ready = _baro_buffer.pop_first_older_than(_imu_sample_delayed.time_us, &_baro_sample_delayed);

	// if we have a new baro sample save the delta time between this sample and the last sample which is
	// used below for baro offset calculations
	if (_baro_data_ready && (delta_time_prev_baro_us != 0)) {
		_delta_time_baro_us = _baro_sample_delayed.time_us - delta_time_prev_baro_us;
	}


	{
		// Get range data from buffer and check validity
		const uint64_t delta_time_prev_rng_us = _range_sensor.sample().time_us;
		const bool is_rng_data_ready = _range_buffer.pop_first_older_than(_imu_sample_delayed.time_us, _range_sensor.getSampleAddress());
		_range_sensor.setDataReadiness(is_rng_data_ready);

		// if we have a new sample save the delta time between this sample and the last sample which is used for height offset calculations
		if (is_rng_data_ready && (delta_time_prev_rng_us != 0)) {
			_delta_time_rng_us = _range_sensor.sample().time_us - delta_time_prev_rng_us;
		}

		// update range sensor angle parameters in case they have changed
		_range_sensor.setPitchOffset(_params.rng_sens_pitch);
		_range_sensor.setCosMaxTilt(_params.range_cos_max_tilt);
		_range_sensor.setQualityHysteresis(_params.range_valid_quality_s);

		_range_sensor.runChecks(_imu_sample_delayed.time_us, _R_to_earth);
	}

	if (_range_sensor.isDataHealthy()) {
		// correct the range data for position offset relative to the IMU
		const Vector3f pos_offset_body = _params.rng_pos_body - _params.imu_pos_body;
		const Vector3f pos_offset_earth = _R_to_earth * pos_offset_body;
		_range_sensor.setRange(_range_sensor.getRange() + pos_offset_earth(2) / _range_sensor.getCosTilt());
	}

	// We don't fuse flow data immediately because we have to wait for the mid integration point to fall behind the fusion time horizon.
	// This means we stop looking for new data until the old data has been fused, unless we are not fusing optical flow,
	// in this case we need to empty the buffer
	if (!_flow_data_ready || !_control_status.flags.opt_flow) {
		_flow_data_ready = _flow_buffer.pop_first_older_than(_imu_sample_delayed.time_us, &_flow_sample_delayed);
	}

	// check if we should fuse flow data for terrain estimation
	if (!_flow_for_terrain_data_ready && _flow_data_ready && _control_status.flags.in_air) {
		// TODO: WARNING, _flow_data_ready can be modified in controlOpticalFlowFusion
		// due to some checks failing
		// only fuse flow for terrain if range data hasn't been fused for 5 seconds
		_flow_for_terrain_data_ready = isTimedOut(_time_last_hagl_fuse, (uint64_t)5E6);
		// only fuse flow for terrain if the main filter is not fusing flow and we are using gps
		_flow_for_terrain_data_ready &= (!_control_status.flags.opt_flow && _control_status.flags.gps);
	}


	const uint64_t delta_time_prev_ev_us = _ev_sample_delayed.time_us;
	_ev_data_ready = _ext_vision_buffer.pop_first_older_than(_imu_sample_delayed.time_us, &_ev_sample_delayed);

	// if we have a new EV sample save the delta time between this sample and the last sample which is used for height offset calculations
	if (_ev_data_ready && (delta_time_prev_ev_us != 0)) {
		_delta_time_ev_us = _ev_sample_delayed.time_us - delta_time_prev_ev_us;
	}


	_tas_data_ready = _airspeed_buffer.pop_first_older_than(_imu_sample_delayed.time_us, &_airspeed_sample_delayed);

	// control use of observations for aiding
	controlMagFusion();
	controlOpticalFlowFusion();
	controlGpsFusion();
	controlAirDataFusion();
	controlBetaFusion();
	controlDragFusion();
	controlHeightFusion();

	// Additional data odoemtery data from an external estimator can be fused.
	controlExternalVisionFusion();

	// Additional horizontal velocity data from an auxiliary sensor can be fused
	controlAuxVelFusion();

	// Fake position measurement for constraining drift when no other velocity or position measurements
	controlFakePosFusion();

	// check if we are no longer fusing measurements that directly constrain velocity drift
	update_deadreckoning_status();
}

void Ekf::controlExternalVisionFusion()
{
	// Check for new external vision data
	if (_ev_data_ready) {

		if (_inhibit_ev_yaw_use) {
			stopEvYawFusion();
		}

		// if the ev data is not in a NED reference frame, then the transformation between EV and EKF navigation frames
		// needs to be calculated and the observations rotated into the EKF frame of reference
		if ((_params.fusion_mode & MASK_ROTATE_EV) && ((_params.fusion_mode & MASK_USE_EVPOS)
				|| (_params.fusion_mode & MASK_USE_EVVEL)) && !_control_status.flags.ev_yaw) {

			// rotate EV measurements into the EKF Navigation frame
			calcExtVisRotMat();
		}

		// external vision aiding selection logic
		if (_control_status.flags.tilt_align && _control_status.flags.yaw_align) {

			// check for a external vision measurement that has fallen behind the fusion time horizon
			if (isRecent(_time_last_ext_vision, 2 * EV_MAX_INTERVAL)) {
				// turn on use of external vision measurements for position
				if (_params.fusion_mode & MASK_USE_EVPOS && !_control_status.flags.ev_pos) {
					startEvPosFusion();
				}

				// turn on use of external vision measurements for velocity
				if (_params.fusion_mode & MASK_USE_EVVEL && !_control_status.flags.ev_vel) {
					startEvVelFusion();
				}
			}
		}

		// external vision yaw aiding selection logic
		if (!_inhibit_ev_yaw_use && (_params.fusion_mode & MASK_USE_EVYAW) && !_control_status.flags.ev_yaw
		    && _control_status.flags.tilt_align) {

			// don't start using EV data unless data is arriving frequently
			if (isRecent(_time_last_ext_vision, 2 * EV_MAX_INTERVAL)) {
				if (resetYawToEv()) {
					_control_status.flags.yaw_align = true;
					startEvYawFusion();
				}
			}
		}

		// determine if we should use the horizontal position observations
		if (_control_status.flags.ev_pos) {

			Vector3f ev_pos_obs_var;
			Vector2f ev_pos_innov_gates;

			// correct position and height for offset relative to IMU
			const Vector3f pos_offset_body = _params.ev_pos_body - _params.imu_pos_body;
			const Vector3f pos_offset_earth = _R_to_earth * pos_offset_body;
			_ev_sample_delayed.pos -= pos_offset_earth;

			// Use an incremental position fusion method for EV position data if GPS is also used
			if (_params.fusion_mode & MASK_USE_GPS) {
				_fuse_hpos_as_odom = true;

			} else {
				_fuse_hpos_as_odom = false;
			}

			if (_fuse_hpos_as_odom) {
				if (!_hpos_prev_available) {
					// no previous observation available to calculate position change
					_hpos_prev_available = true;

				} else {
					// calculate the change in position since the last measurement
					Vector3f ev_delta_pos = _ev_sample_delayed.pos - _pos_meas_prev;

					// rotate measurement into body frame is required when fusing with GPS
					ev_delta_pos = _R_ev_to_ekf * ev_delta_pos;

					// use the change in position since the last measurement
					_ev_pos_innov(0) = _state.pos(0) - _hpos_pred_prev(0) - ev_delta_pos(0);
					_ev_pos_innov(1) = _state.pos(1) - _hpos_pred_prev(1) - ev_delta_pos(1);

					// observation 1-STD error, incremental pos observation is expected to have more uncertainty
					Matrix3f ev_pos_var = matrix::diag(_ev_sample_delayed.posVar);
					ev_pos_var = _R_ev_to_ekf * ev_pos_var * _R_ev_to_ekf.transpose();
					ev_pos_obs_var(0) = fmaxf(ev_pos_var(0, 0), sq(0.5f));
					ev_pos_obs_var(1) = fmaxf(ev_pos_var(1, 1), sq(0.5f));
				}

				// record observation and estimate for use next time
				_pos_meas_prev = _ev_sample_delayed.pos;
				_hpos_pred_prev = _state.pos.xy();

			} else {
				// use the absolute position
				Vector3f ev_pos_meas = _ev_sample_delayed.pos;
				Matrix3f ev_pos_var = matrix::diag(_ev_sample_delayed.posVar);

				if (_params.fusion_mode & MASK_ROTATE_EV) {
					ev_pos_meas = _R_ev_to_ekf * ev_pos_meas;
					ev_pos_var = _R_ev_to_ekf * ev_pos_var * _R_ev_to_ekf.transpose();
				}

				_ev_pos_innov(0) = _state.pos(0) - ev_pos_meas(0);
				_ev_pos_innov(1) = _state.pos(1) - ev_pos_meas(1);

				ev_pos_obs_var(0) = fmaxf(ev_pos_var(0, 0), sq(0.01f));
				ev_pos_obs_var(1) = fmaxf(ev_pos_var(1, 1), sq(0.01f));

				// check if we have been deadreckoning too long
				if (isTimedOut(_time_last_hor_pos_fuse, _params.reset_timeout_max)) {
					// only reset velocity if we have no another source of aiding constraining it
					if (isTimedOut(_time_last_of_fuse, (uint64_t)1E6) &&
					    isTimedOut(_time_last_hor_vel_fuse, (uint64_t)1E6)) {
						resetVelocity();
					}

					resetHorizontalPosition();
				}
			}

			// innovation gate size
			ev_pos_innov_gates(0) = fmaxf(_params.ev_pos_innov_gate, 1.0f);

			fuseHorizontalPosition(_ev_pos_innov, ev_pos_innov_gates, ev_pos_obs_var, _ev_pos_innov_var, _ev_pos_test_ratio);
		}

		// determine if we should use the velocity observations
		if (_control_status.flags.ev_vel) {

			Vector2f ev_vel_innov_gates;

			_last_vel_obs = getVisionVelocityInEkfFrame();
			_ev_vel_innov = _state.vel - _last_vel_obs;

			// check if we have been deadreckoning too long
			if (isTimedOut(_time_last_hor_vel_fuse, _params.reset_timeout_max)) {
				// only reset velocity if we have no another source of aiding constraining it
				if (isTimedOut(_time_last_of_fuse, (uint64_t)1E6) &&
				    isTimedOut(_time_last_hor_pos_fuse, (uint64_t)1E6)) {
					resetVelocity();
				}
			}

			_last_vel_obs_var = matrix::max(getVisionVelocityVarianceInEkfFrame(), sq(0.05f));

			ev_vel_innov_gates.setAll(fmaxf(_params.ev_vel_innov_gate, 1.0f));

			fuseHorizontalVelocity(_ev_vel_innov, ev_vel_innov_gates, _last_vel_obs_var, _ev_vel_innov_var, _ev_vel_test_ratio);
			fuseVerticalVelocity(_ev_vel_innov, ev_vel_innov_gates, _last_vel_obs_var, _ev_vel_innov_var, _ev_vel_test_ratio);
		}

		// determine if we should use the yaw observation
		if (_control_status.flags.ev_yaw) {
			fuseHeading();
		}

	} else if ((_control_status.flags.ev_pos || _control_status.flags.ev_vel ||  _control_status.flags.ev_yaw)
		   && isTimedOut(_time_last_ext_vision, (uint64_t)_params.reset_timeout_max)) {

		// Turn off EV fusion mode if no data has been received
		stopEvFusion();
		_warning_events.flags.vision_data_stopped = true;
		ECL_WARN("vision data stopped");
	}
}

void Ekf::controlOpticalFlowFusion()
{
	// Check if on ground motion is un-suitable for use of optical flow
	if (!_control_status.flags.in_air) {
		updateOnGroundMotionForOpticalFlowChecks();

	} else {
		resetOnGroundMotionForOpticalFlowChecks();
	}

	// Accumulate autopilot gyro data across the same time interval as the flow sensor
	_imu_del_ang_of += _imu_sample_delayed.delta_ang - _state.delta_ang_bias;
	_delta_time_of += _imu_sample_delayed.delta_ang_dt;

	if (_flow_data_ready) {
		const bool is_quality_good = (_flow_sample_delayed.quality >= _params.flow_qual_min);
		const bool is_magnitude_good = !_flow_sample_delayed.flow_xy_rad.longerThan(_flow_sample_delayed.dt * _flow_max_rate);
		const bool is_tilt_good = (_R_to_earth(2, 2) > _params.range_cos_max_tilt);

		const float delta_time_min = fmaxf(0.7f * _delta_time_of, 0.001f);
		const float delta_time_max = fminf(1.3f * _delta_time_of, 0.2f);
		const bool is_delta_time_good = _flow_sample_delayed.dt >= delta_time_min && _flow_sample_delayed.dt <= delta_time_max;
		const bool is_body_rate_comp_available = calcOptFlowBodyRateComp();

		if (is_quality_good
		    && is_magnitude_good
		    && is_tilt_good
		    && is_body_rate_comp_available
		    && is_delta_time_good) {
			// compensate for body motion to give a LOS rate
			_flow_compensated_XY_rad = _flow_sample_delayed.flow_xy_rad - _flow_sample_delayed.gyro_xyz.xy();

		} else if (!_control_status.flags.in_air) {

			if (!is_delta_time_good) {
				// handle special case of SITL and PX4Flow where dt is forced to
				// zero when the quaity is 0
				_flow_sample_delayed.dt = delta_time_min;
			}

			// don't allow invalid flow gyro_xyz to propagate
			if (!is_body_rate_comp_available) {
				if (!PX4_ISFINITE(_flow_sample_delayed.gyro_xyz(0)) || !PX4_ISFINITE(_flow_sample_delayed.gyro_xyz(1)) || !PX4_ISFINITE(_flow_sample_delayed.gyro_xyz(2))) {
					_flow_sample_delayed.gyro_xyz.zero();
				}
			}

			// when on the ground with poor flow quality,
			// assume zero ground relative velocity and LOS rate
			_flow_compensated_XY_rad.setZero();

		} else {
			// don't use this flow data and wait for the next data to arrive
			_flow_data_ready = false;
			_flow_for_terrain_data_ready = false; // TODO: find a better place
		}
	}

	// New optical flow data is available and is ready to be fused when the midpoint of the sample falls behind the fusion time horizon
	if (_flow_data_ready) {
		// Inhibit flow use if motion is un-suitable or we have good quality GPS
		// Apply hysteresis to prevent rapid mode switching
		const float gps_err_norm_lim = _control_status.flags.opt_flow ? 0.7f : 1.0f;

		// Check if we are in-air and require optical flow to control position drift
		const bool is_flow_required = _control_status.flags.in_air
					      && (_is_dead_reckoning // is doing inertial dead-reckoning so must constrain drift urgently
						  || isOnlyActiveSourceOfHorizontalAiding(_control_status.flags.opt_flow)
						  || (_control_status.flags.gps && (_gps_error_norm > gps_err_norm_lim))); // is using GPS, but GPS is bad


		// inhibit use of optical flow if motion is unsuitable and we are not reliant on it for flight navigation
		const bool preflight_motion_not_ok = !_control_status.flags.in_air
						     && ((_imu_sample_delayed.time_us > (_time_good_motion_us + (uint64_t)1E5))
								     || (_imu_sample_delayed.time_us < (_time_bad_motion_us + (uint64_t)5E6)));
		const bool flight_condition_not_ok = _control_status.flags.in_air && !isTerrainEstimateValid();

		_inhibit_flow_use = ((preflight_motion_not_ok || flight_condition_not_ok) && !is_flow_required)
				    || !_control_status.flags.tilt_align;

		// Handle cases where we are using optical flow but we should not use it anymore
		if (_control_status.flags.opt_flow) {
			if (!(_params.fusion_mode & MASK_USE_OF)
			    || _inhibit_flow_use) {

				stopFlowFusion();
				return;
			}
		}

		// optical flow fusion mode selection logic
		if ((_params.fusion_mode & MASK_USE_OF) // optical flow has been selected by the user
		    && !_control_status.flags.opt_flow // we are not yet using flow data
		    && !_inhibit_flow_use) {
			// If the heading is valid and use is not inhibited , start using optical flow aiding
			if (_control_status.flags.yaw_align) {
				// set the flag and reset the fusion timeout
				_control_status.flags.opt_flow = true;
				_time_last_of_fuse = _time_last_imu;

				// if we are not using GPS or external vision aiding, then the velocity and position states and covariances need to be set
				const bool flow_aid_only = !isOtherSourceOfHorizontalAidingThan(_control_status.flags.opt_flow);

				if (flow_aid_only) {
					resetVelocity();
					resetHorizontalPosition();
				}
			}
		}

		if (_control_status.flags.opt_flow) {
			// Wait until the midpoint of the flow sample has fallen behind the fusion time horizon
			if (_imu_sample_delayed.time_us > (_flow_sample_delayed.time_us - uint32_t(1e6f * _flow_sample_delayed.dt) / 2)) {
				// Fuse optical flow LOS rate observations into the main filter only if height above ground has been updated recently
				// but use a relaxed time criteria to enable it to coast through bad range finder data
				if (isRecent(_time_last_hagl_fuse, (uint64_t)10e6)) {
					fuseOptFlow();
					_last_known_posNE = _state.pos.xy();
				}

				_flow_data_ready = false;
			}

			// handle the case when we have optical flow, are reliant on it, but have not been using it for an extended period
			if (isTimedOut(_time_last_of_fuse, _params.reset_timeout_max)
			    && !isOtherSourceOfHorizontalAidingThan(_control_status.flags.opt_flow)) {

				resetVelocity();
				resetHorizontalPosition();
			}
		}

	} else if (_control_status.flags.opt_flow
		   && (_imu_sample_delayed.time_us >  _flow_sample_delayed.time_us + (uint64_t)10e6)) {

		stopFlowFusion();
	}
}

void Ekf::updateOnGroundMotionForOpticalFlowChecks()
{
	// When on ground check if the vehicle is being shaken or moved in a way that could cause a loss of navigation
	const float accel_norm = _accel_vec_filt.norm();

	const bool motion_is_excessive = ((accel_norm > (CONSTANTS_ONE_G * 1.5f)) // upper g limit
					  || (accel_norm < (CONSTANTS_ONE_G * 0.5f)) // lower g limit
					  || (_ang_rate_magnitude_filt > _flow_max_rate) // angular rate exceeds flow sensor limit
					  || (_R_to_earth(2, 2) < cosf(math::radians(30.0f)))); // tilted excessively

	if (motion_is_excessive) {
		_time_bad_motion_us = _imu_sample_delayed.time_us;

	} else {
		_time_good_motion_us = _imu_sample_delayed.time_us;
	}
}

void Ekf::resetOnGroundMotionForOpticalFlowChecks()
{
	_time_bad_motion_us = 0;
	_time_good_motion_us = _imu_sample_delayed.time_us;
}

void Ekf::controlGpsYawFusion(bool gps_checks_passing, bool gps_checks_failing)
{
	if (!(_params.fusion_mode & MASK_USE_GPSYAW)
	    || _control_status.flags.gps_yaw_fault) {

		stopGpsYawFusion();
		return;
	}

	const bool is_new_data_available = PX4_ISFINITE(_gps_sample_delayed.yaw);

	if (is_new_data_available) {

		const bool continuing_conditions_passing = !gps_checks_failing;

		const bool is_gps_yaw_data_intermittent = !isRecent(_time_last_gps_yaw_data, 2 * GPS_MAX_INTERVAL);

		const bool starting_conditions_passing = continuing_conditions_passing
				&& _control_status.flags.tilt_align
				&& gps_checks_passing
				&& !is_gps_yaw_data_intermittent
				&& !_gps_hgt_intermittent;

		_time_last_gps_yaw_data = _time_last_imu;

		if (_control_status.flags.gps_yaw) {

			if (continuing_conditions_passing) {

				fuseGpsYaw();

				const bool is_fusion_failing = isTimedOut(_time_last_gps_yaw_fuse, _params.reset_timeout_max);

				if (is_fusion_failing) {
					if (_nb_gps_yaw_reset_available > 0) {
						// Data seems good, attempt a reset
						resetYawToGps();

						if (_control_status.flags.in_air) {
							_nb_gps_yaw_reset_available--;
						}

					} else if (starting_conditions_passing) {
						// Data seems good, but previous reset did not fix the issue
						// something else must be wrong, declare the sensor faulty and stop the fusion
						_control_status.flags.gps_yaw_fault = true;
						stopGpsYawFusion();

					} else {
						// A reset did not fix the issue but all the starting checks are not passing
						// This could be a temporary issue, stop the fusion without declaring the sensor faulty
						stopGpsYawFusion();
					}

					// TODO: should we give a new reset credit when the fusion does not fail for some time?
				}

			} else {
				// Stop GPS yaw fusion but do not declare it faulty
				stopGpsYawFusion();
			}

		} else {
			if (starting_conditions_passing) {
				// Try to activate GPS yaw fusion
				startGpsYawFusion();

				if (_control_status.flags.gps_yaw) {
					_nb_gps_yaw_reset_available = 1;
				}
			}
		}

	} else if (_control_status.flags.gps_yaw && isTimedOut(_time_last_gps_yaw_data, _params.reset_timeout_max)) {
		// No yaw data in the message anymore. Stop until it comes back.
		stopGpsYawFusion();
	}

	// Before takeoff, we do not want to continue to rely on the current heading
	// if we had to stop the fusion
	if (!_control_status.flags.in_air
	    && !_control_status.flags.gps_yaw
	    && _control_status_prev.flags.gps_yaw) {
		_control_status.flags.yaw_align = false;
	}
}

void Ekf::checkVerticalAccelerationHealth()
{
	// Check for IMU accelerometer vibration induced clipping as evidenced by the vertical
	// innovations being positive and not stale.
	// Clipping usually causes the average accel reading to move towards zero which makes the INS
	// think it is falling and produces positive vertical innovations.
	// Don't use stale innovation data.
	bool is_inertial_nav_falling = false;
	bool are_vertical_pos_and_vel_independant = false;


	if (isRecent(_vert_pos_gps_fuse_attempt_time_us, 1000000) || isRecent(_vert_pos_ev_fuse_attempt_time_us, 1000000)) {

		if (isRecent(_vert_vel_fuse_time_us, 1000000)) {
			// If vertical position and velocity come from independent sensors then we can
			// trust them more if they disagree with the IMU, but need to check that they agree
			const bool using_gps_for_both = _control_status.flags.gps_hgt && _control_status.flags.gps;
			const bool using_ev_for_both = _control_status.flags.ev_hgt && _control_status.flags.ev_vel;

			are_vertical_pos_and_vel_independant = !(using_gps_for_both || using_ev_for_both);

			if (_control_status.flags.gps_hgt) {
				if (_vert_vel_innov_ratio > _params.vert_innov_test_lim && _vert_pos_innov_ratio_gps > 0) {
					is_inertial_nav_falling = true;
				}

				if (_vert_pos_innov_ratio_gps > _params.vert_innov_test_lim && _vert_vel_innov_ratio > 0) {
					is_inertial_nav_falling = true;
				}
			}

			if (_control_status.flags.ev_hgt) {
				if (_vert_vel_innov_ratio > _params.vert_innov_test_lim && _vert_pos_innov_ratio_ev > 0) {
					is_inertial_nav_falling = true;
				}

				if (_vert_pos_innov_ratio_ev > _params.vert_innov_test_lim && _vert_vel_innov_ratio > 0) {
					is_inertial_nav_falling = true;
				}
			}

		} else {
			// only height sensing available
			if (_control_status.flags.gps_hgt) {
				is_inertial_nav_falling = _vert_pos_innov_ratio_gps > _params.vert_innov_test_lim;
			}

			if (_control_status.flags.ev_hgt) {
				is_inertial_nav_falling = _vert_pos_innov_ratio_ev > _params.vert_innov_test_lim;
			}
		}
	}

	// Check for more than 50% clipping affected IMU samples within the past 1 second
	const uint16_t clip_count_limit = 1000 / FILTER_UPDATE_PERIOD_MS;
	const bool is_clipping = _imu_sample_delayed.delta_vel_clipping[0] ||
				 _imu_sample_delayed.delta_vel_clipping[1] ||
				 _imu_sample_delayed.delta_vel_clipping[2];

	if (is_clipping && _clip_counter < clip_count_limit) {
		_clip_counter++;

	} else if (_clip_counter > 0) {
		_clip_counter--;
	}

	const bool is_clipping_frequently = _clip_counter > 0;

	// if vertical velocity and position are independent and agree, then do not require evidence of clipping if
	// innovations are large
	const bool bad_vert_accel = (are_vertical_pos_and_vel_independant || is_clipping_frequently) && is_inertial_nav_falling;

	if (bad_vert_accel) {
		_time_bad_vert_accel =  _time_last_imu;

	} else {
		_time_good_vert_accel = _time_last_imu;
	}

	// declare a bad vertical acceleration measurement and make the declaration persist
	// for a minimum of BADACC_PROBATION seconds
	if (_fault_status.flags.bad_acc_vertical) {
		_fault_status.flags.bad_acc_vertical = isRecent(_time_bad_vert_accel, BADACC_PROBATION);

	} else {
		_fault_status.flags.bad_acc_vertical = bad_vert_accel;
	}
}

void Ekf::controlHeightFusion()
{
	/*
	 * Handle the case where we have not fused height measurements recently and
	 * uncertainty exceeds the max allowable. Reset using the best available height
	 * measurement source, continue using it after the reset and declare the current
	 * source failed if we have switched.
	*/
	checkVerticalAccelerationHealth();

	// check if height is continuously failing because of accel errors
	const bool continuous_bad_accel_hgt = isTimedOut(_time_good_vert_accel, (uint64_t)_params.bad_acc_reset_delay_us);

	// check for inertial sensing errors in the last BADACC_PROBATION seconds
	const bool prev_bad_vert_accel = isRecent(_time_bad_vert_accel, BADACC_PROBATION);



	// check if height has been inertial deadreckoning for too long
	// in vision hgt mode check for vision data
	const bool baro_hgt_fusion_timeout = isTimedOut(_time_last_baro_hgt_fuse, (uint64_t)5e6);
	const bool gps_hgt_fusion_timeout  = isTimedOut(_time_last_gps_hgt_fuse, (uint64_t)5e6);
	const bool rng_hgt_fusion_timeout  = isTimedOut(_time_last_rng_hgt_fuse, (uint64_t)5e6);
	const bool ev_hgt_fusion_timeout   = isTimedOut(_time_last_ev_hgt_fuse, (uint64_t)5e6);


	bool request_height_reset = false;


	// check for intermittent data (before pop_first_older_than)
	const baroSample &baro_init = _baro_buffer.get_newest();
	bool baro_hgt_faulty = !isRecent(baro_init.time_us, 2 * BARO_MAX_INTERVAL);

	const gpsSample &gps_init = _gps_buffer.get_newest();
	bool gps_hgt_intermittent = !isRecent(gps_init.time_us, 2 * GPS_MAX_INTERVAL);


	// check the baro height source for consistency and freshness
	const float baro_innov = _state.pos(2) - (- baro_init.hgt + _baro_hgt_offset);
	const bool baro_data_consistent = fabsf(baro_innov) < (sq(_params.baro_noise) + P(9, 9)) * sq(_params.baro_innov_gate);

	// check if vision data is available
	const extVisionSample &ev_init = _ext_vision_buffer.get_newest();
	const bool ev_data_available = isRecent(ev_init.time_us, 2 * EV_MAX_INTERVAL);


	if (hgt_fusion_timeout || continuous_bad_accel_hgt) {
		// check if GPS height is available
		const gpsSample &gps_init = _gps_buffer.get_newest();
		const bool gps_hgt_accurate = (gps_init.vacc < _params.req_vacc);

		// reset to GPS if adequate GPS data is available and the timeout cannot be blamed on IMU data
		const bool reset_to_gps = !gps_hgt_intermittent && ((gps_hgt_accurate && !prev_bad_vert_accel) || baro_hgt_faulty);

		// if baro data is acceptable and GPS data is inaccurate, reset height to baro
		const bool reset_to_baro = !baro_hgt_faulty && ((baro_data_consistent && !gps_hgt_accurate) || gps_hgt_intermittent);
	}

	// Reset vertical position and velocity states to the last measurement
	if (request_height_reset) {
		// reset the vertical position
		if (_control_status.flags.rng_hgt) {

			// a fallback from any other height source to rangefinder happened
			if (!_control_status_prev.flags.rng_hgt) {

				if (_control_status.flags.in_air && isTerrainEstimateValid()) {
					_rng_hgt_offset = _terrain_vpos;

				} else if (_control_status.flags.in_air) {
					_rng_hgt_offset = _range_sensor.getDistBottom() + _state.pos(2);

				} else {
					_rng_hgt_offset = _params.rng_gnd_clearance;
				}

			}

			// update the state and associated variance
			resetVerticalPositionTo(_rng_hgt_offset - _range_sensor.getDistBottom());

			// the state variance is the same as the observation
			P.uncorrelateCovarianceSetVariance<1>(9, sq(_params.range_noise));

			// reset the height sensor offsets which is subtracted from the readings if we need to use it as a backup
			_baro_hgt_offset = (_baro_hgt_counter == 0) ? NAN : (_state.pos(2) + _baro_hgt_lpf.getState());
			_gps_hgt_offset  = (_gps_hgt_counter == 0)  ? NAN : (_state.pos(2) + _gps_hgt_lpf.getState() - _gps_alt_ref);
			_ev_hgt_offset   = (_ev_hgt_counter == 0)   ? NAN : (_state.pos(2) + _ev_hgt_lpf.getState());

		} else if (_control_status.flags.baro_hgt) {

			if (!baro_hgt_faulty) {
				// initialize vertical position with newest baro measurement
				resetVerticalPositionTo(-_baro_hgt_lpf.getState() + _baro_hgt_offset);

				// the state variance is the same as the observation
				P.uncorrelateCovarianceSetVariance<1>(9, sq(_params.baro_noise));

			} else {
				// TODO: reset to last known baro based estimate
			}

			// reset the height sensor offsets which is subtracted from the readings if we need to use it as a backup
			_gps_hgt_offset  = (_gps_hgt_counter == 0)  ? NAN : (_state.pos(2) + _gps_hgt_lpf.getState() - _gps_alt_ref);
			_rng_hgt_offset  = (_rng_hgt_counter == 0)  ? NAN : (_state.pos(2) + _range_sensor.getDistBottom());
			_ev_hgt_offset   = (_ev_hgt_counter == 0)   ? NAN : (_state.pos(2) + _ev_hgt_lpf.getState());

		} else if (_control_status.flags.gps_hgt) {
			// initialize vertical position and velocity with newest gps measurement
			if (!gps_hgt_intermittent) {
				gpsSample gps_sample;

				if (!_gps_buffer.peek_first_older_than(_imu_sample_delayed.time_us, &gps_sample)) {
					gps_sample = _gps_buffer.get_newest();
				}

				resetVerticalPositionTo(_gps_hgt_offset - _gps_hgt_lpf.getState() + _gps_alt_ref);

				// the state variance is the same as the observation
				P.uncorrelateCovarianceSetVariance<1>(9, sq(gps_sample.vacc));

			} else {
				// TODO: reset to last known gps based estimate
			}

			// reset the height sensor offsets which is subtracted from the readings if we need to use it as a backup
			_baro_hgt_offset = (_baro_hgt_counter == 0) ? NAN : (_state.pos(2) + _baro_hgt_lpf.getState());
			_rng_hgt_offset  = (_rng_hgt_counter == 0)  ? NAN : (_state.pos(2) + _range_sensor.getDistBottom());
			_ev_hgt_offset   = (_ev_hgt_counter == 0)   ? NAN : (_state.pos(2) + _ev_hgt_lpf.getState());

		} else if (_control_status.flags.ev_hgt) {
			extVisionSample ext_vision_sample;

			if (!_ext_vision_buffer.peek_first_older_than(_imu_sample_delayed.time_us, &ext_vision_sample)) {
				ext_vision_sample = _ext_vision_buffer.get_newest();
			}

			resetVerticalPositionTo(_ev_hgt_lpf.getState() + _ev_hgt_offset);

			P.uncorrelateCovarianceSetVariance<1>(9, sq(ext_vision_sample.posVar(2)));

			// reset the height sensor offsets which is subtracted from the readings if we need to use it as a backup
			_baro_hgt_offset = (_baro_hgt_counter == 0) ? NAN : (_state.pos(2) + _baro_hgt_lpf.getState());
			_gps_hgt_offset  = (_gps_hgt_counter == 0)  ? NAN : (_state.pos(2) + _gps_hgt_lpf.getState() - _gps_alt_ref);
			_rng_hgt_offset  = (_rng_hgt_counter == 0)  ? NAN : (_state.pos(2) + _range_sensor.getDistBottom());
		}

		// reset the vertical velocity state
		if (_control_status.flags.gps && !gps_hgt_intermittent) {
			gpsSample gps_sample;

			if (!_gps_buffer.peek_first_older_than(_imu_sample_delayed.time_us, &gps_sample)) {
				gps_sample = _gps_buffer.get_newest();
			}

			// If we are using GPS, then use it to reset the vertical velocity
			resetVerticalVelocityTo(gps_sample.vel(2));

			// the state variance is the same as the observation
			P.uncorrelateCovarianceSetVariance<1>(6, sq(1.5f * gps_sample.sacc));

		} else if (_control_status.flags.ev_vel) {
			extVisionSample ext_vision_sample;

			if (!_ext_vision_buffer.peek_first_older_than(_imu_sample_delayed.time_us, &ext_vision_sample)) {
				ext_vision_sample = _ext_vision_buffer.get_newest();
			}

			// use the most recent data if it's time offset from the fusion time horizon is smaller
			resetVerticalVelocityTo(ext_vision_sample.vel(2));

			P.uncorrelateCovarianceSetVariance<1>(6, math::max(getVisionVelocityVarianceInEkfFrame()(2), sq(0.05f)));

		} else {
			// we don't know what the vertical velocity is, so set it to zero
			resetVerticalVelocityTo(0.0f);

			// Set the variance to a value large enough to allow the state to converge quickly
			// that does not destabilise the filter
			P.uncorrelateCovarianceSetVariance<1>(6, 10.0f);
		}

	}



	// Baro bias estimation using GPS altitude
	if (_baro_data_ready) {
		const float dt = math::constrain(1e-6f * _delta_time_baro_us, 0.0f, 1.0f);
		_baro_b_est.setMaxStateNoise(_params.baro_noise);
		_baro_b_est.setProcessNoiseStdDev(_params.baro_drift_rate);
		_baro_b_est.predict(dt);
	}

	if (_gps_data_ready && !gps_hgt_intermittent
	    && _gps_checks_passed && _NED_origin_initialised
	    && !baro_hgt_faulty) {
		// Use GPS altitude as a reference to compute the baro bias measurement
		const float baro_bias = (_baro_sample_delayed.hgt - _baro_hgt_offset)
					- (_gps_sample_delayed.hgt - _gps_alt_ref);
		const float baro_bias_var = getGpsHeightVariance() + sq(_params.baro_noise);
		_baro_b_est.fuseBias(baro_bias, baro_bias_var);
	}

	// baro
	if (_baro_data_ready && !baro_hgt_faulty) {

		if (!_control_status.flags.baro_hgt) {
			startBaroHgtFusion();
		}

		fuseBaroHgt();
	}

	// GPS
	if (_gps_data_ready && !gps_hgt_intermittent && _gps_checks_passed) {

		if (!_control_status.flags.gps_hgt) {
			startGpsHgtFusion();
		}

		fuseGpsHgt();
	}

	// range
	if (_params.range_aid == 1) {
		checkRangeAidSuitability();

		if (!_control_status.flags.rng_hgt && isRangeAidSuitable()) {
			startRngAidHgtFusion();
		}

		fuseRngHgt();
	}

	// vision
	if (_ev_data_ready) {
		if (!_control_status.flags.ev_hgt) {
			startEvHgtFusion();
		}

		fuseEvHgt();
	}


	_innov_check_fail_status.flags.reject_ver_pos = _innov_check_fail_status.flags.reject_ver_pos_baro
		|| _innov_check_fail_status.flags.reject_ver_pos_gps
		|| _innov_check_fail_status.flags.reject_ver_pos_rng
		|| _innov_check_fail_status.flags.reject_ver_pos_ev;
}

void Ekf::checkRangeAidSuitability()
{
	if (_control_status.flags.in_air
	    && _range_sensor.isHealthy()
	    && isTerrainEstimateValid()) {
		// check if we can use range finder measurements to estimate height, use hysteresis to avoid rapid switching
		// Note that the 0.7 coefficients and the innovation check are arbitrary values but work well in practice
		const float range_hagl = _terrain_vpos - _state.pos(2);
		const float range_hagl_max = _is_range_aid_suitable ? _params.max_hagl_for_range_aid : (_params.max_hagl_for_range_aid * 0.7f);
		const bool is_in_range = range_hagl < range_hagl_max;

		const float hagl_test_ratio = (_hagl_innov * _hagl_innov / (sq(_params.range_aid_innov_gate) * _hagl_innov_var));
		const bool is_hagl_stable = _is_range_aid_suitable ? (hagl_test_ratio < 1.f) : (hagl_test_ratio < 0.01f);

		if (isHorizontalAidingActive()) {
			const float max_vel = _is_range_aid_suitable ? _params.max_vel_for_range_aid : (_params.max_vel_for_range_aid * 0.7f);
			const bool is_below_max_speed = !_state.vel.xy().longerThan(max_vel);

			_is_range_aid_suitable = is_in_range && is_hagl_stable && is_below_max_speed;

		} else {
			_is_range_aid_suitable = is_in_range && is_hagl_stable;
		}

	} else {
		_is_range_aid_suitable = false;
	}
}

void Ekf::controlAirDataFusion()
{
	// control activation and initialisation/reset of wind states required for airspeed fusion

	// If both airspeed and sideslip fusion have timed out and we are not using a drag observation model then we no longer have valid wind estimates
	const bool airspeed_timed_out = isTimedOut(_time_last_arsp_fuse, (uint64_t)10e6);
	const bool sideslip_timed_out = isTimedOut(_time_last_beta_fuse, (uint64_t)10e6);

	if (_using_synthetic_position || (airspeed_timed_out && sideslip_timed_out && !(_params.fusion_mode & MASK_USE_DRAG))) {
		_control_status.flags.wind = false;
	}

	if (_params.arsp_thr <= 0.f) {
		stopAirspeedFusion();
		return;
	}

	if (_tas_data_ready) {
		const bool continuing_conditions_passing = _control_status.flags.in_air && _control_status.flags.fixed_wing && !_using_synthetic_position;
		const bool is_airspeed_significant = _airspeed_sample_delayed.true_airspeed > _params.arsp_thr;
		const bool starting_conditions_passing = continuing_conditions_passing && is_airspeed_significant;

		if (_control_status.flags.fuse_aspd) {
			if (continuing_conditions_passing) {
				if (is_airspeed_significant) {
					fuseAirspeed();
				}

				const bool is_fusion_failing = isTimedOut(_time_last_arsp_fuse, (uint64_t)10e6);

				if (is_fusion_failing) {
					stopAirspeedFusion();
				}

			} else {
				stopAirspeedFusion();
			}

		} else if (starting_conditions_passing) {
			startAirspeedFusion();
		}

	} else if (_control_status.flags.fuse_aspd && (_imu_sample_delayed.time_us - _airspeed_sample_delayed.time_us > (uint64_t) 1e6)) {
		ECL_WARN("Airspeed data stopped");
		stopAirspeedFusion();
	}
}

void Ekf::controlBetaFusion()
{
	if (_using_synthetic_position) {
		return;
	}

	// Perform synthetic sideslip fusion at regular intervals when in-air and sideslip fuson had been enabled externally:
	const bool beta_fusion_time_triggered = isTimedOut(_time_last_beta_fuse, _params.beta_avg_ft_us);

	if (beta_fusion_time_triggered &&
	    _control_status.flags.fuse_beta &&
	    _control_status.flags.in_air) {
		// If starting wind state estimation, reset the wind states and covariances before fusing any data
		if (!_control_status.flags.wind) {
			// activate the wind states
			_control_status.flags.wind = true;
			// reset the timeout timers to prevent repeated resets
			_time_last_beta_fuse = _time_last_imu;
			resetWind();
		}

		fuseSideslip();
	}
}

void Ekf::controlDragFusion()
{
	if ((_params.fusion_mode & MASK_USE_DRAG) &&
	    !_using_synthetic_position &&
	    _control_status.flags.in_air &&
	    !_mag_inhibit_yaw_reset_req) {

		if (!_control_status.flags.wind) {
			// reset the wind states and covariances when starting drag accel fusion
			_control_status.flags.wind = true;
			resetWind();

		} else if (_drag_buffer.pop_first_older_than(_imu_sample_delayed.time_us, &_drag_sample_delayed)) {
			fuseDrag();
		}

	}
}

void Ekf::controlFakePosFusion()
{
	// if we aren't doing any aiding, fake position measurements at the last known position to constrain drift
	// Coincide fake measurements with baro data for efficiency with a minimum fusion rate of 5Hz

	if (!isHorizontalAidingActive()
	    && !(_control_status.flags.fuse_aspd && _control_status.flags.fuse_beta)) {

		// We now need to use a synthetic position observation to prevent unconstrained drift of the INS states.
		_using_synthetic_position = true;

		// Fuse synthetic position observations every 200msec
		if (isTimedOut(_time_last_fake_pos, (uint64_t)2e5)) {

			// Reset position and velocity states if we re-commence this aiding method
			if (isTimedOut(_time_last_fake_pos, (uint64_t)4e5)) {
				_last_known_posNE = _state.pos.xy();
				resetHorizontalPosition();
				resetVelocity();
				_fuse_hpos_as_odom = false;

				if (_time_last_fake_pos != 0) {
					_warning_events.flags.stopping_navigation = true;
					ECL_WARN("stopping navigation");
				}

			}

			_time_last_fake_pos = _time_last_imu;

			Vector3f fake_pos_obs_var;

			if (_control_status.flags.in_air && _control_status.flags.tilt_align) {
				fake_pos_obs_var(0) = fake_pos_obs_var(1) = sq(fmaxf(_params.pos_noaid_noise, _params.gps_pos_noise));

			} else if (_control_status.flags.vehicle_at_rest) {
				// Accelerate tilt fine alignment by fusing more
				// aggressively when the vehicle is at rest
				fake_pos_obs_var(0) = fake_pos_obs_var(1) = sq(0.1f);

			} else {
				fake_pos_obs_var(0) = fake_pos_obs_var(1) = sq(0.5f);
			}

			_gps_pos_innov.xy() = Vector2f(_state.pos) - _last_known_posNE;

			const Vector2f fake_pos_innov_gate(3.0f, 3.0f);

			fuseHorizontalPosition(_gps_pos_innov, fake_pos_innov_gate, fake_pos_obs_var,
					       _gps_pos_innov_var, _gps_pos_test_ratio, true);
		}

	} else {
		_using_synthetic_position = false;
	}

}

void Ekf::controlAuxVelFusion()
{
	const bool data_ready = _auxvel_buffer.pop_first_older_than(_imu_sample_delayed.time_us, &_auxvel_sample_delayed);

	if (data_ready && isHorizontalAidingActive()) {

		const Vector2f aux_vel_innov_gate(_params.auxvel_gate, _params.auxvel_gate);

		_last_vel_obs = _auxvel_sample_delayed.vel;
		_aux_vel_innov = _state.vel - _last_vel_obs;
		_last_vel_obs_var = _aux_vel_innov_var;

		fuseHorizontalVelocity(_aux_vel_innov, aux_vel_innov_gate, _auxvel_sample_delayed.velVar,
				       _aux_vel_innov_var, _aux_vel_test_ratio);

		// Can be enabled after bit for this is added to EKF_AID_MASK
		// fuseVerticalVelocity(_aux_vel_innov, aux_vel_innov_gate, _auxvel_sample_delayed.velVar,
		//		_aux_vel_innov_var, _aux_vel_test_ratio);

	}
}


bool Ekf::isVelStateAlignedWithObs() const
{
	/* Do sanity check to see if the innovation failures is likely caused by a yaw angle error
	 * by measuring the angle between the velocity estimate and the last velocity observation
	 * Only use those vectors if their norm if they are larger than 4 times their noise standard deviation
	 */
	const float vel_obs_xy_norm_sq = _last_vel_obs.xy().norm_squared();
	const float vel_state_xy_norm_sq = _state.vel.xy().norm_squared();

	const float vel_obs_threshold_sq = fmaxf(sq(4.f) * (_last_vel_obs_var(0) + _last_vel_obs_var(1)), sq(0.4f));
	const float vel_state_threshold_sq = fmaxf(sq(4.f) * (P(4, 4) + P(5, 5)), sq(0.4f));

	if (vel_obs_xy_norm_sq > vel_obs_threshold_sq && vel_state_xy_norm_sq > vel_state_threshold_sq) {
		const float obs_dot_vel = Vector2f(_last_vel_obs).dot(_state.vel.xy());
		const float cos_sq = sq(obs_dot_vel) / (vel_state_xy_norm_sq * vel_obs_xy_norm_sq);

		if (cos_sq < sq(cosf(math::radians(25.f))) || obs_dot_vel < 0.f) {
			// The angle between the observation and the velocity estimate is greater than 25 degrees
			return false;
		}
	}

	return true;
}

bool Ekf::hasHorizontalAidingTimedOut() const
{
	return isTimedOut(_time_last_hor_pos_fuse, _params.reset_timeout_max)
	       && isTimedOut(_time_last_delpos_fuse, _params.reset_timeout_max)
	       && isTimedOut(_time_last_hor_vel_fuse, _params.reset_timeout_max)
	       && isTimedOut(_time_last_of_fuse, _params.reset_timeout_max);
}

void Ekf::processVelPosResetRequest()
{
	if (_velpos_reset_request) {
		resetVelocity();
		resetHorizontalPosition();
		_velpos_reset_request = false;

		// Reset the timeout counters
		_time_last_hor_pos_fuse = _time_last_imu;
		_time_last_hor_vel_fuse = _time_last_imu;
	}
}
