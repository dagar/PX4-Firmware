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

void Ekf::controlMag3dFusion(const magSample &mag_sample, estimator_aid_source3d_s &aid_src)
{
	static constexpr const char *AID_SRC_NAME = "Mag 3D";

	resetEstimatorAidStatus(aid_src);
	aid_src.timestamp_sample = mag_sample.time_us;

	checkYawAngleObservability();
	checkMagBiasObservability();

	if (_mag_bias_observable || _yaw_angle_observable) {
		_time_last_mov_3d_mag_suitable = _imu_sample_delayed.time_us;
	}

	// sensor or calibration has changed, clear any mag bias and reset low pass filter
	if (mag_sample.reset) {
		// Zero the magnetometer bias states
		_state.mag_B.zero();

		// Zero the corresponding covariances and set
		// variances to the values use for initial alignment
		P.uncorrelateCovarianceSetVariance<3>(19, sq(_params.mag_noise));

		// reset any saved covariance data for re-use when auto-switching between heading and 3-axis fusion
		_saved_mag_bf_covmat.zero();
	}

	// determine if we should use mag 3d
	// Use of 3D fusion requires an in-air heading alignment but it should not
	// be used when the heading and mag biases are not observable for more than 2 seconds
	bool continuing_conditions_passing = (_params.mag_ctrl & static_cast<int32_t>(MagCtrl::3D))
					     && _control_status.flags.tilt_align
					     && _control_status.flags.yaw_align
					     && isRecent(_time_last_mov_3d_mag_suitable, (uint64_t)2e6)
					     && mag_sample.mag.isAllFinite();

	const bool starting_conditions_passing = continuing_conditions_passing
			&& (Vector3f(aid_src.test_ratio).max() < 0.1f);


	// mag declination
	{
		// if we are using 3-axis magnetometer fusion, but without external NE aiding,
		// then the declination must be fused as an observation to prevent long term heading drift
		// fusing declination when gps aiding is available is optional, but recommended to prevent
		// problem if the vehicle is static for extended periods of time
		const bool mag_declination_user_selected = (_params.mag_declination_source & GeoDeclinationMask::FUSE_DECL);
		const bool not_using_ne_aiding = !_control_status.flags.gps;
		_control_status.flags.mag_dec = _control_status.flags.mag && ((not_using_ne_aiding || mag_declination_user_selected));
	}

	if (_control_status.flags.mag) {
		aid_src.fusion_enabled = true;

		if (continuing_conditions_passing) {

			if (!_mag_decl_cov_reset) {
				if (_control_status.flags.mag) {
					// After any magnetic field covariance reset event the earth field state
					// covariances need to be corrected to incorporate knowledge of the declination
					// before fusing magnetomer data to prevent rapid rotation of the earth field
					// states for the first few observations.
					fuseDeclination(0.02f);
					_mag_decl_cov_reset = true;
				}

				fuseMag(mag_sample.mag, aid_src);

			} else {
				// For the first few seconds after in-flight alignment we allow the magnetic field state estimates to stabilise
				// before they are used to constrain heading drift
				const bool update_all_states = ((_imu_sample_delayed.time_us - _flt_mag_align_start_time) > (uint64_t)5e6)
							       && _control_status.flags.mag_aligned_in_flight
							       && !_control_status.flags.mag_fault
							       && !magFieldStrengthDisturbed(mag_sample.mag - _state.mag_B);

				_control_status.flags.mag_3D = update_all_states;
				// TODO: check variances and remove 5s

				// The normal sequence is to fuse the magnetometer data first before fusing
				// declination angle at a higher uncertainty to allow some learning of
				// declination angle over time.
				fuseMag(mag_sample.mag, aid_src, update_all_states);

				if (_control_status.flags.mag_dec) {
					fuseDeclination(0.5f);
				}
			}

			const bool is_fusion_failing = isTimedOut(aid_src.time_last_fuse, _params.no_aid_timeout_max); // 1 second

			if (is_fusion_failing) {

				if (_nb_mag_reset_available > 0) {
					// Data seems good, attempt a reset

					//ECL_WARN("%s fusion failing, resetting", AID_SRC_NAME);

					aid_src.time_last_fuse = _imu_sample_delayed.time_us;

					if (_control_status.flags.in_air) {
						_nb_mag_reset_available--;
					}

				} else if (starting_conditions_passing) {
					// Data seems good, but previous reset did not fix the issue
					// something else must be wrong, declare the sensor faulty and stop the fusion
					//_control_status.flags.mag_fault = true;
					ECL_WARN("stopping %s fusion, starting conditions failing", AID_SRC_NAME);
					stopMagFusion();

				} else {
					// A reset did not fix the issue but all the starting checks are not passing
					// This could be a temporary issue, stop the fusion without declaring the sensor faulty
					ECL_WARN("stopping %s, fusion failing", AID_SRC_NAME);
					stopMagFusion();
				}
			}

		} else {
			// Stop fusion but do not declare it faulty
			ECL_WARN("stopping %s fusion, continuing conditions failing", AID_SRC_NAME);
			stopMagFusion();
		}

	} else {
		if (starting_conditions_passing) {
			loadMagCovData();

			if (!_control_status.flags.yaw_align) {
				magYawReset(_mag_lpf.getState());
			}



			// TODO: _control_status.flags.mag_aligned_in_flight?


			// activate fusion, only reset if necessary
			ECL_INFO("starting %s fusion", AID_SRC_NAME);

			aid_src.time_last_fuse = _imu_sample_delayed.time_us;

			_nb_mag_reset_available = 5;
			_control_status.flags.mag = true;
		}
	}
}

void Ekf::stopMagFusion()
{
	if (_control_status.flags.mag || _control_status.flags.mag_3D) {
		ECL_INFO("stopping mag fusion");

		_control_status.flags.mag = false;
		_control_status.flags.mag_3D = false;

		saveMagCovData();

		_control_status.flags.mag_dec = false;

		_fault_status.flags.bad_mag_x = false;
		_fault_status.flags.bad_mag_y = false;
		_fault_status.flags.bad_mag_z = false;

		_fault_status.flags.bad_mag_decl = false;
	}
}

void Ekf::checkYawAngleObservability()
{
	if (_control_status.flags.gps || (_control_status.flags.yaw_align && _control_status.flags.ev_pos)) {
		// Check if there has been enough change in horizontal velocity to make yaw observable
		// Apply hysteresis to check to avoid rapid toggling
		const float accel_NE_threshold = _yaw_angle_observable ? _params.mag_acc_gate : 2.f * _params.mag_acc_gate;

		_yaw_angle_observable = _accel_lpf_NE.norm() > accel_NE_threshold;

	} else {
		_yaw_angle_observable = false;
	}
}

void Ekf::checkMagBiasObservability()
{
	// check if there is enough yaw rotation to make the mag bias states observable
	if (!_mag_bias_observable && (fabsf(_yaw_rate_lpf_ef) > _params.mag_yaw_rate_gate)) {
		// initial yaw motion is detected
		_mag_bias_observable = true;

	} else if (_mag_bias_observable) {
		// require sustained yaw motion of 50% the initial yaw rate threshold
		const float yaw_dt = 1e-6f * (float)(_imu_sample_delayed.time_us - _time_yaw_started);
		const float min_yaw_change_req = 0.5f * _params.mag_yaw_rate_gate * yaw_dt;
		_mag_bias_observable = fabsf(_yaw_delta_ef) > min_yaw_change_req;
	}

	_yaw_delta_ef = 0.0f;
	_time_yaw_started = _imu_sample_delayed.time_us;
}
