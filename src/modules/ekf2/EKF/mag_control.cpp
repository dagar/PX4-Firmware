/****************************************************************************
 *
 *   Copyright (c) 2019 Estimation and Control Library (ECL). All rights reserved.
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
 * @file mag_control.cpp
 * Control functions for ekf magnetic field fusion
 */

#include "ekf.h"
#include <mathlib/mathlib.h>

void Ekf::controlMagFusion()
{
	if (_params.mag_fusion_type >= MagFuseType::NONE) {
		stopMagFusion();
		return;
	}

	// If we are on ground, reset the flight alignment flag so that the mag fields will be
	// re-initialised next time we achieve flight altitude
	if (!_control_status_prev.flags.in_air && _control_status.flags.in_air) {
		_control_status.flags.mag_aligned_in_flight = false;
	}

	magSample mag_sample;

	if (_mag_buffer && _mag_buffer->pop_first_older_than(_imu_sample_delayed.time_us, &mag_sample)) {

		_mag_lpf.update(mag_sample.mag);
		_mag_counter++;

		// if enabled, use knowledge of theoretical magnetic field vector to calculate a synthetic magnetomter Z component value.
		// this is useful if there is a lot of interference on the sensor measurement.
		if (_params.synthesize_mag_z && (_params.mag_declination_source & GeoDeclinationMask::USE_GEO_DECL)
		    && (_NED_origin_initialised || PX4_ISFINITE(_mag_declination_gps))
		   ) {
			const Vector3f mag_earth_pred = Dcmf(Eulerf(0, -_mag_inclination_gps,
							     _mag_declination_gps)) * Vector3f(_mag_strength_gps, 0, 0);
			mag_sample.mag(2) = calculate_synthetic_mag_z_measurement(mag_sample.mag, mag_earth_pred);
			_control_status.flags.synthetic_mag_z = true;

		} else {
			_control_status.flags.synthetic_mag_z = false;
		}

		yawAngleObservable();
		checkMagBiasObservability();

		if (_mag_bias_observable || _yaw_angle_observable) {
			_time_last_mov_3d_mag_suitable = _imu_sample_delayed.time_us;
		}

		if (_control_status.flags.tilt_align && !_control_status.flags.ev_yaw && !_control_status.flags.gps_yaw) {
			// Determine if we should use simple magnetic heading fusion which works better when
			// there are large external disturbances or the more accurate 3-axis fusion
			switch (_params.mag_fusion_type) {
			default:

			// FALLTHROUGH
			case MagFuseType::AUTO:

				// Use of 3D fusion requires an in-air heading alignment but it should not
				// be used when the heading and mag biases are not observable for more than 2 seconds
				if (_control_status.flags.mag_aligned_in_flight && isRecent(_time_last_mov_3d_mag_suitable, (uint64_t)2e6)) {
					startMag3DFusion();

				} else {
					startMagHdgFusion();
				}

				break;

			/* fallthrough */
			case MagFuseType::HEADING:
				startMagHdgFusion();
				break;

			case MagFuseType::MAG_3D:
				startMag3DFusion();
				break;
			}
		}

		bool mag_yaw_reset_req = false;

		const bool mag_enabled = (_control_status.flags.mag_hdg || _control_status.flags.mag_3D_orientation);

		// We need to reset the yaw angle after climbing away from the ground to enable
		// recovery from ground level magnetic interference.
		if (mag_enabled
		    && _control_status.flags.in_air
		    && !_control_status.flags.mag_aligned_in_flight
		    && isTerrainEstimateValid()
		   ) {
			// Check if height has increased sufficiently to be away from ground magnetic anomalies
			// and request a yaw reset if not already requested.
			static constexpr float mag_anomalies_max_hagl = 1.5f;
			mag_yaw_reset_req = (getTerrainVPos() - _state.pos(2)) > mag_anomalies_max_hagl;
		}

		if (mag_enabled && (!_control_status.flags.yaw_align || mag_yaw_reset_req)) {

			const Vector3f mag_init_bias_corrected = _mag_lpf.getState() - _state.mag_B;

			if (!magFieldStrengthDisturbed(mag_init_bias_corrected)) {
				resetMagHeading(mag_init_bias_corrected);

			} else if (!magFieldStrengthDisturbed(_mag_lpf.getState())) {
				resetMagHeading(_mag_lpf.getState());
				_state.mag_B.zero();
			}
		}


		{
			resetEstimatorAidStatus(_aid_src_mag);
			_aid_src_mag.timestamp_sample = mag_sample.time_us;
			_aid_src_mag.fusion_enabled = _control_status.flags.mag_3D_orientation;

			// compute magnetometer innovations (for estimator_aid_src_mag logging)
			//  rotate magnetometer earth field state into body frame
			const Vector3f mag_observation = mag_sample.mag - _state.mag_B;
			const Vector3f mag_I_body = _state.quat_nominal.rotateVectorInverse(_state.mag_I);
			const Vector3f mag_innov = mag_I_body - mag_observation;

			mag_observation.copyTo(_aid_src_mag.observation);
			mag_innov.copyTo(_aid_src_mag.innovation);
		}

		if (_control_status.flags.yaw_align && isRecent(_time_last_mov_3d_mag_suitable, (uint64_t)2e6)) {

			// if we are using 3-axis magnetometer fusion, but without external NE aiding,
			// then the declination must be fused as an observation to prevent long term heading drift
			// fusing declination when gps aiding is available is optional, but recommended to prevent
			// problem if the vehicle is static for extended periods of time
			const bool mag_decl_user_selected = (_params.mag_declination_source & GeoDeclinationMask::FUSE_DECL);
			const bool not_using_ne_aiding = !_control_status.flags.gps;
			_control_status.flags.mag_dec = (_control_status.flags.mag_3D && (not_using_ne_aiding || mag_decl_user_selected));

			// For the first few seconds after in-flight alignment we allow the magnetic field state estimates to stabilise
			// before they are used to constrain heading drift
			const bool update_all_states = _control_status.flags.mag_3D
						       && (P(19, 19) < 0.001f) && (P(20, 20) < 0.001f) && (P(21, 21) < 0.001f)
						       && _control_status.flags.mag_aligned_in_flight
						       && !_control_status.flags.mag_fault
						       && !magFieldStrengthDisturbed(mag_sample.mag - _state.mag_B);

			if (!_mag_decl_cov_reset) {
				// After any magnetic field covariance reset event the earth field state
				// covariances need to be corrected to incorporate knowledge of the declination
				// before fusing magnetomer data to prevent rapid rotation of the earth field
				// states for the first few observations.
				fuseDeclination(0.02f);
				_mag_decl_cov_reset = true;
				fuseMag(mag_sample.mag, _aid_src_mag, update_all_states);

			} else {
				// The normal sequence is to fuse the magnetometer data first before fusing
				// declination angle at a higher uncertainty to allow some learning of
				// declination angle over time.
				fuseMag(mag_sample.mag, _aid_src_mag, update_all_states);

				if (_control_status.flags.mag_dec) {
					fuseDeclination(0.5f);
				}

				if (_control_status.flags.mag_fault && magFieldStrengthDisturbed(mag_sample.mag - _state.mag_B)) {

					if ((P(16, 16) < sq(0.01f)) && (P(17, 17) < sq(0.01f)) && (P(18, 18) < sq(0.01f))
					 && (P(19, 19) < sq(0.01f)) && (P(20, 20) < sq(0.01f)) && (P(21, 21) < sq(0.01f))
					) {
						if ((_aid_src_mag.test_ratio[0] < 0.1f)
						&& (_aid_src_mag.test_ratio[1] < 0.1f)
						&& (_aid_src_mag.test_ratio[2] < 0.1f)
						) {
							// clear mag fault
							_control_status.flags.mag_fault = false;
						}

					}
				}
			}
		}


		// mag heading
		resetEstimatorAidStatus(_aid_src_mag_heading);
		_aid_src_mag_heading.timestamp_sample = mag_sample.time_us;
		_aid_src_mag_heading.fusion_enabled = _control_status.flags.mag_hdg;

		// compute mag heading innovation (for estimator_aid_src_mag_heading logging)
		const Vector3f mag_observation = mag_sample.mag - _state.mag_B;

		// Rotate the measurements into earth frame using the zero yaw angle
		const Dcmf R_to_earth = updateYawInRotMat(0.f, _R_to_earth);
		const Vector3f mag_earth_pred = R_to_earth * mag_observation;

		//  the angle of the projection onto the horizontal gives the yaw angle
		//  calculate the yaw innovation and wrap to the interval between +-pi
		float measured_hdg = -atan2f(mag_earth_pred(1), mag_earth_pred(0)) + getMagDeclination();
		float innovation = wrap_pi(getEulerYaw(_R_to_earth) - measured_hdg);
		float heading_obs_var = fmaxf(sq(_params.mag_heading_noise), sq(0.01f));

		_control_status.flags.mag_field_disturbed = magFieldStrengthDisturbed(mag_observation);

		_aid_src_mag_heading.observation = measured_hdg;
		_aid_src_mag_heading.observation_variance = heading_obs_var;
		_aid_src_mag_heading.innovation = innovation;

		if (_control_status.flags.yaw_align
		    && _control_status.flags.mag_hdg
		    && !_control_status.flags.mag_fault
		    && !_control_status.flags.mag_field_disturbed
		   ) {
			fuseYaw(innovation, heading_obs_var, _aid_src_mag_heading);
		}
	}
}

bool Ekf::yawAngleObservable()
{
	if (_control_status.flags.yaw_align && (_control_status.flags.gps || _control_status.flags.ev_pos)) {
		// Check if there has been enough change in horizontal velocity to make yaw observable
		float mag_acc_gate = _params.mag_acc_gate;

		if (!_yaw_angle_observable) {
			// Apply hysteresis to check to avoid rapid toggling
			mag_acc_gate *= 2;
		}

		_yaw_angle_observable = (_accel_lpf_NE.norm() > mag_acc_gate);

	} else {
		_yaw_angle_observable = false;
	}

	return _yaw_angle_observable;
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

bool Ekf::magFieldStrengthDisturbed(const Vector3f &mag_sample) const
{
	if (_params.check_mag_strength) {

		if (PX4_ISFINITE(_mag_strength_gps)) {
			constexpr float wmm_gate_size = 0.2f; // +/- Gauss
			return !isMeasuredMatchingExpected(mag_sample.length(), _mag_strength_gps, wmm_gate_size);

		} else {
			constexpr float average_earth_mag_field_strength = 0.45f; // Gauss
			constexpr float average_earth_mag_gate_size = 0.40f; // +/- Gauss
			return !isMeasuredMatchingExpected(mag_sample.length(), average_earth_mag_field_strength, average_earth_mag_gate_size);
		}
	}

	return false;
}

bool Ekf::isMeasuredMatchingExpected(const float measured, const float expected, const float gate)
{
	return (measured >= expected - gate)
	       && (measured <= expected + gate);
}

bool Ekf::resetMagHeading(const Vector3f &mag)
{
	ECL_INFO("reset mag heading resetting yaw");

	// rotate the magnetometer measurements into earth frame using a zero yaw angle
	const Dcmf R_to_earth = updateYawInRotMat(0.f, _R_to_earth);

	// the angle of the projection onto the horizontal gives the yaw angle
	const Vector3f mag_earth_pred = R_to_earth * mag;

	// calculate the observed yaw angle and yaw variance
	float yaw_new = -atan2f(mag_earth_pred(1), mag_earth_pred(0)) + getMagDeclination();
	float yaw_new_variance = math::max(sq(_params.mag_heading_noise), sq(0.01f));

	// update quaternion states and corresponding covarainces
	resetQuatStateYaw(yaw_new, yaw_new_variance);

	// set the earth magnetic field states using the updated rotation
	_state.mag_I = _R_to_earth * mag;

	resetMagCov();

	// Handle the special case where we have not been constraining yaw drift or learning yaw bias due
	// to assumed invalid mag field associated with indoor operation with a downwards looking flow sensor.
	if (isTimedOut(_aid_src_mag_heading.time_last_fuse, 5e6)
	    && isTimedOut(_aid_src_mag.time_last_fuse, 5e6)
	    && isTimedOut(_aid_src_gnss_yaw.time_last_fuse, 5e6)
	    && isTimedOut(_aid_src_ev_yaw.time_last_fuse, 5e6)
	   ) {
		// Zero the gyro yaw bias covariance and set the variance to the initial alignment uncertainty
		resetZDeltaAngBiasCov();
	}

	_aid_src_mag_heading.time_last_fuse = _imu_sample_delayed.time_us;

	_control_status.flags.yaw_align = true;

	if (_control_status.flags.in_air) {
		// record the time for the magnetic field alignment event
		_control_status.flags.mag_aligned_in_flight = true;
	}

	return true;
}

bool Ekf::resetMagStates()
{
	bool reset = false;

	// reinit mag states
	const bool mag_available = (_mag_counter != 0) && isNewestSampleRecent(_time_last_mag_buffer_push, 500'000);

	// if world magnetic model (inclination, declination, strength) available then use it to reset mag states
	if (PX4_ISFINITE(_mag_inclination_gps) && PX4_ISFINITE(_mag_declination_gps) && PX4_ISFINITE(_mag_strength_gps)) {
		// use predicted earth field to reset states
		const Vector3f mag_earth_pred = Dcmf(Eulerf(0, -_mag_inclination_gps,
						     _mag_declination_gps)) * Vector3f(_mag_strength_gps, 0, 0);
		_state.mag_I = mag_earth_pred;

		// TODO: ECL_DEBUG
		ECL_INFO("resetting mag I to [%.3f, %.3f, %.3f]",
			 (double)_state.mag_I(0), (double)_state.mag_I(1), (double)_state.mag_I(2));

		if (mag_available) {
			const Dcmf R_to_body = quatToInverseRotMat(_state.quat_nominal);
			_state.mag_B = _mag_lpf.getState() - (R_to_body * mag_earth_pred);

			// TODO: ECL_DEBUG
			ECL_INFO("resetting mag B to [%.3f, %.3f, %.3f]",
				 (double)_state.mag_B(0), (double)_state.mag_B(1), (double)_state.mag_B(2));

		} else {
			_state.mag_B.zero();
		}

		reset = true;

	} else if (mag_available) {
		// Use the last magnetometer measurements to reset the field states

		// calculate initial earth magnetic field states
		_state.mag_I = _R_to_earth * _mag_lpf.getState();
		_state.mag_B.zero();

		// TODO: ECL_DEBUG
		ECL_INFO("resetting mag I to [%.3f, %.3f, %.3f]",
			 (double)_state.mag_I(0), (double)_state.mag_I(1), (double)_state.mag_I(2));

		reset = true;
	}

	if (reset) {
		resetMagCov();

		if (mag_available) {
			if (_control_status.flags.in_air) {
				// record the start time for the magnetic field alignment
				_control_status.flags.mag_aligned_in_flight = true;
			}
		}

		return true;
	}

	return false;
}
