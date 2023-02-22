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
	// reset the flight alignment flag so that the mag fields will be
	// re-initialised next time we achieve flight altitude
	if (!_control_status_prev.flags.in_air && _control_status.flags.in_air) {
		_control_status.flags.mag_aligned_in_flight = false;
	}

	// check mag state observability
	checkYawAngleObservability();
	checkMagBiasObservability();

	if (_mag_bias_observable || _yaw_angle_observable) {
		_time_last_mov_3d_mag_suitable = _time_delayed_us;
	}

	// stop mag (require a reset before using again) if there was an external yaw reset (yaw estimator, GPS yaw, etc) or yaw alignment change
	if (!_control_status.flags.mag && _mag_decl_cov_reset) {
		const bool yaw_estimator_reset = _information_events.flags.yaw_aligned_to_imu_gps;
		const bool heading_was_reset = (_state_reset_status.reset_count.quat != _state_reset_count_prev.quat);

		if (yaw_estimator_reset || heading_was_reset) {
			ECL_INFO("yaw reset, stopping mag fusion to force reset/reinit");
			stopMagFusion();
			resetMagCov();
		}
	}

	magSample mag_sample;

	if (_mag_buffer && _mag_buffer->pop_first_older_than(_time_delayed_us, &mag_sample)) {

		if (mag_sample.reset || (_mag_counter == 0)) {
			// sensor or calibration has changed, reset low pass filter (reset handled by controlMag3DFusion/controlMagHeadingFusion)
			_control_status.flags.mag_fault = false;

			_state.mag_B.zero();
			resetMagCov();

			_mag_lpf.reset(mag_sample.mag);
			_mag_counter = 1;

		} else {
			_mag_lpf.update(mag_sample.mag);
			_mag_counter++;
		}

		// if enabled, use knowledge of theoretical magnetic field vector to calculate a synthetic magnetomter Z component value.
		// this is useful if there is a lot of interference on the sensor measurement.
		if (_params.synthesize_mag_z && (_params.mag_declination_source & GeoDeclinationMask::USE_GEO_DECL)
		    && (PX4_ISFINITE(_mag_inclination_gps) && PX4_ISFINITE(_mag_declination_gps) && PX4_ISFINITE(_mag_strength_gps))
		   ) {
			const Vector3f mag_earth_pred = Dcmf(Eulerf(0, -_mag_inclination_gps, _mag_declination_gps))
							* Vector3f(_mag_strength_gps, 0, 0);

			mag_sample.mag(2) = calculate_synthetic_mag_z_measurement(mag_sample.mag, mag_earth_pred);

			_control_status.flags.synthetic_mag_z = true;

		} else {
			_control_status.flags.synthetic_mag_z = false;
		}

		const bool starting_conditions_passing = (_params.mag_fusion_type != MagFuseType::NONE)
				&& _control_status.flags.tilt_align
				&& !_control_status.flags.mag_fault
				&& _mag_lpf.getState().longerThan(0.1f) // WMM magnitude 0.25-0.65 Gauss
				&& (_mag_counter > 5) // wait until we have a few samples through the filter
				&& (_state_reset_status.reset_count.quat == _state_reset_count_prev.quat) // no yaw reset this frame
				&& (_control_status.flags.yaw_align == _control_status_prev.flags.yaw_align) // no yaw alignment change this frame
				&& !_information_events.flags.yaw_aligned_to_imu_gps // don't allow starting on same frame as yaw estimator reset
				&& isNewestSampleRecent(_time_last_mag_buffer_push, MAG_MAX_INTERVAL);

		// allow mag to yaw align if necessary
		if (!_control_status.flags.yaw_align) {
			if (starting_conditions_passing
			    && !magFieldStrengthDisturbed(_mag_lpf.getState())
			    && !_control_status.flags.ev_yaw
			   ) {
				resetMagHeading(_mag_lpf.getState());
				_control_status.flags.yaw_align = true;
			}
		}

		controlMag3DFusion(mag_sample, starting_conditions_passing, _aid_src_mag);
		controlMagHeadingFusion(mag_sample, starting_conditions_passing, _aid_src_mag_heading);

	} else if (!isNewestSampleRecent(_time_last_mag_buffer_push, 2 * MAG_MAX_INTERVAL)) {
		// No data anymore. Stop until it comes back.
		stopMagHdgFusion();
		stopMagFusion();
	}
}

bool Ekf::checkHaglYawResetReq()
{
	// We need to reset the yaw angle after climbing away from the ground to enable
	// recovery from ground level magnetic interference.
	if (_control_status.flags.in_air && _control_status.flags.yaw_align && !_control_status.flags.mag_aligned_in_flight) {
		// Check if height has increased sufficiently to be away from ground magnetic anomalies
		// and request a yaw reset if not already requested.
		static constexpr float mag_anomalies_max_hagl = 1.5f;
		const bool above_mag_anomalies = (getTerrainVPos() - _state.pos(2)) > mag_anomalies_max_hagl;
		return above_mag_anomalies;
	}

	return false;
}

bool Ekf::resetMagHeading(const Vector3f &mag)
{
	const float R_MAG = math::max(sq(_params.mag_noise), sq(0.01f));

	if (isYawEmergencyEstimateAvailable()
	&& PX4_ISFINITE(_mag_inclination_gps) && PX4_ISFINITE(_mag_declination_gps) && PX4_ISFINITE(_mag_strength_gps)
	) {
		// use expected earth field to reset states
		const Vector3f mag_earth_pred = Dcmf(Eulerf(0, -_mag_inclination_gps, _mag_declination_gps))
						* Vector3f(_mag_strength_gps, 0, 0);

		const Vector3f mag_B_before_reset = _state.mag_B;

		const Dcmf R_to_earth = updateYawInRotMat(_yawEstimator.getYaw(), _R_to_earth);
		_state.mag_B = mag - (R_to_earth.transpose() * mag_earth_pred);

		ECL_INFO("resetMagHeading using yaw estimator to reset mag bias [%.3f, %.3f, %.3f] -> [%.3f, %.3f, %.3f]",
				(double)mag_B_before_reset(0), (double)mag_B_before_reset(1), (double)mag_B_before_reset(2),
				(double)_state.mag_B(0), (double)_state.mag_B(1), (double)_state.mag_B(2)
		);

		P.uncorrelateCovarianceSetVariance<3>(19, R_MAG);

		// save the XYZ body covariance sub-matrix
		_saved_mag_bf_covmat = P.slice<3, 3>(19, 19);
	}

	Vector3f mag_bias{};

	// use mag bias if variance good (unless configured for HEADING only)
	if (_params.mag_fusion_type != MagFuseType::HEADING) {

		const Vector3f mag_bias_var = getMagBiasVariance();

		if ((mag_bias_var.min() > 0.f) && (mag_bias_var.max() <= R_MAG)) {
			mag_bias = _state.mag_B;
		}
	}

	// calculate mag heading
	// Rotate the measurements into earth frame using the zero yaw angle
	const Dcmf R_to_earth = updateYawInRotMat(0.f, _R_to_earth);

	// calculate the observed yaw angle and yaw variance
	// the angle of the projection onto the horizontal gives the yaw angle
	const Vector3f mag_earth_pred = R_to_earth * (mag - mag_bias);
	const float declination = getMagDeclination();

	const float mag_heading = -atan2f(mag_earth_pred(1), mag_earth_pred(0)) + declination;

	ECL_INFO("reset mag heading %.3f -> %.3f rad (declination %.1f)",
		(double)getEulerYaw(_R_to_earth), (double)mag_heading, (double)declination);

	resetQuatStateYaw(mag_heading, R_MAG);
	_mag_heading_last_declination = declination;

	return true;
}

void Ekf::resetMagStates(const Vector3f &mag, bool reset_heading)
{
	const float R_MAG = math::max(sq(_params.mag_noise), sq(0.01f));

	// reinit mag states
	const Vector3f mag_I_before_reset = _state.mag_I;
	const Vector3f mag_B_before_reset = _state.mag_B;

	// if world magnetic model (inclination, declination, strength) available then use it to reset mag states
	if (PX4_ISFINITE(_mag_inclination_gps) && PX4_ISFINITE(_mag_declination_gps) && PX4_ISFINITE(_mag_strength_gps)) {
		// use expected earth field to reset states
		const Vector3f mag_earth_pred = Dcmf(Eulerf(0, -_mag_inclination_gps, _mag_declination_gps))
						* Vector3f(_mag_strength_gps, 0, 0);

		// mag_B: reset
		if (isYawEmergencyEstimateAvailable()) {
			// TODO: review
			const Dcmf R_to_earth = updateYawInRotMat(_yawEstimator.getYaw(), _R_to_earth);
			_state.mag_B = mag - (R_to_earth.transpose() * mag_earth_pred);

			ECL_INFO("resetMagStates using yaw estimator to reset mag bias [%.3f, %.3f, %.3f] -> [%.3f, %.3f, %.3f]",
				 (double)mag_B_before_reset(0), (double)mag_B_before_reset(1), (double)mag_B_before_reset(2),
				 (double)_state.mag_B(0), (double)_state.mag_B(1), (double)_state.mag_B(2)
			);

		} else {
			if (_control_status.flags.yaw_align) {
				const Dcmf R_to_body = quatToInverseRotMat(_state.quat_nominal);
				_state.mag_B = mag - (R_to_body * mag_earth_pred);

			} else {
				_state.mag_B.zero();
			}
		}
		// reset covariances to default
		P.uncorrelateCovarianceSetVariance<3>(19, R_MAG);


		// mag_I: reset, skipped if no change in state and variance good
		constexpr float kResetThreadholdGauss = 0.02f;

		for (int i = 0; i < 3; i++) {
			int state_index = 16 + i;

			// reset state if changed by more than threshold or existing variance is bad
			if ((fabsf(_state.mag_I(i) - mag_earth_pred(i)) > kResetThreadholdGauss)
			    || (P(state_index, state_index) > R_MAG)
			    || (P(state_index, state_index) < sq(0.0001f))
			   ) {
				_state.mag_I(i) = mag_earth_pred(i);
				P.uncorrelateCovarianceSetVariance<1>(state_index, R_MAG);
			}
		}

		if (reset_heading) {
			resetMagHeading(mag);
		}

	} else {
		// Use the last magnetometer measurements to reset the field states
		if (reset_heading) {
			resetMagHeading(mag);
		}

		// calculate initial earth magnetic field states
		const Vector3f mag_earth_pred = _R_to_earth * mag;
		_state.mag_I = mag_earth_pred;

		_state.mag_B.zero();

		// reset to default
		P.uncorrelateCovarianceSetVariance<3>(16, R_MAG);
		P.uncorrelateCovarianceSetVariance<3>(19, R_MAG);
	}

	if (!mag_I_before_reset.longerThan(0.f)) {
		ECL_INFO("initializing mag I [%.3f, %.3f, %.3f], mag B [%.3f, %.3f, %.3f]",
			 (double)_state.mag_I(0), (double)_state.mag_I(1), (double)_state.mag_I(2),
			 (double)_state.mag_B(0), (double)_state.mag_B(1), (double)_state.mag_B(2)
			);

	} else {
		ECL_INFO("resetting mag I [%.3f, %.3f, %.3f] -> [%.3f, %.3f, %.3f]",
			 (double)mag_I_before_reset(0), (double)mag_I_before_reset(1), (double)mag_I_before_reset(2),
			 (double)_state.mag_I(0), (double)_state.mag_I(1), (double)_state.mag_I(2)
			);

		if (mag_B_before_reset.longerThan(0.f) || _state.mag_B.longerThan(0.f)) {
			ECL_INFO("resetting mag B [%.3f, %.3f, %.3f] -> [%.3f, %.3f, %.3f]",
				 (double)mag_B_before_reset(0), (double)mag_B_before_reset(1), (double)mag_B_before_reset(2),
				 (double)_state.mag_B(0), (double)_state.mag_B(1), (double)_state.mag_B(2)
				);
		}
	}

	_mag_decl_cov_reset = false;

	// record the start time for the magnetic field alignment
	if (_control_status.flags.in_air) {
		_flt_mag_align_start_time = _time_delayed_us;
		_control_status.flags.mag_aligned_in_flight = true;

	} else {
		_flt_mag_align_start_time = 0;
		_control_status.flags.mag_aligned_in_flight = false;
	}
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

float Ekf::getMagDeclination()
{
	// set source of magnetic declination for internal use
	if (_state.mag_I.longerThan(0.1f)) {
		// Use value consistent with earth field state
		return atan2f(_state.mag_I(1), _state.mag_I(0));

	} else if (_params.mag_declination_source & GeoDeclinationMask::USE_GEO_DECL) {
		// use parameter value until GPS is available, then use value returned by geo library
		if (PX4_ISFINITE(_mag_declination_gps)) {
			return _mag_declination_gps;
		}
	}

	// otherwise use the parameter value
	return math::radians(_params.mag_declination_deg);
}

void Ekf::saveMagCovData()
{
	// save the NED axis covariance sub-matrix
	_saved_mag_ef_covmat = P.slice<3, 3>(16, 16);

	// save the XYZ body covariance sub-matrix
	_saved_mag_bf_covmat = P.slice<3, 3>(19, 19);
}

void Ekf::loadMagCovData()
{
	// re-instate the NE axis covariance sub-matrix
	P.uncorrelateCovarianceSetVariance<3>(16, 0.f);
	P.slice<3, 3>(16, 16) = _saved_mag_ef_covmat;

	// re-instate the XYZ body axis covariance sub-matrix
	P.uncorrelateCovarianceSetVariance<3>(19, 0.f);
	P.slice<3, 3>(19, 19) = _saved_mag_bf_covmat;
}
