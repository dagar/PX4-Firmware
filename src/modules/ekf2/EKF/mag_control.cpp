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
	// If we are on ground, reset the flight alignment flag so that the mag fields will be
	// re-initialised next time we achieve flight altitude
	if (!_control_status.flags.in_air) {
		_control_status.flags.mag_aligned_in_flight = false;
	}

	magSample mag_sample;

	if (_mag_buffer && _mag_buffer->pop_first_older_than(_imu_sample_delayed.time_us, &mag_sample)) {

		// if enabled, use knowledge of theoretical magnetic field vector to calculate a synthetic magnetometer Z component value.
		// this is useful if there is a lot of interference on the sensor measurement.
		if (_params.synthesize_mag_z
		    && (_params.mag_declination_source & GeoDeclinationMask::USE_GEO_DECL)
		    && PX4_ISFINITE(_mag_inclination_gps) && PX4_ISFINITE(_mag_declination_gps) && PX4_ISFINITE(_mag_strength_gps)
		   ) {
			const Vector3f mag_earth_pred = Dcmf(Eulerf(0, -_mag_inclination_gps, _mag_declination_gps))
							* Vector3f(_mag_strength_gps, 0, 0);
			mag_sample.mag(2) = calculate_synthetic_mag_z_measurement(mag_sample.mag, mag_earth_pred);
			_control_status.flags.synthetic_mag_z = true;

		} else {
			_control_status.flags.synthetic_mag_z = false;
		}

		// sensor or calibration has changed, clear any mag bias and reset low pass filter
		if (mag_sample.reset) {

			_mag_lpf.reset(mag_sample.mag);
			_mag_counter = 1;

			_control_status.flags.mag_fault = false;

		} else {
			_mag_lpf.update(mag_sample.mag);
			_mag_counter++;
		}

		_control_status.flags.mag_field_disturbed = magFieldStrengthDisturbed(mag_sample.mag - _state.mag_B);

		controlMag3dFusion(mag_sample, _aid_src_mag);
		controlMagHeadingFusion(mag_sample, _aid_src_mag_heading);

	} else if ((_control_status.flags.mag_hdg || _control_status.flags.mag || _control_status.flags.mag_3D)
		   && !isNewestSampleRecent(_time_last_mag_buffer_push, (int32_t)5e6)) {

		stopMagFusion();
		stopMagHdgFusion();
	}
}

bool Ekf::haglYawResetReq() const
{
	// We need to reset the yaw angle after climbing away from the ground to enable
	// recovery from ground level magnetic interference.
	if (_control_status.flags.in_air && !_control_status.flags.mag_aligned_in_flight) {
		// Check if height has increased sufficiently to be away from ground magnetic anomalies
		// and request a yaw reset if not already requested.
		static constexpr float mag_anomalies_max_hagl = 1.5f;
		const bool above_mag_anomalies = (getTerrainVPos() - _state.pos(2)) > mag_anomalies_max_hagl;
		return above_mag_anomalies;
	}

	return false;
}

bool Ekf::magYawReset(const Vector3f &mag)
{
	bool has_realigned_yaw = false;

	// use yaw estimator if available
	if (_control_status.flags.gps && isYawEmergencyEstimateAvailable()) {

		resetQuatStateYaw(_yawEstimator.getYaw(), _yawEstimator.getYawVar());

		_information_events.flags.yaw_aligned_to_imu_gps = true;

		// if world magnetic model (inclination, declination, strength) available then use it to reset mag states
		if (PX4_ISFINITE(_mag_inclination_gps) && PX4_ISFINITE(_mag_declination_gps) && PX4_ISFINITE(_mag_strength_gps)) {
			// use predicted earth field to reset states
			const Vector3f mag_earth_pred = Dcmf(Eulerf(0, -_mag_inclination_gps,
							     _mag_declination_gps)) * Vector3f(_mag_strength_gps, 0, 0);
			_state.mag_I = mag_earth_pred;

			const Dcmf R_to_body = quatToInverseRotMat(_state.quat_nominal);
			_state.mag_B = _mag_lpf.getState() - (R_to_body * mag_earth_pred);

		} else {
			// Use the last magnetometer measurements to reset the field states
			// calculate initial earth magnetic field states
			_state.mag_I = _R_to_earth * _mag_lpf.getState();
			_state.mag_B.zero();
		}

		ECL_DEBUG("resetting mag I: [%.3f, %.3f, %.3f], B: [%.3f, %.3f, %.3f]",
			  (double)_state.mag_I(0), (double)_state.mag_I(1), (double)_state.mag_I(2),
			  (double)_state.mag_B(0), (double)_state.mag_B(1), (double)_state.mag_B(2)
			 );

		resetMagCov();

		has_realigned_yaw = true;

	} else if (!magFieldStrengthDisturbed(mag - _state.mag_B)) {
		const Vector3f mag_reset = mag - _state.mag_B;

		// rotate the magnetometer measurements into earth frame using a zero yaw angle
		const Dcmf R_to_earth = updateYawInRotMat(0.f, _R_to_earth);

		// the angle of the projection onto the horizontal gives the yaw angle
		const Vector3f mag_earth_pred = R_to_earth * mag;

		// calculate the observed yaw angle and yaw variance
		_mag_declination = getMagDeclination();
		float yaw_new = -atan2f(mag_earth_pred(1), mag_earth_pred(0)) + _mag_declination;
		float yaw_new_variance = sq(fmaxf(_params.mag_heading_noise, 1.e-2f));

		// update quaternion states and corresponding covarainces
		resetQuatStateYaw(yaw_new, yaw_new_variance);

		// set the earth magnetic field states using the updated rotation
		_state.mag_I = _R_to_earth * mag;

		resetMagCov();

		has_realigned_yaw = true;
	}

	if (has_realigned_yaw) {
		_control_status.flags.yaw_align = true;

		if (_control_status.flags.in_air) {
			_control_status.flags.mag_aligned_in_flight = true;

			// record the time for the magnetic field alignment event
			_flt_mag_align_start_time = _imu_sample_delayed.time_us;
		}

		// Handle the special case where we have not been constraining yaw drift or learning yaw bias due
		// to assumed invalid mag field associated with indoor operation with a downwards looking flow sensor.
		static constexpr uint64_t YAW_AIDING_TIMEOUT_US = (uint64_t)5e6;

		if (!_yaw_angle_observable
		    && isTimedOut(_aid_src_mag_heading.time_last_fuse, YAW_AIDING_TIMEOUT_US)
		    && isTimedOut(_aid_src_mag.time_last_fuse, YAW_AIDING_TIMEOUT_US)
		    && isTimedOut(_aid_src_gnss_yaw.time_last_fuse, YAW_AIDING_TIMEOUT_US)
		    && isTimedOut(_aid_src_ev_yaw.time_last_fuse, YAW_AIDING_TIMEOUT_US)
		   ) {
			// Zero the yaw bias covariance and set the variance to the initial alignment uncertainty
			resetZDeltaAngBiasCov();
		}

		_aid_src_mag.time_last_fuse = _imu_sample_delayed.time_us;
		_aid_src_mag_heading.time_last_fuse = _imu_sample_delayed.time_us;

		return true;
	}

	return false;
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
