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
 * Control functions for ekf magnetometer heading fusion
 */

#include "ekf.h"

void Ekf::controlMagHeadingFusion(const magSample &mag_sample, estimator_aid_source1d_s &aid_src)
{
	static constexpr const char *AID_SRC_NAME = "Mag heading";


	// mag heading
	const Vector3f mag_observation = mag_sample.mag - _state.Vector3f;

	_control_status.flags.mag_hdg_field_disturbed = magFieldStrengthDisturbed(mag_observation);

	// Rotate the measurements into earth frame using the zero yaw angle
	const Dcmf R_to_earth = updateYawInRotMat(0.f, _R_to_earth);
	const Vector3f mag_earth_pred = R_to_earth * mag_observation;

	const float mag_declination = getMagDeclination();

	// TODO: reset if declination changed
	const bool declination_changed = (fabsf(mag_declination - _mag_declination) > math::radians(1.f));


	// the angle of the projection onto the horizontal gives the yaw angle
	// calculate the yaw innovation and wrap to the interval between +-pi

	const float measured_hdg = -atan2f(mag_earth_pred(1), mag_earth_pred(0)) + mag_declination;

	resetEstimatorAidStatus(aid_src);
	aid_src.timestamp_sample = mag_sample.time_us;
	aid_src.observation = measured_hdg;
	aid_src.observation_variance = math::max(sq(_params.mag_hdg_heading_noise), sq(0.01f));
	aid_src.innovation = wrap_pi(getEulerYaw(_R_to_earth) - measured_hdg);

	// determine if we should use EV velocity aiding
	bool continuing_conditions_passing = (_params.mag_hdg_ctrl & static_cast<int32_t>(MagCtrl::heading))
					     && _control_status.flags.yaw_align
					     && _control_status.flags.tilt_align
					     && !_control_status.flags.mag_field_disturbed
					     && !_control_status.flags.mag_fault
					     && !_control_status.flags.mag_3D
					     && !_control_status.flags.gps_yaw
					     && !_control_status.flags.ev_yaw
					     && mag_sample.mag.isAllFinite();

	const bool starting_conditions_passing = continuing_conditions_passing
			&& (_mag_counter > 10);

	if (_control_status.flags.mag_hdg) {
		aid_src.fusion_enabled = true;

		if (continuing_conditions_passing) {

			if (mag_sample.reset || declination_changed) {
				// reset
				const Vector3f mag_sample_reset = _mag_lpf.getState() - _state.mag_B;
				const Vector3f mag_lpf_earth_pred = R_to_earth * _mag_lpf.getState();
				const float heading_lpf_new = -atan2f(mag_earth_pred(1), mag_earth_pred(0)) + mag_declination;
				resetQuatStateYaw(heading_lpf_new, aid_src.observation_variance);
				aid_src.time_last_fuse = _imu_sample_delayed.time_us;

			} else {
				fuseYaw(aid_src);
			}

			_mag_declination = mag_declination;

			const bool is_fusion_failing = isTimedOut(aid_src.time_last_fuse, _params.no_aid_timeout_max); // 1 second

			if (is_fusion_failing) {

				if (_nb_mag_heading_reset_available > 0) {
					// Data seems good, attempt a reset

					//ECL_WARN("%s fusion failing, resetting", AID_SRC_NAME);

					aid_src.time_last_fuse = _imu_sample_delayed.time_us;

					if (_control_status.flags.in_air) {
						_nb_mag_heading_reset_available--;
					}

				} else if (starting_conditions_passing) {
					// Data seems good, but previous reset did not fix the issue
					// something else must be wrong, declare the sensor faulty and stop the fusion
					//_control_status.flags.mag_hdg_fault = true;
					ECL_WARN("stopping %s fusion, starting conditions failing", AID_SRC_NAME);
					stopMagHdgFusion();

				} else {
					// A reset did not fix the issue but all the starting checks are not passing
					// This could be a temporary issue, stop the fusion without declaring the sensor faulty
					ECL_WARN("stopping %s, fusion failing", AID_SRC_NAME);
					stopMagHdgFusion();
				}
			}

		} else {
			// Stop fusion but do not declare it faulty
			ECL_WARN("stopping %s fusion, continuing conditions failing", AID_SRC_NAME);
			stopMagHdgFusion();
		}

	} else {
		if (starting_conditions_passing) {
			// activate fusion, only reset if necessary

			// the angle of the projection onto the horizontal gives the yaw angle
			const Vector3f mag_sample_reset = _mag_lpf.getState() - _state.mag_B;
			const Vector3f mag_lpf_earth_pred = R_to_earth * _mag_lpf.getState();
			float heading_lpf_new = -atan2f(mag_earth_pred(1), mag_earth_pred(0)) + mag_declination;

			if (fabsf(heading_lpf_new - getEulerYaw()) > math::radians(5.f)) {
				ECL_INFO("starting %s fusion, resetting heading", AID_SRC_NAME);
				// reset
				// update quaternion states and corresponding covarainces
				resetQuatStateYaw(heading_lpf_new, aid_src.observation_variance);
				_mag_declination = mag_declination;

			} else {
				ECL_INFO("starting %s fusion", AID_SRC_NAME);
			}

			aid_src.time_last_fuse = _imu_sample_delayed.time_us;

			_nb_mag_heading_reset_available = 5;
			_control_status.flags.mag_hdg = true;
		}
	}
}

void Ekf::stopMagHdgFusion()
{
	if (_control_status.flags.mag_hdg) {
		_control_status.flags.mag_hdg = false;

		_fault_status.flags.bad_hdg = false;
	}
}
