/****************************************************************************
 *
 *   Copyright (c) 2018-2023 PX4 Development Team. All rights reserved.
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
 * @file gnss_yaw.cpp
 * Definition of functions required to use yaw obtained from GPS dual antenna measurements.
 * Equations generated using EKF/python/ekf_derivation/main.py
 *
 * @author Paul Riseborough <p_riseborough@live.com.au>
 *
 */

#include "ekf.h"
#include <ekf_derivation/generated/compute_gnss_yaw_pred_innov_var_and_h.h>

#include <mathlib/mathlib.h>
#include <cstdlib>

void Ekf::controlGNSSYawFusion(const gnssSample &gnss_sample)
{
	static constexpr const char *AID_SRC_NAME = "GNSS yaw";

	estimator_aid_source1d_s &aid_src = _aid_src_gnss_yaw;

	if (PX4_ISFINITE(gnss_sample.yaw)) {

		const float antenna_yaw_offset = PX4_ISFINITE(gnss_sample.yaw_offset) ? gnss_sample.yaw_offset : 0.f;

		// calculate the observed yaw angle of antenna array, converting a from body to antenna yaw measurement
		const float measured_hdg = wrap_pi(gnss_sample.yaw + antenna_yaw_offset);

		const float yaw_acc = PX4_ISFINITE(gnss_sample.yaw_acc) ? gnss_sample.yaw_acc : 0.f;
		const float R_YAW = sq(math::max(yaw_acc, _params.gps_heading_noise, 0.01f));

		float heading_pred;
		float heading_innov_var;
		VectorState H;
		sym::ComputeGnssYawPredInnovVarAndH(_state.vector(), P, antenna_yaw_offset, R_YAW, FLT_EPSILON,
						    &heading_pred, &heading_innov_var, &H);

		updateAidSourceStatus(aid_src,
				      gnss_sample.time_us,                         // sample timestamp
				      measured_hdg,                                // observation
				      R_YAW,                                       // observation variance
				      wrap_pi(heading_pred - measured_hdg),        // innovation
				      heading_innov_var,                           // innovation variance
				      math::max(_params.heading_innov_gate, 1.f)); // innovation gate

		const bool continuing_conditions_passing = (_params.gnss_ctrl & static_cast<int32_t>(GnssCtrl::YAW))
				&& !_control_status.flags.gps_yaw_fault
				&& _control_status.flags.tilt_align;

		const bool starting_conditions_passing = continuing_conditions_passing
				&& _gps_checks_passed
				&& isNewestSampleRecent(_time_last_gps_yaw_buffer_push, 2 * GNSS_YAW_MAX_INTERVAL);

		if (_control_status.flags.gps_yaw) {
			if (continuing_conditions_passing) {
				if (!fuseGpsYaw(H, aid_src, antenna_yaw_offset) && isTimedOut(aid_src.time_last_fuse, _params.reset_timeout_max)) {
					ECL_INFO("fusion failing, stopping %s", AID_SRC_NAME);
					_control_status.flags.gps_yaw = false;

					// Before takeoff, we do not want to continue to rely on the current heading
					// if we had to stop the fusion
					if (!_control_status.flags.in_air) {
						ECL_INFO("clearing yaw alignment");
						_control_status.flags.yaw_align = false;
					}
				}

			} else {
				// Stop GPS yaw fusion but do not declare it faulty
				ECL_INFO("continuing conditions failing, stopping %s", AID_SRC_NAME);
				_control_status.flags.gps_yaw = false;
			}

		} else {
			if (starting_conditions_passing) {
				// Try to activate GPS yaw fusion
				const bool not_using_ne_aiding = !_control_status.flags.gps && !_control_status.flags.aux_gpos;

				if (!_control_status.flags.in_air
				    || !_control_status.flags.yaw_align
				    || not_using_ne_aiding) {

					// Reset before starting the fusion

					// define the predicted antenna array vector and rotate into earth frame
					const Vector3f ant_vec_bf = {cosf(antenna_yaw_offset), sinf(antenna_yaw_offset), 0.0f};
					const Vector3f ant_vec_ef = _R_to_earth * ant_vec_bf;

					// check if antenna array vector is within 30 degrees of vertical and therefore unable to provide a reliable heading
					if (fabsf(ant_vec_ef(2)) <= cosf(math::radians(30.f)))  {
						// GPS yaw measurement is already compensated for antenna offset in the driver
						resetQuatStateYaw(measured_hdg, R_YAW);
						_control_status.flags.yaw_align = true;

						resetAidSourceStatusZeroInnovation(aid_src);

						ECL_INFO("starting %s fusion, resetting", AID_SRC_NAME);
						_control_status.flags.gps_yaw = true;
					}

				} else if (fuseGpsYaw(H, aid_src, antenna_yaw_offset)) {
					// Do not force a reset but wait for the consistency check to pass
					ECL_INFO("starting %s fusion", AID_SRC_NAME);
					_control_status.flags.gps_yaw = true;
				}
			}
		}

	} else if (_control_status.flags.gps_yaw
		   && !isNewestSampleRecent(_time_last_gps_yaw_buffer_push, _params.reset_timeout_max)) {

		// No yaw data in the message anymore. Stop until it comes back.
		ECL_INFO("stopping %s fusion, timeout", AID_SRC_NAME);
		_control_status.flags.gps_yaw = false;
	}
}

bool Ekf::fuseGpsYaw(const VectorState &H, estimator_aid_source1d_s &aid_src, float antenna_yaw_offset)
{
	if (aid_src.innovation_rejected) {
		_innov_check_fail_status.flags.reject_yaw = true;
		return false;
	}

	// check if the innovation variance calculation is badly conditioned
	if (aid_src.innovation_variance < aid_src.observation_variance) {
		// the innovation variance contribution from the state covariances is negative which means the covariance matrix is badly conditioned
		_fault_status.flags.bad_hdg = true;

		// we reinitialise the covariance matrix and abort this fusion step
		initialiseCovariance();
		ECL_ERR("GPS yaw numerical error - covariance reset");
		return false;
	}

	_fault_status.flags.bad_hdg = false;
	_innov_check_fail_status.flags.reject_yaw = false;

	if ((fabsf(aid_src.test_ratio_filtered) > 0.2f)
	    && !_control_status.flags.in_air && isTimedOut(aid_src.time_last_fuse, (uint64_t)1e6)
	   ) {
		// A constant large signed test ratio is a sign of wrong gyro bias
		// Reset the yaw gyro variance to converge faster and avoid
		// being stuck on a previous bad estimate
		resetGyroBiasZCov();
	}

	// calculate the Kalman gains
	// only calculate gains for states we are using
	VectorState Kfusion = P * H / aid_src.innovation_variance;

	const bool is_fused = measurementUpdate(Kfusion, H, aid_src.observation_variance, aid_src.innovation);
	_fault_status.flags.bad_hdg = !is_fused;
	aid_src.fused = is_fused;

	if (is_fused) {
		_time_last_heading_fuse = _time_delayed_us;
		aid_src.time_last_fuse = _time_delayed_us;
	}

	return is_fused;
}
