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
 * @file gps_control.cpp
 * Control functions for ekf range finder height fusion
 */

#include "ekf.h"

void Ekf::controlRangeHeightFusion()
{
	auto &aid_src = _aid_src_rng_hgt;
	HeightBiasEstimator &bias_est = _rng_hgt_b_est;

	bias_est.predict(_dt_ekf_avg);

	if (_rng_data_ready) {

		const float measurement = math::max(_range_sensor.getDistBottom(), _params.rng_gnd_clearance);
		const float measurement_var = math::max(sq(0.01f), sq(_params.range_noise) + sq(_params.range_noise_scaler * _range_sensor.getDistBottom()));

		const float innov_gate = math::max(_params.range_innov_gate, 1.f);

		// vertical position innovation - baro measurement has opposite sign to earth z axis
		updateVerticalPositionAidSrcStatus(_range_sensor.getSampleAddress()->time_us,
						   -(measurement - bias_est.getBias()),
						   measurement_var + bias_est.getBiasVar(),
						   innov_gate,
						   aid_src);

		// update the bias estimator before updating the main filter but after
		// using its current state to compute the vertical position innovation
		if (_range_sensor.isDataHealthy()
		    && PX4_ISFINITE(measurement) && PX4_ISFINITE(measurement_var)
		   ) {
			bias_est.setMaxStateNoise(sqrtf(measurement_var));
			bias_est.setProcessNoiseSpectralDensity(_params.rng_hgt_bias_nsd);
			bias_est.fuseBias(measurement - (-_state.pos(2)), measurement_var + P(9, 9));
		}

		// determine if we should use baro height aiding
		const bool do_conditional_range_aid = (_params.rng_ctrl == RngCtrl::CONDITIONAL) && isConditionalRangeAidSuitable();
		const bool continuing_conditions_passing = ((_params.rng_ctrl == RngCtrl::ENABLED) || do_conditional_range_aid)
				&& PX4_ISFINITE(_range_sensor.getDistBottom())
				&& _range_sensor.isDataHealthy();

		const bool starting_conditions_passing = continuing_conditions_passing
				&& _range_sensor.isRegularlySendingData()
				&& isNewestSampleRecent(_time_last_range_buffer_push, 2 * RNG_MAX_INTERVAL);

		if (_control_status.flags.rng_hgt) {
			aid_src.fusion_enabled = true;

			if (continuing_conditions_passing) {

				fuseVerticalPosition(aid_src);

				const bool is_fusion_failing = isTimedOut(aid_src.time_last_fuse, _params.hgt_fusion_timeout_max);

				if (isHeightResetRequired()) {
					// All height sources are failing
					ECL_INFO("RNG height fusion reset required, all height sources failing");
					resetHeightToRng(measurement, measurement_var);

					// reset vertical velocity
					resetVerticalVelocityToZero();

				} else if (is_fusion_failing) {
					// Some other height source is still working
					ECL_INFO("stopping RNG height fusion, fusion failing");
					stopRngHgtFusion();
					_control_status.flags.rng_fault = true;
					_range_sensor.setFaulty();
				}

			} else {
				ECL_INFO("stopping RNG height fusion, continuing conditions not passing");
				stopRngHgtFusion();
			}

		} else {
			if (starting_conditions_passing) {
				if ((_params.height_sensor_ref == HeightSensor::RANGE) && (_params.rng_ctrl == RngCtrl::CONDITIONAL)) {
					// Range finder is used while hovering to stabilize the height estimate. Don't reset but use it as height reference.
					ECL_INFO("starting RNG height fusion, resetting height to RNG");
					bias_est.setBias(_state.pos(2) + _range_sensor.getDistBottom());
					_height_sensor_ref = HeightSensor::RANGE;

				} else if ((_params.height_sensor_ref == HeightSensor::RANGE) && (_params.rng_ctrl != RngCtrl::CONDITIONAL)) {
					// Range finder is the primary height source, the ground is now the datum used
					// to compute the local vertical position
					ECL_INFO("starting RNG height fusion, resetting height to RNG");
					bias_est.reset();
					_height_sensor_ref = HeightSensor::RANGE;
					resetHeightToRng(measurement, measurement_var);

				} else {
					ECL_INFO("starting RNG height fusion");
					bias_est.setBias(_state.pos(2) + _range_sensor.getDistBottom());
				}

				aid_src.time_last_fuse = _imu_sample_delayed.time_us;

				_control_status.flags.rng_hgt = true;
				bias_est.setFusionActive();
			}
		}

	} else if (_control_status.flags.rng_hgt && !isNewestSampleRecent(_time_last_range_buffer_push, 2 * RNG_MAX_INTERVAL)) {
		// No data anymore. Stop until it comes back.
		ECL_INFO("stopping RNG height fusion, no data");
		stopRngHgtFusion();
	}
}

void Ekf::resetHeightToRng(const float obs, const float obs_var)
{
	_information_events.flags.reset_hgt_to_rng = true;

	resetVerticalPositionTo(-(obs - _rng_hgt_b_est.getBias()));

	// the state variance is the same as the observation
	P.uncorrelateCovarianceSetVariance<1>(9, obs_var);

	_baro_b_est.setBias(_baro_b_est.getBias() + _state_reset_status.posD_change);
	_ev_hgt_b_est.setBias(_ev_hgt_b_est.getBias() - _state_reset_status.posD_change);
	_gps_hgt_b_est.setBias(_gps_hgt_b_est.getBias() + _state_reset_status.posD_change);
	//_rng_hgt_b_est.setBias(_rng_hgt_b_est.getBias() + _state_reset_status.posD_change);

	_aid_src_rng_hgt.time_last_fuse = _imu_sample_delayed.time_us;
}

void Ekf::stopRngHgtFusion()
{
	if (_control_status.flags.rng_hgt) {

		if (_height_sensor_ref == HeightSensor::RANGE) {
			_height_sensor_ref = HeightSensor::UNKNOWN;
		}

		_control_status.flags.rng_hgt = false;
		_rng_hgt_b_est.setFusionInactive();
		resetEstimatorAidStatus(_aid_src_rng_hgt);
	}
}

bool Ekf::isConditionalRangeAidSuitable()
{
	if (_control_status.flags.in_air
	    && _range_sensor.isHealthy()
	    && isTerrainEstimateValid()) {
		// check if we can use range finder measurements to estimate height, use hysteresis to avoid rapid switching
		// Note that the 0.7 coefficients and the innovation check are arbitrary values but work well in practice
		float range_hagl_max = _params.max_hagl_for_range_aid;
		float max_vel_xy = _params.max_vel_for_range_aid;

		const float hagl_test_ratio = (_hagl_innov * _hagl_innov / (sq(_params.range_aid_innov_gate) * _hagl_innov_var));

		bool is_hagl_stable = (hagl_test_ratio < 1.f);

		if (!_control_status.flags.rng_hgt) {
			range_hagl_max = 0.7f * _params.max_hagl_for_range_aid;
			max_vel_xy = 0.7f * _params.max_vel_for_range_aid;
			is_hagl_stable = (hagl_test_ratio < 0.01f);
		}

		const float range_hagl = _terrain_vpos - _state.pos(2);

		const bool is_in_range = (range_hagl < range_hagl_max);

		bool is_below_max_speed = true;

		if (isHorizontalAidingActive()) {
			is_below_max_speed = !_state.vel.xy().longerThan(max_vel_xy);
		}

		return is_in_range && is_hagl_stable && is_below_max_speed;
	}

	return false;
}
