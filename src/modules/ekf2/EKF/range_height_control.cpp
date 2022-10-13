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
 * @file range_height_control.cpp
 * Control functions for ekf range finder height fusion
 */

#include "ekf.h"

void Ekf::controlRangeHeightFusion()
{
	static constexpr char HGT_SRC_NAME[]{"RNG"};

	HeightBiasEstimator &bias_est = _rng_hgt_b_est;

	bias_est.predict(_dt_ekf_avg);

	if (_rng_data_ready && _range_sensor.getSampleAddress()) {

		auto &aid_src = _aid_src_rng_hgt;

		const bool hgt_src_enabled = (_params.rng_ctrl == RngCtrl::ENABLED);

		const uint64_t time_us = _range_sensor.getSampleAddress()->time_us;
		const float measurement = math::max(_range_sensor.getDistBottom(), _params.rng_gnd_clearance);
		const float measurement_var = math::max(sq(0.01f), sq(_params.range_noise) + sq(_params.range_noise_scaler * _range_sensor.getDistBottom()));
		const float innov_gate = math::max(_params.range_innov_gate, 1.f);

		const bool measurement_valid = PX4_ISFINITE(measurement) && PX4_ISFINITE(measurement_var);

		if (measurement_valid) {

			if (_rng_counter == 0) {
				_rng_lpf.reset(measurement);

			} else {
				_rng_lpf.update(measurement);
			}

			if (_rng_counter <= _obs_buffer_length) {
				// Initialize the offset
				bias_est.setBias(_state.pos(2) + _rng_lpf.getState());
				_rng_counter++;
			}
		}

		// vertical position - measurement has opposite sign of earth z axis
		updateVerticalPositionAidSrcStatus(time_us,
						   -(measurement - bias_est.getBias()),
						   measurement_var + bias_est.getBiasVar(),
						   innov_gate,
						   aid_src);

		// update the bias estimator before updating the main filter but after
		// using its current state to compute the vertical position innovation
		if (measurement_valid && _range_sensor.isDataHealthy()) {
			bias_est.setMaxStateNoise(sqrtf(measurement_var));
			bias_est.setProcessNoiseSpectralDensity(_params.rng_hgt_bias_nsd);
			bias_est.fuseBias(measurement - (-_state.pos(2)), measurement_var + P(9, 9));
		}

		// clear fault if test_ratio very good
		if (_control_status.flags.rng_fault) {
			if (aid_src.test_ratio < 0.01f) {
				_control_status.flags.rng_fault = false;
				_range_sensor.setFaulty(false);
			}
		}

		// determine if we should use height aiding
		const bool do_conditional_range_aid = (_params.rng_ctrl == RngCtrl::CONDITIONAL) && isConditionalRangeAidSuitable();
		const bool continuing_conditions_passing = (hgt_src_enabled || do_conditional_range_aid)
				&& measurement_valid
				&& !_control_status.flags.rng_fault
				&& _range_sensor.isDataHealthy();

		const bool starting_conditions_passing = continuing_conditions_passing
				&& (_rng_counter > _obs_buffer_length)
				&& isNewestSampleRecent(_time_last_range_buffer_push, 2 * RNG_MAX_INTERVAL)
				&& _range_sensor.isRegularlySendingData();

		const float measurement_lpf = _rng_lpf.getState();

		if (_control_status.flags.rng_hgt) {
			aid_src.fusion_enabled = true;

			if (continuing_conditions_passing) {

				fuseVerticalPosition(aid_src);

				const bool is_fusion_failing = isTimedOut(aid_src.time_last_fuse, _params.hgt_fusion_timeout_max);

				if (isHeightResetRequired()) {
					// All height sources are failing
					ECL_WARN("%s height fusion reset required, all height sources failing", HGT_SRC_NAME);
					_information_events.flags.reset_hgt_to_rng = true;
					resetVerticalPositionTo(-(measurement_lpf - _rng_hgt_b_est.getBias()));
					bias_est.setBias(_state.pos(2) + measurement_lpf);

					// reset vertical velocity
					resetVerticalVelocityToZero();

				} else if (is_fusion_failing) {
					// Some other height source is still working
					ECL_WARN("stopping %s height fusion, fusion failing", HGT_SRC_NAME);
					stopRngHgtFusion();
					_control_status.flags.rng_fault = true;
					_range_sensor.setFaulty(true);
				}

			} else {
				ECL_WARN("stopping %s height fusion, continuing conditions failing", HGT_SRC_NAME);
				stopRngHgtFusion();
			}

		} else {
			if (starting_conditions_passing) {
				if ((_params.height_sensor_ref == HeightSensor::RANGE) && (_params.rng_ctrl == RngCtrl::CONDITIONAL)) {
					// Range finder is used while hovering to stabilize the height estimate. Don't reset but use it as height reference.
					ECL_INFO("starting conditional %s height fusion", HGT_SRC_NAME);
					bias_est.setBias(_state.pos(2) + measurement_lpf);
					_height_sensor_ref = HeightSensor::RANGE;

				} else if ((_params.height_sensor_ref == HeightSensor::RANGE) && (_params.rng_ctrl != RngCtrl::CONDITIONAL)) {
					// Range finder is the primary height source, the ground is now the datum used
					// to compute the local vertical position
					ECL_INFO("starting %s height fusion, resetting height to %s", HGT_SRC_NAME);
					_information_events.flags.reset_hgt_to_rng = true;
					_height_sensor_ref = HeightSensor::RANGE;
					resetVerticalPositionTo(-measurement_lpf, measurement_var);
					bias_est.reset();

				} else {
					ECL_INFO("starting %s height fusion", HGT_SRC_NAME);
					bias_est.setBias(_state.pos(2) + measurement_lpf);
				}

				aid_src.time_last_fuse = _imu_sample_delayed.time_us;

				_control_status.flags.rng_hgt = true;
				bias_est.setFusionActive();
			}
		}

	} else if (_control_status.flags.rng_hgt
		   && !isNewestSampleRecent(_time_last_range_buffer_push, 2 * RNG_MAX_INTERVAL)) {
		// No data anymore. Stop until it comes back.
		ECL_WARN("stopping %s height fusion, no data", HGT_SRC_NAME);
		stopRngHgtFusion();
	}
}

void Ekf::stopRngHgtFusion()
{
	if (_control_status.flags.rng_hgt) {

		if (_height_sensor_ref == HeightSensor::RANGE) {
			_height_sensor_ref = HeightSensor::UNKNOWN;
		}

		_rng_hgt_b_est.setFusionInactive();
		_rng_hgt_b_est.reset();

		_rng_lpf.reset(0.f);
		_rng_counter = 0;

		resetEstimatorAidStatus(_aid_src_rng_hgt);

		_control_status.flags.rng_hgt = false;
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
