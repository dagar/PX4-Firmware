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
 * @file baro_height_control.cpp
 * Control functions for ekf barometric height fusion
 */

#include "ekf.h"

void Ekf::controlBaroHeightFusion()
{
	static constexpr char HGT_SRC_NAME[]{"baro"};

	HeightBiasEstimator &bias_est = _baro_b_est;

	if (_height_sensor_ref != HeightSensor::BARO) {
		bias_est.predict(_dt_ekf_avg);
	}

	baroSample baro_sample;

	if (_baro_buffer && _baro_buffer->pop_first_older_than(_imu_sample_delayed.time_us, &baro_sample)) {

		auto &aid_src = _aid_src_baro_hgt;

		const bool hgt_src_enabled = (_params.baro_ctrl == 1);

		const uint64_t time_us = baro_sample.time_us;
		float measurement = baro_sample.hgt;
		float measurement_var = math::max(sq(0.01f), sq(_params.baro_noise));
		const float innov_gate = math::max(_params.baro_innov_gate, 1.f);

		const bool measurement_valid = PX4_ISFINITE(measurement) && PX4_ISFINITE(measurement_var);

		if (measurement_valid) {
			if (_baro_counter == 0) {
				_baro_lpf.reset(measurement);

			} else {
				_baro_lpf.update(measurement);
			}

			if (_baro_counter <= _obs_buffer_length) {
				// Initialize the pressure offset (included in the baro bias)
				bias_est.setBias(_state.pos(2) + _baro_lpf.getState());
				_baro_counter++;
			}
		}

		// vertical position - measurement has opposite sign of earth z axis
		if (_height_sensor_ref == HeightSensor::BARO) {
			updateVerticalPositionAidSrcStatus(time_us,
							-(measurement - bias_est.getBias()),
							measurement_var + bias_est.getBiasVar(),
							innov_gate,
							aid_src);

		} else {
			updateVerticalPositionAidSrcStatus(time_us,
							-measurement,
							measurement_var,
							innov_gate,
							aid_src);
		}

		// Compensate for positive static pressure transients (negative vertical position innovations)
		// caused by rotor wash ground interaction by applying a temporary deadzone to baro innovations.
		if (_control_status.flags.gnd_effect && (_params.gnd_effect_deadzone > 0.f)) {

			const float deadzone_start = 0.0f;
			const float deadzone_end = deadzone_start + _params.gnd_effect_deadzone;

			if (aid_src.innovation < -deadzone_start) {
				if (aid_src.innovation <= -deadzone_end) {
					aid_src.innovation += deadzone_end;

				} else {
					aid_src.innovation = -deadzone_start;
				}
			}
		}

		// update the bias estimator before updating the main filter but after
		// using its current state to compute the vertical position innovation
		if (measurement_valid) {
			bias_est.setMaxStateNoise(sqrtf(measurement_var));
			bias_est.setProcessNoiseSpectralDensity(_params.baro_bias_nsd);
			bias_est.fuseBias(measurement - (-_state.pos(2)), measurement_var + P(9, 9));
		}

		// clear fault if test_ratio very good
		if (_control_status.flags.baro_fault) {
			if (aid_src.test_ratio < 0.01f) {
				_control_status.flags.baro_fault = false;
			}
		}

		// determine if we should use height aiding
		const bool continuing_conditions_passing = hgt_src_enabled
				&& measurement_valid
				&& !_control_status.flags.baro_fault;

		const bool starting_conditions_passing = continuing_conditions_passing
				&& (_baro_counter > _obs_buffer_length)
				&& isNewestSampleRecent(_time_last_baro_buffer_push, 2 * BARO_MAX_INTERVAL);

		const float measurement_lpf = _baro_lpf.getState();

		if (_control_status.flags.baro_hgt) {
			aid_src.fusion_enabled = true;

			if (continuing_conditions_passing) {

				fuseVerticalPosition(aid_src);

				const bool is_fusion_failing = isTimedOut(aid_src.time_last_fuse, _params.hgt_fusion_timeout_max);

				if (isHeightResetRequired()) {
					// All height sources are failing
					ECL_WARN("%s height fusion reset required, all height sources failing", HGT_SRC_NAME);
					_information_events.flags.reset_hgt_to_baro = true;

					if (_height_sensor_ref == HeightSensor::BARO) {
						resetVerticalPositionTo(-measurement_lpf, measurement_var);
						bias_est.reset();

					} else {
						resetVerticalPositionTo(-(measurement_lpf - bias_est.getBias()), measurement_var + bias_est.getBiasVar());
						bias_est.setBias(_state.pos(2) + measurement_lpf);
					}

					_aid_src_baro_hgt.time_last_fuse = _imu_sample_delayed.time_us;

					// reset vertical velocity
					resetVerticalVelocityToZero();

				} else if (is_fusion_failing) {
					// Some other height source is still working
					ECL_WARN("stopping %s height fusion, fusion failing", HGT_SRC_NAME);
					stopBaroHgtFusion();
					_control_status.flags.baro_fault = true;
				}

			} else {
				ECL_WARN("stopping %s height fusion, continuing conditions failing", HGT_SRC_NAME);
				stopBaroHgtFusion();
			}

		} else {
			if (starting_conditions_passing) {
				if (_params.height_sensor_ref == HeightSensor::BARO) {
					ECL_INFO("starting %s height fusion, resetting height to %s", HGT_SRC_NAME);
					_information_events.flags.reset_hgt_to_baro = true;
					_height_sensor_ref = HeightSensor::BARO;

					resetVerticalPositionTo(-measurement_lpf, measurement_var);
					bias_est.reset();

				} else {
					ECL_INFO("starting %s height fusion", HGT_SRC_NAME);
					bias_est.setBias(_state.pos(2) + measurement_lpf);
				}

				aid_src.time_last_fuse = _imu_sample_delayed.time_us;

				_control_status.flags.baro_hgt = true;
				bias_est.setFusionActive();
			}
		}

	} else if (_control_status.flags.baro_hgt
		   && !isNewestSampleRecent(_time_last_baro_buffer_push, 2 * BARO_MAX_INTERVAL)) {
		// No data anymore. Stop until it comes back.
		ECL_WARN("stopping %s height fusion, no data", HGT_SRC_NAME);
		stopBaroHgtFusion();
	}
}

void Ekf::stopBaroHgtFusion()
{
	if (_control_status.flags.baro_hgt) {

		if (_height_sensor_ref == HeightSensor::BARO) {
			_height_sensor_ref = HeightSensor::UNKNOWN;
		}

		_baro_b_est.setFusionInactive();
		_baro_b_est.reset();

		_baro_lpf.reset(0.f);
		_baro_counter = 0;

		resetEstimatorAidStatus(_aid_src_baro_hgt);

		_control_status.flags.baro_hgt = false;
	}
}
