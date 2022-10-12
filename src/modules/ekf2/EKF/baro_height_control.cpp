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
	HeightBiasEstimator &bias_est = _baro_b_est;

	bias_est.predict(_dt_ekf_avg);

	baroSample baro_sample;

	if (_baro_buffer && _baro_buffer->pop_first_older_than(_imu_sample_delayed.time_us, &baro_sample)) {

		// determine if we should use baro height aiding
		bool continuing_conditions_passing = (_params.baro_ctrl == 1)
						     && PX4_ISFINITE(baro_sample.hgt)
						     && !_baro_hgt_faulty;

		if (_baro_counter == 0) {
			_baro_lpf.reset(baro_sample.hgt);

		} else {
			_baro_lpf.update(baro_sample.hgt);
		}

		if (_baro_counter < _obs_buffer_length) {
			// Initialize the pressure offset (included in the baro bias)
			bias_est.setBias(_state.pos(2) + _baro_lpf.getState());
			_baro_counter++;
		}

		auto &aid_src = _aid_src_baro_hgt;

		const float innov_gate = fmaxf(_params.baro_innov_gate, 1.f);

		const float measurement = baro_sample.hgt;
		const float measurement_var = sq(fmaxf(_params.baro_noise, 0.01f));

		// vertical position innovation - baro measurement has opposite sign to earth z axis
		updateVerticalPositionAidSrcStatus(baro_sample.time_us,
						   -(measurement - bias_est.getBias()),
						   measurement_var + bias_est.getBiasVar(),
						   innov_gate,
						   aid_src);

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
		if ((_baro_counter >= _obs_buffer_length)
		    && PX4_ISFINITE(measurement) && PX4_ISFINITE(measurement_var)
		   ) {
			bias_est.setMaxStateNoise(_params.baro_noise);
			bias_est.setProcessNoiseSpectralDensity(_params.baro_bias_nsd);
			bias_est.fuseBias(measurement - (-_state.pos(2)), measurement_var + P(9, 9));
		}

		bool starting_conditions_passing = continuing_conditions_passing
						   && (_baro_counter >= _obs_buffer_length)
						   && isNewestSampleRecent(_time_last_baro_buffer_push, 2 * BARO_MAX_INTERVAL);

		if (_control_status.flags.baro_hgt) {
			aid_src.fusion_enabled = true;

			if (continuing_conditions_passing) {

				fuseVerticalPosition(aid_src);

				const bool is_fusion_failing = isTimedOut(aid_src.time_last_fuse, _params.hgt_fusion_timeout_max);

				if (isHeightResetRequired()) {
					// All height sources are failing
					ECL_INFO("baro height fusion reset required, all height sources failing");
					resetHeightToBaro(baro_sample);
					resetVerticalVelocityToZero();

				} else if (is_fusion_failing) {
					// Some other height source is still working
					ECL_INFO("stopping baro height fusion, fusion failing");
					stopBaroHgtFusion();
					_baro_hgt_faulty = true;
				}

			} else {
				ECL_INFO("stopping baro height fusion, continuing conditions not passing");
				stopBaroHgtFusion();
			}

		} else {
			if (starting_conditions_passing) {
				if (_params.height_sensor_ref == HeightSensor::BARO) {
					// bias_est.reset(); // TODO: review
					_height_sensor_ref = HeightSensor::BARO;
					resetHeightToBaro(baro_sample);

				} else {
					bias_est.setBias(_state.pos(2) + _baro_lpf.getState());
				}

				aid_src.time_last_fuse = _imu_sample_delayed.time_us;

				_control_status.flags.baro_hgt = true;
				bias_est.setFusionActive();
				ECL_INFO("starting baro height fusion");
			}
		}

	} else if (_control_status.flags.baro_hgt
		   && !isNewestSampleRecent(_time_last_baro_buffer_push, 2 * BARO_MAX_INTERVAL)) {
		// No data anymore. Stop until it comes back.
		ECL_INFO("stopping baro height fusion, no data");
		stopBaroHgtFusion();
	}
}

void Ekf::resetHeightToBaro(const baroSample &baro_sample)
{
	ECL_INFO("reset height to baro");
	_information_events.flags.reset_hgt_to_baro = true;

	resetVerticalPositionTo(-(baro_sample.hgt - _baro_b_est.getBias()));

	// the state variance is the same as the observation
	P.uncorrelateCovarianceSetVariance<1>(9, sq(_params.baro_noise));

	//_baro_b_est.setBias(_baro_b_est.getBias() + _state_reset_status.posD_change);
	_ev_hgt_b_est.setBias(_ev_hgt_b_est.getBias() - _state_reset_status.posD_change);
	_gps_hgt_b_est.setBias(_gps_hgt_b_est.getBias() + _state_reset_status.posD_change);
	_rng_hgt_b_est.setBias(_rng_hgt_b_est.getBias() + _state_reset_status.posD_change);
}

void Ekf::stopBaroHgtFusion()
{
	if (_control_status.flags.baro_hgt) {

		if (_height_sensor_ref == HeightSensor::BARO) {
			_height_sensor_ref = HeightSensor::UNKNOWN;
		}

		_control_status.flags.baro_hgt = false;
		_baro_b_est.setFusionInactive();
		resetEstimatorAidStatus(_aid_src_baro_hgt);
		ECL_INFO("stopping baro height fusion");
	}
}
