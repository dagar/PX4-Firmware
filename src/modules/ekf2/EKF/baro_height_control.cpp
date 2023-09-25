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
	static constexpr const char *HGT_SRC_NAME = "baro";

	auto &aid_src = _aid_src_baro_hgt;
	BiasEstimator &bias_est = _baro_b_est;

	if (_params.height_sensor_ref != HeightSensor::BARO) {
		bias_est.predict(_dt_ekf_avg);
	}

	baroSample baro_sample;

	if (_baro_buffer && _baro_buffer->pop_first_older_than(_time_delayed_us, &baro_sample)) {

#if defined(CONFIG_EKF2_BARO_COMPENSATION)
		const float measurement = compensateBaroForDynamicPressure(baro_sample.hgt);
#else
		const float measurement = baro_sample.hgt;
#endif

		const float measurement_var = sq(_params.baro_noise);

		const float innov_gate = math::max(_params.baro_innov_gate, 1.f);

		const bool measurement_valid = PX4_ISFINITE(measurement) && PX4_ISFINITE(measurement_var);

		if (measurement_valid) {
			if ((_baro_counter == 0) || baro_sample.reset) {
				_baro_lpf.reset(measurement);
				_baro_counter = 1;

			} else {
				_baro_lpf.update(measurement);
				_baro_counter++;
			}

			// baro height mode update bias preflight
			if (_baro_counter <= _obs_buffer_length) {
				// Initialize the pressure offset (included in the baro bias)
				bias_est.setBias(_state.pos(2) + _baro_lpf.getState());
			}
		}

		if (_params.height_sensor_ref == HeightSensor::BARO) {
			// vertical position innovation - baro measurement has opposite sign to earth z axis
			updateVerticalPositionAidSrcStatus(baro_sample.time_us,
							   -(measurement - bias_est.getBias()),
							   measurement_var,
							   innov_gate,
							   aid_src);

		} else {
			// vertical position innovation - baro measurement has opposite sign to earth z axis
			updateVerticalPositionAidSrcStatus(baro_sample.time_us,
							   -(measurement - bias_est.getBias()),
							   measurement_var + bias_est.getBiasVar(),
							   innov_gate,
							   aid_src);

			// update the bias estimator before updating the main filter but after
			// using its current state to compute the vertical position innovation
			if (measurement_valid) {
				bias_est.setMaxStateNoise(sqrtf(measurement_var));
				bias_est.setProcessNoiseSpectralDensity(_params.baro_bias_nsd);
				bias_est.fuseBias(measurement - (-_state.pos(2)), measurement_var + P(State::pos.idx + 2, State::pos.idx + 2));
			}
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

				// recompute test ratio
				aid_src.test_ratio = sq(aid_src.innovation) / (sq(innov_gate) * aid_src.innovation_variance);
				aid_src.innovation_rejected = (aid_src.test_ratio > 1.f);
			}
		}


		// determine if we should use height aiding
		const bool continuing_conditions_passing = (_params.baro_ctrl == 1)
				&& measurement_valid
				&& (_baro_counter > _obs_buffer_length)
				&& !_baro_hgt_faulty;

		const bool starting_conditions_passing = continuing_conditions_passing
				&& isNewestSampleRecent(_time_last_baro_buffer_push, 2 * BARO_MAX_INTERVAL);

		if (_control_status.flags.baro_hgt) {
			if (continuing_conditions_passing) {

				fuseVerticalPosition(aid_src);

				const bool is_fusion_failing = isTimedOut(aid_src.time_last_fuse, _params.hgt_fusion_timeout_max);

				if (isHeightResetRequired()) {
					// All height sources are failing
					ECL_WARN("%s height fusion reset required, all height sources failing", HGT_SRC_NAME);

					if (resetVerticalPosition(aid_src)) {
						_information_events.flags.reset_hgt_to_baro = true;
					}

					// reset vertical velocity
					resetVerticalVelocityToZero();

				} else if (is_fusion_failing) {
					// Some other height source is still working
					ECL_WARN("stopping %s height fusion, fusion failing", HGT_SRC_NAME);
					stopBaroHgtFusion();
					_baro_hgt_faulty = true;
				}

			} else {
				ECL_WARN("stopping %s height fusion, continuing conditions failing", HGT_SRC_NAME);
				stopBaroHgtFusion();
			}

		} else {
			if (starting_conditions_passing) {
				if (_params.height_sensor_ref == HeightSensor::BARO) {
					// TODO: use _baro_lpf.getState() 
					// float reset_z = -(_baro_lpf.getState() - bias_est.getBias());
					if (resetVerticalPosition(aid_src)) {
						ECL_INFO("starting %s height fusion, resetting height", HGT_SRC_NAME);
						_height_sensor_ref = HeightSensor::BARO;
						_information_events.flags.reset_hgt_to_baro = true;
						_control_status.flags.baro_hgt = true;
					}

				} else {
					if (fuseVerticalPosition(aid_src)) {
						ECL_INFO("starting %s height fusion", HGT_SRC_NAME);
						_control_status.flags.baro_hgt = true;

					} else {
						// not starting because vertical position fusion failed, reset bias to try again next time
						bias_est.setBias(_state.pos(2) + _baro_lpf.getState());
					}
				}
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

		_control_status.flags.baro_hgt = false;
	}
}
