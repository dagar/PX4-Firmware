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
 * @file ev_height_control.cpp
 * Control functions for ekf external vision height fusion
 */

#include "ekf.h"

void Ekf::controlEvHeightFusion(const extVisionSample &ev_sample)
{
	auto &aid_src = _aid_src_ev_hgt;
	HeightBiasEstimator &bias_est = _ev_hgt_b_est;

	bias_est.predict(_dt_ekf_avg);

	if (_ev_data_ready) {

		const float measurement = ev_sample.pos(2);
		const float measurement_var = math::max(ev_sample.posVar(2), sq(0.01f));

		const float innov_gate = math::max(_params.ev_pos_innov_gate, 1.f);

		updateVerticalPositionAidSrcStatus(ev_sample.time_us,
						   measurement - bias_est.getBias(),
						   measurement_var + bias_est.getBiasVar(),
						   innov_gate,
						   aid_src);

		// update the bias estimator before updating the main filter but after
		// using its current state to compute the vertical position innovation
		if (PX4_ISFINITE(measurement) && PX4_ISFINITE(measurement_var)
		   ) {
			bias_est.setMaxStateNoise(sqrtf(measurement_var));
			bias_est.setProcessNoiseSpectralDensity(_params.ev_hgt_bias_nsd);
			bias_est.fuseBias(measurement - _state.pos(2), measurement_var + P(9, 9));
		}

		const bool continuing_conditions_passing = ((_params.fusion_mode & SensorFusionMask::USE_EXT_VIS_POS) || (_params.height_sensor_ref == HeightSensor::EV)) // TODO: (_params.ev_ctrl & EvCtrl::VPOS)
				&& PX4_ISFINITE(measurement);

		const bool starting_conditions_passing = continuing_conditions_passing
				&& isNewestSampleRecent(_time_last_ext_vision_buffer_push, 2 * EV_MAX_INTERVAL);

		if (_control_status.flags.ev_hgt) {
			aid_src.fusion_enabled = true;

			if (continuing_conditions_passing) {

				fuseVerticalPosition(aid_src);

				const bool is_fusion_failing = isTimedOut(aid_src.time_last_fuse, _params.hgt_fusion_timeout_max);

				if (isHeightResetRequired()) {
					// All height sources are failing
					ECL_INFO("EV height fusion reset required, all height sources failing");
					resetHeightToEv(measurement, measurement_var);

					// reset vertical velocity
					if (PX4_ISFINITE(ev_sample.vel(2)) && (_params.fusion_mode & SensorFusionMask::USE_EXT_VIS_POS)) {
						resetVerticalVelocityTo(ev_sample.vel(2));
						P.uncorrelateCovarianceSetVariance<1>(6, math::max(sq(0.01f), ev_sample.velVar(2)));

					} else {
						resetVerticalVelocityToZero();
					}

				} else if (is_fusion_failing) {
					// Some other height source is still working
					ECL_INFO("stopping EV height fusion, fusion failing");
					stopEvHgtFusion();
				}

			} else {
				ECL_INFO("stopping EV height fusion, continuing conditions not passing");
				stopEvHgtFusion();
			}

		} else {
			if (starting_conditions_passing) {
				if (_params.height_sensor_ref == HeightSensor::EV) {
					ECL_INFO("starting EV height fusion, resetting height to EV");
					bias_est.reset();
					_height_sensor_ref = HeightSensor::EV;
					resetHeightToEv(measurement, measurement_var);

				} else {
					ECL_INFO("starting EV height fusion");
					bias_est.setBias(-_state.pos(2) + measurement);
				}

				aid_src.time_last_fuse = _imu_sample_delayed.time_us;

				_control_status.flags.ev_hgt = true;
				bias_est.setFusionActive();
			}
		}

	} else if (_control_status.flags.ev_hgt
		   && !isNewestSampleRecent(_time_last_ext_vision_buffer_push, 2 * EV_MAX_INTERVAL)) {
		// No data anymore. Stop until it comes back.
		ECL_INFO("stopping EV height fusion, no data");
		stopEvHgtFusion();
	}
}

void Ekf::resetHeightToEv(const float obs, const float obs_var)
{
	_information_events.flags.reset_hgt_to_ev = true;

	resetVerticalPositionTo(obs - _ev_hgt_b_est.getBias());

	// the state variance is the same as the observation
	P.uncorrelateCovarianceSetVariance<1>(9, math::max(sq(0.01f), obs_var));

	_baro_b_est.setBias(_baro_b_est.getBias() + _state_reset_status.posD_change);
	//_ev_hgt_b_est.setBias(_ev_hgt_b_est.getBias() - _state_reset_status.posD_change);
	_gps_hgt_b_est.setBias(_gps_hgt_b_est.getBias() + _state_reset_status.posD_change);
	_rng_hgt_b_est.setBias(_rng_hgt_b_est.getBias() + _state_reset_status.posD_change);

	_aid_src_ev_hgt.time_last_fuse = _imu_sample_delayed.time_us;
}

void Ekf::stopEvHgtFusion()
{
	if (_control_status.flags.ev_hgt) {

		if (_height_sensor_ref == HeightSensor::EV) {
			_height_sensor_ref = HeightSensor::UNKNOWN;
		}

		_control_status.flags.ev_hgt = false;
		_ev_hgt_b_est.setFusionInactive();
		resetEstimatorAidStatus(_aid_src_ev_hgt);
	}
}
