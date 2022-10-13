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
 * @file gnss_height_control.cpp
 * Control functions for ekf GNSS height fusion
 */

#include "ekf.h"

void Ekf::controlGnssHeightFusion(const gpsSample &gps_sample)
{
	auto &aid_src = _aid_src_gnss_hgt;
	HeightBiasEstimator &bias_est = _gps_hgt_b_est;

	bias_est.predict(_dt_ekf_avg);

	if (_gps_data_ready) {

		// relax the upper observation noise limit which prevents bad GPS perturbing the position estimate
		float noise = math::max(0.01f, gps_sample.vacc, 1.5f * _params.gps_pos_noise); // use 1.5 as a typical ratio of vacc/hacc

		if (!isOnlyActiveSourceOfVerticalPositionAiding(_control_status.flags.gps_hgt)) {
			// if we are not using another source of aiding, then we are reliant on the GPS
			// observations to constrain attitude errors and must limit the observation noise value.
			if (noise > _params.pos_noaid_noise) {
				noise = _params.pos_noaid_noise;
			}
		}

		const float measurement = gps_sample.hgt - getEkfGlobalOriginAltitude();
		const float measurement_var = sq(math::max(noise, 0.01f));

		const float innov_gate = math::max(_params.gps_pos_innov_gate, 1.f);

		// GNSS position, vertical position GNSS measurement has opposite sign to earth z axis
		updateVerticalPositionAidSrcStatus(gps_sample.time_us,
						   -(measurement - bias_est.getBias()),
						   measurement_var + bias_est.getBiasVar(),
						   innov_gate,
						   aid_src);

		const bool gps_checks_passing = isTimedOut(_last_gps_fail_us, (uint64_t)5e6);
		const bool gps_checks_failing = isTimedOut(_last_gps_pass_us, (uint64_t)5e6);

		// update the bias estimator before updating the main filter but after
		// using its current state to compute the vertical position innovation
		if (gps_checks_passing && !gps_checks_failing
		    && PX4_ISFINITE(measurement) && PX4_ISFINITE(measurement_var)
		   ) {
			bias_est.setMaxStateNoise(sqrtf(measurement_var));
			bias_est.setProcessNoiseSpectralDensity(_params.gps_hgt_bias_nsd);
			bias_est.fuseBias(measurement - (-_state.pos(2)), measurement_var + P(9, 9));
		}

		// determine if we should use GNSS height aiding
		const bool continuing_conditions_passing = (_params.gnss_ctrl & GnssCtrl::VPOS)
				&& PX4_ISFINITE(measurement)
				&& _NED_origin_initialised
				&& _gps_checks_passed;

		const bool starting_conditions_passing = continuing_conditions_passing
				&& _gps_checks_passed
				&& gps_checks_passing
				&& !gps_checks_failing
				&& isNewestSampleRecent(_time_last_gps_buffer_push, 2 * GPS_MAX_INTERVAL);

		if (_control_status.flags.gps_hgt) {
			aid_src.fusion_enabled = true;

			if (continuing_conditions_passing) {

				fuseVerticalPosition(aid_src);

				const bool is_fusion_failing = isTimedOut(aid_src.time_last_fuse, _params.hgt_fusion_timeout_max);

				if (isHeightResetRequired()) {
					// All height sources are failing
					ECL_INFO("GPS height fusion reset required, all height sources failing");
					resetHeightToGps(measurement, measurement_var);

					// reset vertical velocity
					if (PX4_ISFINITE(gps_sample.vel(2)) && (_params.gnss_ctrl & GnssCtrl::VEL)) {
						resetVerticalVelocityTo(gps_sample.vel(2));
						P.uncorrelateCovarianceSetVariance<1>(6, sq(math::max(0.01f, 1.5f * gps_sample.sacc)));

					} else {
						resetVerticalVelocityToZero();
					}

				} else if (is_fusion_failing) {
					// Some other height source is still working
					ECL_INFO("stopping GPS height fusion, fusion failing");
					stopGpsHgtFusion();
				}

			} else {
				ECL_INFO("stopping GPS height fusion, continuing conditions not passing");
				stopGpsHgtFusion();
			}

		} else {
			if (starting_conditions_passing) {
				if (_params.height_sensor_ref == HeightSensor::GNSS) {
					ECL_INFO("starting GPS height fusion, resetting height to GPS");
					bias_est.reset();
					_height_sensor_ref = HeightSensor::GNSS;
					resetHeightToGps(measurement, measurement_var);

				} else {
					ECL_INFO("starting GPS height fusion");
					bias_est.setBias(_state.pos(2) + measurement);
				}

				aid_src.time_last_fuse = _imu_sample_delayed.time_us;

				_control_status.flags.gps_hgt = true;
				bias_est.setFusionActive();
			}
		}

	} else if (_control_status.flags.gps_hgt && !isNewestSampleRecent(_time_last_gps_buffer_push, 2 * GPS_MAX_INTERVAL)) {
		// No data anymore. Stop until it comes back.
		ECL_INFO("stopping GPS height fusion, no data");
		stopGpsHgtFusion();
	}
}

void Ekf::resetHeightToGps(const float gps_sample_height, const float obs_var)
{
	_information_events.flags.reset_hgt_to_gps = true;

	resetVerticalPositionTo(-(gps_sample_height - _gps_hgt_b_est.getBias()));

	// the state variance is the same as the observation
	P.uncorrelateCovarianceSetVariance<1>(9, obs_var);

	_baro_b_est.setBias(_baro_b_est.getBias() + _state_reset_status.posD_change);
	//_gps_hgt_b_est.setBias(_gps_hgt_b_est.getBias() + _state_reset_status.posD_change);
	_ev_hgt_b_est.setBias(_ev_hgt_b_est.getBias() - _state_reset_status.posD_change);
	_rng_hgt_b_est.setBias(_rng_hgt_b_est.getBias() + _state_reset_status.posD_change);

	_aid_src_gnss_hgt.time_last_fuse = _imu_sample_delayed.time_us;
}

void Ekf::stopGpsHgtFusion()
{
	if (_control_status.flags.gps_hgt) {

		if (_height_sensor_ref == HeightSensor::GNSS) {
			_height_sensor_ref = HeightSensor::UNKNOWN;
		}

		_control_status.flags.gps_hgt = false;
		_gps_hgt_b_est.setFusionInactive();
		resetEstimatorAidStatus(_aid_src_gnss_hgt);
	}
}
