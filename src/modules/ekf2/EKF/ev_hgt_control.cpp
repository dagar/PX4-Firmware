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
 * @file ev_hgt_control.cpp
 * Control functions for ekf external vision height fusion
 */

#include "ekf.h"

void Ekf::controlEvHeightFusion(const extVisionSample &ev_sample, bool starting_conditions_passing, bool ev_reset,
				bool quality_sufficient, estimator_aid_source1d_s &aid_src)
{
	static constexpr const char *HGT_SRC_NAME = "EV";

	HeightBiasEstimator &bias_est = _ev_hgt_b_est;

	bias_est.predict(_dt_ekf_avg);

	// correct position for offset relative to IMU
	const Vector3f pos_offset_body = _params.ev_pos_body - _params.imu_pos_body;
	const Vector3f pos_offset_earth = _R_to_earth * pos_offset_body;

	const float measurement = (ev_sample.pos - pos_offset_earth)(2);
	const float measurement_var = ev_sample.posVar(2);

	const float innov_gate = math::max(_params.ev_pos_innov_gate, 1.f);

	const bool measurement_valid = PX4_ISFINITE(measurement) && PX4_ISFINITE(measurement_var);

	updateVerticalPositionAidSrcStatus(ev_sample.time_us,
					   measurement - bias_est.getBias(),
					   measurement_var + bias_est.getBiasVar(),
					   innov_gate,
					   aid_src);

	// update the bias estimator before updating the main filter but after
	// using its current state to compute the vertical position innovation
	if (measurement_valid && quality_sufficient) {
		bias_est.setMaxStateVar(measurement_var);
		bias_est.setProcessNoiseSpectralDensity(_params.ev_hgt_bias_nsd);
		bias_est.fuseBias(measurement - _state.pos(2), measurement_var + P(9, 9));
	}

	const bool continuing_conditions_passing = ((_params.ev_ctrl & static_cast<int32_t>(EvCtrl::VPOS)) || (_params.height_sensor_ref == HeightSensor::EV))
			&& measurement_valid;

	starting_conditions_passing &= continuing_conditions_passing;

	if (_control_status.flags.ev_hgt) {
		aid_src.fusion_enabled = true;

		if (continuing_conditions_passing) {

			if (ev_reset && quality_sufficient) {
				// EV sample indicates reset
				ECL_INFO("%s height reset", HGT_SRC_NAME);

				if (_height_sensor_ref == HeightSensor::EV) {
					_information_events.flags.reset_hgt_to_ev = true;
					resetVerticalPositionTo(measurement, measurement_var);
					bias_est.reset();

				} else {
					bias_est.setBias(-_state.pos(2) + measurement);
				}

				aid_src.time_last_fuse = _imu_sample_delayed.time_us;

			} else if (quality_sufficient) {
				fuseVerticalPosition(aid_src);

			} else {
				aid_src.innovation_rejected = true;
			}

			const bool is_fusion_failing = isTimedOut(aid_src.time_last_fuse, _params.hgt_fusion_timeout_max);

			if (isHeightResetRequired()) {
				// All height sources are failing
				ECL_WARN("%s height fusion reset required, all height sources failing", HGT_SRC_NAME);

				_information_events.flags.reset_hgt_to_ev = true;
				resetVerticalPositionTo(measurement - bias_est.getBias(), measurement_var);
				bias_est.setBias(-_state.pos(2) + measurement);

				// reset vertical velocity
				if (PX4_ISFINITE(ev_sample.vel(2)) && (_params.ev_ctrl & static_cast<int32_t>(EvCtrl::VEL))) {

					// correct velocity for offset relative to IMU
					const Vector3f vel_offset_body = _ang_rate_delayed_raw % pos_offset_body;
					const Vector3f vel_offset_earth = _R_to_earth * vel_offset_body;

					switch (ev_sample.vel_frame) {
					case VelocityFrame::LOCAL_FRAME_NED:
					case VelocityFrame::LOCAL_FRAME_FRD: {
							const Vector3f reset_vel = ev_sample.vel - vel_offset_earth;
							resetVerticalVelocityTo(reset_vel(2), ev_sample.velVar(2));
						}
						break;

					case VelocityFrame::BODY_FRAME_FRD: {
							const Vector3f reset_vel = _R_to_earth * (ev_sample.vel - vel_offset_body);
							resetVerticalVelocityTo(reset_vel(2), ev_sample.velVar(2));
						}
						break;
					}

				} else {
					resetVerticalVelocityToZero();
				}

				aid_src.time_last_fuse = _imu_sample_delayed.time_us;

			} else if (is_fusion_failing) {
				// Some other height source is still working
				ECL_WARN("stopping %s height fusion, fusion failing", HGT_SRC_NAME);
				stopEvHgtFusion();
			}

		} else {
			ECL_WARN("stopping %s height fusion, continuing conditions failing", HGT_SRC_NAME);
			stopEvHgtFusion();
		}

	} else {
		if (starting_conditions_passing) {
			if (_params.height_sensor_ref == HeightSensor::EV) {
				ECL_INFO("starting %s height fusion, resetting height", HGT_SRC_NAME);
				_height_sensor_ref = HeightSensor::EV;

				_information_events.flags.reset_hgt_to_ev = true;
				resetVerticalPositionTo(measurement, measurement_var);
				bias_est.reset();

			} else {
				ECL_INFO("starting %s height fusion", HGT_SRC_NAME);
				bias_est.setBias(-_state.pos(2) + measurement);
			}

			aid_src.time_last_fuse = _imu_sample_delayed.time_us;
			bias_est.setFusionActive();
			_control_status.flags.ev_hgt = true;
		}
	}
}

void Ekf::stopEvHgtFusion()
{
	if (_control_status.flags.ev_hgt) {

		if (_height_sensor_ref == HeightSensor::EV) {
			_height_sensor_ref = HeightSensor::UNKNOWN;
		}

		_ev_hgt_b_est.setFusionInactive();
		resetEstimatorAidStatus(_aid_src_ev_hgt);

		_control_status.flags.ev_hgt = false;
	}
}
