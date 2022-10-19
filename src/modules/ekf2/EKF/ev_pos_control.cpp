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
 * @file ev_pos_control.cpp
 * Control functions for ekf external vision position fusion
 */

#include "ekf.h"

void Ekf::controlEvPosFusion(const extVisionSample &ev_sample, bool starting_conditions_passing, bool ev_reset,
			     bool quality_sufficient, estimator_aid_source2d_s &aid_src)
{
	static constexpr const char *AID_SRC_NAME = "EV";

	// determine if we should use EV position aiding
	bool continuing_conditions_passing = (_params.ev_ctrl & static_cast<int32_t>(EvCtrl::HPOS))
					     && _control_status.flags.tilt_align
					     && PX4_ISFINITE(ev_sample.pos(0))
					     && PX4_ISFINITE(ev_sample.pos(1));

	// correct position for offset relative to IMU
	const Vector3f pos_offset_body = _params.ev_pos_body - _params.imu_pos_body;
	const Vector3f pos_offset_earth = _R_to_earth * pos_offset_body;

	// rotate measurement into correct earth frame if required
	Vector3f pos{NAN, NAN, NAN};
	Matrix3f pos_cov{};

	switch (ev_sample.pos_frame) {
	case PositionFrame::LOCAL_FRAME_NED:
		if (_control_status.flags.yaw_align) {
			pos = ev_sample.pos - pos_offset_earth;
			pos_cov = matrix::diag(ev_sample.posVar);

			if (_control_status.flags.gps) {
				_ev_pos_b_est.setFusionActive();

			} else {
				_ev_pos_b_est.setFusionInactive();
			}

		} else {
			continuing_conditions_passing = false;
			_ev_pos_b_est.setFusionInactive();
			_ev_pos_b_est.reset();
		}

		break;

	case PositionFrame::LOCAL_FRAME_FRD:
		if (_control_status.flags.ev_yaw) {
			// using EV frame
			pos = ev_sample.pos - pos_offset_earth;
			pos_cov = matrix::diag(ev_sample.posVar);

			_ev_pos_b_est.setFusionInactive();
			_ev_pos_b_est.reset();

		} else {
			// rotate EV to the EKF reference frame
			const Quatf q_error((_state.quat_nominal * ev_sample.quat.inversed()).normalized());
			const Dcmf R_ev_to_ekf = Dcmf(q_error);

			pos = R_ev_to_ekf * ev_sample.pos - pos_offset_earth;
			pos_cov = R_ev_to_ekf * matrix::diag(ev_sample.posVar) * R_ev_to_ekf.transpose();

			if (_control_status.flags.gps) {
				_ev_pos_b_est.setFusionActive();

			} else {
				_ev_pos_b_est.setFusionInactive();
			}
		}

		break;

	default:
		continuing_conditions_passing = false;
		_ev_pos_b_est.setFusionInactive();
		_ev_pos_b_est.reset();
		break;
	}

	const Vector2f measurement{pos(0), pos(1)};

	const Vector2f measurement_var{
		math::max(pos_cov(0, 0), sq(0.01f)),
		math::max(pos_cov(1, 1), sq(0.01f)),
	};

	const bool measurement_valid = measurement.isAllFinite() && measurement_var.isAllFinite();

	updateHorizontalPositionAidSrcStatus(ev_sample.time_us,
					     measurement - _ev_pos_b_est.getBias(),        // observation
					     measurement_var + _ev_pos_b_est.getBiasVar(), // observation variance
					     math::max(_params.ev_pos_innov_gate, 1.f),    // innovation gate
					     aid_src);

	// update the bias estimator before updating the main filter but after
	// using its current state to compute the vertical position innovation
	if (measurement_valid && quality_sufficient) {
		_ev_pos_b_est.setMaxStateNoise(Vector2f(sqrtf(measurement_var(0)), sqrtf(measurement_var(1))));
		_ev_pos_b_est.setProcessNoiseSpectralDensity(_params.ev_hgt_bias_nsd); // TODO
		_ev_pos_b_est.fuseBias(measurement - Vector2f(_state.pos.xy()), measurement_var + Vector2f(P(7, 7), P(8, 8)));
	}

	if (!measurement_valid) {
		continuing_conditions_passing = false;
	}

	starting_conditions_passing &= continuing_conditions_passing
				       && ((Vector2f(aid_src.test_ratio).max() < 1.f) || !_control_status.flags.gps);

	if (_control_status.flags.ev_pos) {
		aid_src.fusion_enabled = true;

		if (continuing_conditions_passing) {

			if (ev_reset && quality_sufficient) {
				// EV sample indicates reset
				ECL_INFO("EV %s height reset", AID_SRC_NAME);

				if (!_control_status.flags.gps) {
					// reset count changed in EV sample, reset vision position unless GPS is active
					_information_events.flags.reset_pos_to_vision = true;
					ECL_INFO("reset horizontal position to EV");
					resetHorizontalPositionTo(measurement);
					P.uncorrelateCovarianceSetVariance<2>(7, measurement_var);

					_ev_pos_b_est.reset();

				} else {
					_ev_pos_b_est.setBias(-Vector2f(_state.pos.xy()) + measurement);
				}

				aid_src.time_last_fuse = _imu_sample_delayed.time_us;

			} else if (quality_sufficient) {
				fuseHorizontalPosition(aid_src);

			} else {
				aid_src.innovation_rejected = true;
			}

			const bool is_fusion_failing = isTimedOut(aid_src.time_last_fuse, _params.no_aid_timeout_max);

			if (is_fusion_failing) {
				bool pos_xy_fusion_failing = isTimedOut(_time_last_hor_pos_fuse, _params.no_aid_timeout_max);

				if ((_nb_ev_pos_reset_available > 0) && quality_sufficient) {

					if (_control_status.flags.gps && !pos_xy_fusion_failing) {
						// reset EV position bias
						_ev_pos_b_est.setBias(-Vector2f(_state.pos.xy()) + measurement);

					} else {
						// Data seems good, attempt a reset
						_information_events.flags.reset_pos_to_vision = true;
						ECL_WARN("reset horiztonal position to EV (fusion failing)");

						if (_control_status.flags.gps) {
							resetHorizontalPositionTo(measurement - _ev_pos_b_est.getBias());
							P.uncorrelateCovarianceSetVariance<2>(7, measurement_var + _ev_pos_b_est.getBiasVar());

							_ev_pos_b_est.setBias(-Vector2f(_state.pos.xy()) + measurement);

						} else {
							resetHorizontalPositionTo(measurement);
							P.uncorrelateCovarianceSetVariance<2>(7, measurement_var);

							_ev_pos_b_est.reset();
						}
					}

					aid_src.time_last_fuse = _imu_sample_delayed.time_us;

					if (_control_status.flags.in_air) {
						_nb_ev_pos_reset_available--;
					}

				} else if (starting_conditions_passing) {
					// Data seems good, but previous reset did not fix the issue
					// something else must be wrong, declare the sensor faulty and stop the fusion
					//_control_status.flags.ev_pos_fault = true;
					ECL_WARN("stopping %s position fusion, starting conditions failing", AID_SRC_NAME);
					stopEvPosFusion();

				} else {
					// A reset did not fix the issue but all the starting checks are not passing
					// This could be a temporary issue, stop the fusion without declaring the sensor faulty
					ECL_WARN("stopping %s position fusion, fusion failing", AID_SRC_NAME);
					stopEvPosFusion();
				}
			}

		} else {
			// Stop fusion but do not declare it faulty
			ECL_WARN("stopping %s position fusion, continuing conditions failing", AID_SRC_NAME);
			stopEvPosFusion();
		}

	} else {
		if (starting_conditions_passing) {
			// activate EV position fusion
			// TODO:  (_params.position_sensor_ref == PositionSensor::EV)
			if (_control_status.flags.gps) {
				ECL_INFO("starting %s position fusion", AID_SRC_NAME);
				_ev_pos_b_est.setBias(-Vector2f(_state.pos.xy()) + measurement);
				_ev_pos_b_est.setFusionActive();

			} else {
				ECL_INFO("starting %s position fusion, resetting position", AID_SRC_NAME);
				//_position_sensor_ref = PositionSensor::EV;

				_information_events.flags.reset_pos_to_vision = true;
				resetHorizontalPositionTo(measurement);
				P.uncorrelateCovarianceSetVariance<2>(7, measurement_var);

				_ev_pos_b_est.reset();
			}

			aid_src.time_last_fuse = _imu_sample_delayed.time_us;

			_nb_ev_pos_reset_available = 5;
			_information_events.flags.starting_vision_pos_fusion = true;
			ECL_INFO("starting vision pos fusion");
			_control_status.flags.ev_pos = true;
		}
	}
}

void Ekf::stopEvPosFusion()
{
	if (_control_status.flags.ev_pos) {
		resetEstimatorAidStatus(_aid_src_ev_pos);
		_control_status.flags.ev_pos = false;
	}
}
