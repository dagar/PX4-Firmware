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
 * @file external_vision_control.cpp
 * Control functions for ekf external vision control
 */

#include "ekf.h"

void Ekf::controlExternalVisionFusion()
{
	// Check for new external vision data
	if (_ev_data_ready) {
		// if the ev data is not in a NED reference frame, then the transformation between EV and EKF navigation frames
		// needs to be calculated and the observations rotated into the EKF frame of reference
		if (_params.fusion_mode & MASK_ROTATE_EV && !_control_status.flags.ev_yaw) {
			// update the rotation matrix which rotates EV measurements into the EKF's navigation frame
			// Calculate the quaternion delta that rotates from the EV to the EKF reference frame at the EKF fusion time horizon.
			const Quatf q_error((_state.quat_nominal * _ev_sample_delayed.quat.inversed()).normalized());
			_R_ev_to_ekf = Dcmf(q_error);

		} else {
			_R_ev_to_ekf.setIdentity();
		}

		// correct position and height for offset relative to IMU
		const Vector3f pos_offset_body = _params.ev_pos_body - _params.imu_pos_body;
		const Vector3f pos_offset_earth = _R_to_earth * pos_offset_body;
		_ev_sample_delayed.pos = _ev_sample_delayed.pos - pos_offset_earth;

		controlEvPosFusion();
		controlEvVelFusion();
		controlEvYawFusion();

		// record observation and estimate for use next time
		_ev_sample_delayed_prev = _ev_sample_delayed;
		_hpos_pred_prev = _state.pos;
		_hpos_prev_available = true;

	} else if ((_control_status.flags.ev_pos || _control_status.flags.ev_vel ||  _control_status.flags.ev_yaw)
		   && isTimedOut(_time_last_ext_vision, (uint64_t)_params.reset_timeout_max)) {

		// Turn off EV fusion mode if no data has been received
		stopEvPosFusion();
		stopEvVelFusion();
		stopEvYawFusion();

		_warning_events.flags.vision_data_stopped = true;
		ECL_WARN("vision data stopped");
	}
}

void Ekf::controlEvPosFusion()
{
	if (!(_params.fusion_mode & MASK_USE_EVPOS)) {
		//if (!(_params.fusion_mode & MASK_USE_EVPOS) || _control_status.flags.ev_pos_fault) {
		stopEvPosFusion();
		return;
	}

	if (_control_status.flags.ev_pos) {
		if (_control_status_prev.flags.gps && !_control_status.flags.gps) {
			// GPS is no longer active
			resetHorizontalPositionToVision();
		}
	}

	if (_ev_data_ready) {

		// determine if we should use the horizontal position observations
		const bool continuing_conditions_passing = isRecent(_aid_src_ev_pos.time_last_fuse[0], 2 * EV_MAX_INTERVAL);
		const bool starting_conditions_passing = continuing_conditions_passing && _control_status.flags.tilt_align
				&& _control_status.flags.yaw_align;

		if (_control_status.flags.ev_pos) {

			// TODO: GPS <=> EV transitions
			if (continuing_conditions_passing) {

				if (_control_status_prev.flags.ev_pos && (_ev_sample_delayed.reset_counter != _ev_sample_delayed_prev.reset_counter)) {
					// reset count changed in EV sample, reset vision position unless GPS is active
					if (!_control_status.flags.gps) {
						resetHorizontalPositionToVision();
					}

				} else {
					// Use an incremental position fusion method for EV position data if GPS is also used
					if (_control_status.flags.gps) {
						// GPS active
						fuseEvPositionDelta();

					} else {
						fuseEvPosition();
					}

					const bool is_fusion_failing = isTimedOut(_aid_src_ev_pos.time_last_fuse[0], _params.reset_timeout_max);

					if (is_fusion_failing) {
						if (_aid_src_ev_pos.num_resets_available > 0) {
							// Data seems good, attempt a reset
							resetHorizontalPositionToVision();

							if (_control_status.flags.in_air) {
								_aid_src_ev_pos.num_resets_available--;
							}

						} else if (starting_conditions_passing) {
							// Data seems good, but previous reset did not fix the issue
							// something else must be wrong, declare the sensor faulty and stop the fusion
							//_control_status.flags.ev_pos_fault = true;
							stopEvPosFusion();

						} else {
							// A reset did not fix the issue but all the starting checks are not passing
							// This could be a temporary issue, stop the fusion without declaring the sensor faulty
							stopEvPosFusion();
						}
					}
				}

			} else {
				// Stop fusion but do not declare it faulty
				stopEvPosFusion();
			}

		} else {
			if (starting_conditions_passing) {
				// Try to activate EV position fusion
				startEvPosFusion();

				if (_control_status.flags.ev_pos) {
					_aid_src_ev_pos.num_resets_available = 3;
				}
			}
		}

	} else if (_control_status.flags.ev_pos && isTimedOut(_aid_src_ev_pos.time_last_fuse[0], _params.reset_timeout_max)) {
		// No data anymore. Stop until it comes back.
		stopEvPosFusion();
	}
}

void Ekf::controlEvVelFusion()
{
	const bool enabled = (_params.fusion_mode & MASK_USE_EVVEL);

	//if (!enabled || _control_status.flags.ev_vel_fault) {
	if (!enabled) {
		stopEvVelFusion();
		//return;
	}

	if (_ev_data_ready) {

		_aid_src_ev_vel.timestamp_sample = _ev_sample_delayed.time_us;

		// correct velocity for offset relative to IMU
		const Vector3f pos_offset_body = _params.ev_pos_body - _params.imu_pos_body;
		const Vector3f vel_offset_body = _ang_rate_delayed_raw % pos_offset_body;
		const Vector3f vel_offset_earth = _R_to_earth * vel_offset_body;

		Vector3f vel{};
		Matrix3f vel_cov{};

		// rotate measurement and covariance into correct earth frame if required
		switch (_ev_sample_delayed.vel_frame) {
		case velocity_frame_t::BODY_FRAME_FRD:
			vel = _R_to_earth * (_ev_sample_delayed.vel - vel_offset_body);
			vel_cov = _R_to_earth * vel_cov * _R_to_earth.transpose();
			break;

		case velocity_frame_t::LOCAL_FRAME_FRD:
			if (_params.fusion_mode & MASK_ROTATE_EV) {
				vel = _R_ev_to_ekf * _ev_sample_delayed.vel - vel_offset_earth;
				vel_cov = _R_ev_to_ekf * vel_cov * _R_ev_to_ekf.transpose();

			} else {
				vel = _ev_sample_delayed.vel - vel_offset_earth;
				vel_cov = _ev_sample_delayed.velCov;
			}

			break;
		}

		const Vector3f obs_var = matrix::max(vel_cov.diag(), sq(0.05f));
		const float innov_gate = fmaxf(_params.ev_vel_innov_gate, 1.f);


		const Vector3f innov{_state.vel - vel};

		const Vector3f innov_var{
			P(4, 4) + obs_var(0),
			P(5, 5) + obs_var(1),
			P(6, 7) + obs_var(2)};

		const Vector3f test_ratio{
			sq(innov(0)) / (sq(innov_gate) * innov_var(0)),
			sq(innov(1)) / (sq(innov_gate) * innov_var(1)),
			sq(innov(2)) / (sq(innov_gate) * innov_var(2))};

		vel.copyTo(_aid_src_ev_vel.observation);
		innov.copyTo(_aid_src_ev_vel.innovation);
		innov_var.copyTo(_aid_src_ev_vel.innovation_variance);
		test_ratio.copyTo(_aid_src_ev_vel.test_ratio);

		const bool continuing_conditions_passing = isRecent(_aid_src_ev_vel.time_last_fuse[0], 2 * EV_MAX_INTERVAL);
		const bool starting_conditions_passing = continuing_conditions_passing && _control_status.flags.tilt_align
				&& _control_status.flags.yaw_align;



		// TODO: test ratio passing or no other options?


		// TODO: enable _ev_vel_status.filter_control_status_fuse[i]


		for (int i = 0; i < 3; i++) {
			const bool test_ratio_passed = (test_ratio(i) > 0.f) && (test_ratio(i) < 1.f);

			_aid_src_ev_vel.innovation_rejected[i] = !test_ratio_passed;

			if (test_ratio_passed) {
				if (_aid_src_ev_vel.fusion_enabled[i]) {
					if (fuseVelPosHeight(innov(i), innov_var(i), i)) {
						_aid_src_ev_vel.time_last_fuse[i] = _time_last_imu;
						_aid_src_ev_vel.fused[i] = false;

					} else {
						_aid_src_ev_vel.fused[i] = true;
					}
				}
			}
		}

		// TODO: continuous data update

		if (_control_status.flags.ev_vel) {

			if (continuing_conditions_passing) {

				if (_control_status_prev.flags.ev_vel && (_ev_sample_delayed.reset_counter != _ev_sample_delayed_prev.reset_counter)) {
					// reset count changed in EV sample
					// resetVelocityToVision();
					_information_events.flags.reset_vel_to_vision = true;
					ECL_INFO("reset to vision velocity");
					resetVelocityTo(vel);
					P.uncorrelateCovarianceSetVariance<3>(4, obs_var);

				} else {
					fuseEvVelocity();

					const bool is_fusion_failing = isTimedOut(_aid_src_ev_vel.time_last_fuse[0], _params.reset_timeout_max);

					if (is_fusion_failing) {
						if (_aid_src_ev_vel.num_resets_available > 0) {
							// Data seems good, attempt a reset
							resetVelocityToVision();

							if (_control_status.flags.in_air) {
								_aid_src_ev_vel.num_resets_available--;
							}

						} else if (starting_conditions_passing) {
							// Data seems good, but previous reset did not fix the issue
							// something else must be wrong, declare the sensor faulty and stop the fusion
							//_control_status.flags.ev_vel_fault = true;
							stopEvVelFusion();

						} else {
							// A reset did not fix the issue but all the starting checks are not passing
							// This could be a temporary issue, stop the fusion without declaring the sensor faulty
							stopEvVelFusion();
						}
					}
				}

			} else {
				// Stop fusion but do not declare it faulty
				stopEvVelFusion();
			}

		} else {
			if (starting_conditions_passing) {
				// Try to activate EV velocity fusion
				startEvVelFusion();

				if (_control_status.flags.ev_vel) {
					_aid_src_ev_vel.num_resets_available = 3;
				}
			}
		}

	} else {
		_aid_src_ev_vel.timestamp_sample = 0;

		if (_control_status.flags.ev_vel && isTimedOut(_aid_src_ev_vel.time_last_fuse[0], _params.reset_timeout_max)) {
			// No data anymore. Stop until it comes back.
			stopEvVelFusion();
		}
	}
}

void Ekf::controlEvYawFusion()
{
	if (!(_params.fusion_mode & MASK_USE_EVYAW)) {
		//if (!(_params.fusion_mode & MASK_USE_EVYAW) || _control_status.flags.ev_vel_fault) {
		stopEvYawFusion();
		return;
	}

	if (_ev_data_ready) {

		// TODO: mag, gps yaw heading review?
		const bool continuing_conditions_passing = isRecent(_aid_src_ev_yaw.time_last_fuse, 2 * EV_MAX_INTERVAL)
				&& !_control_status.flags.gps && !_control_status.flags.gps_yaw;
		const bool starting_conditions_passing = continuing_conditions_passing && _control_status.flags.tilt_align; // TODO

		if (_control_status.flags.ev_yaw) {

			if (continuing_conditions_passing) {

				if (_control_status_prev.flags.ev_yaw && (_ev_sample_delayed.reset_counter != _ev_sample_delayed_prev.reset_counter)) {
					// reset count changed in EV sample
					resetYawToEv();

				} else {
					fuseEvYaw();

					const bool is_fusion_failing = isTimedOut(_aid_src_ev_yaw.time_last_fuse, _params.reset_timeout_max);

					if (is_fusion_failing) {
						if (_aid_src_ev_yaw.num_resets_available > 0) {
							// Data seems good, attempt a reset
							resetYawToEv();

							if (_control_status.flags.in_air) {
								_aid_src_ev_yaw.num_resets_available--;
							}

						} else if (starting_conditions_passing) {
							// Data seems good, but previous reset did not fix the issue
							// something else must be wrong, declare the sensor faulty and stop the fusion
							//_control_status.flags.ev_yaw_fault = true;
							stopEvYawFusion();

						} else {
							// A reset did not fix the issue but all the starting checks are not passing
							// This could be a temporary issue, stop the fusion without declaring the sensor faulty
							stopEvYawFusion();
						}
					}
				}

			} else {
				// Stop fusion but do not declare it faulty
				stopEvYawFusion();
			}

		} else {
			if (starting_conditions_passing) {
				// Try to activate EV yaw fusion
				startEvYawFusion();

				if (_control_status.flags.ev_yaw) {
					_aid_src_ev_yaw.num_resets_available = 3;
				}
			}
		}

	} else if (_control_status.flags.ev_yaw && isTimedOut(_aid_src_ev_yaw.time_last_fuse, _params.reset_timeout_max)) {
		// No data anymore. Stop until it comes back.
		stopEvYawFusion();
	}
}

void Ekf::startEvPosFusion()
{
	if (!_control_status.flags.ev_pos) {
		_control_status.flags.ev_pos = true;

		// TODO: ev_pos vs delta
		if (!(_control_status.flags.gps && isTimedOut(_last_gps_fail_us, (uint64_t)_min_gps_health_time_us))) {
			resetHorizontalPositionToVision();
		}

		_information_events.flags.starting_vision_pos_fusion = true;
		ECL_INFO("starting vision pos fusion");
	}
}

void Ekf::startEvVelFusion()
{
	if (!_control_status.flags.ev_vel) {
		_control_status.flags.ev_vel = true;

		if (!(_control_status.flags.gps && isTimedOut(_last_gps_fail_us, (uint64_t)_min_gps_health_time_us))) {
			resetVelocityToVision();
		}

		_information_events.flags.starting_vision_vel_fusion = true;
		ECL_INFO("starting vision vel fusion");
	}
}

void Ekf::startEvYawFusion()
{
	if (!_control_status.flags.ev_yaw) {
		// turn on fusion of external vision yaw measurements and disable all magnetometer fusion
		_control_status.flags.ev_yaw = true;

		stopMagFusion();

		resetYawToEv();

		_control_status.flags.yaw_align = true;

		_information_events.flags.starting_vision_yaw_fusion = true;
		ECL_INFO("starting vision yaw fusion");
	}
}

void Ekf::stopEvPosFusion()
{
	if (_control_status.flags.ev_pos) {
		_control_status.flags.ev_pos = false;

		// reset
		resetEstimatorAidStatus(_aid_src_ev_pos);
		_hpos_prev_available = false;
	}
}

void Ekf::stopEvVelFusion()
{
	if (_control_status.flags.ev_vel) {
		_control_status.flags.ev_vel = false;

		// reset
		resetEstimatorAidStatus(_aid_src_ev_vel);
	}
}

void Ekf::stopEvYawFusion()
{
	if (_control_status.flags.ev_yaw) {
		_control_status.flags.ev_yaw = false;

		_heading_innov = 0.f;
		_heading_innov_var = 0.f;
		_yaw_test_ratio = 0.f;

		// reset
		resetEstimatorAidStatus(_aid_src_ev_yaw);
	}
}

void Ekf::fuseEvPosition()
{
	if (!PX4_ISFINITE(_ev_hgt_offset)) {
		_ev_hgt_offset = _state.pos(2) - _ev_sample_delayed.pos(2);
	}

	if (!_control_status.flags.ev_hgt) {
		// apply a 10 second first order low pass filter to offset
		const float local_time_step = math::constrain(1e-6f * _delta_time_ev_us, 0.f, 1.f);

		float innov = _state.pos(2) - _ev_sample_delayed.pos(2) - _ev_hgt_offset;
		const float offset_rate_correction = 0.1f * innov;
		_ev_hgt_offset += local_time_step * math::constrain(offset_rate_correction, -0.1f, 0.1f);
	}

	_aid_src_ev_pos.timestamp_sample = _ev_sample_delayed.time_us;

	Vector3f ev_pos_meas = _R_ev_to_ekf * _ev_sample_delayed.pos;

	// observation variance
	Matrix3f ev_pos_var = _R_ev_to_ekf * matrix::diag(_ev_sample_delayed.posVar) * _R_ev_to_ekf.transpose();

	// innovation gate size
	const float innov_gate = fmaxf(_params.ev_pos_innov_gate, 1.f);

	for (int i = 0; i < 3; i++) {

		float obs_var = fmaxf(ev_pos_var(i, i), sq(0.01f));

		_aid_src_ev_pos.observation[i] = ev_pos_meas(i);
		_aid_src_ev_pos.observation_variance[i] = obs_var;

		_aid_src_ev_pos.innovation[i] = _state.pos(i) - ev_pos_meas(i);
		_aid_src_ev_pos.innovation_variance[i] = P(7 + i, 7 + i) + fmaxf(ev_pos_var(i, i), sq(0.01f));
		_aid_src_ev_pos.test_ratio[i] = sq(_aid_src_ev_pos.innovation[i]) / (sq(innov_gate) *
						_aid_src_ev_pos.innovation_variance[i]);

		if (_aid_src_ev_pos.test_ratio[i] > 0.f && _aid_src_ev_pos.test_ratio[i] <= 1.f) {
			// innovation check passed
			_aid_src_ev_pos.innovation_rejected[i] = false;

			if (_aid_src_ev_pos.fusion_enabled[i]) {
				int pos_index = 3 + i;

				if (fuseVelPosHeight(_aid_src_ev_pos.innovation[i], _aid_src_ev_pos.innovation_variance[i], pos_index)) {
					_aid_src_ev_pos.fused[i] = false;
					_aid_src_ev_pos.time_last_fuse[i] = _time_last_imu;

				} else {
					_aid_src_ev_pos.fused[i] = true;
				}
			}

		} else {
			_aid_src_ev_pos.innovation_rejected[i] = true;
		}
	}
}

void Ekf::fuseEvPositionDelta()
{
	if (_hpos_prev_available) {
		_aid_src_ev_pos.timestamp_sample = _ev_sample_delayed.time_us;

		// calculate the change in position since the last measurement
		Vector3f delta_pos = _state.pos - _ev_sample_delayed_prev.pos;

		// rotate measurement into body frame is required when fusing with GPS
		Vector3f ev_delta_pos = _R_ev_to_ekf * Vector3f(_ev_sample_delayed.pos - _ev_sample_delayed_prev.pos);

		// use the change in position since the last measurement
		Vector3f innov = delta_pos - ev_delta_pos;


		// observation variance
		// observation 1-STD error, incremental pos observation is expected to have more uncertainty
		Matrix3f ev_pos_var = matrix::diag(_ev_sample_delayed.posVar);
		ev_pos_var = _R_ev_to_ekf * ev_pos_var * _R_ev_to_ekf.transpose();

		// innovation gate size
		const float innov_gate = fmaxf(_params.ev_pos_innov_gate, 1.f);

		for (int i = 0; i < 3; i++) {

			float obs_var = fmaxf(ev_pos_var(i, i), sq(0.01f));

			_aid_src_ev_pos.observation[i] = delta_pos(i);
			_aid_src_ev_pos.observation_variance[i] = obs_var;

			_aid_src_ev_pos.innovation[i] = innov(i);
			_aid_src_ev_pos.innovation_variance[i] = P(7 + i, 7 + i) + obs_var;
			_aid_src_ev_pos.test_ratio[i] = sq(_aid_src_ev_pos.innovation[i]) / (sq(innov_gate) *
							_aid_src_ev_pos.innovation_variance[i]);

			if (_aid_src_ev_pos.test_ratio[i] > 0.f && _aid_src_ev_pos.test_ratio[i] <= 1.f) {
				// innovation check passed
				_aid_src_ev_pos.innovation_rejected[i] = false;

				if (_aid_src_ev_pos.fusion_enabled[i]) {
					int pos_index = 3 + i;

					if (fuseVelPosHeight(_aid_src_ev_pos.innovation[i], _aid_src_ev_pos.innovation_variance[i], pos_index)) {
						_aid_src_ev_pos.fusion_enabled[i] = false;
						_aid_src_ev_pos.time_last_fuse[i] = _time_last_imu;

					} else {
						_aid_src_ev_pos.fusion_enabled[i] = true;
					}
				}

			} else {
				_aid_src_ev_pos.innovation_rejected[i] = true;
			}
		}
	}
}

void Ekf::fuseEvVelocity()
{
	_aid_src_ev_vel.timestamp_sample = _ev_sample_delayed.time_us;

	Vector3f ev_vel_meas = _R_ev_to_ekf * _ev_sample_delayed.vel;

	// observation variance
	Matrix3f ev_vel_var = _R_ev_to_ekf * matrix::diag(_ev_sample_delayed.posVar) * _R_ev_to_ekf.transpose();

	// innovation gate size
	const float innov_gate = fmaxf(_params.ev_vel_innov_gate, 1.f);

	for (int i = 0; i < 3; i++) {

		float obs_var = fmaxf(ev_vel_var(i, i), sq(0.01f));

		_aid_src_ev_vel.observation[i] = ev_vel_meas(i);
		_aid_src_ev_vel.observation_variance[i] = obs_var;

		_aid_src_ev_vel.innovation[i] = _state.pos(i) - ev_vel_meas(i);
		_aid_src_ev_vel.innovation_variance[i] = P(4 + i, 4 + i) + obs_var;
		_aid_src_ev_vel.test_ratio[i] = sq(_aid_src_ev_vel.innovation[i]) / (sq(innov_gate) *
						_aid_src_ev_vel.innovation_variance[i]);

		if (_aid_src_ev_vel.test_ratio[i] > 0.f && _aid_src_ev_vel.test_ratio[i] <= 1.f) {
			// innovation check passed
			_aid_src_ev_vel.innovation_rejected[i] = false;

			if (_aid_src_ev_vel.fusion_enabled[i]) {
				if (fuseVelPosHeight(_aid_src_ev_vel.innovation[i], _aid_src_ev_vel.innovation_variance[i], i)) {
					_aid_src_ev_vel.fused[i] = false;
					_aid_src_ev_vel.time_last_fuse[i] = _time_last_imu;

				} else {
					_aid_src_ev_vel.fused[i] = true;
				}
			}

		} else {
			_aid_src_ev_vel.innovation_rejected[i] = true;
		}
	}

	// // used by checkVerticalAccelerationHealth
	// _vert_vel_innov_ratio = _external_vision->status_ev_pos.innov[2] / sqrtf(_ev_vel_innov_var(2));
	// _vert_vel_fuse_time_us = _time_last_imu;
}

void Ekf::fuseEvYaw()
{
	_aid_src_ev_yaw.timestamp_sample = _ev_sample_delayed.time_us;

	const Quatf q{_ev_sample_delayed.quat};
	const float angular_variance = fmaxf(_ev_sample_delayed.angVar, 1.e-4f); ///< angular heading variance (rad**2)

	if (shouldUse321RotationSequence(_R_to_earth)) {
		float measured_hdg = getEuler321Yaw(q);

		_aid_src_ev_yaw.observation = measured_hdg;
		_aid_src_ev_yaw.observation_variance = angular_variance;

		// TODO:
		//_aid_src_ev_yaw.innov = wrap_pi(atan2f(_R_to_earth(1, 0), _R_to_earth(0, 0)) - wrap_pi(measured_hdg));
		//_aid_src_ev_yaw.innov_var = P(7 + i, 7 + i) + angular_variance;
		//_aid_src_ev_yaw.test_ratio = sq(_aid_src_ev_yaw.innov[i]) / (sq(innov_gate) * _aid_src_ev_yaw.innov_var[i]);
		if (_aid_src_ev_yaw.fusion_enabled) {
			if (fuseYaw321(measured_hdg, angular_variance)) {
				_aid_src_ev_yaw.fused = false;
				_aid_src_ev_yaw.time_last_fuse = _time_last_imu;

			} else {
				_aid_src_ev_yaw.fused = true;
			}
		}

	} else {
		float measured_hdg = getEuler312Yaw(q);

		_aid_src_ev_yaw.observation = measured_hdg;
		_aid_src_ev_yaw.observation_variance = angular_variance;

		// TODO:
		//_aid_src_ev_yaw.innov = wrap_pi(atan2f(_R_to_earth(1, 0), _R_to_earth(0, 0)) - wrap_pi(measured_hdg));
		//_aid_src_ev_yaw.innov_var = P(7 + i, 7 + i) + angular_variance;
		//_aid_src_ev_yaw.test_ratio = sq(_aid_src_ev_yaw.innov[i]) / (sq(innov_gate) * _aid_src_ev_yaw.innov_var[i]);
		if (_aid_src_ev_yaw.fusion_enabled) {
			if (fuseYaw312(measured_hdg, angular_variance)) {
				_aid_src_ev_yaw.fused = false;
				_aid_src_ev_yaw.time_last_fuse = _time_last_imu;

			} else {
				_aid_src_ev_yaw.fused = true;
			}
		}
	}

	// TODO:
	_aid_src_ev_yaw.innovation = _heading_innov;
	_aid_src_ev_yaw.innovation_variance = _heading_innov_var;
	_aid_src_ev_yaw.test_ratio = _yaw_test_ratio;

	_aid_src_ev_yaw.innovation_rejected = (_aid_src_ev_yaw.test_ratio <= 0.f && _aid_src_ev_yaw.test_ratio > 1.f);
}
