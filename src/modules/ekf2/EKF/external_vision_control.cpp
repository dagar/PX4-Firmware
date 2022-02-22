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
	auto &ev = _external_vision[0];

	if (ev.buffer) {
		ev.data_ready = ev.buffer->pop_first_older_than(_imu_sample_delayed.time_us, &ev.sample_delayed);
	}

	// Check for new external vision data
	if (ev.data_ready) {
		// if the ev data is not in a NED reference frame, then the transformation between EV and EKF navigation frames
		// needs to be calculated and the observations rotated into the EKF frame of reference
		if (_params.fusion_mode & MASK_ROTATE_EV && !_control_status.flags.ev_yaw) {
			// update the rotation matrix which rotates EV measurements into the EKF's navigation frame
			// Calculate the quaternion delta that rotates from the EV to the EKF reference frame at the EKF fusion time horizon.
			const Quatf q_error((_state.quat_nominal * ev.sample_delayed.quat.inversed()).normalized());
			ev.R_ev_to_ekf = Dcmf(q_error);

		} else {
			ev.R_ev_to_ekf.setIdentity();
		}

		// correct position and height for offset relative to IMU
		const Vector3f pos_offset_body = _params.ev_pos_body - _params.imu_pos_body;
		const Vector3f pos_offset_earth = _R_to_earth * pos_offset_body;
		ev.sample_delayed.pos = ev.sample_delayed.pos - pos_offset_earth;

		controlEvPosFusion();
		controlEvVelFusion();
		controlEvYawFusion();

		// record observation and estimate for use next time
		ev.sample_delayed_prev = ev.sample_delayed;
		ev.pos_pred_prev = _state.pos;
		ev.hpos_prev_available = true;

	} else if ((_control_status.flags.ev_pos || _control_status.flags.ev_vel ||  _control_status.flags.ev_yaw)
		   && isTimedOut(ev.time_last_measurement, (uint64_t)_params.reset_timeout_max)) {

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

	auto &ev = _external_vision[0];

	// reset
	ev.status_ev_pos.timestamp_sample = 0;

	if (ev.data_ready) {

		// determine if we should use the horizontal position observations
		const bool continuing_conditions_passing = isRecent(ev.status_ev_pos.time_last_fuse[0], 2 * EV_MAX_INTERVAL);
		const bool starting_conditions_passing = continuing_conditions_passing && _control_status.flags.tilt_align
				&& _control_status.flags.yaw_align;

		if (_control_status.flags.ev_pos) {

			// TODO: GPS <=> EV transitions
			if (continuing_conditions_passing) {

				if (_control_status_prev.flags.ev_pos && (ev.sample_delayed.reset_counter != ev.sample_delayed_prev.reset_counter)) {
					// reset count changed in EV sample, reset vision position unless GPS is active
					if (!_control_status.flags.gps) {
						resetHorizontalPositionToVision();
					}

				} else {
					// Use an incremental position fusion method for EV position data if GPS is also used
					if (_control_status.flags.gps) {
						// GPS active
						fuseEvPositionDelta(ev);

					} else {
						fuseEvPosition(ev);
					}

					const bool is_fusion_failing = isTimedOut(ev.status_ev_pos.time_last_fuse[0], _params.reset_timeout_max);

					if (is_fusion_failing) {
						if (ev.status_ev_pos.num_resets_available > 0) {
							// Data seems good, attempt a reset
							resetHorizontalPosition();

							if (_control_status.flags.in_air) {
								ev.status_ev_pos.num_resets_available--;
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
					ev.status_ev_pos.num_resets_available = 3;
				}
			}
		}

	} else if (_control_status.flags.ev_pos && isTimedOut(ev.status_ev_pos.time_last_fuse[0], _params.reset_timeout_max)) {
		// No data anymore. Stop until it comes back.
		stopEvPosFusion();
	}
}

void Ekf::controlEvVelFusion()
{
	const bool enabled = (_params.fusion_mode & MASK_USE_EVVEL);

	auto &ev = _external_vision[0];

	// reset
	ev.status_ev_vel.timestamp_sample = 0;

	//if (!enabled || _control_status.flags.ev_vel_fault) {
	if (!enabled) {
		stopEvVelFusion();
		//return;
	}

	if (ev.data_ready) {

		ev.status_ev_vel.timestamp_sample = ev.sample_delayed.time_us;

		// correct velocity for offset relative to IMU
		const Vector3f pos_offset_body = _params.ev_pos_body - _params.imu_pos_body;
		const Vector3f vel_offset_body = _ang_rate_delayed_raw % pos_offset_body;
		const Vector3f vel_offset_earth = _R_to_earth * vel_offset_body;

		Vector3f vel{};
		Matrix3f vel_cov{};

		// rotate measurement and covariance into correct earth frame if required
		switch (ev.sample_delayed.vel_frame) {
		case velocity_frame_t::BODY_FRAME_FRD:
			vel = _R_to_earth * (ev.sample_delayed.vel - vel_offset_body);
			vel_cov = _R_to_earth * vel_cov * _R_to_earth.transpose();
			break;

		case velocity_frame_t::LOCAL_FRAME_FRD:
			if (_params.fusion_mode & MASK_ROTATE_EV) {
				vel = ev.R_ev_to_ekf * ev.sample_delayed.vel - vel_offset_earth;
				vel_cov = ev.R_ev_to_ekf * vel_cov * ev.R_ev_to_ekf.transpose();

			} else {
				vel = ev.sample_delayed.vel - vel_offset_earth;
				vel_cov = ev.sample_delayed.velCov;
			}

			break;
		}

		const Vector3f obs_var = matrix::max(vel_cov.diag(), sq(0.05f));
		const float innov_gate = fmaxf(ev.vel_innov_gate, 1.f);


		const Vector3f innov{_state.vel - vel};

		const Vector3f innov_var{
			P(4, 4) + obs_var(0),
			P(5, 5) + obs_var(1),
			P(6, 7) + obs_var(2)};

		const Vector3f test_ratio{
			sq(innov(0)) / (sq(innov_gate) * innov_var(0)),
			sq(innov(1)) / (sq(innov_gate) * innov_var(1)),
			sq(innov(2)) / (sq(innov_gate) * innov_var(2))};

		vel.copyTo(ev.status_ev_vel.measurement_adjusted);
		innov.copyTo(ev.status_ev_vel.innov);
		innov_var.copyTo(ev.status_ev_vel.innov_var);
		test_ratio.copyTo(ev.status_ev_vel.test_ratio);


		const bool continuing_conditions_passing = isRecent(ev.status_ev_vel.time_last_fuse[0], 2 * EV_MAX_INTERVAL);
		const bool starting_conditions_passing = continuing_conditions_passing && _control_status.flags.tilt_align
				&& _control_status.flags.yaw_align;



		// TODO: test ratio passing or no other options?


		// TODO: enable _ev_vel_status.filter_control_status_fuse[i]


		for (int i = 0; i < 3; i++) {
			const bool test_ratio_passed = (test_ratio(i) > 0.f) && (test_ratio(i) < 1.f);

			ev.status_ev_vel.innovation_fault_status_rejected[i] = !test_ratio_passed;

			if (test_ratio_passed) {
				if (ev.status_ev_vel.filter_control_status_fuse[i]) {
					if (fuseVelPosHeight(innov(i), innov_var(i), i)) {
						ev.status_ev_vel.time_last_fuse[i] = _time_last_imu;
						ev.status_ev_vel.fault_status_numerical_error[i] = false;

					} else {
						ev.status_ev_vel.fault_status_numerical_error[i] = true;
					}
				}
			}
		}

		// TODO: continous data update

		if (_control_status.flags.ev_vel) {

			if (continuing_conditions_passing) {

				if (_control_status_prev.flags.ev_vel && (ev.sample_delayed.reset_counter != ev.sample_delayed_prev.reset_counter)) {
					// reset count changed in EV sample
					// resetVelocityToVision();
					_information_events.flags.reset_vel_to_vision = true;
					ECL_INFO("reset to vision velocity");
					resetVelocityTo(vel);
					P.uncorrelateCovarianceSetVariance<3>(4, obs_var);

				} else {
					fuseEvVelocity(ev);

					const bool is_fusion_failing = isTimedOut(ev.status_ev_vel.time_last_fuse[0], _params.reset_timeout_max);

					if (is_fusion_failing) {
						if (ev.status_ev_vel.num_resets_available > 0) {
							// Data seems good, attempt a reset
							resetVelocity();

							if (_control_status.flags.in_air) {
								ev.status_ev_vel.num_resets_available--;
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
					ev.status_ev_vel.num_resets_available = 3;
				}
			}
		}

	} else {
		ev.status_ev_vel.timestamp_sample = 0;

		if (_control_status.flags.ev_vel && isTimedOut(ev.status_ev_vel.time_last_fuse[0], _params.reset_timeout_max)) {
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

	auto &ev = _external_vision[0];

	// reset
	ev.status_ev_yaw.timestamp_sample = 0;

	if (ev.data_ready) {

		// TODO: mag, gps yaw heading review?
		const bool continuing_conditions_passing = isRecent(ev.status_ev_yaw.time_last_fuse, 2 * EV_MAX_INTERVAL)
				&& !_control_status.flags.gps && !_control_status.flags.gps_yaw;
		const bool starting_conditions_passing = continuing_conditions_passing && _control_status.flags.tilt_align; // TODO

		if (_control_status.flags.ev_yaw) {

			if (continuing_conditions_passing) {

				if (_control_status_prev.flags.ev_yaw && (ev.sample_delayed.reset_counter != ev.sample_delayed_prev.reset_counter)) {
					// reset count changed in EV sample
					resetYawToEv();

				} else {
					fuseEvYaw(ev);

					const bool is_fusion_failing = isTimedOut(ev.status_ev_yaw.time_last_fuse, _params.reset_timeout_max);

					if (is_fusion_failing) {
						if (ev.status_ev_yaw.num_resets_available > 0) {
							// Data seems good, attempt a reset
							resetYawToEv();

							if (_control_status.flags.in_air) {
								ev.status_ev_yaw.num_resets_available--;
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
					ev.status_ev_yaw.num_resets_available = 3;
				}
			}
		}

	} else if (_control_status.flags.ev_yaw && isTimedOut(ev.status_ev_yaw.time_last_fuse, _params.reset_timeout_max)) {
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
			// resetVelocityToVision();
			_information_events.flags.reset_vel_to_vision = true;
			//ECL_INFO("reset to vision velocity");
			//resetVelocityTo(vel);
			//P.uncorrelateCovarianceSetVariance<3>(4, getVisionVelocityVarianceInEkfFrame());
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
		auto &ev = _external_vision[0];
		ev.hpos_prev_available = false;
		ev.status_ev_pos = {};
	}
}

void Ekf::stopEvVelFusion()
{
	if (_control_status.flags.ev_vel) {
		_control_status.flags.ev_vel = false;

		// reset
		auto &ev = _external_vision[0];
		ev.status_ev_vel = {};
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
		auto &ev = _external_vision[0];
		ev.status_ev_yaw = {};
	}
}

void Ekf::fuseEvPosition(AidSourceVision &ev)
{
	ev.status_ev_pos.timestamp_sample = ev.sample_delayed.time_us;

	Vector3f ev_pos_meas = ev.R_ev_to_ekf * ev.sample_delayed.pos;

	// observation variance
	Matrix3f ev_pos_var = ev.R_ev_to_ekf * matrix::diag(ev.sample_delayed.posVar) * ev.R_ev_to_ekf.transpose();

	// innovation gate size
	const float innov_gate = fmaxf(_params.ev_pos_innov_gate, 1.f);

	for (int i = 0; i < 3; i++) {

		float obs_var = fmaxf(ev_pos_var(i, i), sq(0.01f));

		ev.status_ev_pos.measurement_adjusted[i] = ev_pos_meas(i);
		ev.status_ev_pos.measurement_variance[i] = obs_var;

		ev.status_ev_pos.innov[i] = _state.pos(i) - ev_pos_meas(i);
		ev.status_ev_pos.innov_var[i] = P(7 + i, 7 + i) + fmaxf(ev_pos_var(i, i), sq(0.01f));
		ev.status_ev_pos.test_ratio[i] = sq(ev.status_ev_pos.innov[i]) / (sq(innov_gate) * ev.status_ev_pos.innov_var[i]);

		if (ev.status_ev_pos.test_ratio[i] > 0.f && ev.status_ev_pos.test_ratio[i] <= 1.f) {
			// innovation check passed
			ev.status_ev_pos.innovation_fault_status_rejected[i] = false;

			if (ev.status_ev_pos.filter_control_status_fuse[i]) {
				int pos_index = 3 + i;

				if (fuseVelPosHeight(ev.status_ev_pos.innov[i], ev.status_ev_pos.innov_var[i], pos_index)) {
					ev.status_ev_pos.fault_status_numerical_error[i] = false;
					ev.status_ev_pos.time_last_fuse[i] = _time_last_imu;

				} else {
					ev.status_ev_pos.fault_status_numerical_error[i] = true;
				}
			}

		} else {
			ev.status_ev_pos.innovation_fault_status_rejected[i] = true;
		}
	}
}

void Ekf::fuseEvPositionDelta(AidSourceVision &ev)
{
	if (ev.hpos_prev_available) {
		ev.status_ev_pos.timestamp_sample = ev.sample_delayed.time_us;

		// calculate the change in position since the last measurement
		Vector3f delta_pos = _state.pos - ev.pos_pred_prev;

		// rotate measurement into body frame is required when fusing with GPS
		Vector3f ev_delta_pos = ev.R_ev_to_ekf * Vector3f(ev.sample_delayed.pos - ev.sample_delayed_prev.pos);

		// use the change in position since the last measurement
		Vector3f innov = delta_pos - ev_delta_pos;


		// observation variance
		// observation 1-STD error, incremental pos observation is expected to have more uncertainty
		Matrix3f ev_pos_var = matrix::diag(ev.sample_delayed.posVar);
		ev_pos_var = ev.R_ev_to_ekf * ev_pos_var * ev.R_ev_to_ekf.transpose();

		// innovation gate size
		const float innov_gate = fmaxf(_params.ev_pos_innov_gate, 1.f);

		for (int i = 0; i < 3; i++) {

			float obs_var = fmaxf(ev_pos_var(i, i), sq(0.01f));

			ev.status_ev_pos.measurement_adjusted[i] = delta_pos(i);
			ev.status_ev_pos.measurement_variance[i] = obs_var;

			ev.status_ev_pos.innov[i] = innov(i);
			ev.status_ev_pos.innov_var[i] = P(7 + i, 7 + i) + obs_var;
			ev.status_ev_pos.test_ratio[i] = sq(ev.status_ev_pos.innov[i]) / (sq(innov_gate) * ev.status_ev_pos.innov_var[i]);

			if (ev.status_ev_pos.test_ratio[i] > 0.f && ev.status_ev_pos.test_ratio[i] <= 1.f) {
				// innovation check passed
				ev.status_ev_pos.innovation_fault_status_rejected[i] = false;

				if (ev.status_ev_pos.filter_control_status_fuse[i]) {
					int pos_index = 3 + i;

					if (fuseVelPosHeight(ev.status_ev_pos.innov[i], ev.status_ev_pos.innov_var[i], pos_index)) {
						ev.status_ev_pos.fault_status_numerical_error[i] = false;
						ev.status_ev_pos.time_last_fuse[i] = _time_last_imu;

					} else {
						ev.status_ev_pos.fault_status_numerical_error[i] = true;
					}
				}

			} else {
				ev.status_ev_pos.innovation_fault_status_rejected[i] = true;
			}
		}
	}
}

void Ekf::fuseEvVelocity(AidSourceVision &ev)
{
	ev.status_ev_vel.timestamp_sample = ev.sample_delayed.time_us;

	Vector3f ev_vel_meas = ev.R_ev_to_ekf * ev.sample_delayed.vel;

	// observation variance
	Matrix3f ev_vel_var = ev.R_ev_to_ekf * matrix::diag(ev.sample_delayed.posVar) * ev.R_ev_to_ekf.transpose();

	// innovation gate size
	const float innov_gate = fmaxf(_params.ev_vel_innov_gate, 1.f);

	for (int i = 0; i < 3; i++) {

		float obs_var = fmaxf(ev_vel_var(i, i), sq(0.01f));

		ev.status_ev_vel.measurement_adjusted[i] = ev_vel_meas(i);
		ev.status_ev_vel.measurement_variance[i] = obs_var;

		ev.status_ev_vel.innov[i] = _state.pos(i) - ev_vel_meas(i);
		ev.status_ev_vel.innov_var[i] = P(4 + i, 4 + i) + obs_var;
		ev.status_ev_vel.test_ratio[i] = sq(ev.status_ev_vel.innov[i]) / (sq(innov_gate) * ev.status_ev_vel.innov_var[i]);

		if (ev.status_ev_vel.test_ratio[i] > 0.f && ev.status_ev_vel.test_ratio[i] <= 1.f) {
			// innovation check passed
			ev.status_ev_vel.innovation_fault_status_rejected[i] = false;

			if (ev.status_ev_vel.filter_control_status_fuse[i]) {
				if (fuseVelPosHeight(ev.status_ev_vel.innov[i], ev.status_ev_vel.innov_var[i], i)) {
					ev.status_ev_vel.fault_status_numerical_error[i] = false;
					ev.status_ev_vel.time_last_fuse[i] = _time_last_imu;

				} else {
					ev.status_ev_vel.fault_status_numerical_error[i] = true;
				}
			}

		} else {
			ev.status_ev_vel.innovation_fault_status_rejected[i] = true;
		}
	}

	// // used by checkVerticalAccelerationHealth
	// _vert_vel_innov_ratio = _external_vision->status_ev_pos.innov[2] / sqrtf(_ev_vel_innov_var(2));
	// _vert_vel_fuse_time_us = _time_last_imu;
}

void Ekf::fuseEvYaw(AidSourceVision &ev)
{
	ev.status_ev_yaw.timestamp_sample = ev.sample_delayed.time_us;

	const Quatf q{ev.sample_delayed.quat};
	const float angular_variance = fmaxf(ev.sample_delayed.angVar, 1.e-4f); ///< angular heading variance (rad**2)

	if (shouldUse321RotationSequence(_R_to_earth)) {
		float measured_hdg = getEuler321Yaw(q);

		ev.status_ev_yaw.measurement_adjusted = measured_hdg;
		ev.status_ev_yaw.measurement_variance = angular_variance;

		// TODO:
		//ev.status_ev_yaw.innov = wrap_pi(atan2f(_R_to_earth(1, 0), _R_to_earth(0, 0)) - wrap_pi(measured_hdg));
		//ev.status_ev_yaw.innov_var = P(7 + i, 7 + i) + angular_variance;
		//ev.status_ev_yaw.test_ratio = sq(ev.status_ev_yaw.innov[i]) / (sq(innov_gate) * ev.status_ev_yaw.innov_var[i]);
		if (ev.status_ev_yaw.filter_control_status_fuse) {
			if (fuseYaw321(measured_hdg, angular_variance)) {
				ev.status_ev_yaw.fault_status_numerical_error = false;
				ev.status_ev_yaw.time_last_fuse = _time_last_imu;

			} else {
				ev.status_ev_yaw.fault_status_numerical_error = true;
			}
		}

	} else {
		float measured_hdg = getEuler312Yaw(q);

		ev.status_ev_yaw.measurement_adjusted = measured_hdg;
		ev.status_ev_yaw.measurement_variance = angular_variance;

		// TODO:
		//ev.status_ev_yaw.innov = wrap_pi(atan2f(_R_to_earth(1, 0), _R_to_earth(0, 0)) - wrap_pi(measured_hdg));
		//ev.status_ev_yaw.innov_var = P(7 + i, 7 + i) + angular_variance;
		//ev.status_ev_yaw.test_ratio = sq(ev.status_ev_yaw.innov[i]) / (sq(innov_gate) * ev.status_ev_yaw.innov_var[i]);
		if (ev.status_ev_yaw.filter_control_status_fuse) {
			if (fuseYaw312(measured_hdg, angular_variance)) {
				ev.status_ev_yaw.fault_status_numerical_error = false;
				ev.status_ev_yaw.time_last_fuse = _time_last_imu;

			} else {
				ev.status_ev_yaw.fault_status_numerical_error = true;
			}
		}
	}

	// TODO:
	ev.status_ev_yaw.innov = _heading_innov;
	ev.status_ev_yaw.innov_var = _heading_innov_var;
	ev.status_ev_yaw.test_ratio = _yaw_test_ratio;

	ev.status_ev_yaw.innovation_fault_status_rejected = (ev.status_ev_yaw.test_ratio > 0.f
			&& ev.status_ev_yaw.test_ratio <= 1.f);
}
