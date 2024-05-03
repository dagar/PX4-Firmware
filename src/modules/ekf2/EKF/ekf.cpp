/****************************************************************************
 *
 *   Copyright (c) 2015-2023 PX4 Development Team. All rights reserved.
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
 * @file ekf.cpp
 * Core functions for ekf attitude and position estimator.
 *
 * @author Roman Bast <bapstroman@gmail.com>
 * @author Paul Riseborough <p_riseborough@live.com.au>
 */

#include "ekf.h"

#include <mathlib/mathlib.h>

void Ekf::reset()
{
	ECL_INFO("reset");

	_time_delayed_us = 0;
	_time_latest_us = 0;

	_output_predictor.reset();

	_imu_updated = false;
	_NED_origin_initialised = false;

	_is_first_imu_sample = true;
	_imu_buffer.reset();
	_imu_down_sampler.reset();

	_accel_lpf.reset({});
	_gyro_lpf.reset({});

	_accel_bias_inhibit[0] = false;
	_accel_bias_inhibit[1] = false;
	_accel_bias_inhibit[2] = false;

	_gyro_bias_inhibit[0] = false;
	_gyro_bias_inhibit[1] = false;
	_gyro_bias_inhibit[2] = false;

	_accel_magnitude_filt = 0.f;
	_ang_rate_magnitude_filt = 0.f;

	_time_bad_vert_accel = 0;
	_time_good_vert_accel = 0;
	_clip_counter = 0;


	_innov_check_fail_status.value = 0;
	_fault_status.value = 0;

	_control_status.value = 0;
	_control_status.flags.in_air = true;
	_control_status_prev.value = _control_status.value;

	_warning_events.value = 0;
	_information_events.value = 0;

	_horizontal_deadreckon_time_exceeded = true;
	_vertical_position_deadreckon_time_exceeded = true;
	_vertical_velocity_deadreckon_time_exceeded = true;

	_time_last_on_ground_us = 0;
	_time_last_in_air = 0;
	_time_last_gnd_effect_on = 0;

	_pos_ref = {};
	_gps_alt_ref = NAN;
	_gpos_origin_eph = 0.f;
	_gpos_origin_epv = 0.f;

	_wmm_gps_time_last_checked = 0;
	_wmm_gps_time_last_set = 0;

	_state_reset_status = {};
	_state_reset_count_prev = {};

	_ang_rate_delayed_raw.zero();

	_state.quat_nominal.setIdentity();
	_state.vel.setZero();
	_state.pos.setZero();
	_state.gyro_bias.setZero();
	_state.accel_bias.setZero();

#if defined(CONFIG_EKF2_MAGNETOMETER)
	_state.mag_I.setZero();
	_state.mag_B.setZero();
#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_WIND)
	_state.wind_vel.setZero();
#endif // CONFIG_EKF2_WIND

	_filter_initialised = false;

	_time_last_horizontal_aiding = 0;
	_time_last_v_pos_aiding = 0;
	_time_last_v_vel_aiding = 0;

	_time_last_hor_pos_fuse = 0;
	_time_last_hgt_fuse = 0;
	_time_last_hor_vel_fuse = 0;
	_time_last_ver_vel_fuse = 0;
	_time_last_heading_fuse = 0;

	_last_known_pos.setZero();
	_time_acc_bias_check = 0;

	_earth_rate_NED = {};

	_R_to_earth.setIdentity();

	_accel_lpf_NE.zero();
	_height_rate_lpf = 0.f;

	P.setZero();


	_height_sensor_ref = HeightSensor::UNKNOWN;



	_aid_src_fake_pos = {};
	_aid_src_fake_hgt = {};

	if (_system_flag_buffer) {
		_system_flag_buffer->reset();
	}


#if defined(CONFIG_EKF2_AIRSPEED)
	if (_airspeed_buffer) {
		_airspeed_buffer->reset();
	}
	_aid_src_airspeed = {};
#endif // CONFIG_EKF2_AIRSPEED


#if defined(CONFIG_EKF2_AUX_GLOBAL_POSITION) && defined(MODULE_NAME)
	//_aux_global_position.reset();
#endif // CONFIG_EKF2_AUX_GLOBAL_POSITION


#if defined(CONFIG_EKF2_AUXVEL)
	if (_auxvel_buffer) {
		_auxvel_buffer->reset();
	}
	_aid_src_aux_vel = {};
#endif // CONFIG_EKF2_AUXVEL


#if defined(CONFIG_EKF2_BAROMETER)
	if (_baro_buffer) {
		_baro_buffer->reset();
	}
	_time_last_baro_buffer_push = 0;
	_aid_src_baro_hgt = {};
	_baro_lpf.reset({});
	_baro_counter = 0;

	_baro_b_est.reset();

	_baro_hgt_faulty = false;
#endif // CONFIG_EKF2_BAROMETER


#if defined(CONFIG_EKF2_DRAG_FUSION)
	if (_drag_buffer) {
		_drag_buffer->reset();
	}
	_drag_down_sampled = {};
	_drag_sample_count = 0;
	_drag_sample_time_dt = 0.f;
	_aid_src_drag = {};
#endif // CONFIG_EKF2_DRAG_FUSION


#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	if (_ext_vision_buffer) {
		_ext_vision_buffer->reset();
	}
	_time_last_ext_vision_buffer_push = 0;

	_aid_src_ev_hgt = {};
	_aid_src_ev_pos = {};
	_aid_src_ev_vel = {};
	_aid_src_ev_yaw = {};

	_ev_hgt_b_est.reset();
	_ev_pos_b_est.reset();
	_ev_q_error_filt.reset({});
	_ev_q_error_initialized = false;

#endif // CONFIG_EKF2_EXTERNAL_VISION


#if defined(CONFIG_EKF2_GNSS)
	if (_gps_buffer) {
		_gps_buffer->reset();
	}
	_time_last_gps_buffer_push = 0;
	_gps_sample_delayed = {};

	resetGpsDriftCheckFilters();

	_gps_pos_prev = {};
	_gps_alt_prev = 0.f;

	_gps_alt_ref = NAN;

	_gps_data_ready = false;
	_gps_pos_deriv_filt.zero();
	_gps_velNE_filt.zero();

	_last_gps_fail_us = 0;
	_last_gps_pass_us = 0;

	_gps_checks_passed = false;

	_gps_check_fail_status.value = 0;

	_gps_intermittent = true;

	_gps_hgt_b_est.reset();

	_aid_src_gnss_hgt = {};
	_aid_src_gnss_pos = {};
	_aid_src_gnss_vel = {};

# if defined(CONFIG_EKF2_GNSS_YAW)
	// innovation consistency check monitoring ratios
	_gnss_yaw_signed_test_ratio_lpf.reset({});
	_time_last_gps_yaw_buffer_push = 0.f;
	_aid_src_gnss_yaw = {};
# endif // CONFIG_EKF2_GNSS_YAW
#endif // CONFIG_EKF2_GNSS


#if defined(CONFIG_EKF2_GRAVITY_FUSION)
	_aid_src_gravity = {};
#endif // CONFIG_EKF2_GRAVITY_FUSION


#if defined(CONFIG_EKF2_MAGNETOMETER)
	if (_mag_buffer) {
		_mag_buffer->reset();
	}
	_time_last_mag_buffer_push = 0;

	_mag_declination_gps = NAN;
	_mag_inclination_gps = NAN;
	_mag_strength_gps = NAN;

	_mag_inclination = NAN;
	_mag_strength = NAN;

	_mag_lpf.reset({});
	_mag_counter = 0;

	_yaw_angle_observable = false;
	_mag_decl_cov_reset = false;

	_aid_src_mag = {};

	_flt_mag_align_start_time = 0;
	_time_last_mag_check_failing = 0;
#endif // CONFIG_EKF2_MAGNETOMETER


#if defined(CONFIG_EKF2_OPTICAL_FLOW)
	if (_flow_buffer) {
		_flow_buffer->reset();
	}
	_flow_sample_delayed = {};
	resetEstimatorAidStatus(_aid_src_optical_flow);

	_flow_gyro_bias.zero();
	_flow_vel_body.zero();
	_flow_vel_ne.zero();
	_ref_body_rate.zero();

	_flow_rate_compensated.zero();
	_flow_data_ready = false;
#endif // CONFIG_EKF2_OPTICAL_FLOW


#if defined(CONFIG_EKF2_RANGE_FINDER)
	_range_sensor.setPitchOffset(_params.rng_sens_pitch);
	_range_sensor.setCosMaxTilt(_params.range_cos_max_tilt);
	_range_sensor.setQualityHysteresis(_params.range_valid_quality_s);

	if (_range_buffer) {
		_range_buffer->reset();
	}
	_time_last_range_buffer_push = 0;

	_range_sensor = {};
	_rng_consistency_check = {};

	_aid_src_rng_hgt = {};

	_rng_hgt_b_est.reset();
#endif // CONFIG_EKF2_RANGE_FINDER


#if defined(CONFIG_EKF2_SIDESLIP)
	_aid_src_sideslip = {};
#endif // CONFIG_EKF2_SIDESLIP


#if defined(CONFIG_EKF2_TERRAIN)
	_terrain_vpos = 0.f;
	_terrain_var = 1e4f;
	_terrain_vpos_reset_counter = 0;

	_hagl_sensor_status = {};
	_last_on_ground_posD = 0.f;

# if defined(CONFIG_EKF2_RANGE_FINDER)
	_aid_src_terrain_range_finder = {};
	_time_last_healthy_rng_data = 0;
# endif // CONFIG_EKF2_RANGE_FINDER

# if defined(CONFIG_EKF2_OPTICAL_FLOW)
	_aid_src_terrain_optical_flow = {};
# endif // CONFIG_EKF2_OPTICAL_FLOW

#endif // CONFIG_EKF2_TERRAIN

	_zero_gyro_update.reset();
	_zero_velocity_update.reset();
}

bool Ekf::update()
{
	if (!_filter_initialised) {

		if (_time_delayed_us == 0) {
			_time_delayed_us = _imu_buffer.get_oldest().time_us;
		}

		if (_time_latest_us == 0) {
			_time_latest_us = _imu_buffer.get_newest().time_us;
		}

		if (!_initialised) {
			_filter_initialised = false;
			_initialised = initialise_interface();
		}

		_filter_initialised = initialiseFilter();

		if (!_initialised || !_filter_initialised) {
			return false;
		}
	}

	// Only run the filter if IMU data in the buffer has been updated
	if (_imu_updated) {
		_imu_updated = false;

		// get the oldest IMU data from the buffer
		// TODO: explicitly pop at desired time horizon
		const imuSample imu_sample_delayed = _imu_buffer.get_oldest();

		// calculate an average filter update time
		//  filter and limit input between -50% and +100% of nominal value
		float input = 0.5f * (imu_sample_delayed.delta_vel_dt + imu_sample_delayed.delta_ang_dt);
		float filter_update_s = 1e-6f * _params.filter_update_interval_us;
		_dt_ekf_avg = 0.99f * _dt_ekf_avg + 0.01f * math::constrain(input, 0.5f * filter_update_s, 2.f * filter_update_s);

		updateIMUBiasInhibit(imu_sample_delayed);

		// perform state and covariance prediction for the main filter
		predictCovariance(imu_sample_delayed);
		predictState(imu_sample_delayed);

		// control fusion of observation data
		controlFusionModes(imu_sample_delayed);

#if defined(CONFIG_EKF2_TERRAIN)
		// run a separate filter for terrain estimation
		runTerrainEstimator(imu_sample_delayed);
#endif // CONFIG_EKF2_TERRAIN

		_output_predictor.correctOutputStates(imu_sample_delayed.time_us, _state.quat_nominal, _state.vel, _state.pos, _state.gyro_bias, _state.accel_bias);

		return true;
	}

	return false;
}

bool Ekf::initialiseFilter()
{
	// Filter accel for tilt initialization
	const imuSample &imu_init = _imu_buffer.get_newest();

	// protect against zero data
	if (imu_init.delta_vel_dt < 1e-4f || imu_init.delta_ang_dt < 1e-4f) {
		return false;
	}

	if (_is_first_imu_sample) {
		_accel_lpf.reset(imu_init.delta_vel / imu_init.delta_vel_dt);
		_gyro_lpf.reset(imu_init.delta_ang / imu_init.delta_ang_dt);
		_is_first_imu_sample = false;

	} else {
		_accel_lpf.update(imu_init.delta_vel / imu_init.delta_vel_dt);
		_gyro_lpf.update(imu_init.delta_ang / imu_init.delta_ang_dt);
	}

	if (!initialiseTilt()) {
		return false;
	}

	// initialise the state covariance matrix now we have starting values for all the states
	initialiseCovariance();

#if defined(CONFIG_EKF2_TERRAIN)
	// Initialise the terrain estimator
	initHagl();
#endif // CONFIG_EKF2_TERRAIN

	// reset the output predictor state history to match the EKF initial values
	_output_predictor.alignOutputFilter(_state.quat_nominal, _state.vel, _state.pos);

	return true;
}

bool Ekf::initialiseTilt()
{
	const float accel_norm = _accel_lpf.getState().norm();
	const float gyro_norm = _gyro_lpf.getState().norm();

	if (accel_norm < 0.8f * CONSTANTS_ONE_G ||
	    accel_norm > 1.2f * CONSTANTS_ONE_G ||
	    gyro_norm > math::radians(15.0f)) {
		return false;
	}

	// get initial tilt estimate from delta velocity vector, assuming vehicle is static
	_state.quat_nominal = Quatf(_accel_lpf.getState(), Vector3f(0.f, 0.f, -1.f));
	_R_to_earth = Dcmf(_state.quat_nominal);

	return true;
}

void Ekf::predictState(const imuSample &imu_delayed)
{
	// apply imu bias corrections
	const Vector3f delta_ang_bias_scaled = getGyroBias() * imu_delayed.delta_ang_dt;
	Vector3f corrected_delta_ang = imu_delayed.delta_ang - delta_ang_bias_scaled;

	// subtract component of angular rate due to earth rotation
	corrected_delta_ang -= _R_to_earth.transpose() * _earth_rate_NED * imu_delayed.delta_ang_dt;

	const Quatf dq(AxisAnglef{corrected_delta_ang});

	// rotate the previous quaternion by the delta quaternion using a quaternion multiplication
	_state.quat_nominal = (_state.quat_nominal * dq).normalized();
	_R_to_earth = Dcmf(_state.quat_nominal);

	// Calculate an earth frame delta velocity
	const Vector3f delta_vel_bias_scaled = getAccelBias() * imu_delayed.delta_vel_dt;
	const Vector3f corrected_delta_vel = imu_delayed.delta_vel - delta_vel_bias_scaled;
	const Vector3f corrected_delta_vel_ef = _R_to_earth * corrected_delta_vel;

	// save the previous value of velocity so we can use trapzoidal integration
	const Vector3f vel_last = _state.vel;

	// calculate the increment in velocity using the current orientation
	_state.vel += corrected_delta_vel_ef;

	// compensate for acceleration due to gravity
	_state.vel(2) += CONSTANTS_ONE_G * imu_delayed.delta_vel_dt;

	// predict position states via trapezoidal integration of velocity
	_state.pos += (vel_last + _state.vel) * imu_delayed.delta_vel_dt * 0.5f;

	// constrain states
	_state.vel = matrix::constrain(_state.vel, -1000.f, 1000.f);
	_state.pos = matrix::constrain(_state.pos, -1.e6f, 1.e6f);

	// some calculations elsewhere in code require a raw angular rate vector so calculate here to avoid duplication
	// protect against possible small timesteps resulting from timing slip on previous frame that can drive spikes into the rate
	// due to insufficient averaging
	if (imu_delayed.delta_ang_dt > 0.25f * _dt_ekf_avg) {
		_ang_rate_delayed_raw = imu_delayed.delta_ang / imu_delayed.delta_ang_dt;
	}


	// calculate a filtered horizontal acceleration with a 1 sec time constant
	// this are used for manoeuvre detection elsewhere
	const float alpha = 1.0f - imu_delayed.delta_vel_dt;
	_accel_lpf_NE = _accel_lpf_NE * alpha + corrected_delta_vel_ef.xy();

	// Calculate low pass filtered height rate
	float alpha_height_rate_lpf = 0.1f * imu_delayed.delta_vel_dt; // 10 seconds time constant
	_height_rate_lpf = _height_rate_lpf * (1.0f - alpha_height_rate_lpf) + _state.vel(2) * alpha_height_rate_lpf;
}

void Ekf::resetGlobalPosToExternalObservation(double lat_deg, double lon_deg, float accuracy, uint64_t timestamp_observation)
{

	if (!_pos_ref.isInitialized()) {
		return;
	}

	// apply a first order correction using velocity at the delated time horizon and the delta time
	timestamp_observation = math::min(_time_latest_us, timestamp_observation);
	const float dt = _time_delayed_us > timestamp_observation ? static_cast<float>(_time_delayed_us - timestamp_observation)
			 * 1e-6f : -static_cast<float>(timestamp_observation - _time_delayed_us) * 1e-6f;

	Vector2f pos_corrected = _pos_ref.project(lat_deg, lon_deg) + _state.vel.xy() * dt;

	resetHorizontalPositionToExternal(pos_corrected, math::max(accuracy, FLT_EPSILON));
}

void Ekf::updateParameters()
{
#if defined(CONFIG_EKF2_AUX_GLOBAL_POSITION) && defined(MODULE_NAME)
	_aux_global_position.updateParameters();
#endif // CONFIG_EKF2_AUX_GLOBAL_POSITION
}

template<typename T>
static void printRingBuffer(const char *name, RingBuffer<T> *rb)
{
	if (rb) {
		printf("%s: %d/%d entries (%d/%d Bytes) (%zu Bytes per entry)\n",
		       name,
		       rb->entries(), rb->get_length(), rb->get_used_size(), rb->get_total_size(),
		       sizeof(T));
	}
}

void Ekf::print_status()
{
	printf("\nStates: (%.4f seconds ago)\n", (_time_latest_us - _time_delayed_us) * 1e-6);
	printf("Orientation (%d-%d): [%.3f, %.3f, %.3f, %.3f] (Euler [%.1f, %.1f, %.1f] deg) var: [%.1e, %.1e, %.1e]\n",
	       State::quat_nominal.idx, State::quat_nominal.idx + State::quat_nominal.dof - 1,
	       (double)_state.quat_nominal(0), (double)_state.quat_nominal(1), (double)_state.quat_nominal(2), (double)_state.quat_nominal(3),
	       (double)math::degrees(matrix::Eulerf(_state.quat_nominal).phi()), (double)math::degrees(matrix::Eulerf(_state.quat_nominal).theta()), (double)math::degrees(matrix::Eulerf(_state.quat_nominal).psi()),
	       (double)getStateVariance<State::quat_nominal>()(0), (double)getStateVariance<State::quat_nominal>()(1), (double)getStateVariance<State::quat_nominal>()(2)
	      );

	printf("Velocity (%d-%d): [%.3f, %.3f, %.3f] var: [%.1e, %.1e, %.1e]\n",
	       State::vel.idx, State::vel.idx + State::vel.dof - 1,
	       (double)_state.vel(0), (double)_state.vel(1), (double)_state.vel(2),
	       (double)getStateVariance<State::vel>()(0), (double)getStateVariance<State::vel>()(1), (double)getStateVariance<State::vel>()(2)
	      );

	printf("Position (%d-%d): [%.3f, %.3f, %.3f] var: [%.1e, %.1e, %.1e]\n",
	       State::pos.idx, State::pos.idx + State::pos.dof - 1,
	       (double)_state.pos(0), (double)_state.pos(1), (double)_state.pos(2),
	       (double)getStateVariance<State::pos>()(0), (double)getStateVariance<State::pos>()(1), (double)getStateVariance<State::pos>()(2)
	      );

	printf("Gyro Bias (%d-%d): [%.6f, %.6f, %.6f] var: [%.1e, %.1e, %.1e]\n",
	       State::gyro_bias.idx, State::gyro_bias.idx + State::gyro_bias.dof - 1,
	       (double)_state.gyro_bias(0), (double)_state.gyro_bias(1), (double)_state.gyro_bias(2),
	       (double)getStateVariance<State::gyro_bias>()(0), (double)getStateVariance<State::gyro_bias>()(1), (double)getStateVariance<State::gyro_bias>()(2)
	      );

	printf("Accel Bias (%d-%d): [%.6f, %.6f, %.6f] var: [%.1e, %.1e, %.1e]\n",
	       State::accel_bias.idx, State::accel_bias.idx + State::accel_bias.dof - 1,
	       (double)_state.accel_bias(0), (double)_state.accel_bias(1), (double)_state.accel_bias(2),
	       (double)getStateVariance<State::accel_bias>()(0), (double)getStateVariance<State::accel_bias>()(1), (double)getStateVariance<State::accel_bias>()(2)
	      );

#if defined(CONFIG_EKF2_MAGNETOMETER)
	printf("Magnetic Field (%d-%d): [%.3f, %.3f, %.3f] var: [%.1e, %.1e, %.1e]\n",
	       State::mag_I.idx, State::mag_I.idx + State::mag_I.dof - 1,
	       (double)_state.mag_I(0), (double)_state.mag_I(1), (double)_state.mag_I(2),
	       (double)getStateVariance<State::mag_I>()(0), (double)getStateVariance<State::mag_I>()(1), (double)getStateVariance<State::mag_I>()(2)
	      );

	printf("Magnetic Bias (%d-%d): [%.3f, %.3f, %.3f] var: [%.1e, %.1e, %.1e]\n",
	       State::mag_B.idx, State::mag_B.idx + State::mag_B.dof - 1,
	       (double)_state.mag_B(0), (double)_state.mag_B(1), (double)_state.mag_B(2),
	       (double)getStateVariance<State::mag_B>()(0), (double)getStateVariance<State::mag_B>()(1),
	       (double)getStateVariance<State::mag_B>()(2)
	      );
#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_WIND)
	printf("Wind velocity (%d-%d): [%.3f, %.3f] var: [%.1e, %.1e]\n",
	       State::wind_vel.idx, State::wind_vel.idx + State::wind_vel.dof - 1,
	       (double)_state.wind_vel(0), (double)_state.wind_vel(1),
	       (double)getStateVariance<State::wind_vel>()(0), (double)getStateVariance<State::wind_vel>()(1)
	      );
#endif // CONFIG_EKF2_WIND

	printf("\nP:\n");
	P.print();

	printf("EKF average dt: %.6f seconds\n", (double)_dt_ekf_avg);
	printf("minimum observation interval %d us\n", _min_obs_interval_us);

	printRingBuffer("IMU buffer", &_imu_buffer);
	printRingBuffer("system flag buffer", _system_flag_buffer);

#if defined(CONFIG_EKF2_AIRSPEED)
	printRingBuffer("airspeed buffer", _airspeed_buffer);
#endif // CONFIG_EKF2_AIRSPEED

#if defined(CONFIG_EKF2_AUXVEL)
	printRingBuffer("aux vel buffer", _auxvel_buffer);
#endif // CONFIG_EKF2_AUXVEL

#if defined(CONFIG_EKF2_BAROMETER)
	printRingBuffer("baro buffer", _baro_buffer);
#endif // CONFIG_EKF2_BAROMETER

#if defined(CONFIG_EKF2_DRAG_FUSION)
	printRingBuffer("drag buffer", _drag_buffer);
#endif // CONFIG_EKF2_DRAG_FUSION

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	printRingBuffer("ext vision buffer", _ext_vision_buffer);
#endif // CONFIG_EKF2_EXTERNAL_VISION

#if defined(CONFIG_EKF2_GNSS)
	printRingBuffer("gps buffer", _gps_buffer);
#endif // CONFIG_EKF2_GNSS

#if defined(CONFIG_EKF2_MAGNETOMETER)
	printRingBuffer("mag buffer", _mag_buffer);
#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_OPTICAL_FLOW)
	printRingBuffer("flow buffer", _flow_buffer);
#endif // CONFIG_EKF2_OPTICAL_FLOW

#if defined(CONFIG_EKF2_RANGE_FINDER)
	printRingBuffer("range buffer", _range_buffer);
#endif // CONFIG_EKF2_RANGE_FINDER


	_output_predictor.print_status();
}
