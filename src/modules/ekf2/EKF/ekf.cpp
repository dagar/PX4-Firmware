/****************************************************************************
 *
 *   Copyright (c) 2015 Estimation and Control Library (ECL). All rights reserved.
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
 * 3. Neither the name ECL nor the names of its contributors may be
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

bool Ekf::init(uint64_t timestamp)
{
	bool ret = initialise_interface(timestamp);
	reset();
	return ret;
}

void Ekf::reset()
{
	_state.vel.setZero();
	_state.pos.setZero();
	_state.delta_ang_bias.setZero();
	_state.delta_vel_bias.setZero();
	_state.mag_I.setZero();
	_state.mag_B.setZero();
	_state.wind_vel.setZero();
	_state.quat_nominal.setIdentity();

	_range_sensor.setPitchOffset(_params.rng_sens_pitch);
	_range_sensor.setCosMaxTilt(_params.range_cos_max_tilt);
	_range_sensor.setQualityHysteresis(_params.range_valid_quality_s);

	_control_status.value = 0;
	_control_status_prev.value = 0;

	_control_status.flags.in_air = true;
	_control_status_prev.flags.in_air = true;

	_ang_rate_delayed_raw.zero();

	_fault_status.value = 0;
	_innov_check_fail_status.value = 0;

	_prev_dvel_bias_var.zero();

	resetGpsDriftCheckFilters();


	// TODO:
	_output_predictor.reset();
}

bool Ekf::update()
{
	bool updated = false;

	if (!_filter_initialised) {
		_filter_initialised = initialiseFilter();

		if (!_filter_initialised) {
			return false;
		}
	}

	// Only run the filter if IMU data in the buffer has been updated
	if (_imu_updated) {
		// perform state and covariance prediction for the main filter
		predictState();
		predictCovariance();

		// control fusion of observation data
		controlFusionModes();

		// run a separate filter for terrain estimation
		runTerrainEstimator();

		updated = true;
	}

	// the output observer always runs

	{
		// Use full rate IMU data at the current time horizon

		// correct delta angles for bias offsets
		const float dt_scale_correction = _dt_imu_avg / _dt_ekf_avg;

		// Apply corrections to the delta angle required to track the quaternion states at the EKF fusion time horizon
		const Vector3f delta_angle(_newest_high_rate_imu_sample.delta_ang - _state.delta_ang_bias * dt_scale_correction);
		const float delta_angle_dt = _newest_high_rate_imu_sample.delta_ang_dt;

		const Vector3f delta_velocity(_newest_high_rate_imu_sample.delta_vel - _state.delta_ang_bias * dt_scale_correction);
		const float delta_velocity_dt = _newest_high_rate_imu_sample.delta_vel_dt;

		_output_predictor.calculateOutputStates(_newest_high_rate_imu_sample.time_us, delta_angle, delta_angle_dt, delta_velocity, delta_velocity_dt);

		if (_imu_updated) {
			_output_predictor.correctOutputStates(_newest_high_rate_imu_sample.time_us, _imu_sample_delayed.time_us, _dt_imu_avg, _dt_ekf_avg,
								_state.quat_nominal, _state.vel, _state.pos);
		}
	}

	return updated;
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

	// Sum the magnetometer measurements
	if (_mag_buffer) {
		magSample mag_sample;

		if (_mag_buffer->pop_first_older_than(_imu_sample_delayed.time_us, &mag_sample)) {
			if (mag_sample.time_us != 0) {
				if (_mag_counter == 0) {
					_mag_lpf.reset(mag_sample.mag);

				} else {
					_mag_lpf.update(mag_sample.mag);
				}

				_mag_counter++;
			}
		}
	}

	// accumulate enough height measurements to be confident in the quality of the data
	if (_baro_buffer && _baro_buffer->pop_first_older_than(_imu_sample_delayed.time_us, &_baro_sample_delayed)) {
		if (_baro_sample_delayed.time_us != 0) {
			if (_baro_counter == 0) {
				_baro_hgt_offset = _baro_sample_delayed.hgt;

			} else {
				_baro_hgt_offset = 0.9f * _baro_hgt_offset + 0.1f * _baro_sample_delayed.hgt;
			}

			_baro_counter++;
		}
	}

	if (_baro_counter < _obs_buffer_length) {
		// not enough baro samples accumulated
		return false;
	}

	// we use baro height initially and switch to GPS/range/EV finder later when it passes checks.
	setControlBaroHeight();

	if (!initialiseTilt()) {
		return false;
	}

	// calculate the initial magnetic field and yaw alignment
	// but do not mark the yaw alignement complete as it needs to be
	// reset once the leveling phase is done
	if (_params.mag_fusion_type <= MagFuseType::MAG_3D) {
		if (_mag_counter > 0) {
			// rotate the magnetometer measurements into earth frame using a zero yaw angle
			// the angle of the projection onto the horizontal gives the yaw angle
			const Vector3f mag_earth_pred = updateYawInRotMat(0.f, _R_to_earth) * _mag_lpf.getState();
			float yaw_new = -atan2f(mag_earth_pred(1), mag_earth_pred(0)) + math::radians(_params.mag_declination_deg);

			// update quaternion states and corresponding covarainces
			// update transformation matrix from body to world frame using the current estimate
			// update the rotation matrix using the new yaw value
			_R_to_earth = updateYawInRotMat(yaw_new, Dcmf(_state.quat_nominal));
			_state.quat_nominal = _R_to_earth;

			// set the earth magnetic field states using the updated rotation
			_state.mag_I = _R_to_earth * _mag_lpf.getState();
			_state.mag_B.zero();

		} else {
			// mag samples required
			return false;
		}
	}

	// initialise the state covariance matrix now we have starting values for all the states
	initialiseCovariance();

	// update the yaw angle variance using the variance of the measurement
	if (_params.mag_fusion_type <= MagFuseType::MAG_3D) {
		// using magnetic heading tuning parameter
		increaseQuatYawErrVariance(sq(fmaxf(_params.mag_heading_noise, 1.0e-2f)));
	}

	// Initialise the terrain estimator
	initHagl();

	// reset the essential fusion timeout counters
	_time_last_hgt_fuse = _time_last_imu;
	_time_last_hor_pos_fuse = _time_last_imu;
	_time_last_hor_vel_fuse = _time_last_imu;
	_time_last_hagl_fuse = _time_last_imu;
	_time_last_flow_terrain_fuse = _time_last_imu;
	_time_last_of_fuse = _time_last_imu;

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

	// get initial roll and pitch estimate from delta velocity vector, assuming vehicle is static
	const Vector3f gravity_in_body = _accel_lpf.getState().normalized();
	const float pitch = asinf(gravity_in_body(0));
	const float roll = atan2f(-gravity_in_body(1), -gravity_in_body(2));

	_state.quat_nominal = Quatf{Eulerf{roll, pitch, 0.0f}};
	_R_to_earth = Dcmf(_state.quat_nominal);

	return true;
}

void Ekf::predictState()
{
	// apply imu bias corrections
	Vector3f corrected_delta_ang = _imu_sample_delayed.delta_ang - _state.delta_ang_bias;

	// subtract component of angular rate due to earth rotation
	corrected_delta_ang -= _R_to_earth.transpose() * _earth_rate_NED * _imu_sample_delayed.delta_ang_dt;

	const Quatf dq(AxisAnglef{corrected_delta_ang});

	// rotate the previous quaternion by the delta quaternion using a quaternion multiplication
	_state.quat_nominal = (_state.quat_nominal * dq).normalized();
	_R_to_earth = Dcmf(_state.quat_nominal);

	// Calculate an earth frame delta velocity
	const Vector3f corrected_delta_vel = _imu_sample_delayed.delta_vel - _state.delta_vel_bias;
	const Vector3f corrected_delta_vel_ef = _R_to_earth * corrected_delta_vel;

	// save the previous value of velocity so we can use trapzoidal integration
	const Vector3f vel_last = _state.vel;

	// calculate the increment in velocity using the current orientation
	_state.vel += corrected_delta_vel_ef;

	// compensate for acceleration due to gravity
	_state.vel(2) += CONSTANTS_ONE_G * _imu_sample_delayed.delta_vel_dt;

	// predict position states via trapezoidal integration of velocity
	_state.pos += (vel_last + _state.vel) * _imu_sample_delayed.delta_vel_dt * 0.5f;

	constrainStates();

	// calculate an average filter update time
	float input = 0.5f * (_imu_sample_delayed.delta_vel_dt + _imu_sample_delayed.delta_ang_dt);

	// filter and limit input between -50% and +100% of nominal value
	const float filter_update_s = 1e-6f * _params.filter_update_interval_us;
	input = math::constrain(input, 0.5f * filter_update_s, 2.f * filter_update_s);
	_dt_ekf_avg = 0.99f * _dt_ekf_avg + 0.01f * input;

	// some calculations elsewhere in code require a raw angular rate vector so calculate here to avoid duplication
	// protect against possible small timesteps resulting from timing slip on previous frame that can drive spikes into the rate
	// due to insufficient averaging
	if (_imu_sample_delayed.delta_ang_dt > 0.25f * _dt_ekf_avg) {
		_ang_rate_delayed_raw = _imu_sample_delayed.delta_ang / _imu_sample_delayed.delta_ang_dt;
	}

	// calculate a filtered horizontal acceleration with a 1 sec time constant
	// this are used for manoeuvre detection elsewhere
	const float alpha = 1.0f - _imu_sample_delayed.delta_vel_dt;
	_accel_lpf_NE = _accel_lpf_NE * alpha + corrected_delta_vel_ef.xy();

	// calculate a yaw change about the earth frame vertical
	const float spin_del_ang_D = corrected_delta_ang.dot(Vector3f(_R_to_earth.row(2)));
	_yaw_delta_ef += spin_del_ang_D;

	// Calculate filtered yaw rate to be used by the magnetometer fusion type selection logic
	// Note fixed coefficients are used to save operations. The exact time constant is not important.
	_yaw_rate_lpf_ef = 0.95f * _yaw_rate_lpf_ef + 0.05f * spin_del_ang_D / _imu_sample_delayed.delta_ang_dt;
}
