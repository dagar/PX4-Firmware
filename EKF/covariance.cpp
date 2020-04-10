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
 * @file covariance.cpp
 * Contains functions for initialising, predicting and updating the state
 * covariance matrix
 *
 * @author Roman Bast <bastroman@gmail.com>
 *
 */

#include "ekf.h"

#include <ecl.h>
#include <math.h>
#include <mathlib/mathlib.h>

void Ekf::initialiseCovariance()
{
	for (unsigned i = 0; i < _k_num_states; i++) {
		for (unsigned j = 0; j < _k_num_states; j++) {
			P[i][j] = 0.0f;
		}
	}

	_delta_angle_bias_var_accum.setZero();
	_delta_vel_bias_var_accum.setZero();

	// calculate average prediction time step in sec
	float dt = FILTER_UPDATE_PERIOD_S;

	// define the initial angle uncertainty as variances for a rotation vector
	Vector3f rot_vec_var;
	rot_vec_var(2) = rot_vec_var(1) = rot_vec_var(0) = sq(_params.initial_tilt_err);

	// update the quaternion state covariances
	initialiseQuatCovariances(rot_vec_var);

	// velocity
	P[4][4] = sq(fmaxf(_params.gps_vel_noise, 0.01f));
	P[5][5] =(double)P[4][4];
	P[6][6] = sq(1.5f) *P[4][4];

	// position
	P[7][7] = sq(fmaxf(_params.gps_pos_noise, 0.01f));
	P[8][8] =P[7][7];

	if (_control_status.flags.rng_hgt) {
		P[9][9] = sq(fmaxf(_params.range_noise, 0.01f));

	} else if (_control_status.flags.gps_hgt) {
		float lower_limit = fmaxf(_params.gps_pos_noise, 0.01f);
		float upper_limit = fmaxf(_params.pos_noaid_noise, lower_limit);
		P[9][9] = sq(1.5f * math::constrain(_gps_sample_delayed.vacc, lower_limit, upper_limit));

	} else {
		P[9][9] = sq(fmaxf(_params.baro_noise, 0.01f));
	}

	// gyro bias
	P[10][10] = sq(_params.switch_on_gyro_bias * dt);
	P[11][11] =P[10][10];
	P[12][12] =P[10][10];

	// accel bias
	_prev_dvel_bias_var(0) =P[13][13] = sq(_params.switch_on_accel_bias * dt);
	_prev_dvel_bias_var(1) =P[14][14] =P[13][13];
	_prev_dvel_bias_var(2) =P[15][15] =P[13][13];

	// record IMU bias state covariance reset time - used to prevent resets being performed too often
	_last_imu_bias_cov_reset_us = _imu_sample_delayed.time_us;

	// variances for optional states

	// earth frame and body frame magnetic field
	// set to observation variance
	for (uint8_t index = 16; index <= 21; index ++) {
		P[index][index] = sq(_params.mag_noise);
	}

	// save covariance data for re-use when auto-switching between heading and 3-axis fusion
	save_mag_cov_data();

	// wind
	P[22][22] = sq(_params.initial_wind_uncertainty);
	P[23][23] = sq(_params.initial_wind_uncertainty);

}

void Ekf::get_pos_var(Vector3f &pos_var)
{
	pos_var(0) =P[7][7];
	pos_var(1) =P[8][8];
	pos_var(2) =P[9][9];
}

void Ekf::get_vel_var(Vector3f &vel_var)
{
	vel_var(0) =P[4][4];
	vel_var(1) =P[5][5];
	vel_var(2) =P[6][6];
}

void Ekf::predictCovariance()
{
	// assign intermediate state variables
	double q0 = _state.quat_nominal(0);
	double q1 = _state.quat_nominal(1);
	double q2 = _state.quat_nominal(2);
	double q3 = _state.quat_nominal(3);

	double dax = _imu_sample_delayed.delta_ang(0);
	double day = _imu_sample_delayed.delta_ang(1);
	double daz = _imu_sample_delayed.delta_ang(2);

	double dvx = _imu_sample_delayed.delta_vel(0);
	double dvy = _imu_sample_delayed.delta_vel(1);
	double dvz = _imu_sample_delayed.delta_vel(2);

	//printf("\n acc: %.5f, %.5f, %.5f\n", (double)dvx, (double)dvy, (double)dvz);

	double dax_b = _state.gyro_bias(0);
	double day_b = _state.gyro_bias(1);
	double daz_b = _state.gyro_bias(2);

	double dvx_b = _state.accel_bias(0)=0.0f;
	double dvy_b = _state.accel_bias(1)=0.0f;
	double dvz_b = _state.accel_bias(2)=0.0f;

/*printf(" \n dt ang: %.9f, constrained: %.9f, limits: (%.9f, %.9f) \n", (double)_imu_sample_delayed.delta_ang_dt,
	   (double)(math::constrain(_imu_sample_delayed.delta_ang_dt, 0.5f * FILTER_UPDATE_PERIOD_S, 2.0f * FILTER_UPDATE_PERIOD_S)),
	   (double)(0.5f * FILTER_UPDATE_PERIOD_S), (double)(2.0f * FILTER_UPDATE_PERIOD_S));*/
	float dt = math::constrain(_imu_sample_delayed.delta_ang_dt, 0.5f * FILTER_UPDATE_PERIOD_S, 2.0f * FILTER_UPDATE_PERIOD_S);
	float dt_inv = 1.0f / dt;

	// compute noise variance for stationary processes
	double process_noise[_k_num_states] = {};

	// convert rate of change of rate gyro bias (rad/s**2) as specified by the parameter to an expected change in delta angle (rad) since the last update
	double d_ang_bias_sig = dt * dt * math::constrain(_params.gyro_bias_p_noise, 0.0f, 1.0f);

	// convert rate of change of accelerometer bias (m/s**3) as specified by the parameter to an expected change in delta velocity (m/s) since the last update
	double d_vel_bias_sig = dt * dt * math::constrain(_params.accel_bias_p_noise, 0.0f, 1.0f);

	// inhibit learning of imu accel bias if the manoeuvre levels are too high to protect against the effect of sensor nonlinearities or bad accel data is detected
	float alpha = math::constrain((dt / _params.acc_bias_learn_tc), 0.0f, 1.0f);
	float beta = 1.0f - alpha;
	_ang_rate_mag_filt = fmaxf(dt_inv * _imu_sample_delayed.delta_ang.norm(), beta * _ang_rate_mag_filt);
	_accel_mag_filt = fmaxf(dt_inv * _imu_sample_delayed.delta_vel.norm(), beta * _accel_mag_filt);
	_accel_vec_filt(0) = alpha * dt_inv * _imu_sample_delayed.delta_vel(0) + beta * _accel_vec_filt(0);
	_accel_vec_filt(1) = alpha * dt_inv * _imu_sample_delayed.delta_vel(1) + beta * _accel_vec_filt(1);
	_accel_vec_filt(2) = alpha * dt_inv * _imu_sample_delayed.delta_vel(2) + beta * _accel_vec_filt(2);

	if (_ang_rate_mag_filt > _params.acc_bias_learn_gyr_lim
	    || _accel_mag_filt > _params.acc_bias_learn_acc_lim
	    || _bad_vert_accel_detected) {

		// store the bias state variances to be reinstated later
		if (!_accel_bias_inhibit) {
			_prev_dvel_bias_var(0) =P[13][13];
			_prev_dvel_bias_var(1) =P[14][14];
			_prev_dvel_bias_var(2) =P[15][15];
		}

		_accel_bias_inhibit = true;

	} else {
		if (_accel_bias_inhibit) {
			// reinstate the bias state variances
			P[13][13] = _prev_dvel_bias_var(0);
			P[14][14] = _prev_dvel_bias_var(1);
			P[15][15] = _prev_dvel_bias_var(2);

		} else {
			// store the bias state variances to be reinstated later
			_prev_dvel_bias_var(0) =P[13][13];
			_prev_dvel_bias_var(1) =P[14][14];
			_prev_dvel_bias_var(2) =P[15][15];
		}

		_accel_bias_inhibit = false;
	}

	// Don't continue to grow the earth field variances if they are becoming too large or we are not doing 3-axis fusion as this can make the covariance matrix badly conditioned
	float mag_I_sig;

	if (_control_status.flags.mag_3D && (P[16][16] + P[17][17] + P[18][18]) < 0.1f) {
		mag_I_sig = dt * math::constrain(_params.mage_p_noise, 0.0f, 1.0f);

	} else {
		mag_I_sig = 0.0f;
	}

	// Don't continue to grow the body field variances if they is becoming too large or we are not doing 3-axis fusion as this can make the covariance matrix badly conditioned
	float mag_B_sig;

	if (_control_status.flags.mag_3D && (P[19][19] +P[20][20] +P[21][21]) < 0.1f) {
		mag_B_sig = dt * math::constrain(_params.magb_p_noise, 0.0f, 1.0f);

	} else {
		mag_B_sig = 0.0f;
	}

	float wind_vel_sig;

	// Calculate low pass filtered height rate
	float alpha_height_rate_lpf = 0.1f * dt; // 10 seconds time constant
	_height_rate_lpf = _height_rate_lpf * (1.0f - alpha_height_rate_lpf) + _state.vel(2) * alpha_height_rate_lpf;

	// Don't continue to grow wind velocity state variances if they are becoming too large or we are not using wind velocity states as this can make the covariance matrix badly conditioned
	if (_control_status.flags.wind && (P[22][22] +P[23][23]) < sq(_params.initial_wind_uncertainty)) {
		wind_vel_sig = dt * math::constrain(_params.wind_vel_p_noise, 0.0f, 1.0f) * (1.0f + _params.wind_vel_p_noise_scaler * fabsf(_height_rate_lpf));

	} else {
		wind_vel_sig = 0.0f;
	}

	// Construct the process noise variance diagonal for those states with a stationary process model
	// These are kinematic states and their error growth is controlled separately by the IMU noise variances
	for (unsigned i = 0; i <= 9; i++) {
		process_noise[i] = 0.0f;
	}

	// delta angle bias states
	process_noise[12] = process_noise[11] = process_noise[10] = dsq(d_ang_bias_sig);
	// delta_velocity bias states
	process_noise[15] = process_noise[14] = process_noise[13] = dsq(d_vel_bias_sig);
	// earth frame magnetic field states
	process_noise[18] = process_noise[17] = process_noise[16] = dsq(mag_I_sig);
	// body frame magnetic field states
	process_noise[21] = process_noise[20] = process_noise[19] = dsq(mag_B_sig);
	// wind velocity states
	process_noise[23] = process_noise[22] = dsq(wind_vel_sig);

	// assign IMU noise variances
	// inputs to the system are 3 delta angles and 3 delta velocities
	double daxVar, dayVar, dazVar;
	double dvxVar, dvyVar, dvzVar;
	float gyro_noise = math::constrain(_params.gyro_noise, 0.0f, 1.0f);
	daxVar = dayVar = dazVar = dsq(dt * gyro_noise);
	float accel_noise = math::constrain(_params.accel_noise, 0.0f, 1.0f);

	if (_bad_vert_accel_detected) { PX4_ERR("bad accel vert detected");
		// Increase accelerometer process noise if bad accel data is detected. Measurement errors due to
		// vibration induced clipping commonly reach an equivalent 0.5g offset.
		accel_noise = BADACC_BIAS_PNOISE;
	}

	dvxVar = dvyVar = dvzVar = dsq(dt * accel_noise);

	// predict the covariance

	// intermediate calculations
	double SF[21];
	SF[0] = dvz - dvz_b;
	SF[1] = dvy - dvy_b;
	SF[2] = dvx - dvx_b;
	SF[3] = 2*q1*SF[2] + 2*q2*SF[1] + 2*q3*SF[0];
	SF[4] = 2*q0*SF[1] - 2*q1*SF[0] + 2*q3*SF[2];
	SF[5] = 2*q0*SF[2] + 2*q2*SF[0] - 2*q3*SF[1];
	SF[6] = day/2 - day_b/2;
	SF[7] = daz/2 - daz_b/2;
	SF[8] = dax/2 - dax_b/2;
	SF[9] = dax_b/2 - dax/2;
	SF[10] = daz_b/2 - daz/2;
	SF[11] = day_b/2 - day/2;
	SF[12] = 2*q1*SF[1];
	SF[13] = 2*q0*SF[0];
	SF[14] = q1/2;
	SF[15] = q2/2;
	SF[16] = q3/2;
	SF[17] = dsq(q3);
	SF[18] = dsq(q2);
	SF[19] = dsq(q1);
	SF[20] = dsq(q0);

	double SG[8];
	SG[0] = q0/2;
	SG[1] = dsq(q3);
	SG[2] = dsq(q2);
	SG[3] = dsq(q1);
	SG[4] = dsq(q0);
	SG[5] = 2*q2*q3;
	SG[6] = 2*q1*q3;
	SG[7] = 2*q1*q2;

	double SQ[11];
	SQ[0] = dvzVar*(SG[5] - 2*q0*q1)*(SG[1] - SG[2] - SG[3] + SG[4]) - dvyVar*(SG[5] + 2*q0*q1)*(SG[1] - SG[2] + SG[3] - SG[4]) + dvxVar*(SG[6] - 2*q0*q2)*(SG[7] + 2*q0*q3);
	SQ[1] = dvzVar*(SG[6] + 2*q0*q2)*(SG[1] - SG[2] - SG[3] + SG[4]) - dvxVar*(SG[6] - 2*q0*q2)*(SG[1] + SG[2] - SG[3] - SG[4]) + dvyVar*(SG[5] + 2*q0*q1)*(SG[7] - 2*q0*q3);
	SQ[2] = dvzVar*(SG[5] - 2*q0*q1)*(SG[6] + 2*q0*q2) - dvyVar*(SG[7] - 2*q0*q3)*(SG[1] - SG[2] + SG[3] - SG[4]) - dvxVar*(SG[7] + 2*q0*q3)*(SG[1] + SG[2] - SG[3] - SG[4]);
	SQ[3] = (dayVar*q1*SG[0])/2 - (dazVar*q1*SG[0])/2 - (daxVar*q2*q3)/4;
	SQ[4] = (dazVar*q2*SG[0])/2 - (daxVar*q2*SG[0])/2 - (dayVar*q1*q3)/4;
	SQ[5] = (daxVar*q3*SG[0])/2 - (dayVar*q3*SG[0])/2 - (dazVar*q1*q2)/4;
	SQ[6] = (daxVar*q1*q2)/4 - (dazVar*q3*SG[0])/2 - (dayVar*q1*q2)/4;
	SQ[7] = (dazVar*q1*q3)/4 - (daxVar*q1*q3)/4 - (dayVar*q2*SG[0])/2;
	SQ[8] = (dayVar*q2*q3)/4 - (daxVar*q1*SG[0])/2 - (dazVar*q2*q3)/4;
	SQ[9] = dsq(SG[0]);
	SQ[10] = dsq(q1);

	double SPP[11] = {};
	SPP[0] = SF[12] + SF[13] - 2*q2*SF[2];
	SPP[1] = SF[17] - SF[18] - SF[19] + SF[20];
	SPP[2] = SF[17] - SF[18] + SF[19] - SF[20];
	SPP[3] = SF[17] + SF[18] - SF[19] - SF[20];
	SPP[4] = 2*q0*q2 - 2*q1*q3;
	SPP[5] = 2*q0*q1 - 2*q2*q3;
	SPP[6] = 2*q0*q3 - 2*q1*q2;
	SPP[7] = 2*q0*q1 + 2*q2*q3;
	SPP[8] = 2*q0*q3 + 2*q1*q2;
	SPP[9] = 2*q0*q2 + 2*q1*q3;
	SPP[10] = SF[16];


	/*float nextP6_1 =(double)P[1][6]*SF[4] -(double)P[2][6]*SF[5] +(double)P[3][6]*SF[3] +(double)P[0][6]*SPP[0] +(double)P[13][6]*SPP[4] -(double)P[14][6]*SPP[7] -(double)P[15][6]*SPP[1] + SF[4]*(P[6][1] +(double)P[1][1]*SF[4] -(double)P[2][1]*SF[5] +(double)P[3][1]*SF[3] +(double)P[0][1]*SPP[0] +(double)P[13][1]*SPP[4] -(double)P[14][1]*SPP[7] -(double)P[15][1]*SPP[1]) - SF[5]*(P[6][2] +(double)P[1][2]*SF[4] -(double)P[2][2]*SF[5] +(double)P[3][2]*SF[3] +(double)P[0][2]*SPP[0] +(double)P[13][2]*SPP[4] -(double)P[14][2]*SPP[7] -(double)P[15][2]*SPP[1]) + SF[3]*(P[6][3] +(double)P[1][3]*SF[4] -(double)P[2][3]*SF[5] +(double)P[3][3]*SF[3] +(double)P[0][3]*SPP[0] +(double)P[13][3]*SPP[4] -(double)P[14][3]*SPP[7] -(double)P[15][3]*SPP[1]) + SPP[0]*(P[6][0] +(double)P[1][0]*SF[4] -(double)P[2][0]*SF[5] +(double)P[3][0]*SF[3] +(double)P[0][0]*SPP[0] +(double)P[13][0]*SPP[4] -(double)P[14][0]*SPP[7] -(double)P[15][0]*SPP[1]) + SPP[4]*(P[6][13] +(double)P[1][13]*SF[4] -(double)P[2][13]*SF[5] +(double)P[3][13]*SF[3] +(double)P[0][13]*SPP[0] +(double)P[13][13]*SPP[4] -(double)P[14][13]*SPP[7] -(double)P[15][13]*SPP[1]) - SPP[7]*(P[6][14] +(double)P[1][14]*SF[4] -(double)P[2][14]*SF[5] +(double)P[3][14]*SF[3] +(double)P[0][14]*SPP[0] +(double)P[13][14]*SPP[4] -(double)P[14][14]*SPP[7] -(double)P[15][14]*SPP[1]) - SPP[1]*(P[6][15] +(double)P[1][15]*SF[4] -(double)P[2][15]*SF[5] +(double)P[3][15]*SF[3] +(double)P[0][15]*SPP[0] +(double)P[13][15]*SPP[4] -(double)P[14][15]*SPP[7] -(double)P[15][15]*SPP[1]);
	float nextP6_2 = dvxVar*dsq(SG[6] - 2*q0*q2) + dvyVar*dsq(SG[5] + 2*q0*q1) + dvzVar*dsq(SG[1] - SG[2] - SG[3] + SG[4]);
	printf(" (from: %.9f add %.9f + %.9f = %.9f tot: %.9f, dt= (%.3f, %.3f, %.3f))",
		   (double)P[6][6], (double)nextP6_1, (double)nextP6_2,
			(double)(nextP6_1+nextP6_2), (double)(nextP6_1+nextP6_2+P[6][6]),
			(double)dt*1000.0, (double)_imu_sample_delayed.delta_ang_dt*1000.0,
			(double)_imu_sample_delayed.delta_vel_dt*1000.0);*/

	/*float nextP0_1 =(double)P[1][0]*SF[9] +(double)P[2][0]*SF[11] +(double)P[3][0]*SF[10] +(double)P[10][0]*SF[14] +(double)P[11][0]*SF[15] +(double)P[12][0]*SPP[10] + (daxVar*SQ[10])/4 + SF[9]*(P[0][1] +(double)P[1][1]*SF[9] +(double)P[2][1]*SF[11] +(double)P[3][1]*SF[10] +(double)P[10][1]*SF[14] +(double)P[11][1]*SF[15] +(double)P[12][1]*SPP[10]) + SF[11]*(P[0][2] +(double)P[1][2]*SF[9] +(double)P[2][2]*SF[11] +(double)P[3][2]*SF[10] +(double)P[10][2]*SF[14] +(double)P[11][2]*SF[15] +(double)P[12][2]*SPP[10]) + SF[10]*(P[0][3] +(double)P[1][3]*SF[9] +(double)P[2][3]*SF[11] +(double)P[3][3]*SF[10] +(double)P[10][3]*SF[14] +(double)P[11][3]*SF[15] +(double)P[12][3]*SPP[10]) + SF[14]*(P[0][10] +(double)P[1][10]*SF[9] +(double)P[2][10]*SF[11] +(double)P[3][10]*SF[10] +(double)P[10][10]*SF[14] +(double)P[11][10]*SF[15] +(double)P[12][10]*SPP[10]) + SF[15]*(P[0][11] +(double)P[1][11]*SF[9] +(double)P[2][11]*SF[11] +(double)P[3][11]*SF[10] +(double)P[10][11]*SF[14] +(double)P[11][11]*SF[15] +(double)P[12][11]*SPP[10]) + SPP[10]*(P[0][12] +(double)P[1][12]*SF[9] +(double)P[2][12]*SF[11] +(double)P[3][12]*SF[10] +(double)P[10][12]*SF[14] +(double)P[11][12]*SF[15] +(double)P[12][12]*SPP[10]);
	float nextP0_2 =  + (dayVar*dsq(q2))/4 + (dazVar*dsq(q3))/4;
	printf(" (from: %.9f add %.9f + %.9f = %.9f tot: %.9f, dt= (%.3f, %.3f, %.3f))",
		   (double)P[0][0], (double)nextP0_1, (double)nextP0_2,
			(double)(nextP0_1+nextP0_2), (double)(nextP0_1+nextP0_2+P[0][0]),
			(double)dt*1000.0, (double)_imu_sample_delayed.delta_ang_dt*1000.0,
			(double)_imu_sample_delayed.delta_vel_dt*1000.0);*/

/*	float nextP9_1 =(double)P[6][9]*dt + dt*(P[9][6] +(double)P[6][6]*dt);
		float nextP9_2 = 0.0f;
		printf(" (from: %.9f add %.9f + %.9f = %.9f tot: %.9f, dt= (%.3f, %.3f, %.3f))",
			   (double)P[9][9], (double)nextP9_1, (double)nextP9_2,
				(double)(nextP9_1+nextP9_2), (double)(nextP9_1+nextP9_2+P[9][9]),
				(double)dt*1000.0, (double)_imu_sample_delayed.delta_ang_dt*1000.0,
				(double)_imu_sample_delayed.delta_vel_dt*1000.0);
*/
	// covariance update
	float nextP[24][24];

	// calculate variances and upper diagonal covariances for quaternion, velocity, position and gyro bias states
	nextP[0][0] =(double)P[0][0] +(double)P[1][0]*SF[9] +(double)P[2][0]*SF[11] +(double)P[3][0]*SF[10] +(double)P[10][0]*SF[14] +(double)P[11][0]*SF[15] +(double)P[12][0]*SPP[10] + (daxVar*SQ[10])/4 + SF[9]*((double)P[0][1] +(double)P[1][1]*SF[9] +(double)P[2][1]*SF[11] +(double)P[3][1]*SF[10] +(double)P[10][1]*SF[14] +(double)P[11][1]*SF[15] +(double)P[12][1]*SPP[10]) + SF[11]*((double)P[0][2] +(double)P[1][2]*SF[9] +(double)P[2][2]*SF[11] +(double)P[3][2]*SF[10] +(double)P[10][2]*SF[14] +(double)P[11][2]*SF[15] +(double)P[12][2]*SPP[10]) + SF[10]*((double)P[0][3] +(double)P[1][3]*SF[9] +(double)P[2][3]*SF[11] +(double)P[3][3]*SF[10] +(double)P[10][3]*SF[14] +(double)P[11][3]*SF[15] +(double)P[12][3]*SPP[10]) + SF[14]*((double)P[0][10] +(double)P[1][10]*SF[9] +(double)P[2][10]*SF[11] +(double)P[3][10]*SF[10] +(double)P[10][10]*SF[14] +(double)P[11][10]*SF[15] +(double)P[12][10]*SPP[10]) + SF[15]*((double)P[0][11] +(double)P[1][11]*SF[9] +(double)P[2][11]*SF[11] +(double)P[3][11]*SF[10] +(double)P[10][11]*SF[14] +(double)P[11][11]*SF[15] +(double)P[12][11]*SPP[10]) + SPP[10]*((double)P[0][12] +(double)P[1][12]*SF[9] +(double)P[2][12]*SF[11] +(double)P[3][12]*SF[10] +(double)P[10][12]*SF[14] +(double)P[11][12]*SF[15] +(double)P[12][12]*SPP[10]) + (dayVar*dsq(q2))/4 + (dazVar*dsq(q3))/4;
	nextP[0][1] =(double)P[0][1] + SQ[8] +(double)P[1][1]*SF[9] +(double)P[2][1]*SF[11] +(double)P[3][1]*SF[10] +(double)P[10][1]*SF[14] +(double)P[11][1]*SF[15] +(double)P[12][1]*SPP[10] + SF[8]*((double)P[0][0] +(double)P[1][0]*SF[9] +(double)P[2][0]*SF[11] +(double)P[3][0]*SF[10] +(double)P[10][0]*SF[14] +(double)P[11][0]*SF[15] +(double)P[12][0]*SPP[10]) + SF[7]*((double)P[0][2] +(double)P[1][2]*SF[9] +(double)P[2][2]*SF[11] +(double)P[3][2]*SF[10] +(double)P[10][2]*SF[14] +(double)P[11][2]*SF[15] +(double)P[12][2]*SPP[10]) + SF[11]*((double)P[0][3] +(double)P[1][3]*SF[9] +(double)P[2][3]*SF[11] +(double)P[3][3]*SF[10] +(double)P[10][3]*SF[14] +(double)P[11][3]*SF[15] +(double)P[12][3]*SPP[10]) - SF[15]*((double)P[0][12] +(double)P[1][12]*SF[9] +(double)P[2][12]*SF[11] +(double)P[3][12]*SF[10] +(double)P[10][12]*SF[14] +(double)P[11][12]*SF[15] +(double)P[12][12]*SPP[10]) + SPP[10]*((double)P[0][11] +(double)P[1][11]*SF[9] +(double)P[2][11]*SF[11] +(double)P[3][11]*SF[10] +(double)P[10][11]*SF[14] +(double)P[11][11]*SF[15] +(double)P[12][11]*SPP[10]) - (q0*((double)P[0][10] +(double)P[1][10]*SF[9] +(double)P[2][10]*SF[11] +(double)P[3][10]*SF[10] +(double)P[10][10]*SF[14] +(double)P[11][10]*SF[15] +(double)P[12][10]*SPP[10]))/2;
	nextP[1][1] =(double)P[1][1] +(double)P[0][1]*SF[8] +(double)P[2][1]*SF[7] +(double)P[3][1]*SF[11] -(double)P[12][1]*SF[15] +(double)P[11][1]*SPP[10] + daxVar*SQ[9] - ((double)P[10][1]*q0)/2 + SF[8]*((double)P[1][0] +(double)P[0][0]*SF[8] +(double)P[2][0]*SF[7] +(double)P[3][0]*SF[11] -(double)P[12][0]*SF[15] +(double)P[11][0]*SPP[10] - ((double)P[10][0]*q0)/2) + SF[7]*((double)P[1][2] +(double)P[0][2]*SF[8] +(double)P[2][2]*SF[7] +(double)P[3][2]*SF[11] -(double)P[12][2]*SF[15] +(double)P[11][2]*SPP[10] - ((double)P[10][2]*q0)/2) + SF[11]*((double)P[1][3] +(double)P[0][3]*SF[8] +(double)P[2][3]*SF[7] +(double)P[3][3]*SF[11] -(double)P[12][3]*SF[15] +(double)P[11][3]*SPP[10] - ((double)P[10][3]*q0)/2) - SF[15]*((double)P[1][12] +(double)P[0][12]*SF[8] +(double)P[2][12]*SF[7] +(double)P[3][12]*SF[11] -(double)P[12][12]*SF[15] +(double)P[11][12]*SPP[10] - ((double)P[10][12]*q0)/2) + SPP[10]*((double)P[1][11] +(double)P[0][11]*SF[8] +(double)P[2][11]*SF[7] +(double)P[3][11]*SF[11] -(double)P[12][11]*SF[15] +(double)P[11][11]*SPP[10] - ((double)P[10][11]*q0)/2) + (dayVar*dsq(q3))/4 + (dazVar*dsq(q2))/4 - (q0*((double)P[1][10] +(double)P[0][10]*SF[8] +(double)P[2][10]*SF[7] +(double)P[3][10]*SF[11] -(double)P[12][10]*SF[15] +(double)P[11][10]*SPP[10] - ((double)P[10][10]*q0)/2))/2;
	nextP[0][2] =(double)P[0][2] + SQ[7] +(double)P[1][2]*SF[9] +(double)P[2][2]*SF[11] +(double)P[3][2]*SF[10] +(double)P[10][2]*SF[14] +(double)P[11][2]*SF[15] +(double)P[12][2]*SPP[10] + SF[6]*((double)P[0][0] +(double)P[1][0]*SF[9] +(double)P[2][0]*SF[11] +(double)P[3][0]*SF[10] +(double)P[10][0]*SF[14] +(double)P[11][0]*SF[15] +(double)P[12][0]*SPP[10]) + SF[10]*((double)P[0][1] +(double)P[1][1]*SF[9] +(double)P[2][1]*SF[11] +(double)P[3][1]*SF[10] +(double)P[10][1]*SF[14] +(double)P[11][1]*SF[15] +(double)P[12][1]*SPP[10]) + SF[8]*((double)P[0][3] +(double)P[1][3]*SF[9] +(double)P[2][3]*SF[11] +(double)P[3][3]*SF[10] +(double)P[10][3]*SF[14] +(double)P[11][3]*SF[15] +(double)P[12][3]*SPP[10]) + SF[14]*((double)P[0][12] +(double)P[1][12]*SF[9] +(double)P[2][12]*SF[11] +(double)P[3][12]*SF[10] +(double)P[10][12]*SF[14] +(double)P[11][12]*SF[15] +(double)P[12][12]*SPP[10]) - SPP[10]*((double)P[0][10] +(double)P[1][10]*SF[9] +(double)P[2][10]*SF[11] +(double)P[3][10]*SF[10] +(double)P[10][10]*SF[14] +(double)P[11][10]*SF[15] +(double)P[12][10]*SPP[10]) - (q0*((double)P[0][11] +(double)P[1][11]*SF[9] +(double)P[2][11]*SF[11] +(double)P[3][11]*SF[10] +(double)P[10][11]*SF[14] +(double)P[11][11]*SF[15] +(double)P[12][11]*SPP[10]))/2;
	nextP[1][2] =(double)P[1][2] + SQ[5] +(double)P[0][2]*SF[8] +(double)P[2][2]*SF[7] +(double)P[3][2]*SF[11] -(double)P[12][2]*SF[15] +(double)P[11][2]*SPP[10] - ((double)P[10][2]*q0)/2 + SF[6]*((double)P[1][0] +(double)P[0][0]*SF[8] +(double)P[2][0]*SF[7] +(double)P[3][0]*SF[11] -(double)P[12][0]*SF[15] +(double)P[11][0]*SPP[10] - ((double)P[10][0]*q0)/2) + SF[10]*((double)P[1][1] +(double)P[0][1]*SF[8] +(double)P[2][1]*SF[7] +(double)P[3][1]*SF[11] -(double)P[12][1]*SF[15] +(double)P[11][1]*SPP[10] - ((double)P[10][1]*q0)/2) + SF[8]*((double)P[1][3] +(double)P[0][3]*SF[8] +(double)P[2][3]*SF[7] +(double)P[3][3]*SF[11] -(double)P[12][3]*SF[15] +(double)P[11][3]*SPP[10] - ((double)P[10][3]*q0)/2) + SF[14]*((double)P[1][12] +(double)P[0][12]*SF[8] +(double)P[2][12]*SF[7] +(double)P[3][12]*SF[11] -(double)P[12][12]*SF[15] +(double)P[11][12]*SPP[10] - ((double)P[10][12]*q0)/2) - SPP[10]*((double)P[1][10] +(double)P[0][10]*SF[8] +(double)P[2][10]*SF[7] +(double)P[3][10]*SF[11] -(double)P[12][10]*SF[15] +(double)P[11][10]*SPP[10] - ((double)P[10][10]*q0)/2) - (q0*((double)P[1][11] +(double)P[0][11]*SF[8] +(double)P[2][11]*SF[7] +(double)P[3][11]*SF[11] -(double)P[12][11]*SF[15] +(double)P[11][11]*SPP[10] - ((double)P[10][11]*q0)/2))/2;
	nextP[2][2] =(double)P[2][2] +(double)P[0][2]*SF[6] +(double)P[1][2]*SF[10] +(double)P[3][2]*SF[8] +(double)P[12][2]*SF[14] -(double)P[10][2]*SPP[10] + dayVar*SQ[9] + (dazVar*SQ[10])/4 - ((double)P[11][2]*q0)/2 + SF[6]*((double)P[2][0] +(double)P[0][0]*SF[6] +(double)P[1][0]*SF[10] +(double)P[3][0]*SF[8] +(double)P[12][0]*SF[14] -(double)P[10][0]*SPP[10] - ((double)P[11][0]*q0)/2) + SF[10]*((double)P[2][1] +(double)P[0][1]*SF[6] +(double)P[1][1]*SF[10] +(double)P[3][1]*SF[8] +(double)P[12][1]*SF[14] -(double)P[10][1]*SPP[10] - ((double)P[11][1]*q0)/2) + SF[8]*((double)P[2][3] +(double)P[0][3]*SF[6] +(double)P[1][3]*SF[10] +(double)P[3][3]*SF[8] +(double)P[12][3]*SF[14] -(double)P[10][3]*SPP[10] - ((double)P[11][3]*q0)/2) + SF[14]*((double)P[2][12] +(double)P[0][12]*SF[6] +(double)P[1][12]*SF[10] +(double)P[3][12]*SF[8] +(double)P[12][12]*SF[14] -(double)P[10][12]*SPP[10] - ((double)P[11][12]*q0)/2) - SPP[10]*((double)P[2][10] +(double)P[0][10]*SF[6] +(double)P[1][10]*SF[10] +(double)P[3][10]*SF[8] +(double)P[12][10]*SF[14] -(double)P[10][10]*SPP[10] - ((double)P[11][10]*q0)/2) + (daxVar*dsq(q3))/4 - (q0*((double)P[2][11] +(double)P[0][11]*SF[6] +(double)P[1][11]*SF[10] +(double)P[3][11]*SF[8] +(double)P[12][11]*SF[14] -(double)P[10][11]*SPP[10] - ((double)P[11][11]*q0)/2))/2;
	nextP[0][3] =(double)P[0][3] + SQ[6] +(double)P[1][3]*SF[9] +(double)P[2][3]*SF[11] +(double)P[3][3]*SF[10] +(double)P[10][3]*SF[14] +(double)P[11][3]*SF[15] +(double)P[12][3]*SPP[10] + SF[7]*((double)P[0][0] +(double)P[1][0]*SF[9] +(double)P[2][0]*SF[11] +(double)P[3][0]*SF[10] +(double)P[10][0]*SF[14] +(double)P[11][0]*SF[15] +(double)P[12][0]*SPP[10]) + SF[6]*((double)P[0][1] +(double)P[1][1]*SF[9] +(double)P[2][1]*SF[11] +(double)P[3][1]*SF[10] +(double)P[10][1]*SF[14] +(double)P[11][1]*SF[15] +(double)P[12][1]*SPP[10]) + SF[9]*((double)P[0][2] +(double)P[1][2]*SF[9] +(double)P[2][2]*SF[11] +(double)P[3][2]*SF[10] +(double)P[10][2]*SF[14] +(double)P[11][2]*SF[15] +(double)P[12][2]*SPP[10]) + SF[15]*((double)P[0][10] +(double)P[1][10]*SF[9] +(double)P[2][10]*SF[11] +(double)P[3][10]*SF[10] +(double)P[10][10]*SF[14] +(double)P[11][10]*SF[15] +(double)P[12][10]*SPP[10]) - SF[14]*((double)P[0][11] +(double)P[1][11]*SF[9] +(double)P[2][11]*SF[11] +(double)P[3][11]*SF[10] +(double)P[10][11]*SF[14] +(double)P[11][11]*SF[15] +(double)P[12][11]*SPP[10]) - (q0*((double)P[0][12] +(double)P[1][12]*SF[9] +(double)P[2][12]*SF[11] +(double)P[3][12]*SF[10] +(double)P[10][12]*SF[14] +(double)P[11][12]*SF[15] +(double)P[12][12]*SPP[10]))/2;
	nextP[1][3] =(double)P[1][3] + SQ[4] +(double)P[0][3]*SF[8] +(double)P[2][3]*SF[7] +(double)P[3][3]*SF[11] -(double)P[12][3]*SF[15] +(double)P[11][3]*SPP[10] - ((double)P[10][3]*q0)/2 + SF[7]*((double)P[1][0] +(double)P[0][0]*SF[8] +(double)P[2][0]*SF[7] +(double)P[3][0]*SF[11] -(double)P[12][0]*SF[15] +(double)P[11][0]*SPP[10] - ((double)P[10][0]*q0)/2) + SF[6]*((double)P[1][1] +(double)P[0][1]*SF[8] +(double)P[2][1]*SF[7] +(double)P[3][1]*SF[11] -(double)P[12][1]*SF[15] +(double)P[11][1]*SPP[10] - ((double)P[10][1]*q0)/2) + SF[9]*((double)P[1][2] +(double)P[0][2]*SF[8] +(double)P[2][2]*SF[7] +(double)P[3][2]*SF[11] -(double)P[12][2]*SF[15] +(double)P[11][2]*SPP[10] - ((double)P[10][2]*q0)/2) + SF[15]*((double)P[1][10] +(double)P[0][10]*SF[8] +(double)P[2][10]*SF[7] +(double)P[3][10]*SF[11] -(double)P[12][10]*SF[15] +(double)P[11][10]*SPP[10] - ((double)P[10][10]*q0)/2) - SF[14]*((double)P[1][11] +(double)P[0][11]*SF[8] +(double)P[2][11]*SF[7] +(double)P[3][11]*SF[11] -(double)P[12][11]*SF[15] +(double)P[11][11]*SPP[10] - ((double)P[10][11]*q0)/2) - (q0*((double)P[1][12] +(double)P[0][12]*SF[8] +(double)P[2][12]*SF[7] +(double)P[3][12]*SF[11] -(double)P[12][12]*SF[15] +(double)P[11][12]*SPP[10] - ((double)P[10][12]*q0)/2))/2;
	nextP[2][3] =(double)P[2][3] + SQ[3] +(double)P[0][3]*SF[6] +(double)P[1][3]*SF[10] +(double)P[3][3]*SF[8] +(double)P[12][3]*SF[14] -(double)P[10][3]*SPP[10] - ((double)P[11][3]*q0)/2 + SF[7]*((double)P[2][0] +(double)P[0][0]*SF[6] +(double)P[1][0]*SF[10] +(double)P[3][0]*SF[8] +(double)P[12][0]*SF[14] -(double)P[10][0]*SPP[10] - ((double)P[11][0]*q0)/2) + SF[6]*((double)P[2][1] +(double)P[0][1]*SF[6] +(double)P[1][1]*SF[10] +(double)P[3][1]*SF[8] +(double)P[12][1]*SF[14] -(double)P[10][1]*SPP[10] - ((double)P[11][1]*q0)/2) + SF[9]*((double)P[2][2] +(double)P[0][2]*SF[6] +(double)P[1][2]*SF[10] +(double)P[3][2]*SF[8] +(double)P[12][2]*SF[14] -(double)P[10][2]*SPP[10] - ((double)P[11][2]*q0)/2) + SF[15]*((double)P[2][10] +(double)P[0][10]*SF[6] +(double)P[1][10]*SF[10] +(double)P[3][10]*SF[8] +(double)P[12][10]*SF[14] -(double)P[10][10]*SPP[10] - ((double)P[11][10]*q0)/2) - SF[14]*((double)P[2][11] +(double)P[0][11]*SF[6] +(double)P[1][11]*SF[10] +(double)P[3][11]*SF[8] +(double)P[12][11]*SF[14] -(double)P[10][11]*SPP[10] - ((double)P[11][11]*q0)/2) - (q0*((double)P[2][12] +(double)P[0][12]*SF[6] +(double)P[1][12]*SF[10] +(double)P[3][12]*SF[8] +(double)P[12][12]*SF[14] -(double)P[10][12]*SPP[10] - ((double)P[11][12]*q0)/2))/2;
	nextP[3][3] =(double)P[3][3] +(double)P[0][3]*SF[7] +(double)P[1][3]*SF[6] +(double)P[2][3]*SF[9] +(double)P[10][3]*SF[15] -(double)P[11][3]*SF[14] + (dayVar*SQ[10])/4 + dazVar*SQ[9] - ((double)P[12][3]*q0)/2 + SF[7]*((double)P[3][0] +(double)P[0][0]*SF[7] +(double)P[1][0]*SF[6] +(double)P[2][0]*SF[9] +(double)P[10][0]*SF[15] -(double)P[11][0]*SF[14] - ((double)P[12][0]*q0)/2) + SF[6]*((double)P[3][1] +(double)P[0][1]*SF[7] +(double)P[1][1]*SF[6] +(double)P[2][1]*SF[9] +(double)P[10][1]*SF[15] -(double)P[11][1]*SF[14] - ((double)P[12][1]*q0)/2) + SF[9]*((double)P[3][2] +(double)P[0][2]*SF[7] +(double)P[1][2]*SF[6] +(double)P[2][2]*SF[9] +(double)P[10][2]*SF[15] -(double)P[11][2]*SF[14] - ((double)P[12][2]*q0)/2) + SF[15]*((double)P[3][10] +(double)P[0][10]*SF[7] +(double)P[1][10]*SF[6] +(double)P[2][10]*SF[9] +(double)P[10][10]*SF[15] -(double)P[11][10]*SF[14] - ((double)P[12][10]*q0)/2) - SF[14]*((double)P[3][11] +(double)P[0][11]*SF[7] +(double)P[1][11]*SF[6] +(double)P[2][11]*SF[9] +(double)P[10][11]*SF[15] -(double)P[11][11]*SF[14] - ((double)P[12][11]*q0)/2) + (daxVar*dsq(q2))/4 - (q0*((double)P[3][12] +(double)P[0][12]*SF[7] +(double)P[1][12]*SF[6] +(double)P[2][12]*SF[9] +(double)P[10][12]*SF[15] -(double)P[11][12]*SF[14] - ((double)P[12][12]*q0)/2))/2;
	nextP[0][4] =(double)P[0][4] +(double)P[1][4]*SF[9] +(double)P[2][4]*SF[11] +(double)P[3][4]*SF[10] +(double)P[10][4]*SF[14] +(double)P[11][4]*SF[15] +(double)P[12][4]*SPP[10] + SF[5]*((double)P[0][0] +(double)P[1][0]*SF[9] +(double)P[2][0]*SF[11] +(double)P[3][0]*SF[10] +(double)P[10][0]*SF[14] +(double)P[11][0]*SF[15] +(double)P[12][0]*SPP[10]) + SF[3]*((double)P[0][1] +(double)P[1][1]*SF[9] +(double)P[2][1]*SF[11] +(double)P[3][1]*SF[10] +(double)P[10][1]*SF[14] +(double)P[11][1]*SF[15] +(double)P[12][1]*SPP[10]) - SF[4]*((double)P[0][3] +(double)P[1][3]*SF[9] +(double)P[2][3]*SF[11] +(double)P[3][3]*SF[10] +(double)P[10][3]*SF[14] +(double)P[11][3]*SF[15] +(double)P[12][3]*SPP[10]) + SPP[0]*((double)P[0][2] +(double)P[1][2]*SF[9] +(double)P[2][2]*SF[11] +(double)P[3][2]*SF[10] +(double)P[10][2]*SF[14] +(double)P[11][2]*SF[15] +(double)P[12][2]*SPP[10]) + SPP[3]*((double)P[0][13] +(double)P[1][13]*SF[9] +(double)P[2][13]*SF[11] +(double)P[3][13]*SF[10] +(double)P[10][13]*SF[14] +(double)P[11][13]*SF[15] +(double)P[12][13]*SPP[10]) + SPP[6]*((double)P[0][14] +(double)P[1][14]*SF[9] +(double)P[2][14]*SF[11] +(double)P[3][14]*SF[10] +(double)P[10][14]*SF[14] +(double)P[11][14]*SF[15] +(double)P[12][14]*SPP[10]) - SPP[9]*((double)P[0][15] +(double)P[1][15]*SF[9] +(double)P[2][15]*SF[11] +(double)P[3][15]*SF[10] +(double)P[10][15]*SF[14] +(double)P[11][15]*SF[15] +(double)P[12][15]*SPP[10]);
	nextP[1][4] =(double)P[1][4] +(double)P[0][4]*SF[8] +(double)P[2][4]*SF[7] +(double)P[3][4]*SF[11] -(double)P[12][4]*SF[15] +(double)P[11][4]*SPP[10] - ((double)P[10][4]*q0)/2 + SF[5]*((double)P[1][0] +(double)P[0][0]*SF[8] +(double)P[2][0]*SF[7] +(double)P[3][0]*SF[11] -(double)P[12][0]*SF[15] +(double)P[11][0]*SPP[10] - ((double)P[10][0]*q0)/2) + SF[3]*((double)P[1][1] +(double)P[0][1]*SF[8] +(double)P[2][1]*SF[7] +(double)P[3][1]*SF[11] -(double)P[12][1]*SF[15] +(double)P[11][1]*SPP[10] - ((double)P[10][1]*q0)/2) - SF[4]*((double)P[1][3] +(double)P[0][3]*SF[8] +(double)P[2][3]*SF[7] +(double)P[3][3]*SF[11] -(double)P[12][3]*SF[15] +(double)P[11][3]*SPP[10] - ((double)P[10][3]*q0)/2) + SPP[0]*((double)P[1][2] +(double)P[0][2]*SF[8] +(double)P[2][2]*SF[7] +(double)P[3][2]*SF[11] -(double)P[12][2]*SF[15] +(double)P[11][2]*SPP[10] - ((double)P[10][2]*q0)/2) + SPP[3]*((double)P[1][13] +(double)P[0][13]*SF[8] +(double)P[2][13]*SF[7] +(double)P[3][13]*SF[11] -(double)P[12][13]*SF[15] +(double)P[11][13]*SPP[10] - ((double)P[10][13]*q0)/2) + SPP[6]*((double)P[1][14] +(double)P[0][14]*SF[8] +(double)P[2][14]*SF[7] +(double)P[3][14]*SF[11] -(double)P[12][14]*SF[15] +(double)P[11][14]*SPP[10] - ((double)P[10][14]*q0)/2) - SPP[9]*((double)P[1][15] +(double)P[0][15]*SF[8] +(double)P[2][15]*SF[7] +(double)P[3][15]*SF[11] -(double)P[12][15]*SF[15] +(double)P[11][15]*SPP[10] - ((double)P[10][15]*q0)/2);
	nextP[2][4] =(double)P[2][4] +(double)P[0][4]*SF[6] +(double)P[1][4]*SF[10] +(double)P[3][4]*SF[8] +(double)P[12][4]*SF[14] -(double)P[10][4]*SPP[10] - ((double)P[11][4]*q0)/2 + SF[5]*((double)P[2][0] +(double)P[0][0]*SF[6] +(double)P[1][0]*SF[10] +(double)P[3][0]*SF[8] +(double)P[12][0]*SF[14] -(double)P[10][0]*SPP[10] - ((double)P[11][0]*q0)/2) + SF[3]*((double)P[2][1] +(double)P[0][1]*SF[6] +(double)P[1][1]*SF[10] +(double)P[3][1]*SF[8] +(double)P[12][1]*SF[14] -(double)P[10][1]*SPP[10] - ((double)P[11][1]*q0)/2) - SF[4]*((double)P[2][3] +(double)P[0][3]*SF[6] +(double)P[1][3]*SF[10] +(double)P[3][3]*SF[8] +(double)P[12][3]*SF[14] -(double)P[10][3]*SPP[10] - ((double)P[11][3]*q0)/2) + SPP[0]*((double)P[2][2] +(double)P[0][2]*SF[6] +(double)P[1][2]*SF[10] +(double)P[3][2]*SF[8] +(double)P[12][2]*SF[14] -(double)P[10][2]*SPP[10] - ((double)P[11][2]*q0)/2) + SPP[3]*((double)P[2][13] +(double)P[0][13]*SF[6] +(double)P[1][13]*SF[10] +(double)P[3][13]*SF[8] +(double)P[12][13]*SF[14] -(double)P[10][13]*SPP[10] - ((double)P[11][13]*q0)/2) + SPP[6]*((double)P[2][14] +(double)P[0][14]*SF[6] +(double)P[1][14]*SF[10] +(double)P[3][14]*SF[8] +(double)P[12][14]*SF[14] -(double)P[10][14]*SPP[10] - ((double)P[11][14]*q0)/2) - SPP[9]*((double)P[2][15] +(double)P[0][15]*SF[6] +(double)P[1][15]*SF[10] +(double)P[3][15]*SF[8] +(double)P[12][15]*SF[14] -(double)P[10][15]*SPP[10] - ((double)P[11][15]*q0)/2);
	nextP[3][4] =(double)P[3][4] +(double)P[0][4]*SF[7] +(double)P[1][4]*SF[6] +(double)P[2][4]*SF[9] +(double)P[10][4]*SF[15] -(double)P[11][4]*SF[14] - ((double)P[12][4]*q0)/2 + SF[5]*((double)P[3][0] +(double)P[0][0]*SF[7] +(double)P[1][0]*SF[6] +(double)P[2][0]*SF[9] +(double)P[10][0]*SF[15] -(double)P[11][0]*SF[14] - ((double)P[12][0]*q0)/2) + SF[3]*((double)P[3][1] +(double)P[0][1]*SF[7] +(double)P[1][1]*SF[6] +(double)P[2][1]*SF[9] +(double)P[10][1]*SF[15] -(double)P[11][1]*SF[14] - ((double)P[12][1]*q0)/2) - SF[4]*((double)P[3][3] +(double)P[0][3]*SF[7] +(double)P[1][3]*SF[6] +(double)P[2][3]*SF[9] +(double)P[10][3]*SF[15] -(double)P[11][3]*SF[14] - ((double)P[12][3]*q0)/2) + SPP[0]*((double)P[3][2] +(double)P[0][2]*SF[7] +(double)P[1][2]*SF[6] +(double)P[2][2]*SF[9] +(double)P[10][2]*SF[15] -(double)P[11][2]*SF[14] - ((double)P[12][2]*q0)/2) + SPP[3]*((double)P[3][13] +(double)P[0][13]*SF[7] +(double)P[1][13]*SF[6] +(double)P[2][13]*SF[9] +(double)P[10][13]*SF[15] -(double)P[11][13]*SF[14] - ((double)P[12][13]*q0)/2) + SPP[6]*((double)P[3][14] +(double)P[0][14]*SF[7] +(double)P[1][14]*SF[6] +(double)P[2][14]*SF[9] +(double)P[10][14]*SF[15] -(double)P[11][14]*SF[14] - ((double)P[12][14]*q0)/2) - SPP[9]*((double)P[3][15] +(double)P[0][15]*SF[7] +(double)P[1][15]*SF[6] +(double)P[2][15]*SF[9] +(double)P[10][15]*SF[15] -(double)P[11][15]*SF[14] - ((double)P[12][15]*q0)/2);
	nextP[4][4] =(double)P[4][4] +(double)P[0][4]*SF[5] +(double)P[1][4]*SF[3] -(double)P[3][4]*SF[4] +(double)P[2][4]*SPP[0] +(double)P[13][4]*SPP[3] +(double)P[14][4]*SPP[6] -(double)P[15][4]*SPP[9] + dvyVar*dsq(SG[7] - 2*q0*q3) + dvzVar*dsq(SG[6] + 2*q0*q2) + SF[5]*((double)P[4][0] +(double)P[0][0]*SF[5] +(double)P[1][0]*SF[3] -(double)P[3][0]*SF[4] +(double)P[2][0]*SPP[0] +(double)P[13][0]*SPP[3] +(double)P[14][0]*SPP[6] -(double)P[15][0]*SPP[9]) + SF[3]*((double)P[4][1] +(double)P[0][1]*SF[5] +(double)P[1][1]*SF[3] -(double)P[3][1]*SF[4] +(double)P[2][1]*SPP[0] +(double)P[13][1]*SPP[3] +(double)P[14][1]*SPP[6] -(double)P[15][1]*SPP[9]) - SF[4]*((double)P[4][3] +(double)P[0][3]*SF[5] +(double)P[1][3]*SF[3] -(double)P[3][3]*SF[4] +(double)P[2][3]*SPP[0] +(double)P[13][3]*SPP[3] +(double)P[14][3]*SPP[6] -(double)P[15][3]*SPP[9]) + SPP[0]*((double)P[4][2] +(double)P[0][2]*SF[5] +(double)P[1][2]*SF[3] -(double)P[3][2]*SF[4] +(double)P[2][2]*SPP[0] +(double)P[13][2]*SPP[3] +(double)P[14][2]*SPP[6] -(double)P[15][2]*SPP[9]) + SPP[3]*((double)P[4][13] +(double)P[0][13]*SF[5] +(double)P[1][13]*SF[3] -(double)P[3][13]*SF[4] +(double)P[2][13]*SPP[0] +(double)P[13][13]*SPP[3] +(double)P[14][13]*SPP[6] -(double)P[15][13]*SPP[9]) + SPP[6]*((double)P[4][14] +(double)P[0][14]*SF[5] +(double)P[1][14]*SF[3] -(double)P[3][14]*SF[4] +(double)P[2][14]*SPP[0] +(double)P[13][14]*SPP[3] +(double)P[14][14]*SPP[6] -(double)P[15][14]*SPP[9]) - SPP[9]*((double)P[4][15] +(double)P[0][15]*SF[5] +(double)P[1][15]*SF[3] -(double)P[3][15]*SF[4] +(double)P[2][15]*SPP[0] +(double)P[13][15]*SPP[3] +(double)P[14][15]*SPP[6] -(double)P[15][15]*SPP[9]) + dvxVar*dsq(SG[1] + SG[2] - SG[3] - SG[4]);
	nextP[0][5] =(double)P[0][5] +(double)P[1][5]*SF[9] +(double)P[2][5]*SF[11] +(double)P[3][5]*SF[10] +(double)P[10][5]*SF[14] +(double)P[11][5]*SF[15] +(double)P[12][5]*SPP[10] + SF[4]*((double)P[0][0] +(double)P[1][0]*SF[9] +(double)P[2][0]*SF[11] +(double)P[3][0]*SF[10] +(double)P[10][0]*SF[14] +(double)P[11][0]*SF[15] +(double)P[12][0]*SPP[10]) + SF[3]*((double)P[0][2] +(double)P[1][2]*SF[9] +(double)P[2][2]*SF[11] +(double)P[3][2]*SF[10] +(double)P[10][2]*SF[14] +(double)P[11][2]*SF[15] +(double)P[12][2]*SPP[10]) + SF[5]*((double)P[0][3] +(double)P[1][3]*SF[9] +(double)P[2][3]*SF[11] +(double)P[3][3]*SF[10] +(double)P[10][3]*SF[14] +(double)P[11][3]*SF[15] +(double)P[12][3]*SPP[10]) - SPP[0]*((double)P[0][1] +(double)P[1][1]*SF[9] +(double)P[2][1]*SF[11] +(double)P[3][1]*SF[10] +(double)P[10][1]*SF[14] +(double)P[11][1]*SF[15] +(double)P[12][1]*SPP[10]) - SPP[8]*((double)P[0][13] +(double)P[1][13]*SF[9] +(double)P[2][13]*SF[11] +(double)P[3][13]*SF[10] +(double)P[10][13]*SF[14] +(double)P[11][13]*SF[15] +(double)P[12][13]*SPP[10]) + SPP[2]*((double)P[0][14] +(double)P[1][14]*SF[9] +(double)P[2][14]*SF[11] +(double)P[3][14]*SF[10] +(double)P[10][14]*SF[14] +(double)P[11][14]*SF[15] +(double)P[12][14]*SPP[10]) + SPP[5]*((double)P[0][15] +(double)P[1][15]*SF[9] +(double)P[2][15]*SF[11] +(double)P[3][15]*SF[10] +(double)P[10][15]*SF[14] +(double)P[11][15]*SF[15] +(double)P[12][15]*SPP[10]);
	nextP[1][5] =(double)P[1][5] +(double)P[0][5]*SF[8] +(double)P[2][5]*SF[7] +(double)P[3][5]*SF[11] -(double)P[12][5]*SF[15] +(double)P[11][5]*SPP[10] - ((double)P[10][5]*q0)/2 + SF[4]*((double)P[1][0] +(double)P[0][0]*SF[8] +(double)P[2][0]*SF[7] +(double)P[3][0]*SF[11] -(double)P[12][0]*SF[15] +(double)P[11][0]*SPP[10] - ((double)P[10][0]*q0)/2) + SF[3]*((double)P[1][2] +(double)P[0][2]*SF[8] +(double)P[2][2]*SF[7] +(double)P[3][2]*SF[11] -(double)P[12][2]*SF[15] +(double)P[11][2]*SPP[10] - ((double)P[10][2]*q0)/2) + SF[5]*((double)P[1][3] +(double)P[0][3]*SF[8] +(double)P[2][3]*SF[7] +(double)P[3][3]*SF[11] -(double)P[12][3]*SF[15] +(double)P[11][3]*SPP[10] - ((double)P[10][3]*q0)/2) - SPP[0]*((double)P[1][1] +(double)P[0][1]*SF[8] +(double)P[2][1]*SF[7] +(double)P[3][1]*SF[11] -(double)P[12][1]*SF[15] +(double)P[11][1]*SPP[10] - ((double)P[10][1]*q0)/2) - SPP[8]*((double)P[1][13] +(double)P[0][13]*SF[8] +(double)P[2][13]*SF[7] +(double)P[3][13]*SF[11] -(double)P[12][13]*SF[15] +(double)P[11][13]*SPP[10] - ((double)P[10][13]*q0)/2) + SPP[2]*((double)P[1][14] +(double)P[0][14]*SF[8] +(double)P[2][14]*SF[7] +(double)P[3][14]*SF[11] -(double)P[12][14]*SF[15] +(double)P[11][14]*SPP[10] - ((double)P[10][14]*q0)/2) + SPP[5]*((double)P[1][15] +(double)P[0][15]*SF[8] +(double)P[2][15]*SF[7] +(double)P[3][15]*SF[11] -(double)P[12][15]*SF[15] +(double)P[11][15]*SPP[10] - ((double)P[10][15]*q0)/2);
	nextP[2][5] =(double)P[2][5] +(double)P[0][5]*SF[6] +(double)P[1][5]*SF[10] +(double)P[3][5]*SF[8] +(double)P[12][5]*SF[14] -(double)P[10][5]*SPP[10] - ((double)P[11][5]*q0)/2 + SF[4]*((double)P[2][0] +(double)P[0][0]*SF[6] +(double)P[1][0]*SF[10] +(double)P[3][0]*SF[8] +(double)P[12][0]*SF[14] -(double)P[10][0]*SPP[10] - ((double)P[11][0]*q0)/2) + SF[3]*((double)P[2][2] +(double)P[0][2]*SF[6] +(double)P[1][2]*SF[10] +(double)P[3][2]*SF[8] +(double)P[12][2]*SF[14] -(double)P[10][2]*SPP[10] - ((double)P[11][2]*q0)/2) + SF[5]*((double)P[2][3] +(double)P[0][3]*SF[6] +(double)P[1][3]*SF[10] +(double)P[3][3]*SF[8] +(double)P[12][3]*SF[14] -(double)P[10][3]*SPP[10] - ((double)P[11][3]*q0)/2) - SPP[0]*((double)P[2][1] +(double)P[0][1]*SF[6] +(double)P[1][1]*SF[10] +(double)P[3][1]*SF[8] +(double)P[12][1]*SF[14] -(double)P[10][1]*SPP[10] - ((double)P[11][1]*q0)/2) - SPP[8]*((double)P[2][13] +(double)P[0][13]*SF[6] +(double)P[1][13]*SF[10] +(double)P[3][13]*SF[8] +(double)P[12][13]*SF[14] -(double)P[10][13]*SPP[10] - ((double)P[11][13]*q0)/2) + SPP[2]*((double)P[2][14] +(double)P[0][14]*SF[6] +(double)P[1][14]*SF[10] +(double)P[3][14]*SF[8] +(double)P[12][14]*SF[14] -(double)P[10][14]*SPP[10] - ((double)P[11][14]*q0)/2) + SPP[5]*((double)P[2][15] +(double)P[0][15]*SF[6] +(double)P[1][15]*SF[10] +(double)P[3][15]*SF[8] +(double)P[12][15]*SF[14] -(double)P[10][15]*SPP[10] - ((double)P[11][15]*q0)/2);
	nextP[3][5] =(double)P[3][5] +(double)P[0][5]*SF[7] +(double)P[1][5]*SF[6] +(double)P[2][5]*SF[9] +(double)P[10][5]*SF[15] -(double)P[11][5]*SF[14] - ((double)P[12][5]*q0)/2 + SF[4]*((double)P[3][0] +(double)P[0][0]*SF[7] +(double)P[1][0]*SF[6] +(double)P[2][0]*SF[9] +(double)P[10][0]*SF[15] -(double)P[11][0]*SF[14] - ((double)P[12][0]*q0)/2) + SF[3]*((double)P[3][2] +(double)P[0][2]*SF[7] +(double)P[1][2]*SF[6] +(double)P[2][2]*SF[9] +(double)P[10][2]*SF[15] -(double)P[11][2]*SF[14] - ((double)P[12][2]*q0)/2) + SF[5]*((double)P[3][3] +(double)P[0][3]*SF[7] +(double)P[1][3]*SF[6] +(double)P[2][3]*SF[9] +(double)P[10][3]*SF[15] -(double)P[11][3]*SF[14] - ((double)P[12][3]*q0)/2) - SPP[0]*((double)P[3][1] +(double)P[0][1]*SF[7] +(double)P[1][1]*SF[6] +(double)P[2][1]*SF[9] +(double)P[10][1]*SF[15] -(double)P[11][1]*SF[14] - ((double)P[12][1]*q0)/2) - SPP[8]*((double)P[3][13] +(double)P[0][13]*SF[7] +(double)P[1][13]*SF[6] +(double)P[2][13]*SF[9] +(double)P[10][13]*SF[15] -(double)P[11][13]*SF[14] - ((double)P[12][13]*q0)/2) + SPP[2]*((double)P[3][14] +(double)P[0][14]*SF[7] +(double)P[1][14]*SF[6] +(double)P[2][14]*SF[9] +(double)P[10][14]*SF[15] -(double)P[11][14]*SF[14] - ((double)P[12][14]*q0)/2) + SPP[5]*((double)P[3][15] +(double)P[0][15]*SF[7] +(double)P[1][15]*SF[6] +(double)P[2][15]*SF[9] +(double)P[10][15]*SF[15] -(double)P[11][15]*SF[14] - ((double)P[12][15]*q0)/2);
	nextP[4][5] =(double)P[4][5] + SQ[2] +(double)P[0][5]*SF[5] +(double)P[1][5]*SF[3] -(double)P[3][5]*SF[4] +(double)P[2][5]*SPP[0] +(double)P[13][5]*SPP[3] +(double)P[14][5]*SPP[6] -(double)P[15][5]*SPP[9] + SF[4]*((double)P[4][0] +(double)P[0][0]*SF[5] +(double)P[1][0]*SF[3] -(double)P[3][0]*SF[4] +(double)P[2][0]*SPP[0] +(double)P[13][0]*SPP[3] +(double)P[14][0]*SPP[6] -(double)P[15][0]*SPP[9]) + SF[3]*((double)P[4][2] +(double)P[0][2]*SF[5] +(double)P[1][2]*SF[3] -(double)P[3][2]*SF[4] +(double)P[2][2]*SPP[0] +(double)P[13][2]*SPP[3] +(double)P[14][2]*SPP[6] -(double)P[15][2]*SPP[9]) + SF[5]*((double)P[4][3] +(double)P[0][3]*SF[5] +(double)P[1][3]*SF[3] -(double)P[3][3]*SF[4] +(double)P[2][3]*SPP[0] +(double)P[13][3]*SPP[3] +(double)P[14][3]*SPP[6] -(double)P[15][3]*SPP[9]) - SPP[0]*((double)P[4][1] +(double)P[0][1]*SF[5] +(double)P[1][1]*SF[3] -(double)P[3][1]*SF[4] +(double)P[2][1]*SPP[0] +(double)P[13][1]*SPP[3] +(double)P[14][1]*SPP[6] -(double)P[15][1]*SPP[9]) - SPP[8]*((double)P[4][13] +(double)P[0][13]*SF[5] +(double)P[1][13]*SF[3] -(double)P[3][13]*SF[4] +(double)P[2][13]*SPP[0] +(double)P[13][13]*SPP[3] +(double)P[14][13]*SPP[6] -(double)P[15][13]*SPP[9]) + SPP[2]*((double)P[4][14] +(double)P[0][14]*SF[5] +(double)P[1][14]*SF[3] -(double)P[3][14]*SF[4] +(double)P[2][14]*SPP[0] +(double)P[13][14]*SPP[3] +(double)P[14][14]*SPP[6] -(double)P[15][14]*SPP[9]) + SPP[5]*((double)P[4][15] +(double)P[0][15]*SF[5] +(double)P[1][15]*SF[3] -(double)P[3][15]*SF[4] +(double)P[2][15]*SPP[0] +(double)P[13][15]*SPP[3] +(double)P[14][15]*SPP[6] -(double)P[15][15]*SPP[9]);
	nextP[5][5] =(double)P[5][5] +(double)P[0][5]*SF[4] +(double)P[2][5]*SF[3] +(double)P[3][5]*SF[5] -(double)P[1][5]*SPP[0] -(double)P[13][5]*SPP[8] +(double)P[14][5]*SPP[2] +(double)P[15][5]*SPP[5] + dvxVar*dsq(SG[7] + 2*q0*q3) + dvzVar*dsq(SG[5] - 2*q0*q1) + SF[4]*((double)P[5][0] +(double)P[0][0]*SF[4] +(double)P[2][0]*SF[3] +(double)P[3][0]*SF[5] -(double)P[1][0]*SPP[0] -(double)P[13][0]*SPP[8] +(double)P[14][0]*SPP[2] +(double)P[15][0]*SPP[5]) + SF[3]*((double)P[5][2] +(double)P[0][2]*SF[4] +(double)P[2][2]*SF[3] +(double)P[3][2]*SF[5] -(double)P[1][2]*SPP[0] -(double)P[13][2]*SPP[8] +(double)P[14][2]*SPP[2] +(double)P[15][2]*SPP[5]) + SF[5]*((double)P[5][3] +(double)P[0][3]*SF[4] +(double)P[2][3]*SF[3] +(double)P[3][3]*SF[5] -(double)P[1][3]*SPP[0] -(double)P[13][3]*SPP[8] +(double)P[14][3]*SPP[2] +(double)P[15][3]*SPP[5]) - SPP[0]*((double)P[5][1] +(double)P[0][1]*SF[4] +(double)P[2][1]*SF[3] +(double)P[3][1]*SF[5] -(double)P[1][1]*SPP[0] -(double)P[13][1]*SPP[8] +(double)P[14][1]*SPP[2] +(double)P[15][1]*SPP[5]) - SPP[8]*((double)P[5][13] +(double)P[0][13]*SF[4] +(double)P[2][13]*SF[3] +(double)P[3][13]*SF[5] -(double)P[1][13]*SPP[0] -(double)P[13][13]*SPP[8] +(double)P[14][13]*SPP[2] +(double)P[15][13]*SPP[5]) + SPP[2]*((double)P[5][14] +(double)P[0][14]*SF[4] +(double)P[2][14]*SF[3] +(double)P[3][14]*SF[5] -(double)P[1][14]*SPP[0] -(double)P[13][14]*SPP[8] +(double)P[14][14]*SPP[2] +(double)P[15][14]*SPP[5]) + SPP[5]*((double)P[5][15] +(double)P[0][15]*SF[4] +(double)P[2][15]*SF[3] +(double)P[3][15]*SF[5] -(double)P[1][15]*SPP[0] -(double)P[13][15]*SPP[8] +(double)P[14][15]*SPP[2] +(double)P[15][15]*SPP[5]) + dvyVar*dsq(SG[1] - SG[2] + SG[3] - SG[4]);
	nextP[0][6] =(double)P[0][6] +(double)P[1][6]*SF[9] +(double)P[2][6]*SF[11] +(double)P[3][6]*SF[10] +(double)P[10][6]*SF[14] +(double)P[11][6]*SF[15] +(double)P[12][6]*SPP[10] + SF[4]*((double)P[0][1] +(double)P[1][1]*SF[9] +(double)P[2][1]*SF[11] +(double)P[3][1]*SF[10] +(double)P[10][1]*SF[14] +(double)P[11][1]*SF[15] +(double)P[12][1]*SPP[10]) - SF[5]*((double)P[0][2] +(double)P[1][2]*SF[9] +(double)P[2][2]*SF[11] +(double)P[3][2]*SF[10] +(double)P[10][2]*SF[14] +(double)P[11][2]*SF[15] +(double)P[12][2]*SPP[10]) + SF[3]*((double)P[0][3] +(double)P[1][3]*SF[9] +(double)P[2][3]*SF[11] +(double)P[3][3]*SF[10] +(double)P[10][3]*SF[14] +(double)P[11][3]*SF[15] +(double)P[12][3]*SPP[10]) + SPP[0]*((double)P[0][0] +(double)P[1][0]*SF[9] +(double)P[2][0]*SF[11] +(double)P[3][0]*SF[10] +(double)P[10][0]*SF[14] +(double)P[11][0]*SF[15] +(double)P[12][0]*SPP[10]) + SPP[4]*((double)P[0][13] +(double)P[1][13]*SF[9] +(double)P[2][13]*SF[11] +(double)P[3][13]*SF[10] +(double)P[10][13]*SF[14] +(double)P[11][13]*SF[15] +(double)P[12][13]*SPP[10]) - SPP[7]*((double)P[0][14] +(double)P[1][14]*SF[9] +(double)P[2][14]*SF[11] +(double)P[3][14]*SF[10] +(double)P[10][14]*SF[14] +(double)P[11][14]*SF[15] +(double)P[12][14]*SPP[10]) - SPP[1]*((double)P[0][15] +(double)P[1][15]*SF[9] +(double)P[2][15]*SF[11] +(double)P[3][15]*SF[10] +(double)P[10][15]*SF[14] +(double)P[11][15]*SF[15] +(double)P[12][15]*SPP[10]);
	nextP[1][6] =(double)P[1][6] +(double)P[0][6]*SF[8] +(double)P[2][6]*SF[7] +(double)P[3][6]*SF[11] -(double)P[12][6]*SF[15] +(double)P[11][6]*SPP[10] - ((double)P[10][6]*q0)/2 + SF[4]*((double)P[1][1] +(double)P[0][1]*SF[8] +(double)P[2][1]*SF[7] +(double)P[3][1]*SF[11] -(double)P[12][1]*SF[15] +(double)P[11][1]*SPP[10] - ((double)P[10][1]*q0)/2) - SF[5]*((double)P[1][2] +(double)P[0][2]*SF[8] +(double)P[2][2]*SF[7] +(double)P[3][2]*SF[11] -(double)P[12][2]*SF[15] +(double)P[11][2]*SPP[10] - ((double)P[10][2]*q0)/2) + SF[3]*((double)P[1][3] +(double)P[0][3]*SF[8] +(double)P[2][3]*SF[7] +(double)P[3][3]*SF[11] -(double)P[12][3]*SF[15] +(double)P[11][3]*SPP[10] - ((double)P[10][3]*q0)/2) + SPP[0]*((double)P[1][0] +(double)P[0][0]*SF[8] +(double)P[2][0]*SF[7] +(double)P[3][0]*SF[11] -(double)P[12][0]*SF[15] +(double)P[11][0]*SPP[10] - ((double)P[10][0]*q0)/2) + SPP[4]*((double)P[1][13] +(double)P[0][13]*SF[8] +(double)P[2][13]*SF[7] +(double)P[3][13]*SF[11] -(double)P[12][13]*SF[15] +(double)P[11][13]*SPP[10] - ((double)P[10][13]*q0)/2) - SPP[7]*((double)P[1][14] +(double)P[0][14]*SF[8] +(double)P[2][14]*SF[7] +(double)P[3][14]*SF[11] -(double)P[12][14]*SF[15] +(double)P[11][14]*SPP[10] - ((double)P[10][14]*q0)/2) - SPP[1]*((double)P[1][15] +(double)P[0][15]*SF[8] +(double)P[2][15]*SF[7] +(double)P[3][15]*SF[11] -(double)P[12][15]*SF[15] +(double)P[11][15]*SPP[10] - ((double)P[10][15]*q0)/2);
	nextP[2][6] =(double)P[2][6] +(double)P[0][6]*SF[6] +(double)P[1][6]*SF[10] +(double)P[3][6]*SF[8] +(double)P[12][6]*SF[14] -(double)P[10][6]*SPP[10] - ((double)P[11][6]*q0)/2 + SF[4]*((double)P[2][1] +(double)P[0][1]*SF[6] +(double)P[1][1]*SF[10] +(double)P[3][1]*SF[8] +(double)P[12][1]*SF[14] -(double)P[10][1]*SPP[10] - ((double)P[11][1]*q0)/2) - SF[5]*((double)P[2][2] +(double)P[0][2]*SF[6] +(double)P[1][2]*SF[10] +(double)P[3][2]*SF[8] +(double)P[12][2]*SF[14] -(double)P[10][2]*SPP[10] - ((double)P[11][2]*q0)/2) + SF[3]*((double)P[2][3] +(double)P[0][3]*SF[6] +(double)P[1][3]*SF[10] +(double)P[3][3]*SF[8] +(double)P[12][3]*SF[14] -(double)P[10][3]*SPP[10] - ((double)P[11][3]*q0)/2) + SPP[0]*((double)P[2][0] +(double)P[0][0]*SF[6] +(double)P[1][0]*SF[10] +(double)P[3][0]*SF[8] +(double)P[12][0]*SF[14] -(double)P[10][0]*SPP[10] - ((double)P[11][0]*q0)/2) + SPP[4]*((double)P[2][13] +(double)P[0][13]*SF[6] +(double)P[1][13]*SF[10] +(double)P[3][13]*SF[8] +(double)P[12][13]*SF[14] -(double)P[10][13]*SPP[10] - ((double)P[11][13]*q0)/2) - SPP[7]*((double)P[2][14] +(double)P[0][14]*SF[6] +(double)P[1][14]*SF[10] +(double)P[3][14]*SF[8] +(double)P[12][14]*SF[14] -(double)P[10][14]*SPP[10] - ((double)P[11][14]*q0)/2) - SPP[1]*((double)P[2][15] +(double)P[0][15]*SF[6] +(double)P[1][15]*SF[10] +(double)P[3][15]*SF[8] +(double)P[12][15]*SF[14] -(double)P[10][15]*SPP[10] - ((double)P[11][15]*q0)/2);
	nextP[3][6] =(double)P[3][6] +(double)P[0][6]*SF[7] +(double)P[1][6]*SF[6] +(double)P[2][6]*SF[9] +(double)P[10][6]*SF[15] -(double)P[11][6]*SF[14] - ((double)P[12][6]*q0)/2 + SF[4]*((double)P[3][1] +(double)P[0][1]*SF[7] +(double)P[1][1]*SF[6] +(double)P[2][1]*SF[9] +(double)P[10][1]*SF[15] -(double)P[11][1]*SF[14] - ((double)P[12][1]*q0)/2) - SF[5]*((double)P[3][2] +(double)P[0][2]*SF[7] +(double)P[1][2]*SF[6] +(double)P[2][2]*SF[9] +(double)P[10][2]*SF[15] -(double)P[11][2]*SF[14] - ((double)P[12][2]*q0)/2) + SF[3]*((double)P[3][3] +(double)P[0][3]*SF[7] +(double)P[1][3]*SF[6] +(double)P[2][3]*SF[9] +(double)P[10][3]*SF[15] -(double)P[11][3]*SF[14] - ((double)P[12][3]*q0)/2) + SPP[0]*((double)P[3][0] +(double)P[0][0]*SF[7] +(double)P[1][0]*SF[6] +(double)P[2][0]*SF[9] +(double)P[10][0]*SF[15] -(double)P[11][0]*SF[14] - ((double)P[12][0]*q0)/2) + SPP[4]*((double)P[3][13] +(double)P[0][13]*SF[7] +(double)P[1][13]*SF[6] +(double)P[2][13]*SF[9] +(double)P[10][13]*SF[15] -(double)P[11][13]*SF[14] - ((double)P[12][13]*q0)/2) - SPP[7]*((double)P[3][14] +(double)P[0][14]*SF[7] +(double)P[1][14]*SF[6] +(double)P[2][14]*SF[9] +(double)P[10][14]*SF[15] -(double)P[11][14]*SF[14] - ((double)P[12][14]*q0)/2) - SPP[1]*((double)P[3][15] +(double)P[0][15]*SF[7] +(double)P[1][15]*SF[6] +(double)P[2][15]*SF[9] +(double)P[10][15]*SF[15] -(double)P[11][15]*SF[14] - ((double)P[12][15]*q0)/2);
	nextP[4][6] =(double)P[4][6] + SQ[1] +(double)P[0][6]*SF[5] +(double)P[1][6]*SF[3] -(double)P[3][6]*SF[4] +(double)P[2][6]*SPP[0] +(double)P[13][6]*SPP[3] +(double)P[14][6]*SPP[6] -(double)P[15][6]*SPP[9] + SF[4]*((double)P[4][1] +(double)P[0][1]*SF[5] +(double)P[1][1]*SF[3] -(double)P[3][1]*SF[4] +(double)P[2][1]*SPP[0] +(double)P[13][1]*SPP[3] +(double)P[14][1]*SPP[6] -(double)P[15][1]*SPP[9]) - SF[5]*((double)P[4][2] +(double)P[0][2]*SF[5] +(double)P[1][2]*SF[3] -(double)P[3][2]*SF[4] +(double)P[2][2]*SPP[0] +(double)P[13][2]*SPP[3] +(double)P[14][2]*SPP[6] -(double)P[15][2]*SPP[9]) + SF[3]*((double)P[4][3] +(double)P[0][3]*SF[5] +(double)P[1][3]*SF[3] -(double)P[3][3]*SF[4] +(double)P[2][3]*SPP[0] +(double)P[13][3]*SPP[3] +(double)P[14][3]*SPP[6] -(double)P[15][3]*SPP[9]) + SPP[0]*((double)P[4][0] +(double)P[0][0]*SF[5] +(double)P[1][0]*SF[3] -(double)P[3][0]*SF[4] +(double)P[2][0]*SPP[0] +(double)P[13][0]*SPP[3] +(double)P[14][0]*SPP[6] -(double)P[15][0]*SPP[9]) + SPP[4]*((double)P[4][13] +(double)P[0][13]*SF[5] +(double)P[1][13]*SF[3] -(double)P[3][13]*SF[4] +(double)P[2][13]*SPP[0] +(double)P[13][13]*SPP[3] +(double)P[14][13]*SPP[6] -(double)P[15][13]*SPP[9]) - SPP[7]*((double)P[4][14] +(double)P[0][14]*SF[5] +(double)P[1][14]*SF[3] -(double)P[3][14]*SF[4] +(double)P[2][14]*SPP[0] +(double)P[13][14]*SPP[3] +(double)P[14][14]*SPP[6] -(double)P[15][14]*SPP[9]) - SPP[1]*((double)P[4][15] +(double)P[0][15]*SF[5] +(double)P[1][15]*SF[3] -(double)P[3][15]*SF[4] +(double)P[2][15]*SPP[0] +(double)P[13][15]*SPP[3] +(double)P[14][15]*SPP[6] -(double)P[15][15]*SPP[9]);
	nextP[5][6] =(double)P[5][6] + SQ[0] +(double)P[0][6]*SF[4] +(double)P[2][6]*SF[3] +(double)P[3][6]*SF[5] -(double)P[1][6]*SPP[0] -(double)P[13][6]*SPP[8] +(double)P[14][6]*SPP[2] +(double)P[15][6]*SPP[5] + SF[4]*((double)P[5][1] +(double)P[0][1]*SF[4] +(double)P[2][1]*SF[3] +(double)P[3][1]*SF[5] -(double)P[1][1]*SPP[0] -(double)P[13][1]*SPP[8] +(double)P[14][1]*SPP[2] +(double)P[15][1]*SPP[5]) - SF[5]*((double)P[5][2] +(double)P[0][2]*SF[4] +(double)P[2][2]*SF[3] +(double)P[3][2]*SF[5] -(double)P[1][2]*SPP[0] -(double)P[13][2]*SPP[8] +(double)P[14][2]*SPP[2] +(double)P[15][2]*SPP[5]) + SF[3]*((double)P[5][3] +(double)P[0][3]*SF[4] +(double)P[2][3]*SF[3] +(double)P[3][3]*SF[5] -(double)P[1][3]*SPP[0] -(double)P[13][3]*SPP[8] +(double)P[14][3]*SPP[2] +(double)P[15][3]*SPP[5]) + SPP[0]*((double)P[5][0] +(double)P[0][0]*SF[4] +(double)P[2][0]*SF[3] +(double)P[3][0]*SF[5] -(double)P[1][0]*SPP[0] -(double)P[13][0]*SPP[8] +(double)P[14][0]*SPP[2] +(double)P[15][0]*SPP[5]) + SPP[4]*((double)P[5][13] +(double)P[0][13]*SF[4] +(double)P[2][13]*SF[3] +(double)P[3][13]*SF[5] -(double)P[1][13]*SPP[0] -(double)P[13][13]*SPP[8] +(double)P[14][13]*SPP[2] +(double)P[15][13]*SPP[5]) - SPP[7]*((double)P[5][14] +(double)P[0][14]*SF[4] +(double)P[2][14]*SF[3] +(double)P[3][14]*SF[5] -(double)P[1][14]*SPP[0] -(double)P[13][14]*SPP[8] +(double)P[14][14]*SPP[2] +(double)P[15][14]*SPP[5]) - SPP[1]*((double)P[5][15] +(double)P[0][15]*SF[4] +(double)P[2][15]*SF[3] +(double)P[3][15]*SF[5] -(double)P[1][15]*SPP[0] -(double)P[13][15]*SPP[8] +(double)P[14][15]*SPP[2] +(double)P[15][15]*SPP[5]);
	nextP[6][6] =(double)P[6][6] +(double)P[1][6]*SF[4] -(double)P[2][6]*SF[5] +(double)P[3][6]*SF[3] +(double)P[0][6]*SPP[0] +(double)P[13][6]*SPP[4] -(double)P[14][6]*SPP[7] -(double)P[15][6]*SPP[1] + dvxVar*dsq(SG[6] - 2*q0*q2) + dvyVar*dsq(SG[5] + 2*q0*q1) + SF[4]*((double)P[6][1] +(double)P[1][1]*SF[4] -(double)P[2][1]*SF[5] +(double)P[3][1]*SF[3] +(double)P[0][1]*SPP[0] +(double)P[13][1]*SPP[4] -(double)P[14][1]*SPP[7] -(double)P[15][1]*SPP[1]) - SF[5]*((double)P[6][2] +(double)P[1][2]*SF[4] -(double)P[2][2]*SF[5] +(double)P[3][2]*SF[3] +(double)P[0][2]*SPP[0] +(double)P[13][2]*SPP[4] -(double)P[14][2]*SPP[7] -(double)P[15][2]*SPP[1]) + SF[3]*((double)P[6][3] +(double)P[1][3]*SF[4] -(double)P[2][3]*SF[5] +(double)P[3][3]*SF[3] +(double)P[0][3]*SPP[0] +(double)P[13][3]*SPP[4] -(double)P[14][3]*SPP[7] -(double)P[15][3]*SPP[1]) + SPP[0]*((double)P[6][0] +(double)P[1][0]*SF[4] -(double)P[2][0]*SF[5] +(double)P[3][0]*SF[3] +(double)P[0][0]*SPP[0] +(double)P[13][0]*SPP[4] -(double)P[14][0]*SPP[7] -(double)P[15][0]*SPP[1]) + SPP[4]*((double)P[6][13] +(double)P[1][13]*SF[4] -(double)P[2][13]*SF[5] +(double)P[3][13]*SF[3] +(double)P[0][13]*SPP[0] +(double)P[13][13]*SPP[4] -(double)P[14][13]*SPP[7] -(double)P[15][13]*SPP[1]) - SPP[7]*((double)P[6][14] +(double)P[1][14]*SF[4] -(double)P[2][14]*SF[5] +(double)P[3][14]*SF[3] +(double)P[0][14]*SPP[0] +(double)P[13][14]*SPP[4] -(double)P[14][14]*SPP[7] -(double)P[15][14]*SPP[1]) - SPP[1]*((double)P[6][15] +(double)P[1][15]*SF[4] -(double)P[2][15]*SF[5] +(double)P[3][15]*SF[3] +(double)P[0][15]*SPP[0] +(double)P[13][15]*SPP[4] -(double)P[14][15]*SPP[7] -(double)P[15][15]*SPP[1]) + dvzVar*dsq(SG[1] - SG[2] - SG[3] + SG[4]);
	nextP[0][7] =(double)P[0][7] +(double)P[1][7]*SF[9] +(double)P[2][7]*SF[11] +(double)P[3][7]*SF[10] +(double)P[10][7]*SF[14] +(double)P[11][7]*SF[15] +(double)P[12][7]*SPP[10] + (double)dt*((double)P[0][4] +(double)P[1][4]*SF[9] +(double)P[2][4]*SF[11] +(double)P[3][4]*SF[10] +(double)P[10][4]*SF[14] +(double)P[11][4]*SF[15] +(double)P[12][4]*SPP[10]);
	nextP[1][7] =(double)P[1][7] +(double)P[0][7]*SF[8] +(double)P[2][7]*SF[7] +(double)P[3][7]*SF[11] -(double)P[12][7]*SF[15] +(double)P[11][7]*SPP[10] - ((double)P[10][7]*q0)/2 + (double)dt*((double)P[1][4] +(double)P[0][4]*SF[8] +(double)P[2][4]*SF[7] +(double)P[3][4]*SF[11] -(double)P[12][4]*SF[15] +(double)P[11][4]*SPP[10] - ((double)P[10][4]*q0)/2);
	nextP[2][7] =(double)P[2][7] +(double)P[0][7]*SF[6] +(double)P[1][7]*SF[10] +(double)P[3][7]*SF[8] +(double)P[12][7]*SF[14] -(double)P[10][7]*SPP[10] - ((double)P[11][7]*q0)/2 + (double)dt*((double)P[2][4] +(double)P[0][4]*SF[6] +(double)P[1][4]*SF[10] +(double)P[3][4]*SF[8] +(double)P[12][4]*SF[14] -(double)P[10][4]*SPP[10] - ((double)P[11][4]*q0)/2);
	nextP[3][7] =(double)P[3][7] +(double)P[0][7]*SF[7] +(double)P[1][7]*SF[6] +(double)P[2][7]*SF[9] +(double)P[10][7]*SF[15] -(double)P[11][7]*SF[14] - ((double)P[12][7]*q0)/2 + (double)dt*((double)P[3][4] +(double)P[0][4]*SF[7] +(double)P[1][4]*SF[6] +(double)P[2][4]*SF[9] +(double)P[10][4]*SF[15] -(double)P[11][4]*SF[14] - ((double)P[12][4]*q0)/2);
	nextP[4][7] =(double)P[4][7] +(double)P[0][7]*SF[5] +(double)P[1][7]*SF[3] -(double)P[3][7]*SF[4] +(double)P[2][7]*SPP[0] +(double)P[13][7]*SPP[3] +(double)P[14][7]*SPP[6] -(double)P[15][7]*SPP[9] + (double)dt*((double)P[4][4] +(double)P[0][4]*SF[5] +(double)P[1][4]*SF[3] -(double)P[3][4]*SF[4] +(double)P[2][4]*SPP[0] +(double)P[13][4]*SPP[3] +(double)P[14][4]*SPP[6] -(double)P[15][4]*SPP[9]);
	nextP[5][7] =(double)P[5][7] +(double)P[0][7]*SF[4] +(double)P[2][7]*SF[3] +(double)P[3][7]*SF[5] -(double)P[1][7]*SPP[0] -(double)P[13][7]*SPP[8] +(double)P[14][7]*SPP[2] +(double)P[15][7]*SPP[5] + (double)dt*((double)P[5][4] +(double)P[0][4]*SF[4] +(double)P[2][4]*SF[3] +(double)P[3][4]*SF[5] -(double)P[1][4]*SPP[0] -(double)P[13][4]*SPP[8] +(double)P[14][4]*SPP[2] +(double)P[15][4]*SPP[5]);
	nextP[6][7] =(double)P[6][7] +(double)P[1][7]*SF[4] -(double)P[2][7]*SF[5] +(double)P[3][7]*SF[3] +(double)P[0][7]*SPP[0] +(double)P[13][7]*SPP[4] -(double)P[14][7]*SPP[7] -(double)P[15][7]*SPP[1] + (double)dt*((double)P[6][4] +(double)P[1][4]*SF[4] -(double)P[2][4]*SF[5] +(double)P[3][4]*SF[3] +(double)P[0][4]*SPP[0] +(double)P[13][4]*SPP[4] -(double)P[14][4]*SPP[7] -(double)P[15][4]*SPP[1]);
	nextP[7][7] =(double)P[7][7] +(double)P[4][7]*(double)dt + (double)dt*((double)P[7][4] +(double)P[4][4]*(double)dt);
	nextP[0][8] =(double)P[0][8] +(double)P[1][8]*SF[9] +(double)P[2][8]*SF[11] +(double)P[3][8]*SF[10] +(double)P[10][8]*SF[14] +(double)P[11][8]*SF[15] +(double)P[12][8]*SPP[10] + (double)dt*((double)P[0][5] +(double)P[1][5]*SF[9] +(double)P[2][5]*SF[11] +(double)P[3][5]*SF[10] +(double)P[10][5]*SF[14] +(double)P[11][5]*SF[15] +(double)P[12][5]*SPP[10]);
	nextP[1][8] =(double)P[1][8] +(double)P[0][8]*SF[8] +(double)P[2][8]*SF[7] +(double)P[3][8]*SF[11] -(double)P[12][8]*SF[15] +(double)P[11][8]*SPP[10] - ((double)P[10][8]*q0)/2 + (double)dt*((double)P[1][5] +(double)P[0][5]*SF[8] +(double)P[2][5]*SF[7] +(double)P[3][5]*SF[11] -(double)P[12][5]*SF[15] +(double)P[11][5]*SPP[10] - ((double)P[10][5]*q0)/2);
	nextP[2][8] =(double)P[2][8] +(double)P[0][8]*SF[6] +(double)P[1][8]*SF[10] +(double)P[3][8]*SF[8] +(double)P[12][8]*SF[14] -(double)P[10][8]*SPP[10] - ((double)P[11][8]*q0)/2 + (double)dt*((double)P[2][5] +(double)P[0][5]*SF[6] +(double)P[1][5]*SF[10] +(double)P[3][5]*SF[8] +(double)P[12][5]*SF[14] -(double)P[10][5]*SPP[10] - ((double)P[11][5]*q0)/2);
	nextP[3][8] =(double)P[3][8] +(double)P[0][8]*SF[7] +(double)P[1][8]*SF[6] +(double)P[2][8]*SF[9] +(double)P[10][8]*SF[15] -(double)P[11][8]*SF[14] - ((double)P[12][8]*q0)/2 + (double)dt*((double)P[3][5] +(double)P[0][5]*SF[7] +(double)P[1][5]*SF[6] +(double)P[2][5]*SF[9] +(double)P[10][5]*SF[15] -(double)P[11][5]*SF[14] - ((double)P[12][5]*q0)/2);
	nextP[4][8] =(double)P[4][8] +(double)P[0][8]*SF[5] +(double)P[1][8]*SF[3] -(double)P[3][8]*SF[4] +(double)P[2][8]*SPP[0] +(double)P[13][8]*SPP[3] +(double)P[14][8]*SPP[6] -(double)P[15][8]*SPP[9] + (double)dt*((double)P[4][5] +(double)P[0][5]*SF[5] +(double)P[1][5]*SF[3] -(double)P[3][5]*SF[4] +(double)P[2][5]*SPP[0] +(double)P[13][5]*SPP[3] +(double)P[14][5]*SPP[6] -(double)P[15][5]*SPP[9]);
	nextP[5][8] =(double)P[5][8] +(double)P[0][8]*SF[4] +(double)P[2][8]*SF[3] +(double)P[3][8]*SF[5] -(double)P[1][8]*SPP[0] -(double)P[13][8]*SPP[8] +(double)P[14][8]*SPP[2] +(double)P[15][8]*SPP[5] + (double)dt*((double)P[5][5] +(double)P[0][5]*SF[4] +(double)P[2][5]*SF[3] +(double)P[3][5]*SF[5] -(double)P[1][5]*SPP[0] -(double)P[13][5]*SPP[8] +(double)P[14][5]*SPP[2] +(double)P[15][5]*SPP[5]);
	nextP[6][8] =(double)P[6][8] +(double)P[1][8]*SF[4] -(double)P[2][8]*SF[5] +(double)P[3][8]*SF[3] +(double)P[0][8]*SPP[0] +(double)P[13][8]*SPP[4] -(double)P[14][8]*SPP[7] -(double)P[15][8]*SPP[1] + (double)dt*((double)P[6][5] +(double)P[1][5]*SF[4] -(double)P[2][5]*SF[5] +(double)P[3][5]*SF[3] +(double)P[0][5]*SPP[0] +(double)P[13][5]*SPP[4] -(double)P[14][5]*SPP[7] -(double)P[15][5]*SPP[1]);
	nextP[7][8] =(double)P[7][8] +(double)P[4][8]*(double)dt + (double)dt*((double)P[7][5] +(double)P[4][5]*(double)dt);
	nextP[8][8] =(double)P[8][8] +(double)P[5][8]*(double)dt + (double)dt*((double)P[8][5] +(double)P[5][5]*(double)dt);
	nextP[0][9] =(double)P[0][9] +(double)P[1][9]*SF[9] +(double)P[2][9]*SF[11] +(double)P[3][9]*SF[10] +(double)P[10][9]*SF[14] +(double)P[11][9]*SF[15] +(double)P[12][9]*SPP[10] + (double)dt*((double)P[0][6] +(double)P[1][6]*SF[9] +(double)P[2][6]*SF[11] +(double)P[3][6]*SF[10] +(double)P[10][6]*SF[14] +(double)P[11][6]*SF[15] +(double)P[12][6]*SPP[10]);
	nextP[1][9] =(double)P[1][9] +(double)P[0][9]*SF[8] +(double)P[2][9]*SF[7] +(double)P[3][9]*SF[11] -(double)P[12][9]*SF[15] +(double)P[11][9]*SPP[10] - ((double)P[10][9]*q0)/2 + (double)dt*((double)P[1][6] +(double)P[0][6]*SF[8] +(double)P[2][6]*SF[7] +(double)P[3][6]*SF[11] -(double)P[12][6]*SF[15] +(double)P[11][6]*SPP[10] - ((double)P[10][6]*q0)/2);
	nextP[2][9] =(double)P[2][9] +(double)P[0][9]*SF[6] +(double)P[1][9]*SF[10] +(double)P[3][9]*SF[8] +(double)P[12][9]*SF[14] -(double)P[10][9]*SPP[10] - ((double)P[11][9]*q0)/2 + (double)dt*((double)P[2][6] +(double)P[0][6]*SF[6] +(double)P[1][6]*SF[10] +(double)P[3][6]*SF[8] +(double)P[12][6]*SF[14] -(double)P[10][6]*SPP[10] - ((double)P[11][6]*q0)/2);
	nextP[3][9] =(double)P[3][9] +(double)P[0][9]*SF[7] +(double)P[1][9]*SF[6] +(double)P[2][9]*SF[9] +(double)P[10][9]*SF[15] -(double)P[11][9]*SF[14] - ((double)P[12][9]*q0)/2 + (double)dt*((double)P[3][6] +(double)P[0][6]*SF[7] +(double)P[1][6]*SF[6] +(double)P[2][6]*SF[9] +(double)P[10][6]*SF[15] -(double)P[11][6]*SF[14] - ((double)P[12][6]*q0)/2);
	nextP[4][9] =(double)P[4][9] +(double)P[0][9]*SF[5] +(double)P[1][9]*SF[3] -(double)P[3][9]*SF[4] +(double)P[2][9]*SPP[0] +(double)P[13][9]*SPP[3] +(double)P[14][9]*SPP[6] -(double)P[15][9]*SPP[9] + (double)dt*((double)P[4][6] +(double)P[0][6]*SF[5] +(double)P[1][6]*SF[3] -(double)P[3][6]*SF[4] +(double)P[2][6]*SPP[0] +(double)P[13][6]*SPP[3] +(double)P[14][6]*SPP[6] -(double)P[15][6]*SPP[9]);
	nextP[5][9] =(double)P[5][9] +(double)P[0][9]*SF[4] +(double)P[2][9]*SF[3] +(double)P[3][9]*SF[5] -(double)P[1][9]*SPP[0] -(double)P[13][9]*SPP[8] +(double)P[14][9]*SPP[2] +(double)P[15][9]*SPP[5] + (double)dt*((double)P[5][6] +(double)P[0][6]*SF[4] +(double)P[2][6]*SF[3] +(double)P[3][6]*SF[5] -(double)P[1][6]*SPP[0] -(double)P[13][6]*SPP[8] +(double)P[14][6]*SPP[2] +(double)P[15][6]*SPP[5]);
	nextP[6][9] =(double)P[6][9] +(double)P[1][9]*SF[4] -(double)P[2][9]*SF[5] +(double)P[3][9]*SF[3] +(double)P[0][9]*SPP[0] +(double)P[13][9]*SPP[4] -(double)P[14][9]*SPP[7] -(double)P[15][9]*SPP[1] + (double)dt*((double)P[6][6] +(double)P[1][6]*SF[4] -(double)P[2][6]*SF[5] +(double)P[3][6]*SF[3] +(double)P[0][6]*SPP[0] +(double)P[13][6]*SPP[4] -(double)P[14][6]*SPP[7] -(double)P[15][6]*SPP[1]);
	nextP[7][9] =(double)P[7][9] +(double)P[4][9]*(double)dt + (double)dt*((double)P[7][6] +(double)P[4][6]*(double)dt);
	nextP[8][9] =(double)P[8][9] +(double)P[5][9]*(double)dt + (double)dt*((double)P[8][6] +(double)P[5][6]*(double)dt);
	nextP[9][9] =(double)P[9][9] +(double)P[6][9]*(double)dt + (double)dt*((double)P[9][6] +(double)P[6][6]*(double)dt);
	nextP[0][10] =(double)P[0][10] +(double)P[1][10]*SF[9] +(double)P[2][10]*SF[11] +(double)P[3][10]*SF[10] +(double)P[10][10]*SF[14] +(double)P[11][10]*SF[15] +(double)P[12][10]*SPP[10];
	nextP[1][10] =(double)P[1][10] +(double)P[0][10]*SF[8] +(double)P[2][10]*SF[7] +(double)P[3][10]*SF[11] -(double)P[12][10]*SF[15] +(double)P[11][10]*SPP[10] - ((double)P[10][10]*q0)/2;
	nextP[2][10] =(double)P[2][10] +(double)P[0][10]*SF[6] +(double)P[1][10]*SF[10] +(double)P[3][10]*SF[8] +(double)P[12][10]*SF[14] -(double)P[10][10]*SPP[10] - ((double)P[11][10]*q0)/2;
	nextP[3][10] =(double)P[3][10] +(double)P[0][10]*SF[7] +(double)P[1][10]*SF[6] +(double)P[2][10]*SF[9] +(double)P[10][10]*SF[15] -(double)P[11][10]*SF[14] - ((double)P[12][10]*q0)/2;
	nextP[4][10] =(double)P[4][10] +(double)P[0][10]*SF[5] +(double)P[1][10]*SF[3] -(double)P[3][10]*SF[4] +(double)P[2][10]*SPP[0] +(double)P[13][10]*SPP[3] +(double)P[14][10]*SPP[6] -(double)P[15][10]*SPP[9];
	nextP[5][10] =(double)P[5][10] +(double)P[0][10]*SF[4] +(double)P[2][10]*SF[3] +(double)P[3][10]*SF[5] -(double)P[1][10]*SPP[0] -(double)P[13][10]*SPP[8] +(double)P[14][10]*SPP[2] +(double)P[15][10]*SPP[5];
	nextP[6][10] =(double)P[6][10] +(double)P[1][10]*SF[4] -(double)P[2][10]*SF[5] +(double)P[3][10]*SF[3] +(double)P[0][10]*SPP[0] +(double)P[13][10]*SPP[4] -(double)P[14][10]*SPP[7] -(double)P[15][10]*SPP[1];
	nextP[7][10] =(double)P[7][10] +(double)P[4][10]*(double)dt;
	nextP[8][10] =(double)P[8][10] +(double)P[5][10]*(double)dt;
	nextP[9][10] =(double)P[9][10] +(double)P[6][10]*(double)dt;
	nextP[10][10] =(double)P[10][10];
	nextP[0][11] =(double)P[0][11] +(double)P[1][11]*SF[9] +(double)P[2][11]*SF[11] +(double)P[3][11]*SF[10] +(double)P[10][11]*SF[14] +(double)P[11][11]*SF[15] +(double)P[12][11]*SPP[10];
	nextP[1][11] =(double)P[1][11] +(double)P[0][11]*SF[8] +(double)P[2][11]*SF[7] +(double)P[3][11]*SF[11] -(double)P[12][11]*SF[15] +(double)P[11][11]*SPP[10] - ((double)P[10][11]*q0)/2;
	nextP[2][11] =(double)P[2][11] +(double)P[0][11]*SF[6] +(double)P[1][11]*SF[10] +(double)P[3][11]*SF[8] +(double)P[12][11]*SF[14] -(double)P[10][11]*SPP[10] - ((double)P[11][11]*q0)/2;
	nextP[3][11] =(double)P[3][11] +(double)P[0][11]*SF[7] +(double)P[1][11]*SF[6] +(double)P[2][11]*SF[9] +(double)P[10][11]*SF[15] -(double)P[11][11]*SF[14] - ((double)P[12][11]*q0)/2;
	nextP[4][11] =(double)P[4][11] +(double)P[0][11]*SF[5] +(double)P[1][11]*SF[3] -(double)P[3][11]*SF[4] +(double)P[2][11]*SPP[0] +(double)P[13][11]*SPP[3] +(double)P[14][11]*SPP[6] -(double)P[15][11]*SPP[9];
	nextP[5][11] =(double)P[5][11] +(double)P[0][11]*SF[4] +(double)P[2][11]*SF[3] +(double)P[3][11]*SF[5] -(double)P[1][11]*SPP[0] -(double)P[13][11]*SPP[8] +(double)P[14][11]*SPP[2] +(double)P[15][11]*SPP[5];
	nextP[6][11] =(double)P[6][11] +(double)P[1][11]*SF[4] -(double)P[2][11]*SF[5] +(double)P[3][11]*SF[3] +(double)P[0][11]*SPP[0] +(double)P[13][11]*SPP[4] -(double)P[14][11]*SPP[7] -(double)P[15][11]*SPP[1];
	nextP[7][11] =(double)P[7][11] +(double)P[4][11]*(double)dt;
	nextP[8][11] =(double)P[8][11] +(double)P[5][11]*(double)dt;
	nextP[9][11] =(double)P[9][11] +(double)P[6][11]*(double)dt;
	nextP[10][11] =(double)P[10][11];
	nextP[11][11] =(double)P[11][11];
	nextP[0][12] =(double)P[0][12] +(double)P[1][12]*SF[9] +(double)P[2][12]*SF[11] +(double)P[3][12]*SF[10] +(double)P[10][12]*SF[14] +(double)P[11][12]*SF[15] +(double)P[12][12]*SPP[10];
	nextP[1][12] =(double)P[1][12] +(double)P[0][12]*SF[8] +(double)P[2][12]*SF[7] +(double)P[3][12]*SF[11] -(double)P[12][12]*SF[15] +(double)P[11][12]*SPP[10] - ((double)P[10][12]*q0)/2;
	nextP[2][12] =(double)P[2][12] +(double)P[0][12]*SF[6] +(double)P[1][12]*SF[10] +(double)P[3][12]*SF[8] +(double)P[12][12]*SF[14] -(double)P[10][12]*SPP[10] - ((double)P[11][12]*q0)/2;
	nextP[3][12] =(double)P[3][12] +(double)P[0][12]*SF[7] +(double)P[1][12]*SF[6] +(double)P[2][12]*SF[9] +(double)P[10][12]*SF[15] -(double)P[11][12]*SF[14] - ((double)P[12][12]*q0)/2;
	nextP[4][12] =(double)P[4][12] +(double)P[0][12]*SF[5] +(double)P[1][12]*SF[3] -(double)P[3][12]*SF[4] +(double)P[2][12]*SPP[0] +(double)P[13][12]*SPP[3] +(double)P[14][12]*SPP[6] -(double)P[15][12]*SPP[9];
	nextP[5][12] =(double)P[5][12] +(double)P[0][12]*SF[4] +(double)P[2][12]*SF[3] +(double)P[3][12]*SF[5] -(double)P[1][12]*SPP[0] -(double)P[13][12]*SPP[8] +(double)P[14][12]*SPP[2] +(double)P[15][12]*SPP[5];
	nextP[6][12] =(double)P[6][12] +(double)P[1][12]*SF[4] -(double)P[2][12]*SF[5] +(double)P[3][12]*SF[3] +(double)P[0][12]*SPP[0] +(double)P[13][12]*SPP[4] -(double)P[14][12]*SPP[7] -(double)P[15][12]*SPP[1];
	nextP[7][12] =(double)P[7][12] +(double)P[4][12]*(double)dt;
	nextP[8][12] =(double)P[8][12] +(double)P[5][12]*(double)dt;
	nextP[9][12] =(double)P[9][12] +(double)P[6][12]*(double)dt;
	nextP[10][12] =(double)P[10][12];
	nextP[11][12] =(double)P[11][12];
	nextP[12][12] =(double)P[12][12];

	double min_var_increment[_k_num_states] = {};
	for(unsigned i = 0; i<=12; i++) {
		min_var_increment[i] = process_noise[i];
	}

 min_var_increment[0] += (dayVar*dsq(q2))/4 + (dazVar*dsq(q3))/4 + (daxVar*SQ[10])/4;
 min_var_increment[1] += (dayVar*dsq(q3))/4 + (dazVar*dsq(q2))/4 + daxVar*SQ[9];
 min_var_increment[2] += (daxVar*dsq(q3))/4 + dayVar*SQ[9] + (dazVar*SQ[10])/4;
 min_var_increment[3] += (daxVar*dsq(q2))/4 + (dayVar*SQ[10])/4 + dazVar*SQ[9];
 min_var_increment[4] += dvyVar*dsq(SG[7] - 2*q0*q3) + dvzVar*dsq(SG[6] + 2*q0*q2) + dvxVar*dsq(SG[1] + SG[2] - SG[3] - SG[4]);
 min_var_increment[5] += dvxVar*dsq(SG[7] + 2*q0*q3) + dvzVar*dsq(SG[5] - 2*q0*q1) + dvyVar*dsq(SG[1] - SG[2] + SG[3] - SG[4]);
 min_var_increment[6] += dvxVar*dsq(SG[6] - 2*q0*q2) + dvyVar*dsq(SG[5] + 2*q0*q1) + dvzVar*dsq(SG[1] - SG[2] - SG[3] + SG[4]);

	// add process noise that is not from the IMU
	//printf(" (before add 0-9 process noise: %.9f ", (double)nextP[6][6]);
	for (unsigned i = 0; i <= 9; i++) {
		//nextP[i][i] += process_noise[i];
		nextP[i][i] = (double)nextP[i][i] + process_noise[i];
	}
	//printf(" after add 0-9: %.9f) ", (double)nextP[6][6]);
	//printf(" (process noise of %.9f) ", (double)process_noise[6]);

	// process noise contribution for delta angle states can be very small compared to
	// the variances, therefore use algorithm to minimise numerical error
	//printf(" (before main kahan: %.9f ", (double)nextP[6][6]);
		//printf(" g bias p noise: %.9f, dt: %.9f, d_ang_bias_sig: %.9f, sq: %.9f", (double)_params.gyro_bias_p_noise, (double)dt, (double)d_ang_bias_sig, (double)dsq(d_ang_bias_sig));
	for (unsigned i = 10; i <=12; i++) {
		//const int index = i-10;
		//nextP[i][i] = kahanSummation(nextP[i][i], process_noise[i], _delta_angle_bias_var_accum(index));
		//nextP[i][i]+=process_noise[i];
		nextP[i][i] = (double)nextP[i][i] + process_noise[i];

	//	printf(" pn: %e, ", (double)process_noise[i]);
	}
	//printf(" after main kahan: %.9f) ", (double)nextP[6][6]);

	// Don't calculate these covariance terms if IMU delta velocity bias estimation is inhibited
	if (!(_params.fusion_mode & MASK_INHIBIT_ACC_BIAS) && !_accel_bias_inhibit) {
//PX4_WARN("accel bias covariance");
		// calculate variances and upper diagonal covariances for IMU delta velocity bias states
		nextP[0][13] =(double)P[0][13] +(double)P[1][13]*SF[9] +(double)P[2][13]*SF[11] +(double)P[3][13]*SF[10] +(double)P[10][13]*SF[14] +(double)P[11][13]*SF[15] +(double)P[12][13]*SPP[10];
		nextP[1][13] =(double)P[1][13] +(double)P[0][13]*SF[8] +(double)P[2][13]*SF[7] +(double)P[3][13]*SF[11] -(double)P[12][13]*SF[15] +(double)P[11][13]*SPP[10] - ((double)P[10][13]*(double)q0)/2;
		nextP[2][13] =(double)P[2][13] +(double)P[0][13]*SF[6] +(double)P[1][13]*SF[10] +(double)P[3][13]*SF[8] +(double)P[12][13]*SF[14] -(double)P[10][13]*SPP[10] - ((double)P[11][13]*(double)q0)/2;
		nextP[3][13] =(double)P[3][13] +(double)P[0][13]*SF[7] +(double)P[1][13]*SF[6] +(double)P[2][13]*SF[9] +(double)P[10][13]*SF[15] -(double)P[11][13]*SF[14] - ((double)P[12][13]*(double)q0)/2;
		nextP[4][13] =(double)P[4][13] +(double)P[0][13]*SF[5] +(double)P[1][13]*SF[3] -(double)P[3][13]*SF[4] +(double)P[2][13]*SPP[0] +(double)P[13][13]*SPP[3] +(double)P[14][13]*SPP[6] -(double)P[15][13]*SPP[9];
		nextP[5][13] =(double)P[5][13] +(double)P[0][13]*SF[4] +(double)P[2][13]*SF[3] +(double)P[3][13]*SF[5] -(double)P[1][13]*SPP[0] -(double)P[13][13]*SPP[8] +(double)P[14][13]*SPP[2] +(double)P[15][13]*SPP[5];
		nextP[6][13] =(double)P[6][13] +(double)P[1][13]*SF[4] -(double)P[2][13]*SF[5] +(double)P[3][13]*SF[3] +(double)P[0][13]*SPP[0] +(double)P[13][13]*SPP[4] -(double)P[14][13]*SPP[7] -(double)P[15][13]*SPP[1];
		nextP[7][13] =(double)P[7][13] +(double)P[4][13]*(double)dt;
		nextP[8][13] =(double)P[8][13] +(double)P[5][13]*(double)dt;
		nextP[9][13] =(double)P[9][13] +(double)P[6][13]*(double)dt;
		nextP[10][13] =(double)P[10][13];
		nextP[11][13] =(double)P[11][13];
		nextP[12][13] =(double)P[12][13];
		nextP[13][13] =(double)P[13][13];
		nextP[0][14] =(double)P[0][14] +(double)P[1][14]*SF[9] +(double)P[2][14]*SF[11] +(double)P[3][14]*SF[10] +(double)P[10][14]*SF[14] +(double)P[11][14]*SF[15] +(double)P[12][14]*SPP[10];
		nextP[1][14] =(double)P[1][14] +(double)P[0][14]*SF[8] +(double)P[2][14]*SF[7] +(double)P[3][14]*SF[11] -(double)P[12][14]*SF[15] +(double)P[11][14]*SPP[10] - ((double)P[10][14]*(double)q0)/2;
		nextP[2][14] =(double)P[2][14] +(double)P[0][14]*SF[6] +(double)P[1][14]*SF[10] +(double)P[3][14]*SF[8] +(double)P[12][14]*SF[14] -(double)P[10][14]*SPP[10] - ((double)P[11][14]*(double)q0)/2;
		nextP[3][14] =(double)P[3][14] +(double)P[0][14]*SF[7] +(double)P[1][14]*SF[6] +(double)P[2][14]*SF[9] +(double)P[10][14]*SF[15] -(double)P[11][14]*SF[14] - ((double)P[12][14]*(double)q0)/2;
		nextP[4][14] =(double)P[4][14] +(double)P[0][14]*SF[5] +(double)P[1][14]*SF[3] -(double)P[3][14]*SF[4] +(double)P[2][14]*SPP[0] +(double)P[13][14]*SPP[3] +(double)P[14][14]*SPP[6] -(double)P[15][14]*SPP[9];
		nextP[5][14] =(double)P[5][14] +(double)P[0][14]*SF[4] +(double)P[2][14]*SF[3] +(double)P[3][14]*SF[5] -(double)P[1][14]*SPP[0] -(double)P[13][14]*SPP[8] +(double)P[14][14]*SPP[2] +(double)P[15][14]*SPP[5];
		nextP[6][14] =(double)P[6][14] +(double)P[1][14]*SF[4] -(double)P[2][14]*SF[5] +(double)P[3][14]*SF[3] +(double)P[0][14]*SPP[0] +(double)P[13][14]*SPP[4] -(double)P[14][14]*SPP[7] -(double)P[15][14]*SPP[1];
		nextP[7][14] =(double)P[7][14] +(double)P[4][14]*(double)dt;
		nextP[8][14] =(double)P[8][14] +(double)P[5][14]*(double)dt;
		nextP[9][14] =(double)P[9][14] +(double)P[6][14]*(double)dt;
		nextP[10][14] =(double)P[10][14];
		nextP[11][14] =(double)P[11][14];
		nextP[12][14] =(double)P[12][14];
		nextP[13][14] =(double)P[13][14];
		nextP[14][14] =(double)P[14][14];
		nextP[0][15] =(double)P[0][15] +(double)P[1][15]*SF[9] +(double)P[2][15]*SF[11] +(double)P[3][15]*SF[10] +(double)P[10][15]*SF[14] +(double)P[11][15]*SF[15] +(double)P[12][15]*SPP[10];
		nextP[1][15] =(double)P[1][15] +(double)P[0][15]*SF[8] +(double)P[2][15]*SF[7] +(double)P[3][15]*SF[11] -(double)P[12][15]*SF[15] +(double)P[11][15]*SPP[10] - ((double)P[10][15]*(double)q0)/2;
		nextP[2][15] =(double)P[2][15] +(double)P[0][15]*SF[6] +(double)P[1][15]*SF[10] +(double)P[3][15]*SF[8] +(double)P[12][15]*SF[14] -(double)P[10][15]*SPP[10] - ((double)P[11][15]*(double)q0)/2;
		nextP[3][15] =(double)P[3][15] +(double)P[0][15]*SF[7] +(double)P[1][15]*SF[6] +(double)P[2][15]*SF[9] +(double)P[10][15]*SF[15] -(double)P[11][15]*SF[14] - ((double)P[12][15]*(double)q0)/2;
		nextP[4][15] =(double)P[4][15] +(double)P[0][15]*SF[5] +(double)P[1][15]*SF[3] -(double)P[3][15]*SF[4] +(double)P[2][15]*SPP[0] +(double)P[13][15]*SPP[3] +(double)P[14][15]*SPP[6] -(double)P[15][15]*SPP[9];
		nextP[5][15] =(double)P[5][15] +(double)P[0][15]*SF[4] +(double)P[2][15]*SF[3] +(double)P[3][15]*SF[5] -(double)P[1][15]*SPP[0] -(double)P[13][15]*SPP[8] +(double)P[14][15]*SPP[2] +(double)P[15][15]*SPP[5];
		nextP[6][15] =(double)P[6][15] +(double)P[1][15]*SF[4] -(double)P[2][15]*SF[5] +(double)P[3][15]*SF[3] +(double)P[0][15]*SPP[0] +(double)P[13][15]*SPP[4] -(double)P[14][15]*SPP[7] -(double)P[15][15]*SPP[1];
		nextP[7][15] =(double)P[7][15] +(double)P[4][15]*(double)dt;
		nextP[8][15] =(double)P[8][15] +(double)P[5][15]*(double)dt;
		nextP[9][15] =(double)P[9][15] +(double)P[6][15]*(double)dt;
		nextP[10][15] =(double)P[10][15];
		nextP[11][15] =(double)P[11][15];
		nextP[12][15] =(double)P[12][15];
		nextP[13][15] =(double)P[13][15];
		nextP[14][15] =(double)P[14][15];
		nextP[15][15] =(double)P[15][15];

		// add process noise that is not from the IMU
		// process noise contributiton for delta velocity states can be very small compared to
		// the variances, therefore use algorithm to minimise numerical error
		//printf(" (before kahan: %.9f ", (double)nextP[6][6]);
		for (unsigned i = 13; i <= 15; i++) {
			//const int index = i-13;
			//nextP[i][i] = kahanSummation(nextP[i][i], process_noise[i], _delta_vel_bias_var_accum(index));

			//nextP[i][i]+=process_noise[i];
			nextP[i][i] = (double)nextP[i][i] + process_noise[i];
			min_var_increment[i] += process_noise[i];
		}
		//printf(" after kahan: %.9f) ", (double)nextP[6][6]);

	} else {
		// Inhibit delta velocity bias learning by zeroing the covariance terms
		//zeroRows(nextP, 13, 15);
		//zeroCols(nextP, 13, 15);
		uint8_t row;

		for (row = 13; row <= 15; row++) {
			memset(&nextP[row][0], 0, sizeof(nextP[0][0]) * 24);
		}

		for (row = 0; row <= 23; row++) {
			memset(&nextP[row][13], 0, sizeof(nextP[0][0]) * (1 + 15 - 13));
		}


		_delta_vel_bias_var_accum.setZero();
	}

	// Don't do covariance prediction on magnetic field states unless we are using 3-axis fusion
	if (_control_status.flags.mag_3D) { PX4_ERR("3d mag covariance");
		// calculate variances and upper diagonal covariances for earth and body magnetic field states
		nextP[0][16] =(double)P[0][16] +(double)P[1][16]*SF[9] +(double)P[2][16]*SF[11] +(double)P[3][16]*SF[10] +(double)P[10][16]*SF[14] +(double)P[11][16]*SF[15] +(double)P[12][16]*SPP[10];
		nextP[1][16] =(double)P[1][16] +(double)P[0][16]*SF[8] +(double)P[2][16]*SF[7] +(double)P[3][16]*SF[11] -(double)P[12][16]*SF[15] +(double)P[11][16]*SPP[10] - ((double)P[10][16]*(double)q0)/2;
		nextP[2][16] =(double)P[2][16] +(double)P[0][16]*SF[6] +(double)P[1][16]*SF[10] +(double)P[3][16]*SF[8] +(double)P[12][16]*SF[14] -(double)P[10][16]*SPP[10] - ((double)P[11][16]*(double)q0)/2;
		nextP[3][16] =(double)P[3][16] +(double)P[0][16]*SF[7] +(double)P[1][16]*SF[6] +(double)P[2][16]*SF[9] +(double)P[10][16]*SF[15] -(double)P[11][16]*SF[14] - ((double)P[12][16]*(double)q0)/2;
		nextP[4][16] =(double)P[4][16] +(double)P[0][16]*SF[5] +(double)P[1][16]*SF[3] -(double)P[3][16]*SF[4] +(double)P[2][16]*SPP[0] +(double)P[13][16]*SPP[3] +(double)P[14][16]*SPP[6] -(double)P[15][16]*SPP[9];
		nextP[5][16] =(double)P[5][16] +(double)P[0][16]*SF[4] +(double)P[2][16]*SF[3] +(double)P[3][16]*SF[5] -(double)P[1][16]*SPP[0] -(double)P[13][16]*SPP[8] +(double)P[14][16]*SPP[2] +(double)P[15][16]*SPP[5];
		nextP[6][16] =(double)P[6][16] +(double)P[1][16]*SF[4] -(double)P[2][16]*SF[5] +(double)P[3][16]*SF[3] +(double)P[0][16]*SPP[0] +(double)P[13][16]*SPP[4] -(double)P[14][16]*SPP[7] -(double)P[15][16]*SPP[1];
		nextP[7][16] =(double)P[7][16] +(double)P[4][16]*(double)(double)dt;
		nextP[8][16] =(double)P[8][16] +(double)P[5][16]*(double)(double)dt;
		nextP[9][16] =(double)P[9][16] +(double)P[6][16]*(double)(double)dt;
		nextP[10][16] =(double)P[10][16];
		nextP[11][16] =(double)P[11][16];
		nextP[12][16] =(double)P[12][16];
		nextP[13][16] =(double)P[13][16];
		nextP[14][16] =(double)P[14][16];
		nextP[15][16] =(double)P[15][16];
		nextP[16][16] =(double)P[16][16];
		nextP[0][17] =(double)P[0][17] +(double)P[1][17]*SF[9] +(double)P[2][17]*SF[11] +(double)P[3][17]*SF[10] +(double)P[10][17]*SF[14] +(double)P[11][17]*SF[15] +(double)P[12][17]*SPP[10];
		nextP[1][17] =(double)P[1][17] +(double)P[0][17]*SF[8] +(double)P[2][17]*SF[7] +(double)P[3][17]*SF[11] -(double)P[12][17]*SF[15] +(double)P[11][17]*SPP[10] - ((double)P[10][17]*(double)q0)/2;
		nextP[2][17] =(double)P[2][17] +(double)P[0][17]*SF[6] +(double)P[1][17]*SF[10] +(double)P[3][17]*SF[8] +(double)P[12][17]*SF[14] -(double)P[10][17]*SPP[10] - ((double)P[11][17]*(double)q0)/2;
		nextP[3][17] =(double)P[3][17] +(double)P[0][17]*SF[7] +(double)P[1][17]*SF[6] +(double)P[2][17]*SF[9] +(double)P[10][17]*SF[15] -(double)P[11][17]*SF[14] - ((double)P[12][17]*(double)q0)/2;
		nextP[4][17] =(double)P[4][17] +(double)P[0][17]*SF[5] +(double)P[1][17]*SF[3] -(double)P[3][17]*SF[4] +(double)P[2][17]*SPP[0] +(double)P[13][17]*SPP[3] +(double)P[14][17]*SPP[6] -(double)P[15][17]*SPP[9];
		nextP[5][17] =(double)P[5][17] +(double)P[0][17]*SF[4] +(double)P[2][17]*SF[3] +(double)P[3][17]*SF[5] -(double)P[1][17]*SPP[0] -(double)P[13][17]*SPP[8] +(double)P[14][17]*SPP[2] +(double)P[15][17]*SPP[5];
		nextP[6][17] =(double)P[6][17] +(double)P[1][17]*SF[4] -(double)P[2][17]*SF[5] +(double)P[3][17]*SF[3] +(double)P[0][17]*SPP[0] +(double)P[13][17]*SPP[4] -(double)P[14][17]*SPP[7] -(double)P[15][17]*SPP[1];
		nextP[7][17] =(double)P[7][17] +(double)P[4][17]*(double)(double)dt;
		nextP[8][17] =(double)P[8][17] +(double)P[5][17]*(double)(double)dt;
		nextP[9][17] =(double)P[9][17] +(double)P[6][17]*(double)(double)dt;
		nextP[10][17] =(double)P[10][17];
		nextP[11][17] =(double)P[11][17];
		nextP[12][17] =(double)P[12][17];
		nextP[13][17] =(double)P[13][17];
		nextP[14][17] =(double)P[14][17];
		nextP[15][17] =(double)P[15][17];
		nextP[16][17] =(double)P[16][17];
		nextP[17][17] =(double)P[17][17];
		nextP[0][18] =(double)P[0][18] +(double)P[1][18]*SF[9] +(double)P[2][18]*SF[11] +(double)P[3][18]*SF[10] +(double)P[10][18]*SF[14] +(double)P[11][18]*SF[15] +(double)P[12][18]*SPP[10];
		nextP[1][18] =(double)P[1][18] +(double)P[0][18]*SF[8] +(double)P[2][18]*SF[7] +(double)P[3][18]*SF[11] -(double)P[12][18]*SF[15] +(double)P[11][18]*SPP[10] - ((double)P[10][18]*(double)q0)/2;
		nextP[2][18] =(double)P[2][18] +(double)P[0][18]*SF[6] +(double)P[1][18]*SF[10] +(double)P[3][18]*SF[8] +(double)P[12][18]*SF[14] -(double)P[10][18]*SPP[10] - ((double)P[11][18]*(double)q0)/2;
		nextP[3][18] =(double)P[3][18] +(double)P[0][18]*SF[7] +(double)P[1][18]*SF[6] +(double)P[2][18]*SF[9] +(double)P[10][18]*SF[15] -(double)P[11][18]*SF[14] - ((double)P[12][18]*(double)q0)/2;
		nextP[4][18] =(double)P[4][18] +(double)P[0][18]*SF[5] +(double)P[1][18]*SF[3] -(double)P[3][18]*SF[4] +(double)P[2][18]*SPP[0] +(double)P[13][18]*SPP[3] +(double)P[14][18]*SPP[6] -(double)P[15][18]*SPP[9];
		nextP[5][18] =(double)P[5][18] +(double)P[0][18]*SF[4] +(double)P[2][18]*SF[3] +(double)P[3][18]*SF[5] -(double)P[1][18]*SPP[0] -(double)P[13][18]*SPP[8] +(double)P[14][18]*SPP[2] +(double)P[15][18]*SPP[5];
		nextP[6][18] =(double)P[6][18] +(double)P[1][18]*SF[4] -(double)P[2][18]*SF[5] +(double)P[3][18]*SF[3] +(double)P[0][18]*SPP[0] +(double)P[13][18]*SPP[4] -(double)P[14][18]*SPP[7] -(double)P[15][18]*SPP[1];
		nextP[7][18] =(double)P[7][18] +(double)P[4][18]*(double)(double)dt;
		nextP[8][18] =(double)P[8][18] +(double)P[5][18]*(double)(double)dt;
		nextP[9][18] =(double)P[9][18] +(double)P[6][18]*(double)(double)dt;
		nextP[10][18] =(double)P[10][18];
		nextP[11][18] =(double)P[11][18];
		nextP[12][18] =(double)P[12][18];
		nextP[13][18] =(double)P[13][18];
		nextP[14][18] =(double)P[14][18];
		nextP[15][18] =(double)P[15][18];
		nextP[16][18] =(double)P[16][18];
		nextP[17][18] =(double)P[17][18];
		nextP[18][18] =(double)P[18][18];
		nextP[0][19] =(double)P[0][19] +(double)P[1][19]*SF[9] +(double)P[2][19]*SF[11] +(double)P[3][19]*SF[10] +(double)P[10][19]*SF[14] +(double)P[11][19]*SF[15] +(double)P[12][19]*SPP[10];
		nextP[1][19] =(double)P[1][19] +(double)P[0][19]*SF[8] +(double)P[2][19]*SF[7] +(double)P[3][19]*SF[11] -(double)P[12][19]*SF[15] +(double)P[11][19]*SPP[10] - ((double)P[10][19]*(double)q0)/2;
		nextP[2][19] =(double)P[2][19] +(double)P[0][19]*SF[6] +(double)P[1][19]*SF[10] +(double)P[3][19]*SF[8] +(double)P[12][19]*SF[14] -(double)P[10][19]*SPP[10] - ((double)P[11][19]*(double)q0)/2;
		nextP[3][19] =(double)P[3][19] +(double)P[0][19]*SF[7] +(double)P[1][19]*SF[6] +(double)P[2][19]*SF[9] +(double)P[10][19]*SF[15] -(double)P[11][19]*SF[14] - ((double)P[12][19]*(double)q0)/2;
		nextP[4][19] =(double)P[4][19] +(double)P[0][19]*SF[5] +(double)P[1][19]*SF[3] -(double)P[3][19]*SF[4] +(double)P[2][19]*SPP[0] +(double)P[13][19]*SPP[3] +(double)P[14][19]*SPP[6] -(double)P[15][19]*SPP[9];
		nextP[5][19] =(double)P[5][19] +(double)P[0][19]*SF[4] +(double)P[2][19]*SF[3] +(double)P[3][19]*SF[5] -(double)P[1][19]*SPP[0] -(double)P[13][19]*SPP[8] +(double)P[14][19]*SPP[2] +(double)P[15][19]*SPP[5];
		nextP[6][19] =(double)P[6][19] +(double)P[1][19]*SF[4] -(double)P[2][19]*SF[5] +(double)P[3][19]*SF[3] +(double)P[0][19]*SPP[0] +(double)P[13][19]*SPP[4] -(double)P[14][19]*SPP[7] -(double)P[15][19]*SPP[1];
		nextP[7][19] =(double)P[7][19] +(double)P[4][19]*(double)dt;
		nextP[8][19] =(double)P[8][19] +(double)P[5][19]*(double)dt;
		nextP[9][19] =(double)P[9][19] +(double)P[6][19]*(double)dt;
		nextP[10][19] =(double)P[10][19];
		nextP[11][19] =(double)P[11][19];
		nextP[12][19] =(double)P[12][19];
		nextP[13][19] =(double)P[13][19];
		nextP[14][19] =(double)P[14][19];
		nextP[15][19] =(double)P[15][19];
		nextP[16][19] =(double)P[16][19];
		nextP[17][19] =(double)P[17][19];
		nextP[18][19] =(double)P[18][19];
		nextP[19][19] =(double)P[19][19];
		nextP[0][20] =(double)P[0][20] +(double)P[1][20]*SF[9] +(double)P[2][20]*SF[11] +(double)P[3][20]*SF[10] +(double)P[10][20]*SF[14] +(double)P[11][20]*SF[15] +(double)P[12][20]*SPP[10];
		nextP[1][20] =(double)P[1][20] +(double)P[0][20]*SF[8] +(double)P[2][20]*SF[7] +(double)P[3][20]*SF[11] -(double)P[12][20]*SF[15] +(double)P[11][20]*SPP[10] - ((double)P[10][20]*(double)q0)/2;
		nextP[2][20] =(double)P[2][20] +(double)P[0][20]*SF[6] +(double)P[1][20]*SF[10] +(double)P[3][20]*SF[8] +(double)P[12][20]*SF[14] -(double)P[10][20]*SPP[10] - ((double)P[11][20]*(double)q0)/2;
		nextP[3][20] =(double)P[3][20] +(double)P[0][20]*SF[7] +(double)P[1][20]*SF[6] +(double)P[2][20]*SF[9] +(double)P[10][20]*SF[15] -(double)P[11][20]*SF[14] - ((double)P[12][20]*(double)q0)/2;
		nextP[4][20] =(double)P[4][20] +(double)P[0][20]*SF[5] +(double)P[1][20]*SF[3] -(double)P[3][20]*SF[4] +(double)P[2][20]*SPP[0] +(double)P[13][20]*SPP[3] +(double)P[14][20]*SPP[6] -(double)P[15][20]*SPP[9];
		nextP[5][20] =(double)P[5][20] +(double)P[0][20]*SF[4] +(double)P[2][20]*SF[3] +(double)P[3][20]*SF[5] -(double)P[1][20]*SPP[0] -(double)P[13][20]*SPP[8] +(double)P[14][20]*SPP[2] +(double)P[15][20]*SPP[5];
		nextP[6][20] =(double)P[6][20] +(double)P[1][20]*SF[4] -(double)P[2][20]*SF[5] +(double)P[3][20]*SF[3] +(double)P[0][20]*SPP[0] +(double)P[13][20]*SPP[4] -(double)P[14][20]*SPP[7] -(double)P[15][20]*SPP[1];
		nextP[7][20] =(double)P[7][20] +(double)P[4][20]*(double)dt;
		nextP[8][20] =(double)P[8][20] +(double)P[5][20]*(double)dt;
		nextP[9][20] =(double)P[9][20] +(double)P[6][20]*(double)dt;
		nextP[10][20] =(double)P[10][20];
		nextP[11][20] =(double)P[11][20];
		nextP[12][20] =(double)P[12][20];
		nextP[13][20] =(double)P[13][20];
		nextP[14][20] =(double)P[14][20];
		nextP[15][20] =(double)P[15][20];
		nextP[16][20] =(double)P[16][20];
		nextP[17][20] =(double)P[17][20];
		nextP[18][20] =(double)P[18][20];
		nextP[19][20] =(double)P[19][20];
		nextP[20][20] =(double)P[20][20];
		nextP[0][21] =(double)P[0][21] +(double)P[1][21]*SF[9] +(double)P[2][21]*SF[11] +(double)P[3][21]*SF[10] +(double)P[10][21]*SF[14] +(double)P[11][21]*SF[15] +(double)P[12][21]*SPP[10];
		nextP[1][21] =(double)P[1][21] +(double)P[0][21]*SF[8] +(double)P[2][21]*SF[7] +(double)P[3][21]*SF[11] -(double)P[12][21]*SF[15] +(double)P[11][21]*SPP[10] - ((double)P[10][21]*(double)q0)/2;
		nextP[2][21] =(double)P[2][21] +(double)P[0][21]*SF[6] +(double)P[1][21]*SF[10] +(double)P[3][21]*SF[8] +(double)P[12][21]*SF[14] -(double)P[10][21]*SPP[10] - ((double)P[11][21]*(double)q0)/2;
		nextP[3][21] =(double)P[3][21] +(double)P[0][21]*SF[7] +(double)P[1][21]*SF[6] +(double)P[2][21]*SF[9] +(double)P[10][21]*SF[15] -(double)P[11][21]*SF[14] - ((double)P[12][21]*(double)q0)/2;
		nextP[4][21] =(double)P[4][21] +(double)P[0][21]*SF[5] +(double)P[1][21]*SF[3] -(double)P[3][21]*SF[4] +(double)P[2][21]*SPP[0] +(double)P[13][21]*SPP[3] +(double)P[14][21]*SPP[6] -(double)P[15][21]*SPP[9];
		nextP[5][21] =(double)P[5][21] +(double)P[0][21]*SF[4] +(double)P[2][21]*SF[3] +(double)P[3][21]*SF[5] -(double)P[1][21]*SPP[0] -(double)P[13][21]*SPP[8] +(double)P[14][21]*SPP[2] +(double)P[15][21]*SPP[5];
		nextP[6][21] =(double)P[6][21] +(double)P[1][21]*SF[4] -(double)P[2][21]*SF[5] +(double)P[3][21]*SF[3] +(double)P[0][21]*SPP[0] +(double)P[13][21]*SPP[4] -(double)P[14][21]*SPP[7] -(double)P[15][21]*SPP[1];
		nextP[7][21] =(double)P[7][21] +(double)P[4][21]*(double)dt;
		nextP[8][21] =(double)P[8][21] +(double)P[5][21]*(double)dt;
		nextP[9][21] =(double)P[9][21] +(double)P[6][21]*(double)dt;
		nextP[10][21] =(double)P[10][21];
		nextP[11][21] =(double)P[11][21];
		nextP[12][21] =(double)P[12][21];
		nextP[13][21] =(double)P[13][21];
		nextP[14][21] =(double)P[14][21];
		nextP[15][21] =(double)P[15][21];
		nextP[16][21] =(double)P[16][21];
		nextP[17][21] =(double)P[17][21];
		nextP[18][21] =(double)P[18][21];
		nextP[19][21] =(double)P[19][21];
		nextP[20][21] =(double)P[20][21];
		nextP[21][21] =(double)P[21][21];

		// add process noise that is not from the IMU
		for (unsigned i = 16; i <= 21; i++) {
			//nextP[i][i] += process_noise[i];
			nextP[i][i] = (double)nextP[i][i] + process_noise[i];
			min_var_increment[i] += process_noise[i];
		}

	}

	// Don't do covariance prediction on wind states unless we are using them
	if (_control_status.flags.wind) {
PX4_ERR("wind");
		// calculate variances and upper diagonal covariances for wind states
		nextP[0][22] =(double)P[0][22] +(double)P[1][22]*SF[9] +(double)P[2][22]*SF[11] +(double)P[3][22]*SF[10] +(double)P[10][22]*SF[14] +(double)P[11][22]*SF[15] +(double)P[12][22]*SPP[10];
		nextP[1][22] =(double)P[1][22] +(double)P[0][22]*SF[8] +(double)P[2][22]*SF[7] +(double)P[3][22]*SF[11] -(double)P[12][22]*SF[15] +(double)P[11][22]*SPP[10] - ((double)P[10][22]*(double)q0)/2;
		nextP[2][22] =(double)P[2][22] +(double)P[0][22]*SF[6] +(double)P[1][22]*SF[10] +(double)P[3][22]*SF[8] +(double)P[12][22]*SF[14] -(double)P[10][22]*SPP[10] - ((double)P[11][22]*(double)q0)/2;
		nextP[3][22] =(double)P[3][22] +(double)P[0][22]*SF[7] +(double)P[1][22]*SF[6] +(double)P[2][22]*SF[9] +(double)P[10][22]*SF[15] -(double)P[11][22]*SF[14] - ((double)P[12][22]*(double)q0)/2;
		nextP[4][22] =(double)P[4][22] +(double)P[0][22]*SF[5] +(double)P[1][22]*SF[3] -(double)P[3][22]*SF[4] +(double)P[2][22]*SPP[0] +(double)P[13][22]*SPP[3] +(double)P[14][22]*SPP[6] -(double)P[15][22]*SPP[9];
		nextP[5][22] =(double)P[5][22] +(double)P[0][22]*SF[4] +(double)P[2][22]*SF[3] +(double)P[3][22]*SF[5] -(double)P[1][22]*SPP[0] -(double)P[13][22]*SPP[8] +(double)P[14][22]*SPP[2] +(double)P[15][22]*SPP[5];
		nextP[6][22] =(double)P[6][22] +(double)P[1][22]*SF[4] -(double)P[2][22]*SF[5] +(double)P[3][22]*SF[3] +(double)P[0][22]*SPP[0] +(double)P[13][22]*SPP[4] -(double)P[14][22]*SPP[7] -(double)P[15][22]*SPP[1];
		nextP[7][22] =(double)P[7][22] +(double)P[4][22]*(double)dt;
		nextP[8][22] =(double)P[8][22] +(double)P[5][22]*(double)dt;
		nextP[9][22] =(double)P[9][22] +(double)P[6][22]*(double)dt;
		nextP[10][22] =(double)P[10][22];
		nextP[11][22] =(double)P[11][22];
		nextP[12][22] =(double)P[12][22];
		nextP[13][22] =(double)P[13][22];
		nextP[14][22] =(double)P[14][22];
		nextP[15][22] =(double)P[15][22];
		nextP[16][22] =(double)P[16][22];
		nextP[17][22] =(double)P[17][22];
		nextP[18][22] =(double)P[18][22];
		nextP[19][22] =(double)P[19][22];
		nextP[20][22] =(double)P[20][22];
		nextP[21][22] =(double)P[21][22];
		nextP[22][22] =(double)P[22][22];
		nextP[0][23] =(double)P[0][23] +(double)P[1][23]*SF[9] +(double)P[2][23]*SF[11] +(double)P[3][23]*SF[10] +(double)P[10][23]*SF[14] +(double)P[11][23]*SF[15] +(double)P[12][23]*SPP[10];
		nextP[1][23] =(double)P[1][23] +(double)P[0][23]*SF[8] +(double)P[2][23]*SF[7] +(double)P[3][23]*SF[11] -(double)P[12][23]*SF[15] +(double)P[11][23]*SPP[10] - ((double)P[10][23]*(double)q0)/2;
		nextP[2][23] =(double)P[2][23] +(double)P[0][23]*SF[6] +(double)P[1][23]*SF[10] +(double)P[3][23]*SF[8] +(double)P[12][23]*SF[14] -(double)P[10][23]*SPP[10] - ((double)P[11][23]*(double)q0)/2;
		nextP[3][23] =(double)P[3][23] +(double)P[0][23]*SF[7] +(double)P[1][23]*SF[6] +(double)P[2][23]*SF[9] +(double)P[10][23]*SF[15] -(double)P[11][23]*SF[14] - ((double)P[12][23]*(double)q0)/2;
		nextP[4][23] =(double)P[4][23] +(double)P[0][23]*SF[5] +(double)P[1][23]*SF[3] -(double)P[3][23]*SF[4] +(double)P[2][23]*SPP[0] +(double)P[13][23]*SPP[3] +(double)P[14][23]*SPP[6] -(double)P[15][23]*SPP[9];
		nextP[5][23] =(double)P[5][23] +(double)P[0][23]*SF[4] +(double)P[2][23]*SF[3] +(double)P[3][23]*SF[5] -(double)P[1][23]*SPP[0] -(double)P[13][23]*SPP[8] +(double)P[14][23]*SPP[2] +(double)P[15][23]*SPP[5];
		nextP[6][23] =(double)P[6][23] +(double)P[1][23]*SF[4] -(double)P[2][23]*SF[5] +(double)P[3][23]*SF[3] +(double)P[0][23]*SPP[0] +(double)P[13][23]*SPP[4] -(double)P[14][23]*SPP[7] -(double)P[15][23]*SPP[1];
		nextP[7][23] =(double)P[7][23] +(double)P[4][23]*(double)dt;
		nextP[8][23] =(double)P[8][23] +(double)P[5][23]*(double)dt;
		nextP[9][23] =(double)P[9][23] +(double)P[6][23]*(double)dt;
		nextP[10][23] =(double)P[10][23];
		nextP[11][23] =(double)P[11][23];
		nextP[12][23] =(double)P[12][23];
		nextP[13][23] =(double)P[13][23];
		nextP[14][23] =(double)P[14][23];
		nextP[15][23] =(double)P[15][23];
		nextP[16][23] =(double)P[16][23];
		nextP[17][23] =(double)P[17][23];
		nextP[18][23] =(double)P[18][23];
		nextP[19][23] =(double)P[19][23];
		nextP[20][23] =(double)P[20][23];
		nextP[21][23] =(double)P[21][23];
		nextP[22][23] =(double)P[22][23];
		nextP[23][23] =(double)P[23][23];

		// add process noise that is not from the IMU
		for (unsigned i = 22; i <= 23; i++) {
			//nextP[i][i] += process_noise[i];
			nextP[i][i] = (double)nextP[i][i] + process_noise[i];
			min_var_increment[i] += process_noise[i];
		}

	}

	// stop position covariance growth if our total position variance reaches 100m
	// this can happen if we lose gps for some time
	if ((P[7][7] +P[8][8]) > 1e4f) { PX4_ERR(" stop position covariance growth ");
		for (uint8_t i = 7; i <= 8; i++) {
			for (uint8_t j = 0; j < _k_num_states; j++) {
				nextP[i][j] =P[i][j];
				nextP[j][i] =P[j][i];
			}
		}
	}

	// covariance matrix is symmetrical, so copy upper half to lower half
	for (unsigned row = 1; row < _k_num_states; row++) {
		for (unsigned column = 0 ; column < row; column++) {
			P[row][column] = P[column][row] = (float)nextP[column][row];
		}
	}

	// copy variances (diagonals)
	for (unsigned i = 0; i < _k_num_states; i++) {
		//if(i<=12) {
		/*if(nextP[i][i]>P[i][i] + min_var_increment[i]){
			//if(nextP[i][i]>P[i][i]) {
				P[i][i] = nextP[i][i];
			//}
		}
		else {
				P[i][i]=P[i][i] + min_var_increment[i];
			}*/
		//} else {(double)P[i][i]=nextP[i][i];}
		P[i][i] = (float)nextP[i][i];

	} //P[6][6]-=0.0000001f;
	// fix gross errors in the covariance matrix and ensure rows and
	// columns for un-used states are zero
	//printf(" (before fix: %.9f ", (double)P[9][9]);
	fixCovarianceErrors();
	//printf(" after fix: %.9f) ", (double)P[9][9]);

}

void Ekf::fixCovarianceErrors()
{
	// NOTE: This limiting is a last resort and should not be relied on
	// TODO: Split covariance prediction into separate F*P*transpose(F) and Q contributions
	// and set corresponding entries in Q to zero when states exceed 50% of the limit
	// Covariance diagonal limits. Use same values for states which
	// belong to the same group (e.g. vel_x, vel_y, vel_z)
	float P_lim[8] = {};
	P_lim[0] = 1.0f;		// quaternion max var
	P_lim[1] = 1e6f;		// velocity max var
	P_lim[2] = 1e6f;		// positiion max var
	P_lim[3] = 1.0f;		// gyro bias max var
	P_lim[4] = 1.0f;		// delta velocity z bias max var
	P_lim[5] = 1.0f;		// earth mag field max var
	P_lim[6] = 1.0f;		// body mag field max var
	P_lim[7] = 1e6f;		// wind max var

	for (int i = 0; i <= 3; i++) {
		// quaternion states
		P[i][i] = math::constrain(P[i][i], 0.0f, P_lim[0]);
	}

	for (int i = 4; i <= 6; i++) {
		// NED velocity states
		P[i][i] = math::constrain(P[i][i], 0.0f, P_lim[1]);
	}

	for (int i = 7; i <= 9; i++) {
		// NED position states
		P[i][i] = math::constrain(P[i][i], 0.0f, P_lim[2]);
	}

	for (int i = 10; i <= 12; i++) {
		// gyro bias states
		P[i][i] = math::constrain(P[i][i], 0.0f, P_lim[3]);
	}

	// force symmetry on the quaternion, velocity and position state covariances
	makeSymmetrical(P, 0, 12);

	// the following states are optional and are deactivated when not required
	// by ensuring the corresponding covariance matrix values are kept at zero

	// accelerometer bias states
	if ((_params.fusion_mode & MASK_INHIBIT_ACC_BIAS) || _accel_bias_inhibit) {
		zeroRows(P, 13, 15);
		zeroCols(P, 13, 15);

	} else {
		// Find the maximum delta velocity bias state variance and request a covariance reset if any variance is below the safe minimum
		const float minSafeStateVar = 1e-9f;
		float maxStateVar = minSafeStateVar;
		bool resetRequired = false;

		for (uint8_t stateIndex = 13; stateIndex <= 15; stateIndex++) {
			if (P[stateIndex][stateIndex] > maxStateVar) {
				maxStateVar =(double)P[stateIndex][stateIndex];

			} else if (P[stateIndex][stateIndex] < minSafeStateVar) {
				resetRequired = true;
			}
		}

		// To ensure stability of the covariance matrix operations, the ratio of a max and min variance must
		// not exceed 100 and the minimum variance must not fall below the target minimum
		// Also limit variance to a maximum equivalent to a 0.1g uncertainty
		const float minStateVarTarget = 5E-8f;
		float minAllowedStateVar = fmaxf(0.01f * maxStateVar, minStateVarTarget);

		for (uint8_t stateIndex = 13; stateIndex <= 15; stateIndex++) {
			P[stateIndex][stateIndex] = math::constrain(P[stateIndex][stateIndex], minAllowedStateVar, sq(0.1f * CONSTANTS_ONE_G * _dt_ekf_avg));
		}

		// If any one axis has fallen below the safe minimum, all delta velocity covariance terms must be reset to zero
		if (resetRequired) {
			float delVelBiasVar[3];

			// store all delta velocity bias variances
			for (uint8_t stateIndex = 13; stateIndex <= 15; stateIndex++) {
				delVelBiasVar[stateIndex - 13] =(double)P[stateIndex][stateIndex];
			}

			// reset all delta velocity bias covariances
			zeroCols(P, 13, 15);

			// restore all delta velocity bias variances
			for (uint8_t stateIndex = 13; stateIndex <= 15; stateIndex++) {
				P[stateIndex][stateIndex] = delVelBiasVar[stateIndex - 13];
			}
		}

		// Run additional checks to see if the delta velocity bias has hit limits in a direction that is clearly wrong
		// calculate accel bias term aligned with the gravity vector
		float dVel_bias_lim = 0.9f * _params.acc_bias_lim * _dt_ekf_avg;
		float down_dvel_bias = 0.0f;

		for (uint8_t axis_index = 0; axis_index < 3; axis_index++) {
			down_dvel_bias += _state.accel_bias(axis_index) * _R_to_earth(2, axis_index);
		}

		// check that the vertical component of accel bias is consistent with both the vertical position and velocity innovation
		bool bad_acc_bias = (fabsf(down_dvel_bias) > dVel_bias_lim
				     && down_dvel_bias * _vel_pos_innov[2] < 0.0f
				     && down_dvel_bias * _vel_pos_innov[5] < 0.0f);

		// record the pass/fail
		if (!bad_acc_bias) {
			_fault_status.flags.bad_acc_bias = false;
			_time_acc_bias_check = _time_last_imu;

		} else {
			_fault_status.flags.bad_acc_bias = true;
		}

		// if we have failed for 7 seconds continuously, reset the accel bias covariances to fix bad conditioning of
		// the covariance matrix but preserve the variances (diagonals) to allow bias learning to continue
		if (_time_last_imu - _time_acc_bias_check > (uint64_t)7e6) {
			float varX =P[13][13];
			float varY =P[14][14];
			float varZ =P[15][15];
			zeroRows(P, 13, 15);
			zeroCols(P, 13, 15);
			P[13][13] = varX;
			P[14][14] = varY;
			P[15][15] = varZ;
			_time_acc_bias_check = _time_last_imu;
			_fault_status.flags.bad_acc_bias = false;
			ECL_WARN_TIMESTAMPED("EKF invalid accel bias - resetting covariance");

		} else {
			// ensure the covariance values are symmetrical
			makeSymmetrical(P, 13, 15);
		}

	}

	// magnetic field states
	if (!_control_status.flags.mag_3D) {
		zeroRows(P, 16, 21);
		zeroCols(P, 16, 21);

	} else {
		// constrain variances
		for (int i = 16; i <= 18; i++) {
			P[i][i] = math::constrain(P[i][i], 0.0f, P_lim[5]);
		}

		for (int i = 19; i <= 21; i++) {
			P[i][i] = math::constrain(P[i][i], 0.0f, P_lim[6]);
		}

		// force symmetry
		makeSymmetrical(P, 16, 21);
	}

	// wind velocity states
	if (!_control_status.flags.wind) {
		zeroRows(P, 22, 23);
		zeroCols(P, 22, 23);

	} else {
		// constrain variances
		for (int i = 22; i <= 23; i++) {
			P[i][i] = math::constrain(P[i][i], 0.0f, P_lim[7]);
		}

		// force symmetry
		makeSymmetrical(P, 22, 23);
	}
}

void Ekf::resetMagCovariance()
{
	// set the quaternion covariance terms to zero
	zeroRows(P, 0, 3);
	zeroCols(P, 0, 3);

	// set the magnetic field covariance terms to zero
	zeroRows(P, 16, 21);
	zeroCols(P, 16, 21);
	_mag_decl_cov_reset = false;

	// set the field state variance to the observation variance
	for (uint8_t rc_index = 16; rc_index <= 21; rc_index ++) {
		P[rc_index][rc_index] = sq(_params.mag_noise);
	}

	// save covariance data for re-use when auto-switching between heading and 3-axis fusion
	save_mag_cov_data();
}

void Ekf::resetWindCovariance()
{
	// set the wind  covariance terms to zero
	zeroRows(P, 22, 23);
	zeroCols(P, 22, 23);

	if (_tas_data_ready && (_imu_sample_delayed.time_us - _airspeed_sample_delayed.time_us < (uint64_t)5e5)) {
		// Use airspeed and zer sideslip assumption to set initial covariance values for wind states

		// calculate the wind speed and bearing
		float spd = sqrtf(sq(_state.wind_vel(0)) + sq(_state.wind_vel(1)));
		float yaw = atan2f(_state.wind_vel(1), _state.wind_vel(0));

		// calculate the uncertainty in wind speed and direction using the uncertainty in airspeed and sideslip angle
		// used to calculate the initial wind speed
		float R_spd = sq(math::constrain(_params.eas_noise, 0.5f, 5.0f) * math::constrain(_airspeed_sample_delayed.eas2tas, 0.9f, 10.0f));
		float R_yaw = sq(math::radians(10.0f));

		// calculate the variance and covariance terms for the wind states
		float cos_yaw = cosf(yaw);
		float sin_yaw = sinf(yaw);
		float cos_yaw_2 = sq(cos_yaw);
		float sin_yaw_2 = sq(sin_yaw);
		float sin_cos_yaw = sin_yaw * cos_yaw;
		float spd_2 = sq(spd);
		P[22][22] = R_yaw * spd_2 * sin_yaw_2 + R_spd * cos_yaw_2;
		P[22][23] = - R_yaw * sin_cos_yaw * spd_2 + R_spd * sin_cos_yaw;
		P[23][22] =P[22][23];
		P[23][23] = R_yaw * spd_2 * cos_yaw_2 + R_spd * sin_yaw_2;

		// Now add the variance due to uncertainty in vehicle velocity that was used to calculate the initial wind speed
		P[22][22] +=P[4][4];
		P[23][23] +=P[5][5];

	} else {
		// without airspeed, start with a small initial uncertainty to improve the initial estimate
		P[22][22] = sq(1.0f);
		P[23][23] = sq(1.0f);

	}
}
