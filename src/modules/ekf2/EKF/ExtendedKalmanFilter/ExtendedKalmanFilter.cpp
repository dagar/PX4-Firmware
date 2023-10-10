/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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

#include "ExtendedKalmanFilter.hpp"

#include <lib/geo/geo.h>

bool ExtendedKalmanFilter::init(const Vector3f &accel)
{
	const float accel_norm = accel.norm();

	if (accel_norm < 0.5f * CONSTANTS_ONE_G ||
	    accel_norm > 1.5f * CONSTANTS_ONE_G) {

		return false;
	}

	reset();

	// get initial tilt estimate from delta velocity vector, assuming vehicle is static
	_state.quat_nominal = Quatf(accel, Vector3f(0.f, 0.f, -1.f));

	_filter_initialised = true;

	return true;
}

void ExtendedKalmanFilter::reset()
{
	_filter_initialised = false;

	// orientation
	_state.quat_nominal.setIdentity();
	_time_last_yaw_fuse = 0;

	// velocity
	_state.vel.setZero();
	_time_last_vel_xy_fuse = 0;
	_time_last_vel_z_fuse = 0;

	// position
	_state.pos.setZero();
	_time_last_pos_xy_fuse = 0;
	_time_last_pos_z_fuse = 0;

	// gyro bias
	_state.gyro_bias.setZero();
	_prev_gyro_bias_var.zero();
	_gyro_bias_inhibit[0] = false;
	_gyro_bias_inhibit[1] = false;
	_gyro_bias_inhibit[2] = false;

	// accel bias
	_state.accel_bias.setZero();
	_prev_accel_bias_var.zero();
	_accel_bias_inhibit[0] = false;
	_accel_bias_inhibit[1] = false;
	_accel_bias_inhibit[2] = false;

#if defined(CONFIG_EKF2_MAGNETOMETER)
	// mag_I, mag_B
	_state.mag_I.setZero();
	_state.mag_B.setZero();
#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_WIND)
	// wind velocity
	_state.wind_vel.setZero();
#endif // CONFIG_EKF2_WIND

	_dt_ekf_avg = 0.01f;

	_earth_rate_NED.zero();

	_height_rate_lpf = 0.f;

	_state_reset_status = {};
	_state_reset_count_prev = {};

	initialiseCovariance();
}

void ExtendedKalmanFilter::initialiseCovariance()
{
	P.zero();

	// orientation
	float tilt_var = sq(math::max(_params.initial_tilt_err, 0.01f));
	float yaw_var = sq(math::max(_params.initial_yaw_uncertainty, 0.01f));
	resetQuatCov(Vector3f(tilt_var, tilt_var, yaw_var));

	// velocity
	const float vel_var = sq(math::max(_params.initial_vel_uncertainty, 0.01f));
	P.uncorrelateCovarianceSetVariance<State::vel.dof>(State::vel.idx, Vector3f(vel_var, vel_var, vel_var));

	// position
	const float pos_var = sq(math::max(_params.initial_pos_uncertainty, 0.01f));
	P.uncorrelateCovarianceSetVariance<State::pos.dof>(State::pos.idx, Vector3f(pos_var, pos_var, pos_var));

	resetGyroBiasCov();

	resetAccelBiasCov();

#if defined(CONFIG_EKF2_MAGNETOMETER)
	resetMagCov();
#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_WIND)
	resetWindCov();
#endif // CONFIG_EKF2_WIND
}

bool ExtendedKalmanFilter::update(const imuSample &imu)
{
	_time_delayed_us = imu.time_us;

	// calculate an average filter update time
	//  filter and limit input between -50% and +100% of nominal value
	float input = 0.5f * (imu.delta_vel_dt + imu.delta_ang_dt);
	_dt_ekf_avg = 0.99f * _dt_ekf_avg + 0.01f * input;

	if (!_filter_initialised) {
		if (init(imu.delta_vel / imu.delta_vel_dt)) {
			return true;
		}

		if (!_filter_initialised) {
			return false;
		}
	}

	// perform state and covariance prediction for the main filter
	predictCovariance(imu);
	predictState(imu);

	return true;
}

void ExtendedKalmanFilter::constrainStateVar(const IdxDof &state, float min, float max)
{
	for (unsigned i = state.idx; i < (state.idx + state.dof); i++) {
		P(i, i) = math::constrain(P(i, i), min, max);
	}
}

void ExtendedKalmanFilter::fixCovarianceErrors(bool force_symmetry)
{
	// NOTE: This limiting is a last resort and should not be relied on
	// TODO: Split covariance prediction into separate F*P*transpose(F) and Q contributions
	// and set corresponding entries in Q to zero when states exceed 50% of the limit
	// Covariance diagonal limits. Use same values for states which
	// belong to the same group (e.g. vel_x, vel_y, vel_z)

	constrainStateVar(State::quat_nominal, 0.f, 1.f);
	constrainStateVar(State::vel, 1e-6f, 1e6);
	constrainStateVar(State::pos, 1e-6f, 1e6);
	constrainStateVar(State::gyro_bias, 0.f, 1.f);
	constrainStateVar(State::accel_bias, 0.f, 1.f);

	// the following states are optional and are deactivated when not required
	// by ensuring the corresponding covariance matrix values are kept at zero

#if defined(CONFIG_EKF2_MAGNETOMETER)

	// magnetic field states
	if (!_control_flags.mag) {
		P.uncorrelateCovarianceSetVariance<State::mag_I.dof>(State::mag_I.idx, 0.f);
		P.uncorrelateCovarianceSetVariance<State::mag_B.dof>(State::mag_B.idx, 0.f);

	} else {
		constrainStateVar(State::mag_I, 0.f, 1.f);
		constrainStateVar(State::mag_B, 0.f, 1.f);

		if (force_symmetry) {
			P.makeRowColSymmetric<State::mag_I.dof>(State::mag_I.idx);
			P.makeRowColSymmetric<State::mag_B.dof>(State::mag_B.idx);
		}
	}

#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_WIND)

	// wind velocity states
	if (!_control_flags.wind) {
		P.uncorrelateCovarianceSetVariance<State::wind_vel.dof>(State::wind_vel.idx, 0.f);

	} else {
		constrainStateVar(State::wind_vel, 0.f, 1e6f);

		if (force_symmetry) {
			P.makeRowColSymmetric<State::wind_vel.dof>(State::wind_vel.idx);
		}
	}

#endif // CONFIG_EKF2_WIND

	// force symmetry on the quaternion, velocity and position state covariances
	if (force_symmetry) {
		P.makeRowColSymmetric<State::quat_nominal.dof>(State::quat_nominal.idx);
		P.makeRowColSymmetric<State::vel.dof>(State::vel.idx);
		P.makeRowColSymmetric<State::pos.dof>(State::pos.idx);
		P.makeRowColSymmetric<State::gyro_bias.dof>(State::gyro_bias.idx); //TODO: needed?
		P.makeRowColSymmetric<State::accel_bias.dof>(State::accel_bias.idx); //TODO: needed?
	}
}
