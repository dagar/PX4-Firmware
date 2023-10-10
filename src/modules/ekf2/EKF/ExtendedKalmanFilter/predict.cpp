#include "ExtendedKalmanFilter.hpp"

#include "derivation/generated/predict_covariance.h"

#include <lib/geo/geo.h>

static constexpr float BADACC_BIAS_PNOISE{4.9f}; ///< The delta velocity process noise is set to this when accel data is declared bad (m/sec**2)

void ExtendedKalmanFilter::predictCovariance(const imuSample &imu)
{
	// Use average update interval to reduce accumulated covariance prediction errors due to small single frame dt values
	const float dt = _dt_ekf_avg;

	// delta angle noise variance
	float gyro_noise = math::constrain(_params.gyro_noise, 0.f, 1.f);
	const float d_ang_var = sq(imu.delta_ang_dt * gyro_noise);

	// delta velocity noise variance
	float accel_noise = math::constrain(_params.accel_noise, 0.f, 1.f);
	Vector3f d_vel_var;

	for (unsigned i = 0; i < 3; i++) {
		if (_control_flags.bad_acc_vertical || imu.delta_vel_clipping[i]) {
			// Increase accelerometer process noise if bad accel data is detected
			d_vel_var(i) = sq(imu.delta_vel_dt * BADACC_BIAS_PNOISE);

		} else {
			d_vel_var(i) = sq(imu.delta_vel_dt * accel_noise);
		}
	}

	// predict the covariance
	SquareMatrixState nextP;

	// calculate variances and upper diagonal covariances for quaternion, velocity, position and gyro bias states
	sym::PredictCovariance(_state.vector(), P,
			       imu.delta_vel, imu.delta_vel_dt, d_vel_var,
			       imu.delta_ang, imu.delta_ang_dt, d_ang_var,
			       &nextP);

	// Construct the process noise variance diagonal for those states with a stationary process model
	// These are kinematic states and their error growth is controlled separately by the IMU noise variances

	// gyro bias: add process noise, or restore previous gyro bias var if state inhibited
	const float gyro_bias_sig = dt * math::constrain(_params.gyro_bias_p_noise, 0.f, 1.f);
	const float gyro_bias_process_noise = sq(gyro_bias_sig);

	for (unsigned index = 0; index < State::gyro_bias.dof; index++) {
		const unsigned i = State::gyro_bias.idx + index;

		if (!_gyro_bias_inhibit[index]) {
			nextP(i, i) += gyro_bias_process_noise;

		} else {
			nextP.uncorrelateCovarianceSetVariance<1>(i, _prev_gyro_bias_var(index));
		}
	}

	// accel bias: add process noise, or restore previous accel bias var if state inhibited
	const float accel_bias_sig = dt * math::constrain(_params.accel_bias_p_noise, 0.f, 1.f);
	const float accel_bias_process_noise = sq(accel_bias_sig);

	for (unsigned index = 0; index < State::accel_bias.dof; index++) {
		const unsigned i = State::accel_bias.idx + index;

		if (!_accel_bias_inhibit[index]) {
			nextP(i, i) += accel_bias_process_noise;

		} else {
			nextP.uncorrelateCovarianceSetVariance<1>(i, _prev_accel_bias_var(index));
		}
	}

#if defined(CONFIG_EKF2_MAGNETOMETER)

	if (_control_flags.mag) {
		// Don't continue to grow the earth field variances if they are becoming too large or we are not doing 3-axis fusion as this can make the covariance matrix badly conditioned
		if (P.trace<State::mag_I.dof>(State::mag_I.idx) < 0.1f) {

			float mag_I_sig = dt * math::constrain(_params.mage_p_noise, 0.f, 1.f);
			float mag_I_process_noise = sq(mag_I_sig);

			for (unsigned index = 0; index < State::mag_I.dof; index++) {
				unsigned i = State::mag_I.idx + index;
				nextP(i, i) += mag_I_process_noise;
			}
		}

		// Don't continue to grow the body field variances if they are becoming too large or we are not doing 3-axis fusion as this can make the covariance matrix badly conditioned
		if (P.trace<State::mag_B.dof>(State::mag_B.idx) < 0.1f) {

			float mag_B_sig = dt * math::constrain(_params.magb_p_noise, 0.f, 1.f);
			float mag_B_process_noise = sq(mag_B_sig);

			for (unsigned index = 0; index < State::mag_B.dof; index++) {
				unsigned i = State::mag_B.idx + index;
				nextP(i, i) += mag_B_process_noise;
			}
		}

	} else {
		// keep previous covariance
		for (unsigned i = 0; i < State::mag_I.dof; i++) {
			unsigned row = State::mag_I.idx + i;

			for (unsigned col = 0; col < State::size; col++) {
				nextP(row, col) = nextP(col, row) = P(row, col);
			}
		}

		for (unsigned i = 0; i < State::mag_B.dof; i++) {
			unsigned row = State::mag_B.idx + i;

			for (unsigned col = 0; col < State::size; col++) {
				nextP(row, col) = nextP(col, row) = P(row, col);
			}
		}
	}

#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_WIND)

	if (_control_flags.wind) {
		// Don't continue to grow wind velocity state variances if they are becoming too large or we are not using wind velocity states as this can make the covariance matrix badly conditioned
		if (P.trace<State::wind_vel.dof>(State::wind_vel.idx) < sq(_params.initial_wind_uncertainty)) {

			float wind_vel_nsd_scaled = math::constrain(_params.wind_vel_nsd, 0.f, 1.f) * (1.f + _params.wind_vel_nsd_scaler * fabsf(_height_rate_lpf));

			const float wind_vel_process_noise = sq(wind_vel_nsd_scaled) * dt;

			for (unsigned index = 0; index < State::wind_vel.dof; index++) {
				unsigned i = State::wind_vel.idx + index;
				nextP(i, i) += wind_vel_process_noise;
			}
		}

	} else {
		// keep previous covariance
		for (unsigned i = 0; i < State::wind_vel.dof; i++) {
			unsigned row = State::wind_vel.idx + i;

			for (unsigned col = 0; col < State::size; col++) {
				nextP(row, col) = nextP(col, row) = P(row, col);
			}
		}
	}

#endif // CONFIG_EKF2_WIND

	// covariance matrix is symmetrical, so copy upper half to lower half
	for (unsigned row = 0; row < State::size; row++) {
		for (unsigned column = 0 ; column < row; column++) {
			P(row, column) = P(column, row) = nextP(column, row);
		}

		P(row, row) = nextP(row, row);
	}

	// fix gross errors in the covariance matrix and ensure rows and
	// columns for un-used states are zero
	fixCovarianceErrors(false);
}

void ExtendedKalmanFilter::predictState(const imuSample &imu)
{
	// apply imu bias corrections
	const Vector3f delta_ang_bias_scaled = _state.gyro_bias * imu.delta_ang_dt;
	Vector3f corrected_delta_ang = imu.delta_ang - delta_ang_bias_scaled;

	// subtract component of angular rate due to earth rotation
	corrected_delta_ang -= Dcmf(_state.quat_nominal).transpose() * _earth_rate_NED * imu.delta_ang_dt;

	const Quatf dq(AxisAnglef{corrected_delta_ang});

	// rotate the previous quaternion by the delta quaternion using a quaternion multiplication
	_state.quat_nominal = (_state.quat_nominal * dq).normalized();
	const Dcmf R_to_earth = Dcmf(_state.quat_nominal);

	// Calculate an earth frame delta velocity
	const Vector3f delta_vel_bias_scaled = _state.accel_bias * imu.delta_vel_dt;
	const Vector3f corrected_delta_vel = imu.delta_vel - delta_vel_bias_scaled;
	const Vector3f corrected_delta_vel_ef = R_to_earth * corrected_delta_vel;

	// save the previous value of velocity so we can use trapzoidal integration
	const Vector3f vel_last = _state.vel;

	// calculate the increment in velocity using the current orientation
	_state.vel += corrected_delta_vel_ef;

	// compensate for acceleration due to gravity
	_state.vel(2) += CONSTANTS_ONE_G * imu.delta_vel_dt;

	// predict position states via trapezoidal integration of velocity
	_state.pos += (vel_last + _state.vel) * imu.delta_vel_dt * 0.5f;

	constrainStates();
}

void ExtendedKalmanFilter::constrainStates()
{
	_state.quat_nominal = matrix::constrain(_state.quat_nominal, -1.f, 1.f);
	_state.vel = matrix::constrain(_state.vel, -1000.f, 1000.f);
	_state.pos = matrix::constrain(_state.pos, -1.e6f, 1.e6f);

	_state.gyro_bias = matrix::constrain(_state.gyro_bias, -_params.gyro_bias_lim, _params.gyro_bias_lim);

	_state.accel_bias = matrix::constrain(_state.accel_bias, -_params.acc_bias_lim, _params.acc_bias_lim);

#if defined(CONFIG_EKF2_MAGNETOMETER)
	_state.mag_I = matrix::constrain(_state.mag_I, -1.f, 1.f);
	_state.mag_B = matrix::constrain(_state.mag_B, -_params.mag_bias_lim, _params.mag_bias_lim);
#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_WIND)
	_state.wind_vel = matrix::constrain(_state.wind_vel, -100.f, 100.f);
#endif // CONFIG_EKF2_WIND
}
