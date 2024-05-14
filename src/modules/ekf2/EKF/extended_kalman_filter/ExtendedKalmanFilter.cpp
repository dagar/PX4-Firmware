
#include "ExtendedKalmanFilter.hpp"

#include <lib/mathlib/mathlib.h>
#include <lib/geo/geo.h>

using matrix::AxisAnglef;
using matrix::Dcmf;
using matrix::Quatf;
using matrix::Vector3f;

ExtendedKalmanFilter::ExtendedKalmanFilter()
{
	reset();
}

void ExtendedKalmanFilter::reset()
{
	//ECL_INFO("reset");

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

	_time_last_heading_fuse = 0;
}

bool ExtendedKalmanFilter::initialiseTilt()
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

bool ExtendedKalmanFilter::update(const imuSample &imu_sample)
{
	// calculate an average filter update time
	float input = 0.5f * (imu_sample.delta_vel_dt + imu_sample.delta_ang_dt);
	_dt_ekf_avg = 0.99f * _dt_ekf_avg + 0.01f * input;

	if (_time_us == 0) {
		_accel_lpf.reset(imu_sample.delta_vel / imu_sample.delta_vel_dt);
		_gyro_lpf.reset(imu_sample.delta_ang / imu_sample.delta_ang_dt);

	} else {
		_accel_lpf.update(imu_sample.delta_vel / imu_sample.delta_vel_dt);
		_gyro_lpf.update(imu_sample.delta_ang / imu_sample.delta_ang_dt);
	}

	_time_us = imu_sample.time_us;




	if (!_filter_initialised) {
		if (!initialiseTilt()) {
			return false;
		}

		// initialise the state covariance matrix now we have starting values for all the states
		initialiseCovariance();

		_filter_initialised = true;
	}




	// perform state and covariance prediction for the main filter
	predictCovariance(imu_sample);
	predictState(imu_sample);

	return true;
}

void ExtendedKalmanFilter::predictState(const imuSample &imu_sample)
{
	// apply imu bias corrections
	const Vector3f delta_ang_bias_scaled = _state.gyro_bias * imu_sample.delta_ang_dt;
	Vector3f corrected_delta_ang = imu_sample.delta_ang - delta_ang_bias_scaled;

	// subtract component of angular rate due to earth rotation
	corrected_delta_ang -= _R_to_earth.transpose() * _earth_rate_NED * imu_sample.delta_ang_dt;

	const Quatf dq(AxisAnglef{corrected_delta_ang});

	// rotate the previous quaternion by the delta quaternion using a quaternion multiplication
	_state.quat_nominal = (_state.quat_nominal * dq).normalized();
	_R_to_earth = Dcmf(_state.quat_nominal);

	// Calculate an earth frame delta velocity
	const Vector3f delta_vel_bias_scaled = _state.accel_bias * imu_sample.delta_vel_dt;
	const Vector3f corrected_delta_vel = imu_sample.delta_vel - delta_vel_bias_scaled;
	const Vector3f corrected_delta_vel_ef = _R_to_earth * corrected_delta_vel;

	// save the previous value of velocity so we can use trapzoidal integration
	const Vector3f vel_last = _state.vel;

	// calculate the increment in velocity using the current orientation
	_state.vel += corrected_delta_vel_ef;

	// compensate for acceleration due to gravity
	_state.vel(2) += CONSTANTS_ONE_G * imu_sample.delta_vel_dt;

	// predict position states via trapezoidal integration of velocity
	_state.pos += (vel_last + _state.vel) * imu_sample.delta_vel_dt * 0.5f;

	// constrain states
	_state.vel = matrix::constrain(_state.vel, -1000.f, 1000.f);
	_state.pos = matrix::constrain(_state.pos, -1.e6f, 1.e6f);

	// some calculations elsewhere in code require a raw angular rate vector so calculate here to avoid duplication
	// protect against possible small timesteps resulting from timing slip on previous frame that can drive spikes into the rate
	// due to insufficient averaging
	if (imu_sample.delta_ang_dt > 0.25f * _dt_ekf_avg) {
		_ang_rate_raw = imu_sample.delta_ang / imu_sample.delta_ang_dt;
	}


	// calculate a filtered horizontal acceleration with a 1 sec time constant
	// this are used for manoeuvre detection elsewhere
	const float alpha = 1.0f - imu_sample.delta_vel_dt;
	_accel_lpf_NE = _accel_lpf_NE * alpha + corrected_delta_vel_ef.xy();

	// Calculate low pass filtered height rate
	float alpha_height_rate_lpf = 0.1f * imu_sample.delta_vel_dt; // 10 seconds time constant
	_height_rate_lpf = _height_rate_lpf * (1.0f - alpha_height_rate_lpf) + _state.vel(2) * alpha_height_rate_lpf;
}

void ExtendedKalmanFilter::clearInhibitedStateKalmanGains(VectorState &K) const
{
	for (unsigned i = 0; i < State::gyro_bias.dof; i++) {
		if (_gyro_bias_inhibit[i]) {
			K(State::gyro_bias.idx + i) = 0.f;
		}
	}

	for (unsigned i = 0; i < State::accel_bias.dof; i++) {
		if (_accel_bias_inhibit[i]) {
			K(State::accel_bias.idx + i) = 0.f;
		}
	}

#if defined(CONFIG_EKF2_MAGNETOMETER) && 0
	if (!_control_status.flags.mag) {
		for (unsigned i = 0; i < State::mag_I.dof; i++) {
			K(State::mag_I.idx + i) = 0.f;
		}
	}

	if (!_control_status.flags.mag) {
		for (unsigned i = 0; i < State::mag_B.dof; i++) {
			K(State::mag_B.idx + i) = 0.f;
		}
	}
#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_WIND) && 0
	if (!_control_status.flags.wind) {
		for (unsigned i = 0; i < State::wind_vel.dof; i++) {
			K(State::wind_vel.idx + i) = 0.f;
		}
	}
#endif // CONFIG_EKF2_WIND
}

bool ExtendedKalmanFilter::fuseDirectStateMeasurement(const float innov, const float innov_var, const float R, const int state_index)
{
	VectorState K;  // Kalman gain vector for any single observation - sequential fusion is used.

	// calculate kalman gain K = PHS, where S = 1/innovation variance
	for (int row = 0; row < State::size; row++) {
		K(row) = P(row, state_index) / innov_var;
	}

	clearInhibitedStateKalmanGains(K);

#if false
	// Matrix implementation of the Joseph stabilized covariance update
	// This is extremely expensive to compute. Use for debugging purposes only.
	auto A = matrix::eye<float, State::size>();
	VectorState H;
	H(state_index) = 1.f;
	A -= K.multiplyByTranspose(H);
	P = A * P;
	P = P.multiplyByTranspose(A);

	const VectorState KR = K * R;
	P += KR.multiplyByTranspose(K);
#else
	// Efficient implementation of the Joseph stabilized covariance update
	// Based on "G. J. Bierman. Factorization Methods for Discrete Sequential Estimation. Academic Press, Dover Publications, New York, 1977, 2006"
	// P = (I - K * H) * P * (I - K * H).T   + K * R * K.T
	//   =      P_temp     * (I - H.T * K.T) + K * R * K.T
	//   =      P_temp - P_temp * H.T * K.T  + K * R * K.T

	// Step 1: conventional update
	// Compute P_temp and store it in P to avoid allocating more memory
	// P is symmetric, so PH == H.T * P.T == H.T * P. Taking the row is faster as matrices are row-major
	VectorState PH = P.row(state_index);

	for (unsigned i = 0; i < State::size; i++) {
		for (unsigned j = 0; j < State::size; j++) {
			P(i, j) -= K(i) * PH(j); // P is now not symmetric if K is not optimal (e.g.: some gains have been zeroed)
		}
	}

	// Step 2: stabilized update
	// P (or "P_temp") is not symmetric so we must take the column
	PH = P.col(state_index);

	for (unsigned i = 0; i < State::size; i++) {
		for (unsigned j = 0; j <= i; j++) {
			P(i, j) = P(i, j) - PH(i) * K(j) + K(i) * R * K(j);
			P(j, i) = P(i, j);
		}
	}

#endif

	constrainStateVariances();

	// apply the state corrections
	fuse(K, innov);
	return true;
}

bool ExtendedKalmanFilter::measurementUpdate(VectorState &K, const VectorState &H, const float R,
		const float innovation)
{
	clearInhibitedStateKalmanGains(K);

#if false
	// Matrix implementation of the Joseph stabilized covariance update
	// This is extremely expensive to compute. Use for debugging purposes only.
	auto A = matrix::eye<float, State::size>();
	A -= K.multiplyByTranspose(H);
	P = A * P;
	P = P.multiplyByTranspose(A);

	const VectorState KR = K * R;
	P += KR.multiplyByTranspose(K);
#else
	// Efficient implementation of the Joseph stabilized covariance update
	// Based on "G. J. Bierman. Factorization Methods for Discrete Sequential Estimation. Academic Press, Dover Publications, New York, 1977, 2006"
	// P = (I - K * H) * P * (I - K * H).T   + K * R * K.T
	//   =      P_temp     * (I - H.T * K.T) + K * R * K.T
	//   =      P_temp - P_temp * H.T * K.T  + K * R * K.T

	// Step 1: conventional update
	// Compute P_temp and store it in P to avoid allocating more memory
	// P is symmetric, so PH == H.T * P.T == H.T * P. Taking the row is faster as matrices are row-major
	VectorState PH = P * H; // H is stored as a column vector. H is in fact H.T

	for (unsigned i = 0; i < State::size; i++) {
		for (unsigned j = 0; j < State::size; j++) {
			P(i, j) -= K(i) * PH(j); // P is now not symmetrical if K is not optimal (e.g.: some gains have been zeroed)
		}
	}

	// Step 2: stabilized update
	PH = P * H; // H is stored as a column vector. H is in fact H.T

	for (unsigned i = 0; i < State::size; i++) {
		for (unsigned j = 0; j <= i; j++) {
			P(i, j) = P(i, j) - PH(i) * K(j) + K(i) * R * K(j);
			P(j, i) = P(i, j);
		}
	}

#endif

	constrainStateVariances();

	// apply the state corrections
	fuse(K, innovation);
	return true;
}

void ExtendedKalmanFilter::fuse(const VectorState &K, float innovation)
{
	// quat_nominal
	Quatf delta_quat(matrix::AxisAnglef(K.slice<State::quat_nominal.dof, 1>(State::quat_nominal.idx, 0) * (-1.f * innovation)));
	_state.quat_nominal = delta_quat * _state.quat_nominal;
	_state.quat_nominal.normalize();
	_R_to_earth = Dcmf(_state.quat_nominal);

	// vel
	_state.vel = matrix::constrain(_state.vel - K.slice<State::vel.dof, 1>(State::vel.idx, 0) * innovation, -1.e3f, 1.e3f);

	// pos
	_state.pos = matrix::constrain(_state.pos - K.slice<State::pos.dof, 1>(State::pos.idx, 0) * innovation, -1.e6f, 1.e6f);

	// gyro_bias
	_state.gyro_bias = matrix::constrain(_state.gyro_bias - K.slice<State::gyro_bias.dof, 1>(State::gyro_bias.idx, 0) * innovation,
					     -getGyroBiasLimit(), getGyroBiasLimit());

	// accel_bias
	_state.accel_bias = matrix::constrain(_state.accel_bias - K.slice<State::accel_bias.dof, 1>(State::accel_bias.idx, 0) * innovation,
					      -getAccelBiasLimit(), getAccelBiasLimit());

#if defined(CONFIG_EKF2_MAGNETOMETER)
	// mag_I, mag_B
	// if (_control_status.flags.mag) {
	// 	_state.mag_I = matrix::constrain(_state.mag_I - K.slice<State::mag_I.dof, 1>(State::mag_I.idx, 0) * innovation, -1.f, 1.f);
	// 	_state.mag_B = matrix::constrain(_state.mag_B - K.slice<State::mag_B.dof, 1>(State::mag_B.idx, 0) * innovation, -getMagBiasLimit(), getMagBiasLimit());
	// }
#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_WIND)
	// wind_vel
	// if (_control_status.flags.wind) {
	// 	_state.wind_vel = matrix::constrain(_state.wind_vel - K.slice<State::wind_vel.dof, 1>(State::wind_vel.idx, 0) * innovation, -1.e2f, 1.e2f);
	// }
#endif // CONFIG_EKF2_WIND
}

Vector3f ExtendedKalmanFilter::getRotVarBody() const
{
	const matrix::SquareMatrix3f rot_cov_body = getStateCovariance<State::quat_nominal>();
	return matrix::SquareMatrix3f(_R_to_earth.T() * rot_cov_body * _R_to_earth).diag();
}

Vector3f ExtendedKalmanFilter::getRotVarNed() const
{
	const matrix::SquareMatrix3f rot_cov_ned = getStateCovariance<State::quat_nominal>();
	return rot_cov_ned.diag();
}

float ExtendedKalmanFilter::getYawVar() const
{
	return getRotVarNed()(2);
}

float ExtendedKalmanFilter::getTiltVariance() const
{
	const Vector3f rot_var_ned = getRotVarNed();
	return rot_var_ned(0) + rot_var_ned(1);
}
