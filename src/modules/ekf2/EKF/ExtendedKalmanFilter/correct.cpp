#include "ExtendedKalmanFilter.hpp"

bool ExtendedKalmanFilter::checkAndFixCovarianceUpdate(const SquareMatrixState &KHP)
{
	bool healthy = true;

	for (int i = 0; i < State::size; i++) {
		if (P(i, i) < KHP(i, i)) {
			P.uncorrelateCovarianceSetVariance<1>(i, 0.f);
			healthy = false;
		}
	}

	return healthy;
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

#if defined(CONFIG_EKF2_MAGNETOMETER)

	if (!_control_flags.mag) {
		for (unsigned i = 0; i < State::mag_I.dof; i++) {
			K(State::mag_I.idx + i) = 0.f;
		}
	}

	if (!_control_flags.mag) {
		for (unsigned i = 0; i < State::mag_B.dof; i++) {
			K(State::mag_B.idx + i) = 0.f;
		}
	}

#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_WIND)

	if (!_control_flags.wind) {
		for (unsigned i = 0; i < State::wind_vel.dof; i++) {
			K(State::wind_vel.idx + i) = 0.f;
		}
	}

#endif // CONFIG_EKF2_WIND
}

bool ExtendedKalmanFilter::measurementUpdate(VectorState &K, float innovation_variance, float innovation)
{
	clearInhibitedStateKalmanGains(K);

	const VectorState KS = K * innovation_variance;
	SquareMatrixState KHP;

	for (unsigned row = 0; row < State::size; row++) {
		for (unsigned col = 0; col < State::size; col++) {
			// Instad of literally computing KHP, use an equvalent
			// equation involving less mathematical operations
			KHP(row, col) = KS(row) * K(col);
		}
	}

	const bool is_healthy = checkAndFixCovarianceUpdate(KHP);

	if (is_healthy) {
		// apply the covariance corrections
		P -= KHP;

		fixCovarianceErrors(true);

		// apply the state corrections
		fuse(K, innovation);
	}

	return is_healthy;
}

void ExtendedKalmanFilter::fuse(const VectorState &K, float innovation)
{
	_state.quat_nominal -= K.slice<State::quat_nominal.dof, 1>(State::quat_nominal.idx, 0) * innovation;
	_state.quat_nominal.normalize();

	_state.vel -= K.slice<State::vel.dof, 1>(State::vel.idx, 0) * innovation;
	_state.pos -= K.slice<State::pos.dof, 1>(State::pos.idx, 0) * innovation;

	_state.gyro_bias -= K.slice<State::gyro_bias.dof, 1>(State::gyro_bias.idx, 0) * innovation;
	_state.accel_bias -= K.slice<State::accel_bias.dof, 1>(State::accel_bias.idx, 0) * innovation;

#if defined(CONFIG_EKF2_MAGNETOMETER)
	_state.mag_I -= K.slice<State::mag_I.dof, 1>(State::mag_I.idx, 0) * innovation;
	_state.mag_B -= K.slice<State::mag_B.dof, 1>(State::mag_B.idx, 0) * innovation;
#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_WIND)
	_state.wind_vel -= K.slice<State::wind_vel.dof, 1>(State::wind_vel.idx, 0) * innovation;
#endif // CONFIG_EKF2_WIND
}
