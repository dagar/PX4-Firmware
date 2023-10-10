#include "ExtendedKalmanFilter.hpp"

void ExtendedKalmanFilter::resetGyroBias()
{
	// zero the gyro bias states
	_state.gyro_bias.zero();

	resetGyroBiasCov();
}

void ExtendedKalmanFilter::resetGyroBiasCov()
{
	// Zero the corresponding covariances and set
	// variances to the values use for initial alignment
	P.uncorrelateCovarianceSetVariance<State::gyro_bias.dof>(State::gyro_bias.idx, sq(_params.switch_on_gyro_bias));

	// set previous values
	_prev_gyro_bias_var = getStateVariance<State::gyro_bias>();
}

void ExtendedKalmanFilter::resetGyroBiasZCov()
{
	P.uncorrelateCovarianceSetVariance<1>(State::gyro_bias.idx + 2, sq(_params.switch_on_gyro_bias));

	_prev_gyro_bias_var(2) = getStateVariance<State::gyro_bias>()(2);
}

void ExtendedKalmanFilter::inhibitGyroBiasAxis(int axis)
{
	// store the bias state variances to be reinstated later
	if (!_gyro_bias_inhibit[axis]) {
		_prev_gyro_bias_var(axis) = getStateVariance<State::gyro_bias>()(axis);
		_gyro_bias_inhibit[axis] = true;
	}
}

void ExtendedKalmanFilter::uninhibitGyroBiasAxis(int axis)
{
	if (_gyro_bias_inhibit[axis]) {
		// reinstate the bias state variances
		P.uncorrelateCovarianceSetVariance<State::gyro_bias.dof>(State::gyro_bias.idx, _prev_gyro_bias_var(axis));
		_gyro_bias_inhibit[axis] = false;
	}
}
