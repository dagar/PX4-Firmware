#include "ExtendedKalmanFilter.hpp"

void ExtendedKalmanFilter::resetAccelBias()
{
	// zero the accel bias states
	_state.accel_bias.zero();

	resetAccelBiasCov();
}

void ExtendedKalmanFilter::resetAccelBiasCov()
{
	// Zero the corresponding covariances and set
	// variances to the values use for initial alignment
	P.uncorrelateCovarianceSetVariance<State::accel_bias.dof>(State::accel_bias.idx, sq(_params.switch_on_accel_bias));

	// set previous values
	_prev_accel_bias_var = getStateVariance<State::accel_bias>();
}

void ExtendedKalmanFilter::inhibitAccelBiasAxis(int axis)
{
	// store the bias state variances to be reinstated later
	if (!_accel_bias_inhibit[axis]) {
		_prev_accel_bias_var(axis) = getStateVariance<State::accel_bias>()(axis);
		_accel_bias_inhibit[axis] = true;
	}
}

void ExtendedKalmanFilter::uninhibitAccelBiasAxis(int axis)
{
	if (_accel_bias_inhibit[axis]) {
		// reinstate the bias state variances
		P.uncorrelateCovarianceSetVariance<State::accel_bias.dof>(State::accel_bias.idx, _prev_accel_bias_var(axis));
		_accel_bias_inhibit[axis] = false;
	}
}
