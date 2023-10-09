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
