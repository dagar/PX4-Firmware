#include "ExtendedKalmanFilter.hpp"

void ExtendedKalmanFilter::resetMagBias()
{
	_state.mag_B.zero();

	resetMagCov()
}

void ExtendedKalmanFilter::resetMagCov()
{
	P.uncorrelateCovarianceSetVariance<State::mag_I.dof>(State::mag_I.idx, sq(_params.initial_mag_uncertainty));

	P.uncorrelateCovarianceSetVariance<State::mag_B.dof>(State::mag_B.idx, sq(_params.initial_mag_uncertainty));

	//saveMagCovData();
}

bool ExtendedKalmanFilter::setMagEarth(const Vector3f &mag_earth_field)
{
	if (mag_earth_field.isAllFinite()) {
		_state.mag_I = mag_earth_field;
		return true;
	}

	return false;
}

bool ExtendedKalmanFilter::setMagBias(const Vector3f &mag_bias)
{
	if (mag_bias.isAllFinite()) {
		_state.mag_B = mag_bias;
		return true;
	}

	return false;
}
