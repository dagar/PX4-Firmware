#include "ExtendedKalmanFilter.hpp"

void ExtendedKalmanFilter::resetMagCov()
{
	P.uncorrelateCovarianceSetVariance<State::mag_I.dof>(State::mag_I.idx, sq(_params.initial_mag_uncertainty));

	P.uncorrelateCovarianceSetVariance<State::mag_B.dof>(State::mag_B.idx, sq(_params.initial_mag_uncertainty));

	//saveMagCovData();
}
