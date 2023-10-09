#include "ExtendedKalmanFilter.hpp"

void ExtendedKalmanFilter::resetWind()
{
	// if (_control_status.flags.fuse_aspd && isRecent(_airspeed_sample_delayed.time_us, 1e6)) {
	// 	resetWindUsingAirspeed(_airspeed_sample_delayed);
	// 	return;
	// }

	resetWindToZero();
}

void ExtendedKalmanFilter::resetWindToZero()
{
	// If we don't have an airspeed measurement, then assume the wind is zero
	_state.wind_vel.setZero();

	resetWindCov();
}

void ExtendedKalmanFilter::resetWindCov()
{
	// start with a small initial uncertainty to improve the initial estimate
	P.uncorrelateCovarianceSetVariance<State::wind_vel.dof>(State::wind_vel.idx, _params.initial_wind_uncertainty);
}
