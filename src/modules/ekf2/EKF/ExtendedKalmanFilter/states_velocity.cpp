#include "ExtendedKalmanFilter.hpp"

ExtendedKalmanFilter::CorrectionResult3 ExtendedKalmanFilter::velocityCorrection(const Vector3f& observation, const Vector3f& observation_variance, float innovation_gate)
{
	CorrectionResult3 ret{};

	innovation_gate = math::max(innovation_gate, 1.f);

	for (int i = 0; i <= 2; i++) {
		ret.innovation(i) = _state.vel(i) - observation(i);
		ret.innovation_variance(i) = getStateVariance<State::vel>()(i) + math::max(observation_variance(i), sq(0.001f));
		ret.test_ratio(i) = sq(ret.innovation(i)) / (sq(innovation_gate) * ret.innovation_variance(i));
		ret.rejected[i] == (ret.test_ratio(i) >= 1.f);
	}

	if (!ret.rejected[0] && !ret.rejected[1] && !ret.rejected[2]) {
		for (int i = 0; i <= 2; i++) {
			if (!fuseVelocity(ret.innovation(0), ret.innovation_variance(0), i)) {
				ret.rejected[i] = true;
				return ret;
			}
		}
	}

	return ret;
}

bool ExtendedKalmanFilter::velocityXYReset(const Vector2f& observation, const Vector2f& observation_variance)
{
	if (observation.isAllFinite() && observation_variance.isAllFinite()) {
		const Vector2f delta_horz_vel = new_horz_vel - Vector2f(_state.vel);
		_state.vel.xy() = new_horz_vel;

		P.uncorrelateCovarianceSetVariance<1>(State::vel.idx + 0, math::max(sq(0.01f), new_horz_vel_var(0)));
		P.uncorrelateCovarianceSetVariance<1>(State::vel.idx + 1, math::max(sq(0.01f), new_horz_vel_var(1)));

		// record the state change
		if (_state_reset_status.reset_count.vel_xy == _state_reset_count_prev.vel_xy) {
			_state_reset_status.vel_xy_change = delta_horz_vel;

		} else {
			// there's already a reset this update, accumulate total delta
			_state_reset_status.vel_xy_change += delta_horz_vel;
		}

		_state_reset_status.reset_count.vel_xy++;

		// Reset the timout timer
		_time_last_vel_xy_fuse = _time_delayed_us;

		return true;
	}

	return false;
}

void ExtendedKalmanFilter::resetVelocityTo(const Vector3f &new_vel, const Vector3f &new_vel_var)
{
	resetHorizontalVelocityTo(Vector2f(new_vel), Vector2f(new_vel_var(0), new_vel_var(1)));
	resetVerticalVelocityTo(new_vel(2), new_vel_var(2));
}

void ExtendedKalmanFilter::resetHorizontalVelocityTo(const Vector2f &new_horz_vel, const Vector2f &new_horz_vel_var)
{
	const Vector2f delta_horz_vel = new_horz_vel - Vector2f(_state.vel);
	_state.vel.xy() = new_horz_vel;

	P.uncorrelateCovarianceSetVariance<1>(State::vel.idx + 0, math::max(sq(0.01f), new_horz_vel_var(0)));
	P.uncorrelateCovarianceSetVariance<1>(State::vel.idx + 1, math::max(sq(0.01f), new_horz_vel_var(1)));

	// record the state change
	if (_state_reset_status.reset_count.vel_xy == _state_reset_count_prev.vel_xy) {
		_state_reset_status.vel_xy_change = delta_horz_vel;

	} else {
		// there's already a reset this update, accumulate total delta
		_state_reset_status.vel_xy_change += delta_horz_vel;
	}

	_state_reset_status.reset_count.vel_xy++;

	// Reset the timout timer
	_time_last_vel_xy_fuse = _time_delayed_us;
}

void ExtendedKalmanFilter::resetVerticalVelocityTo(float new_vert_vel, float new_vert_vel_var)
{
	const float delta_vert_vel = new_vert_vel - _state.vel(2);
	_state.vel(2) = new_vert_vel;

	// the state variance is the same as the observation
	P.uncorrelateCovarianceSetVariance<1>(State::vel.idx + 2, math::max(sq(0.01f), new_vert_vel_var));

	// record the state change
	if (_state_reset_status.reset_count.vel_z == _state_reset_count_prev.vel_z) {
		_state_reset_status.vel_z_change = delta_vert_vel;

	} else {
		// there's already a reset this update, accumulate total delta
		_state_reset_status.vel_z_change += delta_vert_vel;
	}

	_state_reset_status.reset_count.vel_z++;

	// Reset the timout timer
	_time_last_vel_z_fuse = _time_delayed_us;
}

bool ExtendedKalmanFilter::fuseVelocity(const float innov, const float innov_var, const int axis)
{
	VectorState Kfusion; // Kalman gain vector for any single observation - sequential fusion is used.

	// calculate kalman gain K = PHS, where S = 1/innovation variance
	for (int row = 0; row < State::size; row++) {
		Kfusion(row) = P(row, State::vel.idx + axis) / innov_var;
	}

	clearInhibitedStateKalmanGains(Kfusion);

	SquareMatrixState KHP;

	for (unsigned row = 0; row < State::size; row++) {
		for (unsigned column = 0; column < State::size; column++) {
			KHP(row, column) = Kfusion(row) * P(State::vel.idx + axis, column);
		}
	}

	const bool healthy = checkAndFixCovarianceUpdate(KHP);

	if (healthy) {

		switch (axis) {
		case State::vel.idx + 0:
			_time_last_vel_xy_fuse = _time_delayed_us;
			break;

		case State::vel.idx + 1:
			_time_last_vel_xy_fuse = _time_delayed_us;
			break;

		case State::vel.idx + 2:
			_time_last_vel_z_fuse = _time_delayed_us;
			break;
		}

		// apply the covariance corrections
		P -= KHP;

		fixCovarianceErrors(true);

		// apply the state corrections
		fuse(Kfusion, innov);

		return true;
	}

	return false;
}
