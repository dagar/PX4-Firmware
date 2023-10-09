#include "ExtendedKalmanFilter.hpp"

void ExtendedKalmanFilter::resetPositionTo(const Vector3f &new_pos, const Vector3f &new_pos_var)
{
	resetHorizontalVelocityTo(Vector2f(new_pos), Vector2f(new_pos_var(0), new_pos_var(1)));
	resetVerticalVelocityTo(new_pos(2), new_pos_var(2));
}

void ExtendedKalmanFilter::resetHorizontalPositionTo(const Vector2f &new_horz_pos, const Vector2f &new_horz_pos_var)
{
	const Vector2f delta_horz_pos{new_horz_pos - Vector2f{_state.pos}};
	_state.pos.xy() = new_horz_pos;

	P.uncorrelateCovarianceSetVariance<1>(State::pos.idx + 0, math::max(sq(0.01f), new_horz_pos_var(0)));
	P.uncorrelateCovarianceSetVariance<1>(State::pos.idx + 1, math::max(sq(0.01f), new_horz_pos_var(1)));

	// record the state change
	if (_state_reset_status.reset_count.pos_xy == _state_reset_count_prev.pos_xy) {
		_state_reset_status.pos_xy_change = delta_horz_pos;

	} else {
		// there's already a reset this update, accumulate total delta
		_state_reset_status.pos_xy_change += delta_horz_pos;
	}

	_state_reset_status.reset_count.pos_xy++;

	// Reset the timout timer
	_time_last_pos_xy_fuse = _time_delayed_us;
}

void ExtendedKalmanFilter::resetVerticalPositionTo(const float new_vert_pos, const float new_vert_pos_var)
{
	const float old_vert_pos = _state.pos(2);
	_state.pos(2) = new_vert_pos;

	// the state variance is the same as the observation
	P.uncorrelateCovarianceSetVariance<1>(State::pos.idx + 2, math::max(sq(0.01f), new_vert_pos_var));

	const float delta_z = new_vert_pos - old_vert_pos;

	// record the state change
	if (_state_reset_status.reset_count.pos_z == _state_reset_count_prev.pos_z) {
		_state_reset_status.pos_z_change = delta_z;

	} else {
		// there's already a reset this update, accumulate total delta
		_state_reset_status.pos_z_change += delta_z;
	}

	_state_reset_status.reset_count.pos_z++;

	// Reset the timout timer
	_time_last_pos_z_fuse = _time_delayed_us;
}

bool ExtendedKalmanFilter::fusePosition(const float innov, const float innov_var, const int axis)
{
	VectorState Kfusion; // Kalman gain vector for any single observation - sequential fusion is used.

	// calculate kalman gain K = PHS, where S = 1/innovation variance
	for (int row = 0; row < State::size; row++) {
		Kfusion(row) = P(row, State::pos.idx + axis) / innov_var;
	}

	clearInhibitedStateKalmanGains(Kfusion);

	SquareMatrixState KHP;

	for (unsigned row = 0; row < State::size; row++) {
		for (unsigned column = 0; column < State::size; column++) {
			KHP(row, column) = Kfusion(row) * P(State::pos.idx + axis, column);
		}
	}

	const bool healthy = checkAndFixCovarianceUpdate(KHP);

	if (healthy) {

		switch (axis) {
		case State::pos.idx + 0:
			_time_last_pos_xy_fuse = _time_delayed_us;
			break;

		case State::pos.idx + 1:
			_time_last_pos_xy_fuse = _time_delayed_us;
			break;

		case State::pos.idx + 2:
			_time_last_pos_z_fuse = _time_delayed_us;
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
