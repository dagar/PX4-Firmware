#include "ExtendedKalmanFilter.hpp"

#include "derivation/generated/quat_var_to_rot_var.h"
#include "derivation/generated/rot_var_ned_to_lower_triangular_quat_cov.h"

#include <lib/mathlib/math/Utilities.hpp>

using math::Utilities::updateYawInRotMat;

void ExtendedKalmanFilter::resetQuatCov(const Vector3f &rot_var_ned)
{
	matrix::SquareMatrix<float, State::quat_nominal.dof> q_cov;
	sym::RotVarNedToLowerTriangularQuatCov(_state.vector(), rot_var_ned, &q_cov);
	q_cov.copyLowerToUpperTriangle();
	resetStateCovariance<State::quat_nominal>(q_cov);
}

Vector3f ExtendedKalmanFilter::calcRotVecVariances() const
{
	Vector3f rot_var;
	sym::QuatVarToRotVar(_state.vector(), P, FLT_EPSILON, &rot_var);
	return rot_var;
}

float ExtendedKalmanFilter::getYawVar() const
{
	VectorState H_YAW;
	float yaw_var = 0.f;
	computeYawInnovVarAndH(0.f, yaw_var, H_YAW);

	return yaw_var;
}

void ExtendedKalmanFilter::resetQuatStateYaw(float yaw, float yaw_variance)
{
	// save a copy of the quaternion state for later use in calculating the amount of reset change
	const Quatf quat_before_reset = _state.quat_nominal;

	// save a copy of covariance in NED frame to restore it after the quat reset
	Vector3f rot_var;
	sym::QuatVarToRotVar(_state.vector(), P, FLT_EPSILON, &rot_var);
	const matrix::SquareMatrix3f rot_cov = diag(rot_var);

	const Dcmf R_to_earth = Dcmf(_state.quat_nominal);
	Vector3f rot_var_ned_before_reset = matrix::SquareMatrix3f(R_to_earth * rot_cov * R_to_earth.T()).diag();

	// update the yaw angle variance
	if (PX4_ISFINITE(yaw_variance) && (yaw_variance > FLT_EPSILON)) {
		rot_var_ned_before_reset(2) = yaw_variance;
	}

	// update transformation matrix from body to world frame using the current estimate
	// update the rotation matrix using the new yaw value
	const Dcmf R_to_earth_new = updateYawInRotMat(yaw, Dcmf(_state.quat_nominal));

	// calculate the amount that the quaternion has changed by
	const Quatf quat_after_reset(R_to_earth_new);
	const Quatf q_error((quat_after_reset * quat_before_reset.inversed()).normalized());

	// update quaternion states
	_state.quat_nominal = quat_after_reset;

	P.uncorrelateCovarianceBlock<State::quat_nominal.dof>(State::quat_nominal.idx);

	// restore covariance
	resetQuatCov(rot_var_ned_before_reset);

	// record the state change
	if (_state_reset_status.reset_count.quat == _state_reset_count_prev.quat) {
		_state_reset_status.quat_change = q_error;

	} else {
		// there's already a reset this update, accumulate total delta
		_state_reset_status.quat_change = q_error * _state_reset_status.quat_change;
		_state_reset_status.quat_change.normalize();
	}

	_state_reset_status.reset_count.quat++;

	_time_last_yaw_fuse = _time_delayed_us;
}
