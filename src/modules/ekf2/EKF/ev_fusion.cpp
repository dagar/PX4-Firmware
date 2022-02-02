
void Ekf::fuseEvHeading(float measured_hdg, float obs_var)
{


	// observation variance
	float R_YAW = PX4_ISFINITE(obs_var) ? obs_var : 0.01f;

	// update transformation matrix from body to world frame using the current state estimate
	_R_to_earth = Dcmf(_state.quat_nominal);

	const bool use_321_rotation_seq = shouldUse321RotationSequence(_R_to_earth);

	const float predicted_hdg = use_321_rotation_seq ? getEuler321Yaw(_R_to_earth) : getEuler312Yaw(_R_to_earth);

	if (use_321_rotation_seq) {
		if (fuse_zero_innov) {
			fuseYaw321(0.f, R_YAW);
		} else {
			float innovation = wrap_pi(atan2f(_R_to_earth(1, 0), _R_to_earth(0, 0)) - wrap_pi(measured_hdg));
			fuseYaw321(innovation, R_YAW);
		}
	} else {
		if (fuse_zero_innov) {
			fuseYaw312(0.f, R_YAW);
		} else {
			float innovation = wrap_pi(atan2f(-_R_to_earth(0, 1), _R_to_earth(1, 1)) - wrap_pi(measured_hdg));
			fuseYaw312(innovation, R_YAW);
		}
	}



}
