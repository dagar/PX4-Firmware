
if (updated)
{

	// integrate time to monitor time slippage
	if (_start_time_us == 0) {
		_start_time_us = now;
		_last_time_slip_us = 0;

	} else if (_start_time_us > 0) {
		_integrated_time_us += sensors.gyro_integral_dt;
		_last_time_slip_us = (now - _start_time_us) - _integrated_time_us;
	}

	matrix::Quatf q;
	_ukf.copy_quaternion(q.data());

	// In-run bias estimates
	float gyro_bias[3];
	_ukf.get_gyro_bias(gyro_bias);

	{
		// generate vehicle attitude quaternion data
		vehicle_attitude_s att;
		att.timestamp = now;

		q.copyTo(att.q);
		_ukf.get_quat_reset(&att.delta_q_reset[0], &att.quat_reset_counter);

		att.rollspeed = sensors.gyro_rad[0] - gyro_bias[0];
		att.pitchspeed = sensors.gyro_rad[1] - gyro_bias[1];
		att.yawspeed = sensors.gyro_rad[2] - gyro_bias[2];

		// publish vehicle attitude data
		if (_att_pub == nullptr) {
			_att_pub = orb_advertise(ORB_ID(vehicle_attitude), &att);

		} else {
			orb_publish(ORB_ID(vehicle_attitude), _att_pub, &att);
		}
	}

	// generate vehicle local position data
	vehicle_local_position_s &lpos = _vehicle_local_position_pub.get();

	lpos.timestamp = now;

	// Position of body origin in local NED frame
	float position[3];
	_ukf.get_position(position);
	const float lpos_x_prev = lpos.x;
	const float lpos_y_prev = lpos.y;
	lpos.x = (_ukf.local_position_is_valid()) ? position[0] : 0.0f;
	lpos.y = (_ukf.local_position_is_valid()) ? position[1] : 0.0f;
	lpos.z = position[2];

	// Velocity of body origin in local NED frame (m/s)
	float velocity[3];
	_ukf.get_velocity(velocity);
	lpos.vx = velocity[0];
	lpos.vy = velocity[1];
	lpos.vz = velocity[2];

	// vertical position time derivative (m/s)
	_ukf.get_pos_d_deriv(&lpos.z_deriv);

	// Acceleration of body origin in local NED frame
	float vel_deriv[3];
	_ukf.get_vel_deriv_ned(vel_deriv);
	lpos.ax = vel_deriv[0];
	lpos.ay = vel_deriv[1];
	lpos.az = vel_deriv[2];

	// TODO: better status reporting
	lpos.xy_valid = _ukf.local_position_is_valid() && !_preflt_horiz_fail;
	lpos.z_valid = !_preflt_vert_fail;
	lpos.v_xy_valid = _ukf.local_position_is_valid() && !_preflt_horiz_fail;
	lpos.v_z_valid = !_preflt_vert_fail;

	// Position of local NED origin in GPS / WGS84 frame
	map_projection_reference_s ekf_origin;
	uint64_t origin_time;

	// true if position (x,y,z) has a valid WGS-84 global reference (ref_lat, ref_lon, alt)
	const bool ekf_origin_valid = _ukf.get_ekf_origin(&origin_time, &ekf_origin, &lpos.ref_alt);
	lpos.xy_global = ekf_origin_valid;
	lpos.z_global = ekf_origin_valid;

	if (ekf_origin_valid && (origin_time > lpos.ref_timestamp)) {
		lpos.ref_timestamp = origin_time;
		lpos.ref_lat = ekf_origin.lat_rad * 180.0 / M_PI; // Reference point latitude in degrees
		lpos.ref_lon = ekf_origin.lon_rad * 180.0 / M_PI; // Reference point longitude in degrees
	}

	// The rotation of the tangent plane vs. geographical north
	matrix::Eulerf euler(q);
	lpos.yaw = euler.psi();

	lpos.dist_bottom_valid = _ukf.get_terrain_valid();

	float terrain_vpos;
	_ukf.get_terrain_vert_pos(&terrain_vpos);
	lpos.dist_bottom = terrain_vpos - lpos.z; // Distance to bottom surface (ground) in meters

	// constrain the distance to ground to _rng_gnd_clearance
	if (lpos.dist_bottom < _rng_gnd_clearance.get()) {
		lpos.dist_bottom = _rng_gnd_clearance.get();
	}

	lpos.dist_bottom_rate = -lpos.vz; // Distance to bottom surface (ground) change rate

	bool dead_reckoning;
	_ukf.get_ekf_lpos_accuracy(&lpos.eph, &lpos.epv, &dead_reckoning);
	_ukf.get_ekf_vel_accuracy(&lpos.evh, &lpos.evv, &dead_reckoning);

	// get state reset information of position and velocity
	_ukf.get_posD_reset(&lpos.delta_z, &lpos.z_reset_counter);
	_ukf.get_velD_reset(&lpos.delta_vz, &lpos.vz_reset_counter);
	_ukf.get_posNE_reset(&lpos.delta_xy[0], &lpos.xy_reset_counter);
	_ukf.get_velNE_reset(&lpos.delta_vxy[0], &lpos.vxy_reset_counter);

	// get control limit information
	_ukf.get_ekf_ctrl_limits(&lpos.vxy_max, &lpos.limit_hagl);

	// convert NaN to zero
	if (!PX4_ISFINITE(lpos.vxy_max)) {
		lpos.vxy_max = 0.0f;
	}

	// publish vehicle local position data
	_vehicle_local_position_pub.update();

	if (_ukf.global_position_is_valid() && !_preflt_fail) {
		// generate and publish global position data
		vehicle_global_position_s &global_pos = _vehicle_global_position_pub.get();

		global_pos.timestamp = now;

		if (fabsf(lpos_x_prev - lpos.x) > FLT_EPSILON || fabsf(lpos_y_prev - lpos.y) > FLT_EPSILON) {
			map_projection_reproject(&ekf_origin, lpos.x, lpos.y, &global_pos.lat, &global_pos.lon);
		}

		global_pos.lat_lon_reset_counter = lpos.xy_reset_counter;

		global_pos.alt = -lpos.z + lpos.ref_alt; // Altitude AMSL in meters

		// global altitude has opposite sign of local down position
		global_pos.delta_alt = -lpos.delta_z;

		global_pos.vel_n = lpos.vx; // Ground north velocity, m/s
		global_pos.vel_e = lpos.vy; // Ground east velocity, m/s
		global_pos.vel_d = lpos.vz; // Ground downside velocity, m/s

		global_pos.pos_d_deriv = lpos.z_deriv; // vertical position time derivative, m/s

		global_pos.yaw = lpos.yaw; // Yaw in radians -PI..+PI.

		_ukf.get_ekf_gpos_accuracy(&global_pos.eph, &global_pos.epv, &global_pos.dead_reckoning);
		global_pos.evh = lpos.evh;
		global_pos.evv = lpos.evv;

		global_pos.terrain_alt_valid = lpos.dist_bottom_valid;

		if (global_pos.terrain_alt_valid) {
			global_pos.terrain_alt = lpos.ref_alt - terrain_vpos; // Terrain altitude in m, WGS84

		} else {
			global_pos.terrain_alt = 0.0f; // Terrain altitude in m, WGS84
		}

		global_pos.pressure_alt = sensors.baro_alt_meter; // Pressure altitude AMSL (m)

		global_pos.dead_reckoning = _ukf.inertial_dead_reckoning(); // True if this position is estimated through dead-reckoning

		_vehicle_global_position_pub.update();
	}

	{
		// publish all corrected sensor readings and bias estimates after mag calibration is updated above
		float accel_bias[3];
		_ukf.get_accel_bias(accel_bias);

		sensor_bias_s bias;

		bias.timestamp = now;

		bias.gyro_x = sensors.gyro_rad[0] - gyro_bias[0];
		bias.gyro_y = sensors.gyro_rad[1] - gyro_bias[1];
		bias.gyro_z = sensors.gyro_rad[2] - gyro_bias[2];

		bias.accel_x = sensors.accelerometer_m_s2[0] - accel_bias[0];
		bias.accel_y = sensors.accelerometer_m_s2[1] - accel_bias[1];
		bias.accel_z = sensors.accelerometer_m_s2[2] - accel_bias[2];

		bias.mag_x = sensors.magnetometer_ga[0] - (_last_valid_mag_cal[0] / 1000.0f); // mGauss -> Gauss
		bias.mag_y = sensors.magnetometer_ga[1] - (_last_valid_mag_cal[1] / 1000.0f); // mGauss -> Gauss
		bias.mag_z = sensors.magnetometer_ga[2] - (_last_valid_mag_cal[2] / 1000.0f); // mGauss -> Gauss

		bias.gyro_x_bias = gyro_bias[0];
		bias.gyro_y_bias = gyro_bias[1];
		bias.gyro_z_bias = gyro_bias[2];

		bias.accel_x_bias = accel_bias[0];
		bias.accel_y_bias = accel_bias[1];
		bias.accel_z_bias = accel_bias[2];

		bias.mag_x_bias = _last_valid_mag_cal[0];
		bias.mag_y_bias = _last_valid_mag_cal[1];
		bias.mag_z_bias = _last_valid_mag_cal[2];

		if (_sensor_bias_pub == nullptr) {
			_sensor_bias_pub = orb_advertise(ORB_ID(sensor_bias), &bias);

		} else {
			orb_publish(ORB_ID(sensor_bias), _sensor_bias_pub, &bias);
		}
	}
}

// publish estimator status
estimator_status_s status;
status.timestamp = now;
_ukf.get_state_delayed(status.states);
_ukf.get_covariances(status.covariances);
_ukf.get_gps_check_status(&status.gps_check_fail_flags);
_ukf.get_control_mode(&status.control_mode_flags);
_ukf.get_filter_fault_status(&status.filter_fault_flags);
_ukf.get_innovation_test_status(&status.innovation_check_flags, &status.mag_test_ratio,
				&status.vel_test_ratio, &status.pos_test_ratio,
				&status.hgt_test_ratio, &status.tas_test_ratio,
				&status.hagl_test_ratio, &status.beta_test_ratio);

status.pos_horiz_accuracy = _vehicle_local_position_pub.get().eph;
status.pos_vert_accuracy = _vehicle_local_position_pub.get().epv;
_ukf.get_ekf_soln_status(&status.solution_status_flags);
_ukf.get_imu_vibe_metrics(status.vibe);
status.time_slip = _last_time_slip_us / 1e6f;
status.nan_flags = 0.0f; // unused
status.health_flags = 0.0f; // unused
status.timeout_flags = 0.0f; // unused
status.pre_flt_fail = _preflt_fail;

if (_estimator_status_pub == nullptr)
{
	_estimator_status_pub = orb_advertise(ORB_ID(estimator_status), &status);

} else
{
	orb_publish(ORB_ID(estimator_status), _estimator_status_pub, &status);
}

if (updated)
{
	{
		/* Check and save learned magnetometer bias estimates */

		// Check if conditions are OK to for learning of magnetometer bias values
		if (!vehicle_land_detected.landed && // not on ground
		    (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) && // vehicle is armed
		    (status.filter_fault_flags == 0) && // there are no filter faults
		    (status.control_mode_flags & (1 << 5))) { // the EKF is operating in the correct mode

			if (_last_magcal_us == 0) {
				_last_magcal_us = now;

			} else {
				_total_cal_time_us += now - _last_magcal_us;
				_last_magcal_us = now;
			}

		} else if (status.filter_fault_flags != 0) {
			// if a filter fault has occurred, assume previous learning was invalid and do not
			// count it towards total learning time.
			_total_cal_time_us = 0;

			for (bool &cal_available : _valid_cal_available) {
				cal_available = false;
			}
		}

		// Start checking mag bias estimates when we have accumulated sufficient calibration time
		if (_total_cal_time_us > 120 * 1000 * 1000ULL) {
			// we have sufficient accumulated valid flight time to form a reliable bias estimate
			// check that the state variance for each axis is within a range indicating filter convergence
			const float max_var_allowed = 100.0f * _mag_bias_saved_variance.get();
			const float min_var_allowed = 0.01f * _mag_bias_saved_variance.get();

			// Declare all bias estimates invalid if any variances are out of range
			bool all_estimates_invalid = false;

			for (uint8_t axis_index = 0; axis_index <= 2; axis_index++) {
				if (status.covariances[axis_index + 19] < min_var_allowed
				    || status.covariances[axis_index + 19] > max_var_allowed) {
					all_estimates_invalid = true;
				}
			}

			// Store valid estimates and their associated variances
			if (!all_estimates_invalid) {
				for (uint8_t axis_index = 0; axis_index <= 2; axis_index++) {
					_last_valid_mag_cal[axis_index] = status.states[axis_index + 19];
					_valid_cal_available[axis_index] = true;
					_last_valid_variance[axis_index] = status.covariances[axis_index + 19];
				}
			}
		}

		// Check and save the last valid calibration when we are disarmed
		if ((vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_STANDBY)
		    && (status.filter_fault_flags == 0)
		    && (sensor_selection.mag_device_id == _mag_bias_id.get())) {

			BlockParamFloat *mag_biases[] = { &_mag_bias_x, &_mag_bias_y, &_mag_bias_z };

			for (uint8_t axis_index = 0; axis_index <= 2; axis_index++) {
				if (_valid_cal_available[axis_index]) {

					// calculate weighting using ratio of variances and update stored bias values
					const float weighting = constrain(_mag_bias_saved_variance.get() / (_mag_bias_saved_variance.get() +
									  _last_valid_variance[axis_index]), 0.0f, _mag_bias_alpha.get());
					const float mag_bias_saved = mag_biases[axis_index]->get();

					_last_valid_mag_cal[axis_index] = weighting * _last_valid_mag_cal[axis_index] + mag_bias_saved;

					mag_biases[axis_index]->set(_last_valid_mag_cal[axis_index]);
					mag_biases[axis_index]->commit_no_notification();

					_valid_cal_available[axis_index] = false;
				}
			}

			// reset to prevent data being saved too frequently
			_total_cal_time_us = 0;
		}

		{
			// Velocity of body origin in local NED frame (m/s)
			float velocity[3];
			_ukf.get_velocity(velocity);

			matrix::Quatf q;
			_ukf.copy_quaternion(q.data());

			// Calculate wind-compensated velocity in body frame
			Vector3f v_wind_comp(velocity);
			matrix::Dcmf R_to_body(q.inversed());

			float velNE_wind[2];
			_ukf.get_wind_velocity(velNE_wind);

			v_wind_comp(0) -= velNE_wind[0];
			v_wind_comp(1) -= velNE_wind[1];
			_vel_body_wind = R_to_body * v_wind_comp; // TODO: move this elsewhere

			// Publish wind estimate
			wind_estimate_s wind_estimate;
			wind_estimate.timestamp = now;
			wind_estimate.windspeed_north = velNE_wind[0];
			wind_estimate.windspeed_east = velNE_wind[1];
			wind_estimate.variance_north = status.covariances[22];
			wind_estimate.variance_east = status.covariances[23];

			if (_wind_pub == nullptr) {
				_wind_pub = orb_advertise(ORB_ID(wind_estimate), &wind_estimate);

			} else {
				orb_publish(ORB_ID(wind_estimate), _wind_pub, &wind_estimate);
			}
		}
	}

	{
		// publish estimator innovation data
		ekf2_innovations_s innovations;
		innovations.timestamp = now;
		_ukf.get_vel_pos_innov(&innovations.vel_pos_innov[0]);
		_ukf.get_aux_vel_innov(&innovations.aux_vel_innov[0]);
		_ukf.get_mag_innov(&innovations.mag_innov[0]);
		_ukf.get_heading_innov(&innovations.heading_innov);
		_ukf.get_airspeed_innov(&innovations.airspeed_innov);
		_ukf.get_beta_innov(&innovations.beta_innov);
		_ukf.get_flow_innov(&innovations.flow_innov[0]);
		_ukf.get_hagl_innov(&innovations.hagl_innov);
		_ukf.get_drag_innov(&innovations.drag_innov[0]);

		_ukf.get_vel_pos_innov_var(&innovations.vel_pos_innov_var[0]);
		_ukf.get_mag_innov_var(&innovations.mag_innov_var[0]);
		_ukf.get_heading_innov_var(&innovations.heading_innov_var);
		_ukf.get_airspeed_innov_var(&innovations.airspeed_innov_var);
		_ukf.get_beta_innov_var(&innovations.beta_innov_var);
		_ukf.get_flow_innov_var(&innovations.flow_innov_var[0]);
		_ukf.get_hagl_innov_var(&innovations.hagl_innov_var);
		_ukf.get_drag_innov_var(&innovations.drag_innov_var[0]);

		_ukf.get_output_tracking_error(&innovations.output_tracking_error[0]);

		// calculate noise filtered velocity innovations which are used for pre-flight checking
		if (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_STANDBY) {
			// calculate coefficients for LPF applied to innovation sequences
			float alpha = constrain(sensors.accelerometer_integral_dt / 1.e6f * _innov_lpf_tau_inv, 0.0f, 1.0f);
			float beta = 1.0f - alpha;

			// filter the velocity and innvovations
			_vel_ne_innov_lpf(0) = beta * _vel_ne_innov_lpf(0) + alpha * constrain(innovations.vel_pos_innov[0],
					       -_vel_innov_spike_lim, _vel_innov_spike_lim);
			_vel_ne_innov_lpf(1) = beta * _vel_ne_innov_lpf(1) + alpha * constrain(innovations.vel_pos_innov[1],
					       -_vel_innov_spike_lim, _vel_innov_spike_lim);
			_vel_d_innov_lpf = beta * _vel_d_innov_lpf + alpha * constrain(innovations.vel_pos_innov[2],
					   -_vel_innov_spike_lim, _vel_innov_spike_lim);

			// set the max allowed yaw innovaton depending on whether we are not aiding navigation using
			// observations in the NE reference frame.
			filter_control_status_u _ekf_control_mask;
			_ukf.get_control_mode(&_ekf_control_mask.value);
			bool doing_ne_aiding = _ekf_control_mask.flags.gps ||  _ekf_control_mask.flags.ev_pos;

			float yaw_test_limit;

			if (doing_ne_aiding) {
				// use a smaller tolerance when doing NE inertial frame aiding
				yaw_test_limit = _nav_yaw_innov_test_lim;

			} else {
				// use a larger tolerance when not doing NE inertial frame aiding
				yaw_test_limit = _yaw_innov_test_lim;
			}

			// filter the yaw innovations using a decaying envelope filter to prevent innovation sign changes due to angle wrapping allowinging large innvoations to pass checks after filtering.
			_yaw_innov_magnitude_lpf = fmaxf(beta * _yaw_innov_magnitude_lpf,
							 fminf(fabsf(innovations.heading_innov), 2.0f * yaw_test_limit));

			_hgt_innov_lpf = beta * _hgt_innov_lpf + alpha * constrain(innovations.vel_pos_innov[5], -_hgt_innov_spike_lim,
					 _hgt_innov_spike_lim);

			// check the yaw and horizontal velocity innovations
			float vel_ne_innov_length = sqrtf(innovations.vel_pos_innov[0] * innovations.vel_pos_innov[0] +
							  innovations.vel_pos_innov[1] * innovations.vel_pos_innov[1]);
			_preflt_horiz_fail = (_vel_ne_innov_lpf.norm() > _vel_innov_test_lim)
					     || (vel_ne_innov_length > 2.0f * _vel_innov_test_lim)
					     || (_yaw_innov_magnitude_lpf > yaw_test_limit);

			// check the vertical velocity and position innovations
			_preflt_vert_fail = (fabsf(_vel_d_innov_lpf) > _vel_innov_test_lim)
					    || (fabsf(innovations.vel_pos_innov[2]) > 2.0f * _vel_innov_test_lim)
					    || (fabsf(_hgt_innov_lpf) > _hgt_innov_test_lim);

			// master pass-fail status
			_preflt_fail = _preflt_horiz_fail || _preflt_vert_fail;

		} else {
			_vel_ne_innov_lpf.zero();
			_vel_d_innov_lpf = 0.0f;
			_hgt_innov_lpf = 0.0f;
			_preflt_horiz_fail = false;
			_preflt_vert_fail = false;
			_preflt_fail = false;
		}

		if (_estimator_innovations_pub == nullptr) {
			_estimator_innovations_pub = orb_advertise(ORB_ID(ekf2_innovations), &innovations);

		} else {
			orb_publish(ORB_ID(ekf2_innovations), _estimator_innovations_pub, &innovations);
		}
	}
