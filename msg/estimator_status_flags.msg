
# solution status (ekf_solution_status)
bool solution_attitude				# True if the attitude estimate is good
bool solution_velocity_horiz		# True if the horizontal velocity estimate is good
bool solution_velocity_vert			# True if the vertical velocity estimate is good
bool solution_pos_horiz_rel			# True if the horizontal position (relative) estimate is good
bool solution_pos_horiz_abs			# True if the horizontal position (absolute) estimate is good
bool solution_pos_vert_abs			# True if the vertical position (absolute) estimate is good
bool solution_pos_vert_agl			# True if the vertical position (above ground) estimate is good
bool solution_const_pos_mode		# True if the EKF is in a constant position mode and is not using external measurements (eg GPS or optical flow)
bool solution_pred_pos_horiz_rel	# True if the EKF has sufficient data to enter a mode that will provide a (relative) position estimate
bool solution_pred_pos_horiz_abs	# True if the EKF has sufficient data to enter a mode that will provide a (absolute) position estimate
bool solution_gps_glitch			# True if the EKF has detected a GPS glitch
bool solution_accel_error			# True if the EKF has detected bad accelerometer data


# GPS failure (gps_check_fail_status_u)
bool gps_fail_fix					# insufficient fix type (no 3d solution)
bool gps_fail_min_sat_count			# minimum required sat count fail
bool gps_fail_min_gdop				# minimum required gdop fail
bool gps_fail_max_horz_err			# maximum allowed horizontal position error fail
bool gps_fail_max_vert_err			# maximum allowed vertical position error fail
bool gps_fail_max_spd_err			# maximum allowed speed error fail
bool gps_fail_max_horz_drift		# maximum allowed horizontal position drift fail - requires stationary vehicle
bool gps_fail_max_vert_drift		# maximum allowed vertical position drift fail - requires stationary vehicle
bool gps_fail_max_horz_spd_err		# maximum allowed horizontal speed fail - requires stationary vehicle
bool gps_fail_max_vert_spd_err		# maximum allowed vertical velocity discrepancy fail


# control mode (filter_control_status_u)
bool control_mode_tilt_align		# true if the filter tilt alignment is complete
bool control_mode_yaw_align			# true if the filter yaw alignment is complete
bool control_mode_gps				# true if gps measurements are being fused
bool control_mode_opt_flow			# true if optical flow measurements are being fused
bool control_mode_mag_hdg			# true if a simple magnetic yaw heading is being fused
bool control_mode_mag_3d			# true if 3-axis magnetometer measurement are being fused
bool control_mode_mag_dec			# true if synthetic magnetic declination measurements are being fused
bool control_mode_in_air			# true when thought to be airborne
bool control_mode_wind				# true when wind velocity is being estimated
bool control_mode_baro_hgt			# true when baro height is being fused as a primary height reference
bool control_mode_rng_hgt			# true when range finder height is being fused as a primary height reference
bool control_mode_gps_hgt			# true when gps height is being fused as a primary height reference
bool control_mode_ev_pos			# true when local position data from external vision is being fused
bool control_mode_ev_yaw			# true when yaw data from external vision measurements is being fused
bool control_mode_ev_hgt			# true when height data from external vision measurements is being fused
bool control_mode_fuse_beta			# true when synthetic sideslip measurements are being fused
bool control_mode_mag_field			# true when only the magnetic field states are updated by the magnetometer
bool control_mode_fixed_wing		# true when thought to be operating as a fixed wing vehicle with constrained sideslip
bool control_mode_mag_fault			# true when the magnetomer has been declared faulty and is no longer being used
bool control_mode_fuse_airspeed		# true when airspeed measurements are being fused
bool control_mode_gnd_effect		# true when when protection from ground effect induced static pressure rise is active
bool control_mode_rng_stuck			# true when a stuck range finder sensor has been detected


# filter fault status (fault_status_u)
bool fault_bad_mag_x				# true if the fusion of the magnetometer X-axis has encountered a numerical error
bool fault_bad_mag_y				# true if the fusion of the magnetometer Y-axis has encountered a numerical error
bool fault_bad_mag_z				# true if the fusion of the magnetometer Z-axis has encountered a numerical error
bool fault_bad_mag_hdg				# true if the fusion of the magnetic heading has encountered a numerical error
bool fault_bad_mag_decl				# true if the fusion of the magnetic declination has encountered a numerical error
bool fault_bad_airspeed				# true if fusion of the airspeed has encountered a numerical error
bool fault_bad_sideslip				# true if fusion of the synthetic sideslip constraint has encountered a numerical error
bool fault_bad_optflow_X			# true if fusion of the optical flow X axis has encountered a numerical error
bool fault_bad_optflow_Y			# true if fusion of the optical flow Y axis has encountered a numerical error
bool fault_bad_vel_N				# true if fusion of the North velocity has encountered a numerical error
bool fault_bad_vel_E				# true if fusion of the East velocity has encountered a numerical error
bool fault_bad_vel_D				# true if fusion of the Down velocity has encountered a numerical error
bool fault_bad_pos_N				# true if fusion of the North position has encountered a numerical error
bool fault_bad_pos_E				# true if fusion of the East position has encountered a numerical error
bool fault_bad_pos_D				# true if fusion of the Down position has encountered a numerical error
bool fault_bad_acc_bias				# true if bad delta velocity bias estimates have been detected


# innovation fault status (innovation_fault_status_u)
bool reject_vel_NED					# true if velocity observations have been rejected
bool reject_pos_NE					# true if horizontal position observations have been rejected
bool reject_pos_D					# true if true if vertical position observations have been rejected
bool reject_mag_x					# true if the X magnetometer observation has been rejected
bool reject_mag_y					# true if the Y magnetometer observation has been rejected
bool reject_mag_z					# true if the Z magnetometer observation has been rejected
bool reject_yaw						# true if the yaw observation has been rejected
bool reject_airspeed				# true if the airspeed observation has been rejected
bool reject_sideslip				# true if the synthetic sideslip observation has been rejected
bool reject_hagl					# true if the height above ground observation has been rejected
bool reject_optflow_X				# true if the X optical flow observation has been rejected
bool reject_optflow_Y				# true if the Y optical flow observation has been rejected

