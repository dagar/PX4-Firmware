/****************************************************************************
 *
 *   Copyright (c) 2013-2017 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "MulticopterRateControl.hpp"

MulticopterRateControl *mc_rate_control::g_control;

MulticopterRateControl::MulticopterRateControl() :
	/* performance counters */
	_loop_perf(perf_alloc(PC_ELAPSED, "mc_rate_control")),
	_controller_latency_perf(perf_alloc_once(PC_ELAPSED, "ctrl_latency"))
{
	for (uint8_t i = 0; i < MAX_GYRO_COUNT; i++) {
		_sensor_gyro_sub[i] = -1;
	}

	_vehicle_status.is_rotary_wing = true;

	_params.rate_p.zero();
	_params.rate_i.zero();
	_params.rate_int_lim.zero();
	_params.rate_d.zero();
	_params.rate_ff.zero();

	_params.rattitude_thres = 1.0f;

	_params.bat_scale_en = 0;

	_params.board_rotation = 0;

	_params.board_offset[0] = 0.0f;
	_params.board_offset[1] = 0.0f;
	_params.board_offset[2] = 0.0f;

	_rates_prev.zero();
	_rates_sp.zero();
	_rates_int.zero();

	_thrust_sp = 0.0f;
	_att_control.zero();

	_I.identity();
	_board_rotation.identity();

	_params_handles.roll_rate_p		= 	param_find("MC_ROLLRATE_P");
	_params_handles.roll_rate_i		= 	param_find("MC_ROLLRATE_I");
	_params_handles.roll_rate_integ_lim	= 	param_find("MC_RR_INT_LIM");
	_params_handles.roll_rate_d		= 	param_find("MC_ROLLRATE_D");
	_params_handles.roll_rate_ff	= 	param_find("MC_ROLLRATE_FF");

	_params_handles.pitch_rate_p	= 	param_find("MC_PITCHRATE_P");
	_params_handles.pitch_rate_i	= 	param_find("MC_PITCHRATE_I");
	_params_handles.pitch_rate_integ_lim	= 	param_find("MC_PR_INT_LIM");
	_params_handles.pitch_rate_d	= 	param_find("MC_PITCHRATE_D");
	_params_handles.pitch_rate_ff 	= 	param_find("MC_PITCHRATE_FF");

	_params_handles.tpa_breakpoint_p 	= 	param_find("MC_TPA_BREAK_P");
	_params_handles.tpa_breakpoint_i 	= 	param_find("MC_TPA_BREAK_I");
	_params_handles.tpa_breakpoint_d 	= 	param_find("MC_TPA_BREAK_D");
	_params_handles.tpa_rate_p	 	= 	param_find("MC_TPA_RATE_P");
	_params_handles.tpa_rate_i	 	= 	param_find("MC_TPA_RATE_I");
	_params_handles.tpa_rate_d	 	= 	param_find("MC_TPA_RATE_D");

	_params_handles.yaw_rate_p		= 	param_find("MC_YAWRATE_P");
	_params_handles.yaw_rate_i		= 	param_find("MC_YAWRATE_I");
	_params_handles.yaw_rate_integ_lim	= 	param_find("MC_YR_INT_LIM");
	_params_handles.yaw_rate_d		= 	param_find("MC_YAWRATE_D");
	_params_handles.yaw_rate_ff	 	= 	param_find("MC_YAWRATE_FF");

	_params_handles.acro_roll_max	= 	param_find("MC_ACRO_R_MAX");
	_params_handles.acro_pitch_max	= 	param_find("MC_ACRO_P_MAX");
	_params_handles.acro_yaw_max	= 	param_find("MC_ACRO_Y_MAX");

	_params_handles.rattitude_thres = 	param_find("MC_RATT_TH");

	_params_handles.bat_scale_en		= param_find("MC_BAT_SCALE_EN");

	/* rotations */
	_params_handles.board_rotation = param_find("SENS_BOARD_ROT");

	/* rotation offsets */
	_params_handles.board_offset[0] = param_find("SENS_BOARD_X_OFF");
	_params_handles.board_offset[1] = param_find("SENS_BOARD_Y_OFF");
	_params_handles.board_offset[2] = param_find("SENS_BOARD_Z_OFF");

	/* fetch initial parameter values */
	parameters_update();

	/* initialize thermal corrections as we might not immediately get a topic update (only non-zero values) */
	for (unsigned i = 0; i < 3; i++) {
		// used scale factors to unity
		_sensor_correction.gyro_scale_0[i] = 1.0f;
		_sensor_correction.gyro_scale_1[i] = 1.0f;
		_sensor_correction.gyro_scale_2[i] = 1.0f;
	}
}

MulticopterRateControl::~MulticopterRateControl()
{
	if (_control_task != -1) {
		/* task wakes up every 100ms or so at the longest */
		_task_should_exit = true;

		/* wait for a second for the task to quit at our request */
		unsigned i = 0;

		do {
			/* wait 20ms */
			usleep(20000);

			/* if we have given up, kill it */
			if (++i > 50) {
				px4_task_delete(_control_task);
				break;
			}
		} while (_control_task != -1);
	}

	mc_rate_control::g_control = nullptr;
}

int
MulticopterRateControl::parameters_update()
{
	float v;

	/* roll gains */
	param_get(_params_handles.roll_rate_p, &v);
	_params.rate_p(0) = v;

	param_get(_params_handles.roll_rate_i, &v);
	_params.rate_i(0) = v;

	param_get(_params_handles.roll_rate_integ_lim, &v);
	_params.rate_int_lim(0) = v;

	param_get(_params_handles.roll_rate_d, &v);
	_params.rate_d(0) = v;

	param_get(_params_handles.roll_rate_ff, &v);
	_params.rate_ff(0) = v;

	/* pitch gains */
	param_get(_params_handles.pitch_rate_p, &v);
	_params.rate_p(1) = v;

	param_get(_params_handles.pitch_rate_i, &v);
	_params.rate_i(1) = v;

	param_get(_params_handles.pitch_rate_integ_lim, &v);
	_params.rate_int_lim(1) = v;

	param_get(_params_handles.pitch_rate_d, &v);
	_params.rate_d(1) = v;

	param_get(_params_handles.pitch_rate_ff, &v);
	_params.rate_ff(1) = v;

	param_get(_params_handles.tpa_breakpoint_p, &_params.tpa_breakpoint_p);
	param_get(_params_handles.tpa_breakpoint_i, &_params.tpa_breakpoint_i);
	param_get(_params_handles.tpa_breakpoint_d, &_params.tpa_breakpoint_d);
	param_get(_params_handles.tpa_rate_p, &_params.tpa_rate_p);
	param_get(_params_handles.tpa_rate_i, &_params.tpa_rate_i);
	param_get(_params_handles.tpa_rate_d, &_params.tpa_rate_d);

	/* yaw gains */
	param_get(_params_handles.yaw_rate_p, &v);
	_params.rate_p(2) = v;
	param_get(_params_handles.yaw_rate_i, &v);
	_params.rate_i(2) = v;
	param_get(_params_handles.yaw_rate_integ_lim, &v);
	_params.rate_int_lim(2) = v;
	param_get(_params_handles.yaw_rate_d, &v);
	_params.rate_d(2) = v;
	param_get(_params_handles.yaw_rate_ff, &v);
	_params.rate_ff(2) = v;

	/* manual rate control scale and auto mode roll/pitch rate limits */
	param_get(_params_handles.acro_roll_max, &v);
	_params.acro_rate_max(0) = math::radians(v);
	param_get(_params_handles.acro_pitch_max, &v);
	_params.acro_rate_max(1) = math::radians(v);
	param_get(_params_handles.acro_yaw_max, &v);
	_params.acro_rate_max(2) = math::radians(v);

	/* stick deflection needed in rattitude mode to control rates not angles */
	param_get(_params_handles.rattitude_thres, &_params.rattitude_thres);

	param_get(_params_handles.bat_scale_en, &_params.bat_scale_en);

	_actuators_0_circuit_breaker_enabled = circuit_breaker_enabled("CBRK_RATE_CTRL", CBRK_RATE_CTRL_KEY);

	/* rotation of the autopilot relative to the body */
	param_get(_params_handles.board_rotation, &(_params.board_rotation));

	/* fine adjustment of the rotation */
	param_get(_params_handles.board_offset[0], &(_params.board_offset[0]));
	param_get(_params_handles.board_offset[1], &(_params.board_offset[1]));
	param_get(_params_handles.board_offset[2], &(_params.board_offset[2]));

	return OK;
}

void
MulticopterRateControl::parameter_update_poll()
{
	bool updated;

	/* Check if parameters have changed */
	orb_check(_params_sub, &updated);

	if (updated) {
		struct parameter_update_s param_update;
		orb_copy(ORB_ID(parameter_update), _params_sub, &param_update);
		parameters_update();
	}
}

void
MulticopterRateControl::vehicle_control_mode_poll()
{
	bool updated;

	/* Check if vehicle control mode has changed */
	orb_check(_v_control_mode_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_control_mode), _v_control_mode_sub, &_v_control_mode);
	}
}

void
MulticopterRateControl::vehicle_manual_poll()
{
	bool updated;

	/* get pilots inputs */
	orb_check(_manual_control_sp_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(manual_control_setpoint), _manual_control_sp_sub, &_manual_control_sp);
	}
}

void
MulticopterRateControl::vehicle_rates_setpoint_poll()
{
	/* check if there is a new setpoint */
	bool updated;
	orb_check(_v_rates_sp_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_rates_setpoint), _v_rates_sp_sub, &_v_rates_sp);
	}
}

void
MulticopterRateControl::vehicle_status_poll()
{
	/* check if there is new status information */
	bool vehicle_status_updated;
	orb_check(_vehicle_status_sub, &vehicle_status_updated);

	if (vehicle_status_updated) {
		orb_copy(ORB_ID(vehicle_status), _vehicle_status_sub, &_vehicle_status);

		/* set correct uORB ID, depending on if vehicle is VTOL or not */
		if (_rates_sp_id == nullptr) {
			if (_vehicle_status.is_vtol) {
				_rates_sp_id = ORB_ID(mc_virtual_rates_setpoint);
				_actuators_id = ORB_ID(actuator_controls_virtual_mc);

			} else {
				_rates_sp_id = ORB_ID(vehicle_rates_setpoint);
				_actuators_id = ORB_ID(actuator_controls_0);
			}
		}
	}
}

void
MulticopterRateControl::vehicle_motor_limits_poll()
{
	/* check if there is a new message */
	bool updated;
	orb_check(_motor_limits_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(multirotor_motor_limits), _motor_limits_sub, &_motor_limits);
		_saturation_status.value = _motor_limits.saturation_status;
	}
}

void
MulticopterRateControl::battery_status_poll()
{
	/* check if there is a new message */
	bool updated;
	orb_check(_battery_status_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(battery_status), _battery_status_sub, &_battery_status);
	}
}

void
MulticopterRateControl::control_state_poll()
{
	/* check if there is a new message */
	bool updated;
	orb_check(_ctrl_state_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(control_state), _ctrl_state_sub, &_ctrl_state);
	}
}

void
MulticopterRateControl::sensor_correction_poll()
{
	/* check if there is a new message */
	bool updated;
	orb_check(_sensor_correction_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(sensor_correction), _sensor_correction_sub, &_sensor_correction);
	}

	/* update the latest gyro selection */
	if (_sensor_correction.selected_gyro_instance < _gyro_count) {
		_selected_gyro = _sensor_correction.selected_gyro_instance;
	}
}

/*
 * Throttle PID attenuation
 * Function visualization available here https://www.desmos.com/calculator/gn4mfoddje
 * Input: 'tpa_breakpoint', 'tpa_rate', '_thrust_sp'
 * Output: 'pidAttenuationPerAxis' vector
 */
math::Vector<3>
MulticopterRateControl::pid_attenuations(float tpa_breakpoint, float tpa_rate)
{
	/* throttle pid attenuation factor */
	float tpa = 1.0f - tpa_rate * (fabsf(_v_rates_sp.thrust) - tpa_breakpoint) / (1.0f - tpa_breakpoint);
	tpa = fmaxf(TPA_RATE_LOWER_LIMIT, fminf(1.0f, tpa));

	math::Vector<3> pidAttenuationPerAxis;
	pidAttenuationPerAxis(AXIS_INDEX_ROLL) = tpa;
	pidAttenuationPerAxis(AXIS_INDEX_PITCH) = tpa;
	pidAttenuationPerAxis(AXIS_INDEX_YAW) = 1.0;

	return pidAttenuationPerAxis;
}

/*
 * Attitude rates controller.
 * Input: '_rates_sp' vector, '_thrust_sp'
 * Output: '_att_control' vector
 */
void
MulticopterRateControl::control_attitude_rates(float dt)
{
	// weather-vane mode, prevent integrator winding up in weathervane mode
	if ((_v_control_mode.flag_control_velocity_enabled || _v_control_mode.flag_control_auto_enabled) &&
	    _v_rates_sp.disable_mc_yaw_control && !_v_control_mode.flag_control_manual_enabled) {

		_rates_int(2) = 0.0f;
	}

	/* reset integral if disarmed */
	if (!_v_control_mode.flag_armed || !_vehicle_status.is_rotary_wing) {
		_rates_int.zero();
	}

	/* get transformation matrix from sensor/board to body frame */
	get_rot_matrix((enum Rotation)_params.board_rotation, &_board_rotation);

	/* fine tune the rotation */
	math::Matrix<3, 3> board_rotation_offset;
	board_rotation_offset.from_euler(M_DEG_TO_RAD_F * _params.board_offset[0],
					 M_DEG_TO_RAD_F * _params.board_offset[1],
					 M_DEG_TO_RAD_F * _params.board_offset[2]);
	_board_rotation = board_rotation_offset * _board_rotation;

	// get the raw gyro data and correct for thermal errors
	math::Vector<3> rates;

	if (_selected_gyro == 0) {
		rates(0) = (_sensor_gyro.x - _sensor_correction.gyro_offset_0[0]) * _sensor_correction.gyro_scale_0[0];
		rates(1) = (_sensor_gyro.y - _sensor_correction.gyro_offset_0[1]) * _sensor_correction.gyro_scale_0[1];
		rates(2) = (_sensor_gyro.z - _sensor_correction.gyro_offset_0[2]) * _sensor_correction.gyro_scale_0[2];

	} else if (_selected_gyro == 1) {
		rates(0) = (_sensor_gyro.x - _sensor_correction.gyro_offset_1[0]) * _sensor_correction.gyro_scale_1[0];
		rates(1) = (_sensor_gyro.y - _sensor_correction.gyro_offset_1[1]) * _sensor_correction.gyro_scale_1[1];
		rates(2) = (_sensor_gyro.z - _sensor_correction.gyro_offset_1[2]) * _sensor_correction.gyro_scale_1[2];

	} else if (_selected_gyro == 2) {
		rates(0) = (_sensor_gyro.x - _sensor_correction.gyro_offset_2[0]) * _sensor_correction.gyro_scale_2[0];
		rates(1) = (_sensor_gyro.y - _sensor_correction.gyro_offset_2[1]) * _sensor_correction.gyro_scale_2[1];
		rates(2) = (_sensor_gyro.z - _sensor_correction.gyro_offset_2[2]) * _sensor_correction.gyro_scale_2[2];

	} else {
		rates(0) = _sensor_gyro.x;
		rates(1) = _sensor_gyro.y;
		rates(2) = _sensor_gyro.z;
	}

	// rotate corrected measurements from sensor to body frame
	rates = _board_rotation * rates;

	// correct for in-run bias errors
	rates(0) -= _ctrl_state.roll_rate_bias;
	rates(1) -= _ctrl_state.pitch_rate_bias;
	rates(2) -= _ctrl_state.yaw_rate_bias;

	math::Vector<3> rates_p_scaled = _params.rate_p.emult(pid_attenuations(_params.tpa_breakpoint_p, _params.tpa_rate_p));
	//math::Vector<3> rates_i_scaled = _params.rate_i.emult(pid_attenuations(_params.tpa_breakpoint_i, _params.tpa_rate_i));
	math::Vector<3> rates_d_scaled = _params.rate_d.emult(pid_attenuations(_params.tpa_breakpoint_d, _params.tpa_rate_d));

	/* angular rates error */
	math::Vector<3> rates_err = _rates_sp - rates;

	_att_control = rates_p_scaled.emult(rates_err) +
		       _rates_int +
		       rates_d_scaled.emult(_rates_prev - rates) / dt +
		       _params.rate_ff.emult(_rates_sp);

	_rates_prev = rates;

	/* update integral only if motors are providing enough thrust to be effective */
	if (_thrust_sp > MIN_TAKEOFF_THRUST) {
		for (int i = AXIS_INDEX_ROLL; i < AXIS_COUNT; i++) {
			// Check for positive control saturation
			bool positive_saturation =
				((i == AXIS_INDEX_ROLL) && _saturation_status.flags.roll_pos) ||
				((i == AXIS_INDEX_PITCH) && _saturation_status.flags.pitch_pos) ||
				((i == AXIS_INDEX_YAW) && _saturation_status.flags.yaw_pos);

			// Check for negative control saturation
			bool negative_saturation =
				((i == AXIS_INDEX_ROLL) && _saturation_status.flags.roll_neg) ||
				((i == AXIS_INDEX_PITCH) && _saturation_status.flags.pitch_neg) ||
				((i == AXIS_INDEX_YAW) && _saturation_status.flags.yaw_neg);

			// prevent further positive control saturation
			if (positive_saturation) {
				rates_err(i) = math::min(rates_err(i), 0.0f);
			}

			// prevent further negative control saturation
			if (negative_saturation) {
				rates_err(i) = math::max(rates_err(i), 0.0f);
			}

			// Perform the integration using a first order method and do not propaate the result if out of range or invalid
			float rate_i = _rates_int(i) + _params.rate_i(i) * rates_err(i) * dt;

			if (PX4_ISFINITE(rate_i) && rate_i > -_params.rate_int_lim(i) && rate_i < _params.rate_int_lim(i)) {
				_rates_int(i) = rate_i;
			}
		}
	}

	/* explicitly limit the integrator state */
	for (int i = AXIS_INDEX_ROLL; i < AXIS_COUNT; i++) {
		_rates_int(i) = math::constrain(_rates_int(i), -_params.rate_int_lim(i), _params.rate_int_lim(i));
	}
}

void
MulticopterRateControl::task_main_trampoline(int argc, char *argv[])
{
	mc_rate_control::g_control->task_main();
}

void
MulticopterRateControl::task_main()
{
	_v_rates_sp_sub = orb_subscribe(ORB_ID(vehicle_rates_setpoint));
	_ctrl_state_sub = orb_subscribe(ORB_ID(control_state));
	_v_control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	_params_sub = orb_subscribe(ORB_ID(parameter_update));
	_manual_control_sp_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	_vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
	_motor_limits_sub = orb_subscribe(ORB_ID(multirotor_motor_limits));
	_battery_status_sub = orb_subscribe(ORB_ID(battery_status));

	_gyro_count = math::min(orb_group_count(ORB_ID(sensor_gyro)), MAX_GYRO_COUNT);

	if (_gyro_count == 0) {
		_gyro_count = 1;
	}

	for (unsigned s = 0; s < _gyro_count; s++) {
		_sensor_gyro_sub[s] = orb_subscribe_multi(ORB_ID(sensor_gyro), s);
	}

	_sensor_correction_sub = orb_subscribe(ORB_ID(sensor_correction));

	/* initialize parameters cache */
	parameters_update();

	/* wakeup source: gyro data from sensor selected by the sensor app */
	px4_pollfd_struct_t poll_fds = {};
	poll_fds.events = POLLIN;

	while (!_task_should_exit) {
		poll_fds.fd = _sensor_gyro_sub[_selected_gyro];

		/* wait for up to 100ms for data */
		int pret = px4_poll(&poll_fds, 1, 100);

		/* timed out - periodic check for _task_should_exit */
		if (pret == 0) {
			continue;
		}

		/* this is undesirable but not much we can do - might want to flag unhappy status */
		if (pret < 0) {
			PX4_WARN("poll error %d, %d", pret, errno);
			/* sleep a bit before next try */
			usleep(1000);
			continue;
		}

		perf_begin(_loop_perf);

		/* check for updates in other topics */
		control_state_poll();
		parameter_update_poll();
		vehicle_control_mode_poll();
		vehicle_manual_poll();
		vehicle_status_poll();
		vehicle_motor_limits_poll();
		battery_status_poll();
		sensor_correction_poll();

		/* run controller on gyro changes */
		if (poll_fds.revents & POLLIN) {
			static uint64_t last_run = 0;
			float dt = (hrt_absolute_time() - last_run) / 1000000.0f;
			last_run = hrt_absolute_time();

			/* guard against too small (< 2ms) and too large (> 20ms) dt's */
			if (dt < 0.002f) {
				dt = 0.002f;

			} else if (dt > 0.02f) {
				dt = 0.02f;
			}

			/* copy gyro data */
			orb_copy(ORB_ID(sensor_gyro), _sensor_gyro_sub[_selected_gyro], &_sensor_gyro);

			/* Check if we are in rattitude mode and the pilot is above the threshold on pitch
			 * or roll (yaw can rotate 360 in normal att control). If both are true don't
			 * even bother running the attitude controllers */
			if (_v_control_mode.flag_control_rattitude_enabled) {
				if (fabsf(_manual_control_sp.y) > _params.rattitude_thres ||
				    fabsf(_manual_control_sp.x) > _params.rattitude_thres) {
					_v_control_mode.flag_control_manual_enabled = true;
				}
			}

			if (_v_control_mode.flag_control_manual_enabled && !_v_control_mode.flag_control_attitude_enabled) {
				/* manual rates control - ACRO mode */
				_rates_sp = math::Vector<3>(_manual_control_sp.y, -_manual_control_sp.x,
							    _manual_control_sp.r).emult(_params.acro_rate_max);

				_thrust_sp = math::min(_manual_control_sp.z, MANUAL_THROTTLE_MAX_MULTICOPTER);

				/* publish attitude rates setpoint */
				_v_rates_sp.roll = _rates_sp(0);
				_v_rates_sp.pitch = _rates_sp(1);
				_v_rates_sp.yaw = _rates_sp(2);
				_v_rates_sp.thrust = _thrust_sp;
				_v_rates_sp.timestamp = hrt_absolute_time();

				if (_v_rates_sp_pub != nullptr) {
					orb_publish(_rates_sp_id, _v_rates_sp_pub, &_v_rates_sp);

				} else if (_rates_sp_id) {
					_v_rates_sp_pub = orb_advertise(_rates_sp_id, &_v_rates_sp);
				}

			} else {
				/* attitude controller disabled, poll rates setpoint topic */
				vehicle_rates_setpoint_poll();
				_rates_sp(0) = _v_rates_sp.roll;
				_rates_sp(1) = _v_rates_sp.pitch;
				_rates_sp(2) = _v_rates_sp.yaw;
				_thrust_sp = _v_rates_sp.thrust;
			}

			control_attitude_rates(dt);

			/* publish actuator controls */
			actuator_controls_s actuators = {};
			actuators.timestamp = hrt_absolute_time();
			actuators.timestamp_sample = _sensor_gyro.timestamp;
			actuators.control[0] = (PX4_ISFINITE(_att_control(0))) ? _att_control(0) : 0.0f;
			actuators.control[1] = (PX4_ISFINITE(_att_control(1))) ? _att_control(1) : 0.0f;
			actuators.control[2] = (PX4_ISFINITE(_att_control(2))) ? _att_control(2) : 0.0f;
			actuators.control[3] = (PX4_ISFINITE(_thrust_sp)) ? _thrust_sp : 0.0f;
			actuators.control[7] = _v_rates_sp.landing_gear;

			/* scale effort by battery status */
			if (_params.bat_scale_en && _battery_status.scale > 0.0f) {
				for (int i = 0; i < 4; i++) {
					actuators.control[i] *= _battery_status.scale;
				}
			}

			if (_v_control_mode.flag_control_termination_enabled) {
				if (!_vehicle_status.is_vtol) {
					_rates_sp.zero();
					_rates_int.zero();
					_thrust_sp = 0.0f;
					_att_control.zero();

					/* publish actuator controls */
					actuators.control[0] = 0.0f;
					actuators.control[1] = 0.0f;
					actuators.control[2] = 0.0f;
					actuators.control[3] = 0.0f;
				}
			}

			if (_v_control_mode.flag_control_rates_enabled && !_actuators_0_circuit_breaker_enabled) {
				if (_actuators_id) {
					int inst;
					orb_publish_auto(_actuators_id, &_actuators_0_pub, &actuators, &inst, ORB_PRIO_HIGH);

					perf_end(_controller_latency_perf);
				}
			}

			_controller_status.roll_rate_integ = _rates_int(0);
			_controller_status.pitch_rate_integ = _rates_int(1);
			_controller_status.yaw_rate_integ = _rates_int(2);
			_controller_status.timestamp = hrt_absolute_time();

			/* publish controller status */
			if (_controller_status_pub != nullptr) {
				orb_publish(ORB_ID(mc_att_ctrl_status), _controller_status_pub, &_controller_status);

			} else {
				_controller_status_pub = orb_advertise(ORB_ID(mc_att_ctrl_status), &_controller_status);
			}
		}

		perf_end(_loop_perf);
	}

	_control_task = -1;
}

int
MulticopterRateControl::start()
{
	ASSERT(_control_task == -1);

	/* start the task */
	_control_task = px4_task_spawn_cmd("mc_rate_control",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_RATE_CONTROL,
					   1210,
					   (px4_main_t)&MulticopterRateControl::task_main_trampoline,
					   nullptr);

	if (_control_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}
