/****************************************************************************
 *
 *   Copyright (c) 2012-2019 PX4 Development Team. All rights reserved.
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

#include "PX4IO.hpp"

namespace
{
PX4IO	*g_dev = nullptr;
}

PX4IO::PX4IO(device::Device *interface) :
	CDev("/dev/px4io"),
	OutputModuleInterface(MODULE_NAME, px4::wq_configurations::hp_default),
	_interface(interface)
{
	/* we need this potentially before it could be set in task_main */
	g_dev = this;

	/* Fetch initial flight termination circuit breaker state */
	_cb_flighttermination = circuit_breaker_enabled("CBRK_FLIGHTTERM", CBRK_FLIGHTTERM_KEY);

	_mixing_output.setAllMinValues(PWM_DEFAULT_MIN);
	_mixing_output.setAllMaxValues(PWM_DEFAULT_MAX);
}

PX4IO::~PX4IO()
{
	ScheduleClear();

	/* tell the task we want it to go away */
	_task_should_exit = true;

	/* spin waiting for the task to stop */
	for (unsigned i = 0; (i < 10) && (_task != -1); i++) {
		/* give it another 100ms */
		px4_usleep(100000);
	}

	/* clean up the alternate device node */
	if (_primary_pwm_device) {
		unregister_driver(PWM_OUTPUT0_DEVICE_PATH);
	}

	delete _interface;

	/* deallocate perfs */
	perf_free(_perf_update);
	perf_free(_perf_write);
	perf_free(_perf_sample_latency);

	g_dev = nullptr;
}

int PX4IO::detect()
{
	int ret;

	if (_task == -1) {

		/* do regular cdev init */
		ret = CDev::init();

		if (ret != OK) {
			return ret;
		}

		/* get some parameters */
		unsigned protocol = io_reg_get(PX4IO_PAGE_CONFIG, PX4IO_P_CONFIG_PROTOCOL_VERSION);

		if (protocol != PX4IO_PROTOCOL_VERSION) {
			if (protocol == _io_reg_get_error) {
				PX4_ERR("IO not installed");

			} else {
				PX4_ERR("IO version error");
				mavlink_log_emergency(&_mavlink_log_pub, "IO VERSION MISMATCH, PLEASE UPGRADE SOFTWARE!");
			}

			return -1;
		}
	}

	PX4_INFO("IO found");

	return 0;
}

int PX4IO::init(bool rc_handling_disabled, bool hitl_mode)
{
	_rc_handling_disabled = rc_handling_disabled;
	_hitl_mode = hitl_mode;
	return init();
}

int PX4IO::init()
{
	param_t sys_restart_param = param_find("SYS_RESTART_TYPE");
	int32_t sys_restart_val = DM_INIT_REASON_VOLATILE;

	if (sys_restart_param != PARAM_INVALID) {
		/* Indicate restart type is unknown */
		int32_t prev_val;
		param_get(sys_restart_param, &prev_val);

		if (prev_val != DM_INIT_REASON_POWER_ON) {
			param_set_no_notification(sys_restart_param, &sys_restart_val);
		}
	}

	/* do regular cdev init */
	int ret = CDev::init();

	if (ret != OK) {
		return ret;
	}

	/* get some parameters */
	unsigned protocol;
	hrt_abstime start_try_time = hrt_absolute_time();

	do {
		px4_usleep(2000);
		protocol = io_reg_get(PX4IO_PAGE_CONFIG, PX4IO_P_CONFIG_PROTOCOL_VERSION);
	} while (protocol == _io_reg_get_error && (hrt_elapsed_time(&start_try_time) < 700U * 1000U));

	/* if the error still persists after timing out, we give up */
	if (protocol == _io_reg_get_error) {
		mavlink_log_emergency(&_mavlink_log_pub, "Failed to communicate with IO, abort.");
		return -1;
	}

	if (protocol != PX4IO_PROTOCOL_VERSION) {
		mavlink_log_emergency(&_mavlink_log_pub, "IO protocol/firmware mismatch, abort.");
		return -1;
	}

	_hardware      = io_reg_get(PX4IO_PAGE_CONFIG, PX4IO_P_CONFIG_HARDWARE_VERSION);
	_max_actuators = io_reg_get(PX4IO_PAGE_CONFIG, PX4IO_P_CONFIG_ACTUATOR_COUNT);
	_max_rc_input  = io_reg_get(PX4IO_PAGE_CONFIG, PX4IO_P_CONFIG_RC_INPUT_COUNT);

	if ((_max_actuators < 1) || (_max_actuators > 16) ||
	    (_max_rc_input < 1)  || (_max_rc_input > 255)) {

		PX4_ERR("config read error");
		mavlink_log_emergency(&_mavlink_log_pub, "[IO] config read fail, abort.");

		// ask IO to reboot into bootloader as the failure may
		// be due to mismatched firmware versions and we want
		// the startup script to be able to load a new IO
		// firmware

		// If IO has already safety off it won't accept going into bootloader mode,
		// therefore we need to set safety on first.
		io_reg_set(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_FORCE_SAFETY_ON, PX4IO_FORCE_SAFETY_MAGIC);

		// Now the reboot into bootloader mode should succeed.
		io_reg_set(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_REBOOT_BL, PX4IO_REBOOT_BL_MAGIC);
		return -1;
	}

	if (_max_rc_input > input_rc_s::RC_INPUT_MAX_CHANNELS) {
		_max_rc_input = input_rc_s::RC_INPUT_MAX_CHANNELS;
	}

	param_get(param_find("RC_RSSI_PWM_CHAN"), &_rssi_pwm_chan);
	param_get(param_find("RC_RSSI_PWM_MAX"), &_rssi_pwm_max);
	param_get(param_find("RC_RSSI_PWM_MIN"), &_rssi_pwm_min);

	/*
	 * Check for IO flight state - if FMU was flagged to be in
	 * armed state, FMU is recovering from an in-air reset.
	 * Read back status and request the commander to arm
	 * in this case.
	 */

	uint16_t reg;

	/* get IO's last seen FMU state */
	ret = io_reg_get(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_ARMING, &reg, sizeof(reg));

	if (ret != OK) {
		return ret;
	}

	{
		/* dis-arm IO before touching anything */
		io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_ARMING,
			      PX4IO_P_SETUP_ARMING_FMU_ARMED |
			      PX4IO_P_SETUP_ARMING_ALWAYS_PWM_ENABLE |
			      PX4IO_P_SETUP_ARMING_LOCKDOWN, 0);

		/* Indicate restart type is power on */
		sys_restart_val = DM_INIT_REASON_POWER_ON;
		int32_t prev_val;
		param_get(sys_restart_param, &prev_val);

		if (prev_val != sys_restart_val) {
			param_set(sys_restart_param, &sys_restart_val);
		}

	}

	/* set safety to off if circuit breaker enabled */
	if (circuit_breaker_enabled("CBRK_IO_SAFETY", CBRK_IO_SAFETY_KEY)) {
		(void)io_reg_set(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_FORCE_SAFETY_OFF, PX4IO_FORCE_SAFETY_MAGIC);
	}

	/* try to claim the generic PWM output device node as well - it's OK if we fail at this */
	ret = register_driver(PWM_OUTPUT0_DEVICE_PATH, &fops, 0666, (void *)this);

	if (ret == OK) {
		PX4_INFO("default PWM output device");
		_primary_pwm_device = true;
	}

	// minimal update rate with no actuator controls
	ScheduleDelayed(20_ms);

	return OK;
}

void PX4IO::Run()
{
	if (should_exit()) {
		ScheduleClear();
		_mixing_output.unregister();

		/* tell the dtor that we are exiting */
		_task = -1;

		exit_and_cleanup();
		return;
	}

	/* lock against the ioctl handler */
	lock();

	_mixing_output.update();

	/* loop talking to IO */
	if (!_task_should_exit) {

		// schedule minimal update rate if there are no actuator controls
		ScheduleDelayed(20_ms);

		perf_begin(_perf_update);

		if (hrt_elapsed_time(&_poll_last) >= 20_ms) {
			/* run at 50 */
			_poll_last = hrt_absolute_time();

			/* pull status and alarms from IO */
			io_get_status();

			/* get raw R/C input from IO */
			io_publish_raw_rc();

			if (_t_actuator_armed.updated()) {
				io_set_arming_state();
			}
		}

		if (!_armed && (hrt_elapsed_time(&_orb_check_last) >= 200_ms)) {
			/* run at 5Hz */
			_orb_check_last = hrt_absolute_time();

			/* vehicle command */
			if (_t_vehicle_command.updated()) {
				vehicle_command_s cmd{};
				_t_vehicle_command.copy(&cmd);

				// Check for a DSM pairing command
				if (((unsigned int)cmd.command == vehicle_command_s::VEHICLE_CMD_START_RX_PAIR) && ((int)cmd.param1 == 0)) {
					dsm_bind_ioctl((int)cmd.param2);
				}
			}

			/*
			 * If parameters have changed, re-send RC mappings to IO
			 *
			 * XXX this may be a bit spammy
			 */

			// check for parameter updates
			if (_parameter_update_sub.updated() || _param_update_force) {
				// clear update
				parameter_update_s pupdate;
				_parameter_update_sub.copy(&pupdate);

				_param_update_force = false;

				/* Check if the IO safety circuit breaker has been updated */
				bool circuit_breaker_io_safety_enabled = circuit_breaker_enabled("CBRK_IO_SAFETY", CBRK_IO_SAFETY_KEY);
				/* Bypass IO safety switch logic by setting FORCE_SAFETY_OFF */
				(void)io_reg_set(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_FORCE_SAFETY_OFF, circuit_breaker_io_safety_enabled);

				/* Check if the flight termination circuit breaker has been updated */
				_cb_flighttermination = circuit_breaker_enabled("CBRK_FLIGHTTERM", CBRK_FLIGHTTERM_KEY);
				/* Tell IO that it can terminate the flight if FMU is not responding or if a failure has been reported by the FailureDetector logic */
				(void)io_reg_set(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_ENABLE_FLIGHTTERMINATION, !_cb_flighttermination);

				param_get(param_find("RC_RSSI_PWM_CHAN"), &_rssi_pwm_chan);
				param_get(param_find("RC_RSSI_PWM_MAX"), &_rssi_pwm_max);
				param_get(param_find("RC_RSSI_PWM_MIN"), &_rssi_pwm_min);

				param_t thermal_param = param_find("SENS_EN_THERMAL");

				if (thermal_param != PARAM_INVALID) {

					int32_t thermal_p;
					param_get(thermal_param, &thermal_p);

					if (thermal_p != _thermal_control || _param_update_force) {

						_thermal_control = thermal_p;
						/* set power management state for thermal */
						uint16_t tctrl;

						if (_thermal_control < 0) {
							tctrl = PX4IO_THERMAL_IGNORE;

						} else {
							tctrl = PX4IO_THERMAL_OFF;
						}

						io_reg_set(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_THERMAL, tctrl);
					}
				}

				/*
				 * Set invert mask for PWM outputs (does not apply to S.Bus)
				 */
				int16_t pwm_invert_mask = 0;

				for (unsigned i = 0; i < _max_actuators; i++) {
					char pname[16];
					int32_t ival;

					/* fill the channel reverse mask from parameters */
					sprintf(pname, "PWM_MAIN_REV%u", i + 1);
					param_t param_h = param_find(pname);

					if (param_h != PARAM_INVALID) {
						param_get(param_h, &ival);
						pwm_invert_mask |= ((int16_t)(ival != 0)) << i;
					}
				}

				(void)io_reg_set(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_PWM_REVERSE, pwm_invert_mask);

				// update trim values
				struct pwm_output_values pwm_values {};

				//ret = io_reg_get(PX4IO_PAGE_CONTROL_TRIM_PWM, 0, (uint16_t *)pwm_values.values, _max_actuators);

				for (unsigned i = 0; i < _max_actuators; i++) {
					char pname[16];
					float pval;

					/* fetch the trim values from parameters */
					sprintf(pname, "PWM_MAIN_TRIM%u", i + 1);
					param_t param_h = param_find(pname);

					if (param_h != PARAM_INVALID) {

						param_get(param_h, &pval);
						pwm_values.values[i] = (int16_t)(10000 * pval);
					}
				}

				/* S.BUS output */
				int sbus_mode;
				param_t parm_handle = param_find("PWM_SBUS_MODE");

				if (parm_handle != PARAM_INVALID) {
					param_get(parm_handle, &sbus_mode);

					if (sbus_mode == 1) {
						/* enable S.BUS 1 */
						(void)io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_FEATURES, 0, PX4IO_P_SETUP_FEATURES_SBUS1_OUT);

					} else if (sbus_mode == 2) {
						/* enable S.BUS 2 */
						(void)io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_FEATURES, 0, PX4IO_P_SETUP_FEATURES_SBUS2_OUT);

					} else {
						/* disable S.BUS */
						(void)io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_FEATURES,
								    (PX4IO_P_SETUP_FEATURES_SBUS1_OUT | PX4IO_P_SETUP_FEATURES_SBUS2_OUT), 0);
					}
				}
			}

		}

		perf_end(_perf_update);
	}

	unlock();
}

int PX4IO::io_set_arming_state()
{
	uint16_t set = 0;
	uint16_t clear = 0;

	actuator_armed_s armed;

	if (_t_actuator_armed.copy(&armed)) {
		_in_esc_calibration_mode = armed.in_esc_calibration_mode;

		if (armed.armed || _in_esc_calibration_mode) {
			set |= PX4IO_P_SETUP_ARMING_FMU_ARMED;

		} else {
			clear |= PX4IO_P_SETUP_ARMING_FMU_ARMED;
		}

		_armed = armed.armed;

		if (armed.prearmed) {
			set |= PX4IO_P_SETUP_ARMING_FMU_PREARMED;

		} else {
			clear |= PX4IO_P_SETUP_ARMING_FMU_PREARMED;
		}

		if ((armed.lockdown || armed.manual_lockdown) && !_lockdown_override) {
			set |= PX4IO_P_SETUP_ARMING_LOCKDOWN;
			_lockdown_override = true;

		} else if (!(armed.lockdown || armed.manual_lockdown) && _lockdown_override) {
			clear |= PX4IO_P_SETUP_ARMING_LOCKDOWN;
			_lockdown_override = false;
		}

		if (armed.force_failsafe) {
			set |= PX4IO_P_SETUP_ARMING_FORCE_FAILSAFE;

		} else {
			clear |= PX4IO_P_SETUP_ARMING_FORCE_FAILSAFE;
		}

		// XXX this is for future support in the commander
		// but can be removed if unneeded
		// if (armed.termination_failsafe) {
		// 	set |= PX4IO_P_SETUP_ARMING_TERMINATION_FAILSAFE;
		// } else {
		// 	clear |= PX4IO_P_SETUP_ARMING_TERMINATION_FAILSAFE;
		// }

		if (armed.ready_to_arm) {
			set |= PX4IO_P_SETUP_ARMING_IO_ARM_OK;

		} else {
			clear |= PX4IO_P_SETUP_ARMING_IO_ARM_OK;
		}
	}

	if (_last_written_arming_s != set || _last_written_arming_c != clear) {
		_last_written_arming_s = set;
		_last_written_arming_c = clear;
		return io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_ARMING, clear, set);
	}

	return 0;
}

int PX4IO::io_handle_status(uint16_t status)
{
	int ret = 1;
	/**
	 * WARNING: This section handles in-air resets.
	 */

	/* check for IO reset - force it back to armed if necessary */
	if (_status & PX4IO_P_STATUS_FLAGS_SAFETY_OFF && !(status & PX4IO_P_STATUS_FLAGS_SAFETY_OFF)
	    && !(status & PX4IO_P_STATUS_FLAGS_ARM_SYNC)) {
		/* set the arming flag */
		ret = io_reg_modify(PX4IO_PAGE_STATUS, PX4IO_P_STATUS_FLAGS, 0,
				    PX4IO_P_STATUS_FLAGS_SAFETY_OFF | PX4IO_P_STATUS_FLAGS_ARM_SYNC);

		/* set new status */
		_status = status;
		_status &= PX4IO_P_STATUS_FLAGS_SAFETY_OFF;

	} else if (!(_status & PX4IO_P_STATUS_FLAGS_ARM_SYNC)) {

		/* set the sync flag */
		ret = io_reg_modify(PX4IO_PAGE_STATUS, PX4IO_P_STATUS_FLAGS, 0, PX4IO_P_STATUS_FLAGS_ARM_SYNC);
		/* set new status */
		_status = status;

	} else {
		ret = 0;

		/* set new status */
		_status = status;
	}

	/**
	 * Get and handle the safety status
	 */
	const bool safety_off = status & PX4IO_P_STATUS_FLAGS_SAFETY_OFF;

	// publish immediately on change, otherwise at 1 Hz
	if ((hrt_elapsed_time(&_safety.timestamp) >= 1_s)
	    || (_safety.safety_off != safety_off)) {

		_safety.safety_switch_available = true;
		_safety.safety_off = safety_off;
		_safety.timestamp = hrt_absolute_time();

		_to_safety.publish(_safety);
	}

	return ret;
}

void PX4IO::dsm_bind_ioctl(int dsmMode)
{
	if (!(_status & PX4IO_P_STATUS_FLAGS_SAFETY_OFF)) {
		mavlink_log_info(&_mavlink_log_pub, "[IO] binding DSM%s RX", (dsmMode == 0) ? "2" : ((dsmMode == 1) ? "-X" : "-X8"));
		int ret = ioctl(nullptr, DSM_BIND_START,
				(dsmMode == 0) ? DSM2_BIND_PULSES : ((dsmMode == 1) ? DSMX_BIND_PULSES : DSMX8_BIND_PULSES));

		if (ret) {
			mavlink_log_critical(&_mavlink_log_pub, "binding failed.");
		}

	} else {
		mavlink_log_info(&_mavlink_log_pub, "[IO] safety off, bind request rejected");
	}
}

int PX4IO::io_get_status()
{
	/* get
	 * STATUS_FLAGS, STATUS_ALARMS, STATUS_VBATT, STATUS_IBATT,
	 * STATUS_VSERVO, STATUS_VRSSI
	 * in that order */
	uint16_t regs[6] {};
	int ret = io_reg_get(PX4IO_PAGE_STATUS, PX4IO_P_STATUS_FLAGS, &regs[0], sizeof(regs) / sizeof(regs[0]));

	if (ret != OK) {
		return ret;
	}

	const uint16_t STATUS_FLAGS  = regs[0];
	const uint16_t STATUS_ALARMS = regs[1];
	const uint16_t STATUS_VSERVO = regs[4];
	const uint16_t STATUS_VRSSI  = regs[5];

	io_handle_status(STATUS_FLAGS);

	const float rssi_v = STATUS_VRSSI * 0.001f; // voltage is scaled to mV

	if (_analog_rc_rssi_volt < 0.f) {
		_analog_rc_rssi_volt = rssi_v;
	}

	_analog_rc_rssi_volt = _analog_rc_rssi_volt * 0.99f + rssi_v * 0.01f;

	if (_analog_rc_rssi_volt > 2.5f) {
		_analog_rc_rssi_stable = true;
	}

	const uint16_t SETUP_ARMING = io_reg_get(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_ARMING);

	if ((hrt_elapsed_time(&_last_status_publish) >= 1_s)
	    || (_status != STATUS_FLAGS)
	    || (_alarms != STATUS_ALARMS)
	    || (_setup_arming != SETUP_ARMING)
	   ) {

		px4io_status_s status{};

		status.voltage_v = STATUS_VSERVO * 0.001f; // voltage is scaled to mV
		status.rssi_v = rssi_v;

		status.free_memory_bytes = io_reg_get(PX4IO_PAGE_STATUS, PX4IO_P_STATUS_FREEMEM);

		// PX4IO_P_STATUS_FLAGS
		status.status_outputs_armed   = STATUS_FLAGS & PX4IO_P_STATUS_FLAGS_OUTPUTS_ARMED;
		status.status_rc_ok           = STATUS_FLAGS & PX4IO_P_STATUS_FLAGS_RC_OK;
		status.status_rc_ppm          = STATUS_FLAGS & PX4IO_P_STATUS_FLAGS_RC_PPM;
		status.status_rc_dsm          = STATUS_FLAGS & PX4IO_P_STATUS_FLAGS_RC_DSM;
		status.status_rc_sbus         = STATUS_FLAGS & PX4IO_P_STATUS_FLAGS_RC_SBUS;
		status.status_fmu_ok          = STATUS_FLAGS & PX4IO_P_STATUS_FLAGS_FMU_OK;
		status.status_raw_pwm         = STATUS_FLAGS & PX4IO_P_STATUS_FLAGS_RAW_PWM;
		status.status_arm_sync        = STATUS_FLAGS & PX4IO_P_STATUS_FLAGS_ARM_SYNC;
		status.status_init_ok         = STATUS_FLAGS & PX4IO_P_STATUS_FLAGS_INIT_OK;
		status.status_failsafe        = STATUS_FLAGS & PX4IO_P_STATUS_FLAGS_FAILSAFE;
		status.status_safety_off      = STATUS_FLAGS & PX4IO_P_STATUS_FLAGS_SAFETY_OFF;
		status.status_fmu_initialized = STATUS_FLAGS & PX4IO_P_STATUS_FLAGS_FMU_INITIALIZED;
		status.status_rc_st24         = STATUS_FLAGS & PX4IO_P_STATUS_FLAGS_RC_ST24;
		status.status_rc_sumd         = STATUS_FLAGS & PX4IO_P_STATUS_FLAGS_RC_SUMD;

		// PX4IO_P_STATUS_ALARMS
		status.alarm_vbatt_low     = STATUS_ALARMS & PX4IO_P_STATUS_ALARMS_VBATT_LOW;
		status.alarm_temperature   = STATUS_ALARMS & PX4IO_P_STATUS_ALARMS_TEMPERATURE;
		status.alarm_servo_current = STATUS_ALARMS & PX4IO_P_STATUS_ALARMS_SERVO_CURRENT;
		status.alarm_acc_current   = STATUS_ALARMS & PX4IO_P_STATUS_ALARMS_ACC_CURRENT;
		status.alarm_fmu_lost      = STATUS_ALARMS & PX4IO_P_STATUS_ALARMS_FMU_LOST;
		status.alarm_rc_lost       = STATUS_ALARMS & PX4IO_P_STATUS_ALARMS_RC_LOST;
		status.alarm_pwm_error     = STATUS_ALARMS & PX4IO_P_STATUS_ALARMS_PWM_ERROR;
		status.alarm_vservo_fault  = STATUS_ALARMS & PX4IO_P_STATUS_ALARMS_VSERVO_FAULT;

		// PX4IO_P_SETUP_ARMING
		status.arming_io_arm_ok            = SETUP_ARMING & PX4IO_P_SETUP_ARMING_IO_ARM_OK;
		status.arming_fmu_armed            = SETUP_ARMING & PX4IO_P_SETUP_ARMING_FMU_ARMED;
		status.arming_fmu_prearmed         = SETUP_ARMING & PX4IO_P_SETUP_ARMING_FMU_PREARMED;
		status.arming_failsafe_custom      = SETUP_ARMING & PX4IO_P_SETUP_ARMING_FAILSAFE_CUSTOM;
		status.arming_always_pwm_enable    = SETUP_ARMING & PX4IO_P_SETUP_ARMING_ALWAYS_PWM_ENABLE;
		status.arming_lockdown             = SETUP_ARMING & PX4IO_P_SETUP_ARMING_LOCKDOWN;
		status.arming_force_failsafe       = SETUP_ARMING & PX4IO_P_SETUP_ARMING_FORCE_FAILSAFE;
		status.arming_termination_failsafe = SETUP_ARMING & PX4IO_P_SETUP_ARMING_TERMINATION_FAILSAFE;

		for (unsigned i = 0; i < _max_actuators; i++) {
			status.servos[i] = io_reg_get(PX4IO_PAGE_SERVOS, i);
		}

		uint16_t raw_inputs = io_reg_get(PX4IO_PAGE_RAW_RC_INPUT, PX4IO_P_RAW_RC_COUNT);

		for (unsigned i = 0; i < raw_inputs; i++) {
			status.raw_inputs[i] = io_reg_get(PX4IO_PAGE_RAW_RC_INPUT, PX4IO_P_RAW_RC_BASE + i);
		}

		status.timestamp = hrt_absolute_time();
		_px4io_status_pub.publish(status);

		_last_status_publish = status.timestamp;
	}

	_alarms = STATUS_ALARMS;
	_setup_arming = SETUP_ARMING;

	return ret;
}

int PX4IO::io_get_raw_rc_input(input_rc_s &input_rc)
{
	/* we don't have the status bits, so input_source has to be set elsewhere */
	input_rc.input_source = input_rc_s::RC_INPUT_SOURCE_UNKNOWN;

	const unsigned prolog = (PX4IO_P_RAW_RC_BASE - PX4IO_P_RAW_RC_COUNT);
	uint16_t regs[input_rc_s::RC_INPUT_MAX_CHANNELS + prolog];

	/*
	 * Read the channel count and the first 9 channels.
	 *
	 * This should be the common case (9 channel R/C control being a reasonable upper bound).
	 */
	int ret = io_reg_get(PX4IO_PAGE_RAW_RC_INPUT, PX4IO_P_RAW_RC_COUNT, &regs[0], prolog + 9);

	if (ret != OK) {
		return ret;
	}

	/*
	 * Get the channel count any any extra channels. This is no more expensive than reading the
	 * channel count once.
	 */
	uint32_t channel_count = regs[PX4IO_P_RAW_RC_COUNT];

	/* limit the channel count */
	if (channel_count > input_rc_s::RC_INPUT_MAX_CHANNELS) {
		channel_count = input_rc_s::RC_INPUT_MAX_CHANNELS;
	}

	_rc_chan_count = channel_count;

	input_rc.timestamp = hrt_absolute_time();

	input_rc.rc_ppm_frame_length = regs[PX4IO_P_RAW_RC_DATA];

	if (!_analog_rc_rssi_stable) {
		input_rc.rssi = regs[PX4IO_P_RAW_RC_NRSSI];

	} else {
		float rssi_analog = ((_analog_rc_rssi_volt - 0.2f) / 3.0f) * 100.0f;

		if (rssi_analog > 100.0f) {
			rssi_analog = 100.0f;
		}

		if (rssi_analog < 0.0f) {
			rssi_analog = 0.0f;
		}

		input_rc.rssi = rssi_analog;
	}

	input_rc.rc_failsafe = (regs[PX4IO_P_RAW_RC_FLAGS] & PX4IO_P_RAW_RC_FLAGS_FAILSAFE);
	input_rc.rc_lost = !(regs[PX4IO_P_RAW_RC_FLAGS] & PX4IO_P_RAW_RC_FLAGS_RC_OK);
	input_rc.rc_lost_frame_count = regs[PX4IO_P_RAW_LOST_FRAME_COUNT];
	input_rc.rc_total_frame_count = regs[PX4IO_P_RAW_FRAME_COUNT];
	input_rc.channel_count = channel_count;

	/* rc_lost has to be set before the call to this function */
	if (!input_rc.rc_lost && !input_rc.rc_failsafe) {
		_rc_last_valid = input_rc.timestamp;
	}

	input_rc.timestamp_last_signal = _rc_last_valid;

	/* FIELDS NOT SET HERE */
	/* input_rc.input_source is set after this call XXX we might want to mirror the flags in the RC struct */

	if (channel_count > 9) {
		ret = io_reg_get(PX4IO_PAGE_RAW_RC_INPUT, PX4IO_P_RAW_RC_BASE + 9, &regs[prolog + 9], channel_count - 9);

		if (ret != OK) {
			return ret;
		}
	}

	/* last thing set are the actual channel values as 16 bit values */
	for (unsigned i = 0; i < channel_count; i++) {
		input_rc.values[i] = regs[prolog + i];
	}

	/* zero the remaining fields */
	for (unsigned i = channel_count; i < (sizeof(input_rc.values) / sizeof(input_rc.values[0])); i++) {
		input_rc.values[i] = 0;
	}

	/* get RSSI from input channel */
	if (_rssi_pwm_chan > 0 && _rssi_pwm_chan <= input_rc_s::RC_INPUT_MAX_CHANNELS && _rssi_pwm_max - _rssi_pwm_min != 0) {
		int rssi = ((input_rc.values[_rssi_pwm_chan - 1] - _rssi_pwm_min) * 100) /
			   (_rssi_pwm_max - _rssi_pwm_min);
		rssi = rssi > 100 ? 100 : rssi;
		rssi = rssi < 0 ? 0 : rssi;
		input_rc.rssi = rssi;
	}

	return ret;
}

int PX4IO::io_publish_raw_rc()
{
	/* fetch values from IO */
	input_rc_s	rc_val;

	/* set the RC status flag ORDER MATTERS! */
	rc_val.rc_lost = !(_status & PX4IO_P_STATUS_FLAGS_RC_OK);

	int ret = io_get_raw_rc_input(rc_val);

	if (ret != OK) {
		return ret;
	}

	/* sort out the source of the values */
	if (_status & PX4IO_P_STATUS_FLAGS_RC_PPM) {
		rc_val.input_source = input_rc_s::RC_INPUT_SOURCE_PX4IO_PPM;

	} else if (_status & PX4IO_P_STATUS_FLAGS_RC_DSM) {
		rc_val.input_source = input_rc_s::RC_INPUT_SOURCE_PX4IO_SPEKTRUM;

	} else if (_status & PX4IO_P_STATUS_FLAGS_RC_SBUS) {
		rc_val.input_source = input_rc_s::RC_INPUT_SOURCE_PX4IO_SBUS;

	} else if (_status & PX4IO_P_STATUS_FLAGS_RC_ST24) {
		rc_val.input_source = input_rc_s::RC_INPUT_SOURCE_PX4IO_ST24;

	} else {
		rc_val.input_source = input_rc_s::RC_INPUT_SOURCE_UNKNOWN;

		/* only keep publishing RC input if we ever got a valid input */
		if (_rc_last_valid == 0) {
			/* we have never seen valid RC signals, abort */
			return OK;
		}
	}

	_to_input_rc.publish(rc_val);

	return OK;
}

int PX4IO::io_reg_set(uint8_t page, uint8_t offset, const uint16_t *values, unsigned num_values)
{
	int ret = _interface->write((page << 8) | offset, (void *)values, num_values);

	if (ret != (int)num_values) {
		PX4_DEBUG("io_reg_set(%hhu,%hhu,%u): error %d", page, offset, num_values, ret);
		return -1;
	}

	return OK;
}

int PX4IO::io_reg_get(uint8_t page, uint8_t offset, uint16_t *values, unsigned num_values)
{
	int ret = _interface->read((page << 8) | offset, reinterpret_cast<void *>(values), num_values);

	if (ret != (int)num_values) {
		PX4_DEBUG("io_reg_get(%hhu,%hhu,%u): data error %d", page, offset, num_values, ret);
		return -1;
	}

	return OK;
}

uint32_t PX4IO::io_reg_get(uint8_t page, uint8_t offset)
{
	uint16_t value;

	if (io_reg_get(page, offset, &value, 1) != OK) {
		return _io_reg_get_error;
	}

	return value;
}

int PX4IO::io_reg_modify(uint8_t page, uint8_t offset, uint16_t clearbits, uint16_t setbits)
{
	uint16_t value;

	int ret = io_reg_get(page, offset, &value, 1);

	if (ret != OK) {
		return ret;
	}

	value &= ~clearbits;
	value |= setbits;

	return io_reg_set(page, offset, value);
}

int PX4IO::print_debug()
{
#if defined(CONFIG_ARCH_BOARD_PX4_FMU_V2) || defined(CONFIG_ARCH_BOARD_PX4_FMU_V3) ||  defined(CONFIG_ARCH_BOARD_HEX_CUBE_ORANGE)
	int io_fd = -1;

	if (io_fd <= 0) {
		io_fd = ::open("/dev/ttyS0", O_RDONLY | O_NONBLOCK | O_NOCTTY);
	}

	/* read IO's output */
	if (io_fd >= 0) {
		pollfd fds[1];
		fds[0].fd = io_fd;
		fds[0].events = POLLIN;

		px4_usleep(500);
		int pret = ::poll(fds, sizeof(fds) / sizeof(fds[0]), 0);

		if (pret > 0) {
			int count;
			char buf[65];

			do {
				count = ::read(io_fd, buf, sizeof(buf) - 1);

				if (count > 0) {
					/* enforce null termination */
					buf[count] = '\0';
					warnx("IO CONSOLE: %s", buf);
				}

			} while (count > 0);
		}

		::close(io_fd);
		return 0;
	}

#endif
	return 1;

}

void PX4IO::print_status(bool extended_status)
{
	/* basic configuration */
	printf("protocol %u hardware %u bootloader %u crc 0x%04x%04x\n",
	       io_reg_get(PX4IO_PAGE_CONFIG, PX4IO_P_CONFIG_PROTOCOL_VERSION),
	       io_reg_get(PX4IO_PAGE_CONFIG, PX4IO_P_CONFIG_HARDWARE_VERSION),
	       io_reg_get(PX4IO_PAGE_CONFIG, PX4IO_P_CONFIG_BOOTLOADER_VERSION),
	       io_reg_get(PX4IO_PAGE_SETUP,  PX4IO_P_SETUP_CRC),
	       io_reg_get(PX4IO_PAGE_SETUP,  PX4IO_P_SETUP_CRC + 1));

	printf("%u actuators %u R/C inputs\n",
	       io_reg_get(PX4IO_PAGE_CONFIG, PX4IO_P_CONFIG_ACTUATOR_COUNT),
	       io_reg_get(PX4IO_PAGE_CONFIG, PX4IO_P_CONFIG_RC_INPUT_COUNT));

	/* status */
	uORB::SubscriptionData<px4io_status_s> status_sub{ORB_ID(px4io_status)};
	status_sub.update();

	print_message(status_sub.get());

	/* now clear alarms */
	io_reg_set(PX4IO_PAGE_STATUS, PX4IO_P_STATUS_ALARMS, 0x0000);

	uint16_t pwm_invert_mask = io_reg_get(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_PWM_REVERSE);

	printf("\n");
	printf("reversed outputs: [");

	for (unsigned i = 0; i < _max_actuators; i++) {
		printf("%s", (pwm_invert_mask & (1 << i)) ? "x" : "_");
	}

	printf("]");

	uint16_t raw_inputs = io_reg_get(PX4IO_PAGE_RAW_RC_INPUT, PX4IO_P_RAW_RC_COUNT);
	printf("%hu raw R/C inputs", raw_inputs);

	for (unsigned i = 0; i < raw_inputs; i++) {
		printf(" %u", io_reg_get(PX4IO_PAGE_RAW_RC_INPUT, PX4IO_P_RAW_RC_BASE + i));
	}

	printf("\n");

	uint16_t io_status_flags = io_reg_get(PX4IO_PAGE_STATUS, PX4IO_P_STATUS_FLAGS);
	uint16_t flags = io_reg_get(PX4IO_PAGE_RAW_RC_INPUT, PX4IO_P_RAW_RC_FLAGS);
	printf("R/C flags: 0x%04hx%s%s%s%s%s\n", flags,
	       (((io_status_flags & PX4IO_P_STATUS_FLAGS_RC_DSM) && (!(flags & PX4IO_P_RAW_RC_FLAGS_RC_DSM11))) ? " DSM10" : ""),
	       (((io_status_flags & PX4IO_P_STATUS_FLAGS_RC_DSM) && (flags & PX4IO_P_RAW_RC_FLAGS_RC_DSM11)) ? " DSM11" : ""),
	       ((flags & PX4IO_P_RAW_RC_FLAGS_FRAME_DROP) ? " FRAME_DROP" : ""),
	       ((flags & PX4IO_P_RAW_RC_FLAGS_FAILSAFE) ? " FAILSAFE" : "")
	      );

	if ((io_status_flags & PX4IO_P_STATUS_FLAGS_RC_PPM)) {
		int frame_len = io_reg_get(PX4IO_PAGE_RAW_RC_INPUT, PX4IO_P_RAW_RC_DATA);
		printf("RC data (PPM frame len) %d us\n", frame_len);

		if ((frame_len - raw_inputs * 2000 - 3000) < 0) {
			printf("WARNING  WARNING  WARNING! This RC receiver does not allow safe frame detection.\n");
		}
	}

	printf("\n");

	/* setup and state */
	uint16_t features = io_reg_get(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_FEATURES);
	printf("features 0x%04hx%s%s%s%s\n", features,
	       ((features & PX4IO_P_SETUP_FEATURES_SBUS1_OUT) ? " S.BUS1_OUT" : ""),
	       ((features & PX4IO_P_SETUP_FEATURES_SBUS2_OUT) ? " S.BUS2_OUT" : ""),
	       ((features & PX4IO_P_SETUP_FEATURES_PWM_RSSI) ? " RSSI_PWM" : ""),
	       ((features & PX4IO_P_SETUP_FEATURES_ADC_RSSI) ? " RSSI_ADC" : "")
	      );

	printf("rates 0x%04x default %u alt %u sbus %u\n",
	       io_reg_get(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_PWM_RATES),
	       io_reg_get(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_PWM_DEFAULTRATE),
	       io_reg_get(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_PWM_ALTRATE),
	       io_reg_get(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_SBUS_RATE));
	printf("debuglevel %u\n", io_reg_get(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_SET_DEBUG));

	printf("failsafe");

	for (unsigned i = 0; i < _max_actuators; i++) {
		printf(" %u", io_reg_get(PX4IO_PAGE_FAILSAFE_PWM, i));
	}

	printf("\ndisarmed values");

	for (unsigned i = 0; i < _max_actuators; i++) {
		printf(" %u", io_reg_get(PX4IO_PAGE_DISARMED_PWM, i));
	}

	/* IMU heater (Pixhawk 2.1) */
	uint16_t heater_level = io_reg_get(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_THERMAL);

	if (heater_level != UINT16_MAX) {
		if (heater_level == PX4IO_THERMAL_OFF) {
			printf("\nIMU heater off");

		} else {
			printf("\nIMU heater level %d", heater_level);
		}
	}

	if (_hitl_mode) {
		printf("\nHITL Mode");
	}

	printf("\n");
}

int PX4IO::ioctl(file *filep, int cmd, unsigned long arg)
{
	SmartLock lock_guard(_lock);
	int ret = OK;

	/* regular ioctl? */
	switch (cmd) {
	case PWM_SERVO_ARM:
		/* set the 'armed' bit */
		ret = io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_ARMING, 0, PX4IO_P_SETUP_ARMING_FMU_ARMED);
		break;

	case PWM_SERVO_SET_ARM_OK:
		/* set the 'OK to arm' bit */
		ret = io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_ARMING, 0, PX4IO_P_SETUP_ARMING_IO_ARM_OK);
		break;

	case PWM_SERVO_CLEAR_ARM_OK:
		/* clear the 'OK to arm' bit */
		ret = io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_ARMING, PX4IO_P_SETUP_ARMING_IO_ARM_OK, 0);
		break;

	case PWM_SERVO_DISARM:
		/* clear the 'armed' bit */
		ret = io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_ARMING, PX4IO_P_SETUP_ARMING_FMU_ARMED, 0);
		break;

	case PWM_SERVO_GET_DEFAULT_UPDATE_RATE:
		/* get the default update rate */
		*(unsigned *)arg = io_reg_get(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_PWM_DEFAULTRATE);
		break;

	case PWM_SERVO_SET_UPDATE_RATE:
		/* set the requested alternate rate */
		ret = io_reg_set(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_PWM_ALTRATE, arg);
		break;

	case PWM_SERVO_GET_UPDATE_RATE:
		/* get the alternative update rate */
		*(unsigned *)arg = io_reg_get(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_PWM_ALTRATE);
		break;

	case PWM_SERVO_SET_SELECT_UPDATE_RATE: {

			/* blindly clear the PWM update alarm - might be set for some other reason */
			io_reg_set(PX4IO_PAGE_STATUS, PX4IO_P_STATUS_ALARMS, PX4IO_P_STATUS_ALARMS_PWM_ERROR);

			/* attempt to set the rate map */
			io_reg_set(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_PWM_RATES, arg);

			/* check that the changes took */
			uint16_t alarms = io_reg_get(PX4IO_PAGE_STATUS, PX4IO_P_STATUS_ALARMS);

			if (alarms & PX4IO_P_STATUS_ALARMS_PWM_ERROR) {
				ret = -EINVAL;
				io_reg_set(PX4IO_PAGE_STATUS, PX4IO_P_STATUS_ALARMS, PX4IO_P_STATUS_ALARMS_PWM_ERROR);
			}

			break;
		}

	case PWM_SERVO_GET_SELECT_UPDATE_RATE:

		*(unsigned *)arg = io_reg_get(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_PWM_RATES);
		break;

	case PWM_SERVO_SET_FAILSAFE_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			if (pwm->channel_count > _max_actuators)
				/* fail with error */
			{
				return -E2BIG;
			}

			/* copy values to registers in IO */
			ret = io_reg_set(PX4IO_PAGE_FAILSAFE_PWM, 0, pwm->values, pwm->channel_count);
			break;
		}

	case PWM_SERVO_GET_FAILSAFE_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;
			pwm->channel_count = _max_actuators;

			ret = io_reg_get(PX4IO_PAGE_FAILSAFE_PWM, 0, pwm->values, _max_actuators);

			if (ret != OK) {
				ret = -EIO;
			}

			break;
		}

	case PWM_SERVO_SET_DISARMED_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			if (pwm->channel_count > _max_actuators)
				/* fail with error */
			{
				return -E2BIG;
			}

			/* copy values to registers in IO */
			ret = io_reg_set(PX4IO_PAGE_DISARMED_PWM, 0, pwm->values, pwm->channel_count);
			break;
		}

	case PWM_SERVO_GET_DISARMED_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;
			pwm->channel_count = _max_actuators;

			ret = io_reg_get(PX4IO_PAGE_DISARMED_PWM, 0, pwm->values, _max_actuators);

			if (ret != OK) {
				ret = -EIO;
			}

			break;
		}

	case PWM_SERVO_SET_MIN_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			if (pwm->channel_count > _max_actuators)
				/* fail with error */
			{
				return -E2BIG;
			}

			/* copy values to registers in IO */
			//ret = io_reg_set(PX4IO_PAGE_CONTROL_MIN_PWM, 0, pwm->values, pwm->channel_count);
			break;
		}

	case PWM_SERVO_GET_MIN_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;
			pwm->channel_count = _max_actuators;

			//ret = io_reg_get(PX4IO_PAGE_CONTROL_MIN_PWM, 0, pwm->values, _max_actuators);

			if (ret != OK) {
				ret = -EIO;
			}

			break;
		}

	case PWM_SERVO_SET_MAX_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			if (pwm->channel_count > _max_actuators)
				/* fail with error */
			{
				return -E2BIG;
			}

			/* copy values to registers in IO */
			// ret = io_reg_set(PX4IO_PAGE_CONTROL_MAX_PWM, 0, pwm->values, pwm->channel_count);
			break;
		}

	case PWM_SERVO_GET_MAX_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;
			pwm->channel_count = _max_actuators;

			//ret = io_reg_get(PX4IO_PAGE_CONTROL_MAX_PWM, 0, pwm->values, _max_actuators);

			if (ret != OK) {
				ret = -EIO;
			}
		}

		break;

	case PWM_SERVO_SET_TRIM_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			if (pwm->channel_count > _max_actuators)
				/* fail with error */
			{
				return -E2BIG;
			}

			/* copy values to registers in IO */
			// TODO
			break;
		}

	case PWM_SERVO_GET_TRIM_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;
			pwm->channel_count = _max_actuators;

			// TODO

			if (ret != OK) {
				ret = -EIO;
			}
		}

		break;

	case PWM_SERVO_GET_COUNT:
		*(unsigned *)arg = _max_actuators;
		break;

	case PWM_SERVO_SET_DISABLE_LOCKDOWN:
		_lockdown_override = arg;
		break;

	case PWM_SERVO_GET_DISABLE_LOCKDOWN:
		*(unsigned *)arg = _lockdown_override;
		break;

	case PWM_SERVO_SET_FORCE_SAFETY_OFF:
		/* force safety swith off */
		ret = io_reg_set(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_FORCE_SAFETY_OFF, PX4IO_FORCE_SAFETY_MAGIC);
		break;

	case PWM_SERVO_SET_FORCE_SAFETY_ON:
		/* force safety switch on */
		ret = io_reg_set(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_FORCE_SAFETY_ON, PX4IO_FORCE_SAFETY_MAGIC);
		break;

	case PWM_SERVO_SET_FORCE_FAILSAFE:

		/* force failsafe mode instantly */
		if (arg == 0) {
			/* clear force failsafe flag */
			ret = io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_ARMING, PX4IO_P_SETUP_ARMING_FORCE_FAILSAFE, 0);

		} else {
			/* set force failsafe flag */
			ret = io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_ARMING, 0, PX4IO_P_SETUP_ARMING_FORCE_FAILSAFE);
		}

		break;

	case PWM_SERVO_SET_TERMINATION_FAILSAFE:

		/* if failsafe occurs, do not allow the system to recover */
		if (arg == 0) {
			/* clear termination failsafe flag */
			ret = io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_ARMING, PX4IO_P_SETUP_ARMING_TERMINATION_FAILSAFE, 0);

		} else {
			/* set termination failsafe flag */
			ret = io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_ARMING, 0, PX4IO_P_SETUP_ARMING_TERMINATION_FAILSAFE);
		}

		break;

	case PWM_SERVO_SET_SBUS_RATE:
		/* set the requested SBUS frame rate */
		ret = io_reg_set(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_SBUS_RATE, arg);
		break;

	case DSM_BIND_START:

		/* only allow DSM2, DSM-X and DSM-X with more than 7 channels */
		if (arg == DSM2_BIND_PULSES ||
		    arg == DSMX_BIND_PULSES ||
		    arg == DSMX8_BIND_PULSES) {

			io_reg_set(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_DSM, dsm_bind_power_down);
			px4_usleep(500000);
			io_reg_set(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_DSM, dsm_bind_set_rx_out);
			io_reg_set(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_DSM, dsm_bind_power_up);
			px4_usleep(72000);
			io_reg_set(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_DSM, dsm_bind_send_pulses | (arg << 4));
			px4_usleep(50000);
			io_reg_set(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_DSM, dsm_bind_reinit_uart);

			ret = OK;

		} else {
			ret = -EINVAL;
		}

		break;

	case DSM_BIND_POWER_UP:
		io_reg_set(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_DSM, dsm_bind_power_up);
		break;

	case PWM_SERVO_SET(0) ... PWM_SERVO_SET(PWM_OUTPUT_MAX_CHANNELS - 1): {

			/* TODO: we could go lower for e.g. TurboPWM */
			unsigned channel = cmd - PWM_SERVO_SET(0);

			/* PWM needs to be either 0 or in the valid range. */
			if ((arg != 0) && ((channel >= _max_actuators) ||
					   (arg < PWM_LOWEST_MIN) ||
					   (arg > PWM_HIGHEST_MAX))) {
				ret = -EINVAL;

			} else {
				if (!_test_fmu_fail) {
					/* send a direct PWM value */
					ret = io_reg_set(PX4IO_PAGE_DIRECT_PWM, channel, arg);

				} else {
					/* Just silently accept the ioctl without doing anything
					 * in test mode. */
					ret = OK;
				}
			}

			break;
		}

	case PWM_SERVO_GET(0) ... PWM_SERVO_GET(PWM_OUTPUT_MAX_CHANNELS - 1): {

			unsigned channel = cmd - PWM_SERVO_GET(0);

			if (channel >= _max_actuators) {
				ret = -EINVAL;

			} else {
				/* fetch a current PWM value */
				uint32_t value = io_reg_get(PX4IO_PAGE_SERVOS, channel);

				if (value == _io_reg_get_error) {
					ret = -EIO;

				} else {
					*(servo_position_t *)arg = value;
				}
			}

			break;
		}

	case PWM_SERVO_GET_RATEGROUP(0) ... PWM_SERVO_GET_RATEGROUP(PWM_OUTPUT_MAX_CHANNELS - 1): {

			unsigned channel = cmd - PWM_SERVO_GET_RATEGROUP(0);

			*(uint32_t *)arg = io_reg_get(PX4IO_PAGE_PWM_INFO, PX4IO_RATE_MAP_BASE + channel);

			if (*(uint32_t *)arg == _io_reg_get_error) {
				ret = -EIO;
			}

			break;
		}

	case PWM_SERVO_SET_MODE: {
			// reset all channels to disarmed when entering/leaving test mode, so that we don't
			// accidentially use values from previous tests
			pwm_output_values pwm_disarmed;

			if (io_reg_get(PX4IO_PAGE_DISARMED_PWM, 0, pwm_disarmed.values, _max_actuators) == 0) {
				for (unsigned i = 0; i < _max_actuators; ++i) {
					io_reg_set(PX4IO_PAGE_DIRECT_PWM, i, pwm_disarmed.values[i]);
				}
			}

			ret = (arg == PWM_SERVO_ENTER_TEST_MODE || PWM_SERVO_EXIT_TEST_MODE) ? 0 : -EINVAL;
		}
		break;

	case MIXERIOCRESET:
		ret = 0;	/* load always resets */
		break;

	case MIXERIOCLOADBUF: {
			//const char *buf = (const char *)arg;
			//ret = mixer_send(buf, strlen(buf));
			// TODO:
			break;
		}

	case PX4IO_SET_DEBUG:
		/* set the debug level */
		ret = io_reg_set(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_SET_DEBUG, arg);
		break;

	case PX4IO_REBOOT_BOOTLOADER:
		if (system_status() & PX4IO_P_STATUS_FLAGS_SAFETY_OFF) {
			return -EINVAL;
		}

		/* reboot into bootloader - arg must be PX4IO_REBOOT_BL_MAGIC */
		usleep(1);
		io_reg_set(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_REBOOT_BL, arg);
		// we don't expect a reply from this operation
		ret = OK;
		break;

	case PX4IO_CHECK_CRC: {
			/* check IO firmware CRC against passed value */
			uint32_t io_crc = 0;
			ret = io_reg_get(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_CRC, (uint16_t *)&io_crc, 2);

			if (ret != OK) {
				return ret;
			}

			if (io_crc != arg) {
				PX4_DEBUG("crc mismatch 0x%08x 0x%08lx", io_crc, arg);
				return -EINVAL;
			}

			break;
		}

	case RC_INPUT_ENABLE_RSSI_ANALOG:

		if (arg) {
			ret = io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_FEATURES, 0, PX4IO_P_SETUP_FEATURES_ADC_RSSI);

		} else {
			ret = io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_FEATURES, PX4IO_P_SETUP_FEATURES_ADC_RSSI, 0);
		}

		break;

	case SBUS_SET_PROTO_VERSION:

		if (arg == 1) {
			ret = io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_FEATURES, 0, PX4IO_P_SETUP_FEATURES_SBUS1_OUT);

		} else if (arg == 2) {
			ret = io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_FEATURES, 0, PX4IO_P_SETUP_FEATURES_SBUS2_OUT);

		} else {
			ret = io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_FEATURES,
					    (PX4IO_P_SETUP_FEATURES_SBUS1_OUT | PX4IO_P_SETUP_FEATURES_SBUS2_OUT), 0);
		}

		break;

	default:
		/* see if the parent class can make any use of it */
		ret = CDev::ioctl(filep, cmd, arg);
		break;
	}

	return ret;
}

extern "C" __EXPORT int px4io_main(int argc, char *argv[]);

namespace
{

device::Device *get_interface()
{
	device::Device *interface = PX4IO_serial_interface();

	if (interface != nullptr) {
		goto got;
	}

	errx(1, "cannot alloc interface");

got:

	if (interface->init() != OK) {
		delete interface;
		errx(1, "interface init failed");
	}

	return interface;
}

void start(int argc, char *argv[])
{
	if (g_dev != nullptr) {
		errx(0, "already loaded");
	}

	/* allocate the interface */
	device::Device *interface = get_interface();

	/* create the driver - it will set g_dev */
	(void)new PX4IO(interface);

	if (g_dev == nullptr) {
		delete interface;
		errx(1, "driver allocation failed");
	}

	bool rc_handling_disabled = false;
	bool hitl_mode = false;

	/* disable RC handling and/or actuator_output publication on request */
	for (int extra_args = 1; extra_args < argc; extra_args++) {
		if (!strcmp(argv[extra_args], "norc")) {
			rc_handling_disabled = true;

		} else if (!strcmp(argv[extra_args], "hil")) {
			hitl_mode = true;

		} else if (argv[extra_args][0] != '\0') {
			PX4_WARN("unknown argument: %s", argv[extra_args]);
		}
	}

	if (OK != g_dev->init(rc_handling_disabled, hitl_mode)) {
		delete g_dev;
		g_dev = nullptr;
		errx(1, "driver init failed");
	}

	exit(0);
}

void detect(int argc, char *argv[])
{
	if (g_dev != nullptr) {
		errx(0, "already loaded");
	}

	/* allocate the interface */
	device::Device *interface = get_interface();

	/* create the driver - it will set g_dev */
	(void)new PX4IO(interface);

	if (g_dev == nullptr) {
		errx(1, "driver allocation failed");
	}

	int ret = g_dev->detect();

	delete g_dev;
	g_dev = nullptr;

	if (ret) {
		/* nonzero, error */
		exit(1);

	} else {
		exit(0);
	}
}

void checkcrc(int argc, char *argv[])
{
	bool keep_running = false;

	if (g_dev == nullptr) {
		/* allocate the interface */
		device::Device *interface = get_interface();

		/* create the driver - it will set g_dev */
		(void)new PX4IO(interface);

		if (g_dev == nullptr) {
			errx(1, "driver allocation failed");
		}

	} else {
		/* its already running, don't kill the driver */
		keep_running = true;
	}

	/*
	  check IO CRC against CRC of a file
	 */
	if (argc < 2) {
		warnx("usage: px4io checkcrc filename");
		exit(1);
	}

	int fd = open(argv[1], O_RDONLY);

	if (fd == -1) {
		warnx("open of %s failed: %d", argv[1], errno);
		exit(1);
	}

	const uint32_t app_size_max = 0xf000;
	uint32_t fw_crc = 0;
	uint32_t nbytes = 0;

	while (true) {
		uint8_t buf[16];
		int n = read(fd, buf, sizeof(buf));

		if (n <= 0) { break; }

		fw_crc = crc32part(buf, n, fw_crc);
		nbytes += n;
	}

	close(fd);

	while (nbytes < app_size_max) {
		uint8_t b = 0xff;
		fw_crc = crc32part(&b, 1, fw_crc);
		nbytes++;
	}

	int ret = g_dev->ioctl(nullptr, PX4IO_CHECK_CRC, fw_crc);

	if (!keep_running) {
		delete g_dev;
		g_dev = nullptr;
	}

	if (ret != OK) {
		warn("check CRC failed: %d", ret);
		exit(1);
	}

	exit(0);
}

void bind(int argc, char *argv[])
{
	int pulses;

	if (g_dev == nullptr) {
		errx(1, "px4io must be started first");
	}

	if (argc < 3) {
		errx(0, "needs argument, use dsm2, dsmx or dsmx8");
	}

	if (!strcmp(argv[2], "dsm2")) {
		pulses = DSM2_BIND_PULSES;

	} else if (!strcmp(argv[2], "dsmx")) {
		pulses = DSMX_BIND_PULSES;

	} else if (!strcmp(argv[2], "dsmx8")) {
		pulses = DSMX8_BIND_PULSES;

	} else {
		errx(1, "unknown parameter %s, use dsm2, dsmx or dsmx8", argv[2]);
	}

	// Test for custom pulse parameter
	if (argc > 3) {
		pulses = atoi(argv[3]);
	}

	if (g_dev->system_status() & PX4IO_P_STATUS_FLAGS_SAFETY_OFF) {
		errx(1, "system must not be armed");
	}

	g_dev->ioctl(nullptr, DSM_BIND_START, pulses);

	exit(0);
}

void monitor()
{
	/* clear screen */
	printf("\033[2J");

	unsigned cancels = 2;

	for (;;) {
		pollfd fds[1];

		fds[0].fd = 0;
		fds[0].events = POLLIN;

		if (poll(fds, 1, 2000) < 0) {
			errx(1, "poll fail");
		}

		if (fds[0].revents == POLLIN) {
			/* control logic is to cancel with any key */
			char c;
			(void)read(0, &c, 1);

			if (cancels-- == 0) {
				printf("\033[2J\033[H"); /* move cursor home and clear screen */
				exit(0);
			}
		}

		if (g_dev != nullptr) {

			printf("\033[2J\033[H"); /* move cursor home and clear screen */
			(void)g_dev->print_status(false);
			(void)g_dev->print_debug();
			printf("\n\n\n[ Use 'px4io debug <N>' for more output. Hit <enter> three times to exit monitor mode ]\n");

		} else {
			errx(1, "driver not loaded, exiting");
		}
	}
}

void lockdown(int argc, char *argv[])
{
	if (g_dev != nullptr) {

		if (argc > 2 && !strcmp(argv[2], "disable")) {

			warnx("WARNING: ACTUATORS WILL BE LIVE IN HIL! PROCEED?");
			warnx("Press 'y' to enable, any other key to abort.");

			/* check if user wants to abort */
			char c;

			struct pollfd fds;
			int ret;
			hrt_abstime start = hrt_absolute_time();
			const unsigned long timeout = 5000000;

			while (hrt_elapsed_time(&start) < timeout) {
				fds.fd = 0; /* stdin */
				fds.events = POLLIN;
				ret = poll(&fds, 1, 0);

				if (ret > 0) {

					if (read(0, &c, 1) > 0) {

						if (c != 'y') {
							exit(0);

						} else if (c == 'y') {
							break;
						}
					}
				}

				px4_usleep(10000);
			}

			if (hrt_elapsed_time(&start) > timeout) {
				errx(1, "TIMEOUT! ABORTED WITHOUT CHANGES.");
			}

			(void)g_dev->ioctl(0, PWM_SERVO_SET_DISABLE_LOCKDOWN, 1);

			warnx("WARNING: ACTUATORS ARE NOW LIVE IN HIL!");

		} else {
			(void)g_dev->ioctl(0, PWM_SERVO_SET_DISABLE_LOCKDOWN, 0);
			warnx("ACTUATORS ARE NOW SAFE IN HIL.");
		}

	} else {
		errx(1, "driver not loaded, exiting");
	}

	exit(0);
}

} /* namespace */

int px4io_main(int argc, char *argv[])
{
	/* check for sufficient number of arguments */
	if (argc < 2) {
		goto out;
	}

	if (!PX4_MFT_HW_SUPPORTED(PX4_MFT_PX4IO)) {
		errx(1, "PX4IO Not Supported");
	}

	if (!strcmp(argv[1], "start")) {
		start(argc - 1, argv + 1);
	}

	if (!strcmp(argv[1], "detect")) {
		detect(argc - 1, argv + 1);
	}

	if (!strcmp(argv[1], "checkcrc")) {
		checkcrc(argc - 1, argv + 1);
	}

	if (!strcmp(argv[1], "update")) {

		if (g_dev != nullptr) {
			warnx("loaded, detaching first");
			/* stop the driver */
			delete g_dev;
			g_dev = nullptr;
		}

		PX4IO_Uploader *up;

		/* Assume we are using default paths */

		const char *fn[4] {"/etc/extras/px4_io-v2_default.bin", "/fs/microsd/px4_io-v2_default.bin", "/fs/microsd/px4io2.bin", nullptr};

		/* Override defaults if a path is passed on command line */
		if (argc > 2) {
			fn[0] = argv[2];
			fn[1] = nullptr;
		}

		up = new PX4IO_Uploader;
		int ret = up->upload(&fn[0]);
		delete up;

		switch (ret) {
		case OK:
			break;

		case -ENOENT:
			errx(1, "PX4IO firmware file not found");

		case -EEXIST:
		case -EIO:
			errx(1, "error updating PX4IO - check that bootloader mode is enabled");

		case -EINVAL:
			errx(1, "verify failed - retry the update");

		case -ETIMEDOUT:
			errx(1, "timed out waiting for bootloader - power-cycle and try again");

		default:
			errx(1, "unexpected error %d", ret);
		}

		return ret;
	}

	if (!strcmp(argv[1], "forceupdate")) {
		/*
		  force update of the IO firmware without requiring
		  the user to hold the safety switch down
		 */
		if (argc <= 3) {
			warnx("usage: px4io forceupdate MAGIC filename");
			exit(1);
		}

		if (g_dev == nullptr) {
			warnx("px4io is not started, still attempting upgrade");

			/* allocate the interface */
			device::Device *interface = get_interface();

			/* create the driver - it will set g_dev */
			(void)new PX4IO(interface);

			if (g_dev == nullptr) {
				delete interface;
				errx(1, "driver allocation failed");
			}
		}

		uint16_t arg = atol(argv[2]);
		int ret = g_dev->ioctl(nullptr, PX4IO_REBOOT_BOOTLOADER, arg);

		if (ret != OK) {
			warnx("reboot failed - %d", ret);
			exit(1);
		}

		// tear down the px4io instance
		delete g_dev;
		g_dev = nullptr;

		// upload the specified firmware
		const char *fn[2];
		fn[0] = argv[3];
		fn[1] = nullptr;
		PX4IO_Uploader *up = new PX4IO_Uploader;
		up->upload(&fn[0]);
		delete up;
		exit(0);
	}

	/* commands below here require a started driver */

	if (g_dev == nullptr) {
		errx(1, "not started");
	}

	if (!strcmp(argv[1], "safety_off")) {
		int ret = g_dev->ioctl(NULL, PWM_SERVO_SET_FORCE_SAFETY_OFF, 0);

		if (ret != OK) {
			warnx("failed to disable safety");
			exit(1);
		}

		exit(0);
	}

	if (!strcmp(argv[1], "safety_on")) {
		int ret = g_dev->ioctl(NULL, PWM_SERVO_SET_FORCE_SAFETY_ON, 0);

		if (ret != OK) {
			warnx("failed to enable safety");
			exit(1);
		}

		exit(0);
	}

	if (!strcmp(argv[1], "recovery")) {

		/*
		 * Enable in-air restart support.
		 * We can cheat and call the driver directly, as it
		 * doesn't reference filp in ioctl()
		 */
		g_dev->ioctl(nullptr, PX4IO_INAIR_RESTART_ENABLE, 1);
		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {

		/* stop the driver */
		delete g_dev;
		g_dev = nullptr;
		exit(0);
	}


	if (!strcmp(argv[1], "status")) {

		warnx("loaded");
		g_dev->print_status(true);

		exit(0);
	}

	if (!strcmp(argv[1], "debug")) {
		if (argc <= 2) {
			warnx("usage: px4io debug LEVEL");
			exit(1);
		}

		if (g_dev == nullptr) {
			warnx("not started");
			exit(1);
		}

		uint8_t level = atoi(argv[2]);
		/* we can cheat and call the driver directly, as it
		 * doesn't reference filp in ioctl()
		 */
		int ret = g_dev->ioctl(nullptr, PX4IO_SET_DEBUG, level);

		if (ret != 0) {
			warnx("SET_DEBUG failed: %d", ret);
			exit(1);
		}

		warnx("SET_DEBUG %hhu OK", level);
		exit(0);
	}

	if (!strcmp(argv[1], "rx_dsm") ||
	    !strcmp(argv[1], "rx_dsm_10bit") ||
	    !strcmp(argv[1], "rx_dsm_11bit") ||
	    !strcmp(argv[1], "rx_sbus") ||
	    !strcmp(argv[1], "rx_ppm")) {
		errx(0, "receiver type is automatically detected, option '%s' is deprecated", argv[1]);
	}

	if (!strcmp(argv[1], "monitor")) {
		monitor();
	}

	if (!strcmp(argv[1], "bind")) {
		bind(argc, argv);
	}

	if (!strcmp(argv[1], "lockdown")) {
		lockdown(argc, argv);
	}

	if (!strcmp(argv[1], "sbus1_out")) {
		/* we can cheat and call the driver directly, as it
		 * doesn't reference filp in ioctl()
		 */
		int ret = g_dev->ioctl(nullptr, SBUS_SET_PROTO_VERSION, 1);

		if (ret != 0) {
			errx(ret, "S.BUS v1 failed");
		}

		exit(0);
	}

	if (!strcmp(argv[1], "sbus2_out")) {
		/* we can cheat and call the driver directly, as it
		 * doesn't reference filp in ioctl()
		 */
		int ret = g_dev->ioctl(nullptr, SBUS_SET_PROTO_VERSION, 2);

		if (ret != 0) {
			errx(ret, "S.BUS v2 failed");
		}

		exit(0);
	}

	if (!strcmp(argv[1], "rssi_analog")) {
		/* we can cheat and call the driver directly, as it
		 * doesn't reference filp in ioctl()
		 */
		int ret = g_dev->ioctl(nullptr, RC_INPUT_ENABLE_RSSI_ANALOG, 1);

		if (ret != 0) {
			errx(ret, "RSSI analog failed");
		}

		exit(0);
	}

	if (!strcmp(argv[1], "test_fmu_fail")) {
		if (g_dev != nullptr) {
			g_dev->test_fmu_fail(true);
			exit(0);

		} else {

			errx(1, "px4io must be started first");
		}
	}

	if (!strcmp(argv[1], "test_fmu_ok")) {
		if (g_dev != nullptr) {
			g_dev->test_fmu_fail(false);
			exit(0);

		} else {

			errx(1, "px4io must be started first");
		}
	}

out:
	errx(1, "need a command, try 'start', 'stop', 'status', 'monitor', 'debug <level>',\n"
	     "'recovery', 'bind', 'checkcrc', 'safety_on', 'safety_off',\n"
	     "'forceupdate', 'update', 'sbus1_out', 'sbus2_out', 'rssi_analog' or 'rssi_pwm',\n"
	     "'test_fmu_fail', 'test_fmu_ok'");
}

int PX4IO::task_spawn(int argc, char *argv[])
{
	PX4IO *instance = nullptr;//new PX4IO();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init() == PX4_OK) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int PX4IO::print_status()
{
	//perf_print_counter(_cycle_perf);
	//perf_print_counter(_interval_perf);
	_mixing_output.printStatus();

	return 0;
}

int PX4IO::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("px4io", "driver");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start the task");

	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

// extern "C" __EXPORT int px4io_main(int argc, char *argv[])
// {
// 	return PX4IO::main(argc, argv);
// }
