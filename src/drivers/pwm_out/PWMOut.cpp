/****************************************************************************
 *
 *   Copyright (c) 2012-2020 PX4 Development Team. All rights reserved.
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

#include "PWMOut.hpp"

PWMOut::PWMOut() :
	CDev(PX4FMU_DEVICE_PATH),
	OutputModuleInterface(MODULE_NAME, px4::wq_configurations::hp_default),
	_cycle_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")),
	_interval_perf(perf_alloc(PC_INTERVAL, MODULE_NAME": interval"))
{
	_mixing_output.setAllMinValues(PWM_DEFAULT_MIN);
	_mixing_output.setAllMaxValues(PWM_DEFAULT_MAX);

}

PWMOut::~PWMOut()
{
	/* make sure servos are off */
	up_pwm_servo_deinit();

	/* clean up the alternate device node */
	unregister_class_devname(PWM_OUTPUT_BASE_DEVICE_PATH, _class_instance);

	perf_free(_cycle_perf);
	perf_free(_interval_perf);
}

int PWMOut::init()
{
	/* do regular cdev init */
	int ret = CDev::init();

	if (ret != OK) {
		return ret;
	}

	// XXX best would be to register / de-register the device depending on modes

	/* try to claim the generic PWM output device node as well - it's OK if we fail at this */
	_class_instance = register_class_devname(PWM_OUTPUT_BASE_DEVICE_PATH);

	if (_class_instance == CLASS_DEVICE_PRIMARY) {
		/* lets not be too verbose */
	} else if (_class_instance < 0) {
		PX4_ERR("FAILED registering class device");
	}

	_mixing_output.setDriverInstance(_class_instance);

	/* force a reset of the update rate */
	_current_update_rate = 0;

	// Getting initial parameter values
	update_params();

	ScheduleNow();

	return 0;
}

/* When set_pwm_rate is called from either of the 2 IOCTLs:
 *
 * PWM_SERVO_SET_UPDATE_RATE        - Sets the "alternate" channel's rate to the callers's rate specified
 *                                    and the non "alternate" channels to the _pwm_default_rate.
 *
 *                                    rate_map     = _pwm_alt_rate_channels
 *                                    default_rate = _pwm_default_rate
 *                                    alt_rate     = arg of IOCTL (see rates)
 *
 * PWM_SERVO_SET_SELECT_UPDATE_RATE - The caller's specified rate map selects the "alternate" channels
 *                                    to be set to the alt rate. (_pwm_alt_rate)
 *                                    All other channels are set to the default rate. (_pwm_default_rate)
 *
 *                                    rate_map     = arg of IOCTL
 *                                    default_rate = _pwm_default_rate
 *                                    alt_rate     = _pwm_alt_rate

 *  rate_map                        - A mask of 1's for the channels to be set to the
 *                                    alternate rate.
 *                                    N.B. All channels is a given group must be set
 *                                    to the same rate/mode. (default or alt)
 * rates:
 *   alt_rate, default_rate           For PWM is 25 or 400Hz
 *                                    For Oneshot there is no rate, 0 is therefore used
 *                                    to  select Oneshot mode
 */
int PWMOut::set_pwm_rate(uint32_t rate_map, unsigned default_rate, unsigned alt_rate)
{
	PX4_DEBUG("set_pwm_rate %x %u %u", rate_map, default_rate, alt_rate);

	for (unsigned pass = 0; pass < 2; pass++) {

		/* We should note that group is iterated over from 0 to FMU_MAX_ACTUATORS.
		 * This allows for the ideal worlds situation: 1 channel per group
		 * configuration.
		 *
		 * This is typically not what HW supports. A group represents a timer
		 * and channels belongs to a timer.
		 * Therefore all channels in a group are dependent on the timer's
		 * common settings and can not be independent in terms of count frequency
		 * (granularity of pulse width) and rate (period of repetition).
		 *
		 * To say it another way, all channels in a group moust have the same
		 * rate and mode. (See rates above.)
		 */

		for (unsigned group = 0; group < FMU_MAX_ACTUATORS; group++) {

			// get the channel mask for this rate group
			uint32_t mask = up_pwm_servo_get_rate_group(group);

			if (mask == 0) {
				continue;
			}

			// all channels in the group must be either default or alt-rate
			uint32_t alt = rate_map & mask;

			if (pass == 0) {
				// preflight
				if ((alt != 0) && (alt != mask)) {
					PX4_WARN("rate group %u mask %x bad overlap %x", group, mask, alt);
					// not a legal map, bail
					return -EINVAL;
				}

			} else {
				// set it - errors here are unexpected
				if (alt != 0) {
					if (up_pwm_servo_set_rate_group_update(group, alt_rate) != OK) {
						PX4_WARN("rate group set alt failed");
						return -EINVAL;
					}

				} else {
					if (up_pwm_servo_set_rate_group_update(group, default_rate) != OK) {
						PX4_WARN("rate group set default failed");
						return -EINVAL;
					}
				}
			}
		}
	}

	_pwm_alt_rate_channels = rate_map;
	_pwm_default_rate = default_rate;
	_pwm_alt_rate = alt_rate;

	// minimum rate for backup schedule
	unsigned backup_schedule_rate_hz = math::min(_pwm_default_rate, _pwm_alt_rate);

	if (backup_schedule_rate_hz == 0) {
		// OneShot rate is 0
		backup_schedule_rate_hz = 50;
	}

	// constrain reasonably (1 to 50 Hz)
	backup_schedule_rate_hz = math::constrain(backup_schedule_rate_hz, 1u, 50u);

	_backup_schedule_interval_us = roundf(1e6f / backup_schedule_rate_hz);

	_current_update_rate = 0; // force update

	return OK;
}

void PWMOut::update_current_rate()
{
	/*
	* Adjust actuator topic update rate to keep up with
	* the highest servo update rate configured.
	*
	* We always mix at max rate; some channels may update slower.
	*/
	int max_rate = (_pwm_default_rate > _pwm_alt_rate) ? _pwm_default_rate : _pwm_alt_rate;

	// oneshot
	if ((_pwm_default_rate == 0) || (_pwm_alt_rate == 0)) {
		max_rate = 2000;

	} else {
		// run up to twice PWM rate to reduce end-to-end latency
		//  actual pulse width only updated for next period regardless of output module
		max_rate *= 2;
	}

	// max interval 0.5 - 100 ms (10 - 2000Hz)
	const int update_interval_in_us = math::constrain(1000000 / max_rate, 500, 100000);

	_current_update_rate = max_rate;
	_mixing_output.setMaxTopicUpdateRate(update_interval_in_us);
}

void PWMOut::update_pwm_rev_mask()
{
	uint16_t &reverse_pwm_mask = _mixing_output.reverseOutputMask();
	reverse_pwm_mask = 0;

	const char *pname_format;

	if (_class_instance == CLASS_DEVICE_PRIMARY) {
		pname_format = "PWM_MAIN_REV%d";

	} else if (_class_instance == CLASS_DEVICE_SECONDARY) {
		pname_format = "PWM_AUX_REV%d";

	} else {
		PX4_ERR("PWM REV only for MAIN and AUX");
		return;
	}

	for (unsigned i = 0; i < FMU_MAX_ACTUATORS; i++) {
		char pname[16];

		/* fill the channel reverse mask from parameters */
		sprintf(pname, pname_format, i + 1);
		param_t param_h = param_find(pname);

		if (param_h != PARAM_INVALID) {
			int32_t ival = 0;
			param_get(param_h, &ival);
			reverse_pwm_mask |= ((int16_t)(ival != 0)) << i;
		}
	}
}

void PWMOut::update_pwm_trims()
{
	PX4_DEBUG("update_pwm_trims");

	if (!_mixing_output.mixers()) {
		return;
	}

	int16_t values[FMU_MAX_ACTUATORS] = {};

	const char *pname_format;

	if (_class_instance == CLASS_DEVICE_PRIMARY) {
		pname_format = "PWM_MAIN_TRIM%d";

	} else if (_class_instance == CLASS_DEVICE_SECONDARY) {
		pname_format = "PWM_AUX_TRIM%d";

	} else {
		PX4_ERR("PWM TRIM only for MAIN and AUX");
		return;
	}

	for (unsigned i = 0; i < FMU_MAX_ACTUATORS; i++) {
		char pname[16];

		/* fill the struct from parameters */
		sprintf(pname, pname_format, i + 1);
		param_t param_h = param_find(pname);

		if (param_h != PARAM_INVALID) {
			float pval = 0.0f;
			param_get(param_h, &pval);
			values[i] = (int16_t)(10000 * pval);
			PX4_DEBUG("%s: %d", pname, values[i]);
		}
	}

	/* copy the trim values to the mixer offsets */
	unsigned n_out = _mixing_output.mixers()->set_trims(values, FMU_MAX_ACTUATORS);
	PX4_DEBUG("set %d trims", n_out);
}

int PWMOut::task_spawn(int argc, char *argv[])
{
	PWMOut *instance = new PWMOut();

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

void PWMOut::update_pwm_out_state(bool on)
{
	if (on && !_pwm_initialized && _pwm_mask != 0) {
		up_pwm_servo_init(_pwm_mask);
		set_pwm_rate(_pwm_alt_rate_channels, _pwm_default_rate, _pwm_alt_rate);
		_pwm_initialized = true;
	}

	up_pwm_servo_arm(on);
}

bool PWMOut::updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS],
			   unsigned num_outputs, unsigned num_control_groups_updated)
{
	if (_test_mode) {
		return false;
	}

	/* output to the servos */
	if (_pwm_initialized) {
		for (size_t i = 0; i < math::min(_num_outputs, num_outputs); i++) {
			up_pwm_servo_set(i, outputs[i]);
		}
	}

	/* Trigger all timer's channels in Oneshot mode to fire
	 * the oneshots with updated values.
	 */
	if (num_control_groups_updated > 0) {
		up_pwm_update();
	}

	return true;
}

void PWMOut::Run()
{
	if (should_exit()) {
		ScheduleClear();
		_mixing_output.unregister();

		exit_and_cleanup();
		return;
	}

	perf_begin(_cycle_perf);
	perf_count(_interval_perf);

	// push backup schedule
	ScheduleDelayed(_backup_schedule_interval_us);

	_mixing_output.update();

	/* update PWM status if armed or if disarmed PWM values are set */
	bool pwm_on = _mixing_output.armed().armed || (_num_disarmed_set > 0) || _mixing_output.armed().in_esc_calibration_mode;

	if (_pwm_on != pwm_on) {
		_pwm_on = pwm_on;
		update_pwm_out_state(pwm_on);
	}

	// check for parameter updates
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		update_params();
	}

	if (_current_update_rate == 0) {
		update_current_rate();
	}

	// check at end of cycle (updateSubscriptions() can potentially change to a different WorkQueue thread)
	_mixing_output.updateSubscriptions(true, true);

	perf_end(_cycle_perf);
}

void PWMOut::update_params()
{
	update_pwm_rev_mask();
	update_pwm_trims();



	// only activate actual pins

	// PWM_MAIN_PINS/PWM_AUX
	//   - bitmask

	// PWM_AUX
	// PWM_EXTRA

	// ONLY WHEN DISARMED

	const char prefix[] {"PWM_MAIN"};

	for (int i = 0; i < DIRECT_PWM_OUTPUT_CHANNELS; i++) {

		char str[16];

		// PWM_MAIN_MINx
		{
			sprintf(str, "%s_MIN%u", prefix, i);
			int32_t pwm_min{0};

			if (param_get(param_find(str), &pwm_min) == PX4_OK) {
				_mixing_output.minValue(i) = math::constrain(pwm_min, PWM_LOWEST_MIN, PWM_HIGHEST_MIN);

				if (pwm_min != _mixing_output.minValue(i)) {
					int32_t pwm_min_new = _mixing_output.minValue(i);
					param_set(param_find(str), &pwm_min_new);
				}
			}
		}

		// PWM_MAIN_MAXx
		{
			sprintf(str, "%s_MAX%u", prefix, i);
			int32_t pwm_max{0};

			if (param_get(param_find(str), &pwm_max) == PX4_OK) {
				_mixing_output.maxValue(i) = math::constrain(pwm_max, PWM_HIGHEST_MIN, PWM_HIGHEST_MAX);

				if (pwm_max != _mixing_output.maxValue(i)) {
					int32_t pwm_max_new = _mixing_output.maxValue(i);
					param_set(param_find(str), &pwm_max_new);
				}
			}
		}

		// PWM_MAIN_FAILx
		{
			sprintf(str, "%s_FAIL%u", prefix, i);
			int32_t pwm_failsafe{0};

			if (param_get(param_find(str), &pwm_failsafe) == PX4_OK) {
				_mixing_output.failsafeValue(i) = math::constrain(pwm_failsafe, 0, PWM_HIGHEST_MAX);

				if (pwm_failsafe != _mixing_output.failsafeValue(i)) {
					int32_t pwm_fail_new = _mixing_output.failsafeValue(i);
					param_set(param_find(str), &pwm_fail_new);
				}
			}
		}

		// PWM_MAIN_DISx
		{
			sprintf(str, "%s_DIS%u", prefix, i);
			int32_t pwm_disarm{0};

			if (param_get(param_find(str), &pwm_disarm) == PX4_OK) {
				_mixing_output.disarmedValue(i) = math::constrain(pwm_disarm, 0, PWM_HIGHEST_MAX);

				if (pwm_disarm != _mixing_output.disarmedValue(i)) {
					int32_t pwm_disarm_new = _mixing_output.disarmedValue(i);
					param_set(param_find(str), &pwm_disarm_new);
				}
			}
		}

		// PWM_MAIN_TRIMx
		{
			sprintf(str, "%s_TRIM%u", prefix, i);
			int32_t pwm_trim{0};

			if (param_get(param_find(str), &pwm_trim) == PX4_OK) {

				// TODO:
				//_mixing_output.mixers()->set_trims((int16_t *)pwm->values, pwm->channel_count);

			}
		}

		// PWM_MAIN_REVx
		{
			sprintf(str, "%s_REV%u", prefix, i);
			int32_t pwm_rev{0};

			if (param_get(param_find(str), &pwm_rev) == PX4_OK) {

				// TODO:
				//uint16_t &reverse_pwm_mask = _mixing_output.reverseOutputMask();

			}
		}

		// PWM_MAIN_RATEx
		{
			sprintf(str, "%s_RATE%u", prefix, i);
			int32_t pwm_rate{0};

			if (param_get(param_find(str), &pwm_rate) == PX4_OK) {

				// TODO: _pwm_default_rate
				// set_pwm_rate(_pwm_alt_rate_channels, _pwm_default_rate, arg);
			}
		}


	}



	updateParams();
}

int PWMOut::ioctl(file *filp, int cmd, unsigned long arg)
{
	int ret = OK;

	PX4_DEBUG("ioctl cmd: %d, arg: %ld", cmd, arg);

	lock();

	switch (cmd) {
	case PWM_SERVO_GET_DISARMED_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			for (unsigned i = 0; i < FMU_MAX_ACTUATORS; i++) {
				pwm->values[i] = _mixing_output.disarmedValue(i);
			}

			pwm->channel_count = FMU_MAX_ACTUATORS;
		}
		break;

	case PWM_SERVO_SET_MIN_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			/* discard if too many values are sent */
			if (pwm->channel_count > FMU_MAX_ACTUATORS) {
				ret = -EINVAL;
				break;
			}

			for (unsigned i = 0; i < pwm->channel_count; i++) {
				if (pwm->values[i] != 0) {
					/* ignore 0 */
					_mixing_output.minValue(i) = math::constrain(pwm->values[i], (uint16_t)PWM_LOWEST_MIN, (uint16_t)PWM_HIGHEST_MIN);
				}
			}
		}
		break;

	case PWM_SERVO_GET_MIN_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			for (unsigned i = 0; i < FMU_MAX_ACTUATORS; i++) {
				pwm->values[i] = _mixing_output.minValue(i);
			}

			pwm->channel_count = FMU_MAX_ACTUATORS;
			arg = (unsigned long)&pwm;
		}
		break;

	case PWM_SERVO_SET_MAX_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			/* discard if too many values are sent */
			if (pwm->channel_count > FMU_MAX_ACTUATORS) {
				ret = -EINVAL;
				break;
			}

			for (unsigned i = 0; i < pwm->channel_count; i++) {
				if (pwm->values[i] != 0) {
					_mixing_output.maxValue(i) = math::constrain(pwm->values[i], (uint16_t)PWM_LOWEST_MAX, (uint16_t)PWM_HIGHEST_MAX);
				}
			}
		}
		break;

	case PWM_SERVO_GET_MAX_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			for (unsigned i = 0; i < FMU_MAX_ACTUATORS; i++) {
				pwm->values[i] = _mixing_output.maxValue(i);
			}

			pwm->channel_count = FMU_MAX_ACTUATORS;
			arg = (unsigned long)&pwm;
		}
		break;

	case MIXERIOCRESET:
		_mixing_output.resetMixerThreadSafe();
		break;

	case MIXERIOCLOADBUF: {
			const char *buf = (const char *)arg;
			unsigned buflen = strlen(buf);
			ret = _mixing_output.loadMixerThreadSafe(buf, buflen);
			update_pwm_trims();
		}
		break;

	default:
		ret = -ENOTTY;
		break;
	}

	unlock();

	return ret;
}

int PWMOut::custom_command(int argc, char *argv[])
{
	/* start the FMU if not running */
	if (!is_running()) {
		int ret = PWMOut::task_spawn(argc, argv);

		if (ret) {
			return ret;
		}
	}

	return print_usage("unknown command");
}

int PWMOut::print_status()
{
	PX4_INFO("Max update rate: %i Hz", _current_update_rate);

	perf_print_counter(_cycle_perf);
	perf_print_counter(_interval_perf);
	_mixing_output.printStatus();

	return 0;
}

int PWMOut::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This module is responsible for driving the output and reading the input pins. For boards without a separate IO chip
(eg. Pixracer), it uses the main channels. On boards with an IO chip (eg. Pixhawk), it uses the AUX channels, and the
px4io driver is used for main ones.

It listens on the actuator_controls topics, does the mixing and writes the PWM outputs.

### Implementation
By default the module runs on a work queue with a callback on the uORB actuator_controls topic.

### Examples
It is typically started with:
$ pwm_out mode_pwm
To drive all available pins.

Use the `pwm` command for further configurations (PWM rate, levels, ...), and the `mixer` command to load
mixer files.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("pwm_out", "driver");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start the task (without any mode set, use any of the mode_* cmds)");

	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int pwm_out_main(int argc, char *argv[])
{
	return PWMOut::main(argc, argv);
}
