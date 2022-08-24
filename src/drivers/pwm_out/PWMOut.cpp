/****************************************************************************
 *
 *   Copyright (c) 2012-2022 PX4 Development Team. All rights reserved.
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

#include <px4_platform_common/sem.hpp>

pthread_mutex_t pwm_out_module_mutex = PTHREAD_MUTEX_INITIALIZER;
static px4::atomic<PWMOut *> _objects[PWM_OUT_MAX_INSTANCES] {};
static px4::atomic_bool _require_arming[PWM_OUT_MAX_INSTANCES] {};

static bool is_running()
{
	for (auto &obj : _objects) {
		if (obj.load() != nullptr) {
			return true;
		}
	}

	return false;
}

PWMOut::PWMOut(int instance, uint8_t output_base) :
	CDev((instance == 0) ? PX4FMU_DEVICE_PATH : PX4FMU_DEVICE_PATH"1"),
	OutputModuleInterface((instance == 0) ? MODULE_NAME"0" : MODULE_NAME"1", px4::wq_configurations::hp_default),
	_instance(instance),
	_output_base(output_base),
	_cycle_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")),
	_interval_perf(perf_alloc(PC_INTERVAL, MODULE_NAME": interval"))
{
}

PWMOut::~PWMOut()
{
	/* make sure servos are off */
	up_pwm_servo_deinit(_pwm_mask);

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

	/* try to claim the generic PWM output device node as well - it's OK if we fail at this */
	_class_instance = register_class_devname(PWM_OUTPUT_BASE_DEVICE_PATH);

	if (_class_instance == CLASS_DEVICE_PRIMARY) {
		/* lets not be too verbose */
	} else if (_class_instance < 0) {
		PX4_ERR("FAILED registering class device");
	}

	_num_outputs = FMU_MAX_ACTUATORS;

	_pwm_mask = ((1u << _num_outputs) - 1) << _output_base;
	_mixing_output.setMaxNumOutputs(_num_outputs);

	// Getting initial parameter values
	update_params();

	ScheduleNow();

	return 0;
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

	if (_current_update_rate != max_rate) {
		PX4_INFO("instance: %d, max rate: %d, default: %d, alt: %d", _instance, max_rate, _pwm_default_rate, _pwm_alt_rate);
	}

	_current_update_rate = max_rate;
	_mixing_output.setMaxTopicUpdateRate(update_interval_in_us);
}

int PWMOut::task_spawn(int argc, char *argv[])
{
	for (unsigned instance = 0; instance < (sizeof(_objects) / sizeof(_objects[0])); instance++) {

		if (instance < PWM_OUT_MAX_INSTANCES) {
			uint8_t base = instance * MAX_PER_INSTANCE;  // TODO: configurable
			PWMOut *dev = new PWMOut(instance, base);

			if (dev) {
				_objects[instance].store(dev);

				if (dev->init() != PX4_OK) {
					PX4_ERR("%d - init failed", instance);
					delete dev;
					_objects[instance].store(nullptr);
					return PX4_ERROR;
				}

				// only start one instance with dynamic mixing
				//if (dev->_mixing_output.useDynamicMixing()) {
				if (true) {
					break;
				}

			} else {
				PX4_ERR("alloc failed");
			}

		} else {
			// This hardware platform does not support
			// this many devices, set the storage to
			// a sane default
			_objects[instance].store(nullptr);
		}
	}

	return PX4_OK;
}

bool PWMOut::update_pwm_out_state(bool on)
{
	if (on && !_pwm_initialized && _pwm_mask != 0) {

		for (int timer = 0; timer < MAX_IO_TIMERS; ++timer) {
			_timer_rates[timer] = -1;

			uint32_t channels = io_timer_get_group(timer);

			if (channels == 0) {
				continue;
			}

			char param_name[17];
			snprintf(param_name, sizeof(param_name), "%s_TIM%u", _mixing_output.paramPrefix(), timer);

			int32_t tim_config = 0;
			param_t handle = param_find(param_name);
			param_get(handle, &tim_config);

			if (tim_config > 0) {
				_timer_rates[timer] = tim_config;

			} else if (tim_config == -1) { // OneShot
				_timer_rates[timer] = 0;

			} else {
				_pwm_mask &= ~channels; // don't use for pwm
			}
		}

		int ret = up_pwm_servo_init(_pwm_mask);

		if (ret < 0) {
			PX4_ERR("up_pwm_servo_init failed (%i)", ret);
			return false;
		}

		_pwm_mask = ret;

		// set the timer rates
		for (int timer = 0; timer < MAX_IO_TIMERS; ++timer) {
			uint32_t channels = _pwm_mask & up_pwm_servo_get_rate_group(timer);

			if (channels == 0) {
				continue;
			}

			ret = up_pwm_servo_set_rate_group_update(timer, _timer_rates[timer]);

			if (ret != 0) {
				PX4_ERR("up_pwm_servo_set_rate_group_update failed for timer %i, rate %i (%i)", timer, _timer_rates[timer], ret);
				_timer_rates[timer] = -1;
				_pwm_mask &= ~channels;
			}
		}

		_pwm_initialized = true;

		// disable unused functions
		for (unsigned i = 0; i < _num_outputs; ++i) {
			if (((1 << i) & _pwm_mask) == 0) {
				_mixing_output.disableFunction(i);
			}
		}
	}

	_require_arming[_instance].store(false);
	up_pwm_servo_arm(on, _pwm_mask);
	return true;
}

bool PWMOut::updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS],
			   unsigned num_outputs, unsigned num_control_groups_updated)
{
	/* output to the servos */
	if (_pwm_initialized) {
		for (size_t i = 0; i < num_outputs; i++) {
			if (!_mixing_output.isFunctionSet(i)) {
				// do not run any signal on disabled channels
				outputs[i] = 0;
			}

			if (_pwm_mask & (1 << (i + _output_base))) {
				up_pwm_servo_set(_output_base + i, outputs[i]);
			}
		}
	}

	/* Trigger all timer's channels in Oneshot mode to fire
	 * the oneshots with updated values.
	 */
	if (num_control_groups_updated > 0) {
		up_pwm_update(_pwm_mask);
	}

	return true;
}

void PWMOut::Run()
{
	if (should_exit()) {
		ScheduleClear();
		_mixing_output.unregister();

		//exit_and_cleanup();
		return;
	}

	SmartLock lock_guard(_lock);

	perf_begin(_cycle_perf);
	perf_count(_interval_perf);

	_mixing_output.update();

	/* update PWM status if armed or if disarmed PWM values are set */
	bool pwm_on = true;

	if (_pwm_on != pwm_on || _require_arming[_instance].load()) {

		if (update_pwm_out_state(pwm_on)) {
			_pwm_on = pwm_on;
		}
	}

	// check for parameter updates
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		update_params();
	}

	// check at end of cycle (updateSubscriptions() can potentially change to a different WorkQueue thread)
	_mixing_output.updateSubscriptions(true, true);

	perf_end(_cycle_perf);
}

void PWMOut::update_params()
{
	uint32_t previously_set_functions = 0;

	for (size_t i = 0; i < _num_outputs; i++) {
		previously_set_functions |= (uint32_t)_mixing_output.isFunctionSet(i) << i;
	}

	updateParams();

	// Automatically set the PWM rate and disarmed value when a channel is first set to a servo
	if (!_first_param_update) {
		for (size_t i = 0; i < _num_outputs; i++) {
			if ((previously_set_functions & (1u << i)) == 0 && _mixing_output.functionParamHandle(i) != PARAM_INVALID) {
				int32_t output_function;

				if (param_get(_mixing_output.functionParamHandle(i), &output_function) == 0
				    && output_function >= (int)OutputFunction::Servo1
				    && output_function <= (int)OutputFunction::ServoMax) { // Function got set to a servo
					int32_t val = 1500;
					PX4_INFO("Setting disarmed to %i for channel %i", (int) val, i);
					param_set(_mixing_output.disarmedParamHandle(i), &val);

					// If the whole timer group was not set previously, then set the pwm rate to 50 Hz
					for (int timer = 0; timer < MAX_IO_TIMERS; ++timer) {

						uint32_t channels = io_timer_get_group(timer);

						if ((channels & (1u << i)) == 0) {
							continue;
						}

						if ((channels & previously_set_functions) == 0) { // None of the channels was set
							char param_name[17];
							snprintf(param_name, sizeof(param_name), "%s_TIM%u", _mixing_output.paramPrefix(), timer);

							int32_t tim_config = 0;
							param_t handle = param_find(param_name);

							if (param_get(handle, &tim_config) == 0 && tim_config == 400) {
								tim_config = 50;
								PX4_INFO("setting timer %i to %i Hz", timer, (int) tim_config);
								param_set(handle, &tim_config);
							}
						}
					}
				}
			}
		}
	}

	_first_param_update = false;
}

int PWMOut::ioctl(device::file_t *filp, int cmd, unsigned long arg)
{
	SmartLock lock_guard(_lock);

	int ret = pwm_ioctl(filp, cmd, arg);

	/* if nobody wants it, let CDev have it */
	if (ret == -ENOTTY) {
		ret = CDev::ioctl(filp, cmd, arg);
	}

	return ret;
}

int PWMOut::pwm_ioctl(device::file_t *filp, int cmd, unsigned long arg)
{
	int ret = OK;

	PX4_DEBUG("pwm_out%u: ioctl cmd: %d, arg: %ld", _instance, cmd, arg);

	switch (cmd) {
	case PWM_SERVO_ARM:
		update_pwm_out_state(true);
		break;

	case PWM_SERVO_SET_ARM_OK:
	case PWM_SERVO_CLEAR_ARM_OK:
		break;

	case PWM_SERVO_DISARM:

		/* Ignore disarm if disarmed PWM is set already. */
		if (_num_disarmed_set == 0) {
			update_pwm_out_state(false);
		}

		break;

	case PWM_SERVO_GET_DEFAULT_UPDATE_RATE:
		*(uint32_t *)arg = _pwm_default_rate;
		break;

	case PWM_SERVO_SET_UPDATE_RATE:
		ret = -EINVAL;
		break;

	case PWM_SERVO_GET_UPDATE_RATE:
		*(uint32_t *)arg = _pwm_alt_rate;
		break;

	case PWM_SERVO_SET_SELECT_UPDATE_RATE:
		ret = -EINVAL;
		break;

	case PWM_SERVO_GET_SELECT_UPDATE_RATE:
		*(uint32_t *)arg = _pwm_alt_rate_channels;
		break;

	case PWM_SERVO_GET_FAILSAFE_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			for (unsigned i = 0; i < FMU_MAX_ACTUATORS; i++) {
				pwm->values[i] = _mixing_output.failsafeValue(i);
			}

			pwm->channel_count = FMU_MAX_ACTUATORS;
			break;
		}

	case PWM_SERVO_GET_DISARMED_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			for (unsigned i = 0; i < FMU_MAX_ACTUATORS; i++) {
				pwm->values[i] = _mixing_output.disarmedValue(i);
			}

			pwm->channel_count = FMU_MAX_ACTUATORS;
			break;
		}

	case PWM_SERVO_SET_MIN_PWM:
		ret = -EINVAL;
		break;

	case PWM_SERVO_GET_MIN_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			for (unsigned i = 0; i < FMU_MAX_ACTUATORS; i++) {
				pwm->values[i] = _mixing_output.minValue(i);
			}

			pwm->channel_count = FMU_MAX_ACTUATORS;
			arg = (unsigned long)&pwm;
			break;
		}

	case PWM_SERVO_SET_MAX_PWM:
		ret = -EINVAL;
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

#if defined(DIRECT_PWM_OUTPUT_CHANNELS) && DIRECT_PWM_OUTPUT_CHANNELS >= 14

	case PWM_SERVO_GET(13):
	case PWM_SERVO_GET(12):
	case PWM_SERVO_GET(11):
	case PWM_SERVO_GET(10):
	case PWM_SERVO_GET(9):
	case PWM_SERVO_GET(8):
#endif
#if defined(DIRECT_PWM_OUTPUT_CHANNELS) && DIRECT_PWM_OUTPUT_CHANNELS >= 8
	case PWM_SERVO_GET(7):
	case PWM_SERVO_GET(6):
#endif
#if defined(DIRECT_PWM_OUTPUT_CHANNELS) && DIRECT_PWM_OUTPUT_CHANNELS >= 6
	case PWM_SERVO_GET(5):
#endif
#if defined(DIRECT_PWM_OUTPUT_CHANNELS) && DIRECT_PWM_OUTPUT_CHANNELS >= 5
	case PWM_SERVO_GET(4):
#endif
	case PWM_SERVO_GET(3):
	case PWM_SERVO_GET(2):
	case PWM_SERVO_GET(1):
	case PWM_SERVO_GET(0):
		if (cmd - PWM_SERVO_GET(0) >= (int)_num_outputs) {
			ret = -EINVAL;
			break;
		}

		*(servo_position_t *)arg = up_pwm_servo_get(cmd - PWM_SERVO_GET(0) +  _output_base);
		break;

	case PWM_SERVO_GET_RATEGROUP(0):
	case PWM_SERVO_GET_RATEGROUP(1):
	case PWM_SERVO_GET_RATEGROUP(2):
	case PWM_SERVO_GET_RATEGROUP(3):
#if defined(DIRECT_PWM_OUTPUT_CHANNELS) && DIRECT_PWM_OUTPUT_CHANNELS >= 5
	case PWM_SERVO_GET_RATEGROUP(4):
#endif
#if defined(DIRECT_PWM_OUTPUT_CHANNELS) && DIRECT_PWM_OUTPUT_CHANNELS >= 6
	case PWM_SERVO_GET_RATEGROUP(5):
#endif
#if defined(DIRECT_PWM_OUTPUT_CHANNELS) && DIRECT_PWM_OUTPUT_CHANNELS >= 8
	case PWM_SERVO_GET_RATEGROUP(6):
	case PWM_SERVO_GET_RATEGROUP(7):
#endif
#if defined(DIRECT_PWM_OUTPUT_CHANNELS) && DIRECT_PWM_OUTPUT_CHANNELS >= 14
	case PWM_SERVO_GET_RATEGROUP(8):
	case PWM_SERVO_GET_RATEGROUP(9):
	case PWM_SERVO_GET_RATEGROUP(10):
	case PWM_SERVO_GET_RATEGROUP(11):
	case PWM_SERVO_GET_RATEGROUP(12):
	case PWM_SERVO_GET_RATEGROUP(13):
#endif
		*(uint32_t *)arg = _pwm_mask & up_pwm_servo_get_rate_group(cmd - PWM_SERVO_GET_RATEGROUP(0));
		break;

	case PWM_SERVO_GET_COUNT:
		*(unsigned *)arg = _num_outputs;
		break;

	default:
		ret = -ENOTTY;
		break;
	}

	return ret;
}

int PWMOut::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int PWMOut::print_status()
{
	if (_class_instance == CLASS_DEVICE_PRIMARY) {
		PX4_INFO("%d - PWM_MAIN 0x%04" PRIx32, _instance, _pwm_mask);

	} else if (_class_instance == CLASS_DEVICE_SECONDARY) {
		PX4_INFO("%d - PWM_AUX 0x%04" PRIx32, _instance, _pwm_mask);

	} else if (_class_instance == CLASS_DEVICE_TERTIARY) {
		PX4_INFO("%d - PWM_EXTRA 0x%04" PRIx32, _instance, _pwm_mask);
	}

	PX4_INFO("%d - Max update rate: %i Hz", _instance, _current_update_rate);

	perf_print_counter(_cycle_perf);
	perf_print_counter(_interval_perf);
	_mixing_output.printStatus();

	if (_pwm_initialized) {
		for (int timer = 0; timer < MAX_IO_TIMERS; ++timer) {
			if (_timer_rates[timer] >= 0) {
				PX4_INFO_RAW("Timer %i: rate: %3i", timer, _timer_rates[timer]);
				uint32_t channels = _pwm_mask & up_pwm_servo_get_rate_group(timer);

				if (channels > 0) {
					PX4_INFO_RAW(" channels: ");

					for (uint32_t channel = 0; channel < _num_outputs; ++channel) {
						if ((1 << channel) & channels) {
							PX4_INFO_RAW("%" PRIu32 " ", channel);
						}
					}
				}

				PX4_INFO_RAW("\n");
			}
		}
	}

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
This module is responsible for driving the output pins. For boards without a separate IO chip
(eg. Pixracer), it uses the main channels. On boards with an IO chip (eg. Pixhawk), it uses the AUX channels, and the
px4io driver is used for main ones.

It listens on the actuator_controls topics, does the mixing and writes the PWM outputs.

On startup, the module tries to occupy all available pins for PWM/Oneshot output.
It skips all pins already in use (e.g. by a camera trigger module).

### Implementation
By default the module runs on a work queue with a callback on the uORB actuator_controls topic.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("pwm_out", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int pwm_out_main(int argc, char *argv[])
{
	if (argc <= 1 || strcmp(argv[1], "-h") == 0) {
		return PWMOut::print_usage();
	}

	if (strcmp(argv[1], "start") == 0) {

		if (is_running()) {
			return 0;
		}

		int ret = 0;

		PWMOut::lock_module();

		ret = PWMOut::task_spawn(argc - 1, argv + 1);

		if (ret < 0) {
			PX4_ERR("start failed (%i)", ret);
		}

		PWMOut::unlock_module();
		return ret;

	} else if (strcmp(argv[1], "status") == 0) {
		if (PWMOut::trylock_module()) {

			unsigned count = 0;

			for (int i = 0; i < PWM_OUT_MAX_INSTANCES; i++) {
				if (_objects[i].load()) {
					PX4_INFO_RAW("\n");
					_objects[i].load()->print_status();
					count++;
				}
			}

			PWMOut::unlock_module();

			if (count == 0) {
				PX4_INFO("not running");
				return 1;
			}

		} else {
			PX4_WARN("module locked, try again later");
		}

		return 0;

	} else if (strcmp(argv[1], "stop") == 0) {
		PWMOut::lock_module();

		if (argc > 2) {
			int instance = atoi(argv[2]);

			if (instance >= 0 && instance < PWM_OUT_MAX_INSTANCES) {
				PX4_INFO("stopping instance %d", instance);
				PWMOut *inst = _objects[instance].load();

				if (inst) {
					inst->request_stop();
					px4_usleep(20000); // 20 ms
					delete inst;
					_objects[instance].store(nullptr);
				}
			} else {
				PX4_ERR("invalid instance %d", instance);
			}

		} else {
			// otherwise stop everything
			bool was_running = false;

			for (int i = 0; i < PWM_OUT_MAX_INSTANCES; i++) {
				PWMOut *inst = _objects[i].load();

				if (inst) {
					PX4_INFO("stopping pwm_out instance %d", i);
					was_running = true;
					inst->request_stop();
					px4_usleep(20000); // 20 ms
					delete inst;
					_objects[i].store(nullptr);
				}
			}

			if (!was_running) {
				PX4_WARN("not running");
			}
		}

		PWMOut::unlock_module();
		return PX4_OK;
	}

	PWMOut::lock_module(); // Lock here, as the method could access _object.
	int ret = PWMOut::custom_command(argc - 1, argv + 1);
	PWMOut::unlock_module();

	return ret;
}
