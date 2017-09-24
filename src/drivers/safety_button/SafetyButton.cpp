/****************************************************************************
 *
 *   Copyright (c) 2012-2017, 2017 PX4 Development Team. All rights reserved.
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

#include "SafetyButton.hpp"

SafetyButton::~SafetyButton()
{
	orb_unsubscribe(_armed_sub);

	orb_unadvertise(_to_safety);
}

void
SafetyButton::safety_check_button(void)
{
#ifdef GPIO_BTN_SAFETY
	static int counter = 0;
	/*
	 * Debounce the safety button, change state if it has been held for long enough.
	 *
	 */
	bool safety_button_pressed = px4_arch_gpioread(GPIO_BTN_SAFETY);

	/*
	 * Keep pressed for a while to arm.
	 *
	 * Note that the counting sequence has to be same length
	 * for arming / disarming in order to end up as proper
	 * state machine, keep ARM_COUNTER_THRESHOLD the same
	 * length in all cases of the if/else struct below.
	 */
	if (safety_button_pressed && !_safety_off) {
		if (counter < CYCLE_COUNT) {
			counter++;

		} else if (counter == CYCLE_COUNT) {
			/* switch to armed state */
			_safety_off = true;
			counter++;
		}

	} else if (safety_button_pressed && _safety_off) {
		if (counter < CYCLE_COUNT) {
			counter++;

		} else if (counter == CYCLE_COUNT) {
			/* change to disarmed state */
			_safety_off = false;
			counter++;
		}

	} else {
		counter = 0;
	}

#endif /* GPIO_BTN_SAFETY */
}

void
SafetyButton::flash_safety_button()
{
#ifdef GPIO_BTN_SAFETY

	/* Select the appropriate LED flash pattern depending on the current arm state */
	uint16_t pattern = LED_PATTERN_FMU_REFUSE_TO_ARM;

	/* cycle the blink state machine at 10Hz */
	static int blink_counter = 0;

	if (_safety_off) {
		if (_armed) {
			pattern = LED_PATTERN_IO_FMU_ARMED;

		} else {
			pattern = LED_PATTERN_IO_ARMED;
		}

	} else if (_armed) {
		pattern = LED_PATTERN_FMU_ARMED;

	} else {
		pattern = LED_PATTERN_FMU_OK_TO_ARM;
	}

	/* Turn the LED on if we have a 1 at the current bit position */
	px4_arch_gpiowrite(GPIO_LED_SAFETY, !(pattern & (1 << blink_counter++)));

	if (blink_counter > 15) {
		blink_counter = 0;
	}

#endif /* GPIO_BTN_SAFETY */
}

int
SafetyButton::task_spawn(int argc, char *argv[])
{
	SafetyButton *obj = new SafetyButton();

	int ret = obj->start();

	if (ret < 0) {
		delete obj;
		return ret;
	}

	wait_until_running(); // this will wait until _object is set from the cycle method
	_task_id = task_id_is_work_queue;

	return 0;
}

int SafetyButton::start()
{
	/* schedule a cycle to start things */
	return work_queue(HPWORK, &_work, (worker_t)&SafetyButton::cycle_trampoline, this, 0);
}

void
SafetyButton::cycle_trampoline(void *arg)
{
	SafetyButton *dev = reinterpret_cast<SafetyButton *>(arg);
	dev->cycle();
}

void
SafetyButton::cycle()
{
	if (_object == nullptr) { // not initialized yet
		_safety_disabled = circuit_breaker_enabled("CBRK_IO_SAFETY", CBRK_IO_SAFETY_KEY);
		_armed_sub = orb_subscribe(ORB_ID(actuator_armed));
		_object = this;
	}

	/* check arming state */
	bool updated = false;
	orb_check(_armed_sub, &updated);

	if (updated) {
		actuator_armed_s armed;
		orb_copy(ORB_ID(actuator_armed), _armed_sub, &armed);

		_armed = armed.armed;
	}

#ifdef GPIO_BTN_SAFETY

	const bool previous_safety_off = _safety_off;

	if (_safety_disabled) {
		_safety_off = true;

	} else {
		/* read safety switch input and control safety switch LED at 10Hz */
		safety_check_button();
	}

	/* Make the safety button flash anyway, no matter if it's used or not. */
	flash_safety_button();

	safety_s safety = {};
	safety.timestamp = hrt_absolute_time();
	safety.safety_switch_available = true;

	if (_safety_off) {
		safety.safety_off = true;

	} else {
		safety.safety_off = false;
	}

	/* lazily publish the safety status */
	if (_to_safety != nullptr) {
		if (safety.safety_switch_available && (previous_safety_off != safety.safety_off)) {
			orb_publish(ORB_ID(safety), _to_safety, &safety);
		}

	} else {
		_to_safety = orb_advertise(ORB_ID(safety), &safety);
	}

#endif /* GPIO_BTN_SAFETY */

	if (should_exit()) {
		exit_and_cleanup();

	} else {
		// Schedule next cycle.
		work_queue(HPWORK, &_work, (worker_t)&SafetyButton::cycle_trampoline, this,
			   USEC2TICK(1000000 / SAFETY_BUTTON_UPDATE_RATE_HZ));
	}
}

SafetyButton *SafetyButton::instantiate(int argc, char *argv[])
{
	return new SafetyButton();
}

int SafetyButton::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int SafetyButton::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This module is responsible for the safety button on boards without a separate IO chip (eg. Pixracer).
)DESCR_STR");

	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int safety_button_main(int argc, char *argv[]);

int
safety_button_main(int argc, char *argv[])
{
	return SafetyButton::main(argc, argv);
}
