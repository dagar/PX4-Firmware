/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
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

#include "ControlStatusLEDs.hpp"

using namespace time_literals;

bool
ControlStatusLEDs::init()
{
	ScheduleOnInterval(50_ms); // 20 Hz
	return true;
}

void
ControlStatusLEDs::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	actuator_armed_s actuator_armed{};
	_actuator_armed_sub.copy(&actuator_armed);

	vehicle_status_s status{};
	_vehicle_status_sub.copy(&status);

	cpuload_s cpuload{};
	_cpuload_sub.copy(&cpuload);

	// this runs at around 20Hz, full cycle is 16 ticks = 10/16Hz
	if (actuator_armed.armed) {
		if (status.failsafe) {
			BOARD_ARMED_LED_OFF();

			if (_leds_counter % 5 == 0) {
				BOARD_ARMED_STATE_LED_TOGGLE();
			}

		} else {
			BOARD_ARMED_STATE_LED_OFF();

			// armed, solid
			BOARD_ARMED_LED_ON();
		}

	} else if (actuator_armed.ready_to_arm) {
		BOARD_ARMED_LED_OFF();

		// ready to arm, blink at 1Hz
		if (_leds_counter % 20 == 0) {
			BOARD_ARMED_STATE_LED_TOGGLE();
		}

	} else {
		BOARD_ARMED_LED_OFF();

		// not ready to arm, blink at 10Hz
		if (_leds_counter % 2 == 0) {
			BOARD_ARMED_STATE_LED_TOGGLE();
		}
	}

	// give system warnings on error LED
	if (cpuload.load > 0.95f) || (cpuload.ram_usage > 0.98f) {
		if (_leds_counter % 2 == 0) {
			BOARD_OVERLOAD_LED_TOGGLE();
		}

	} else {
		BOARD_OVERLOAD_LED_OFF();
	}

	_leds_counter++;
}

int ControlStatusLEDs::task_spawn(int argc, char *argv[])
{
	ControlStatusLEDs *instance = new ControlStatusLEDs();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
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

int ControlStatusLEDs::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int ControlStatusLEDs::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description


)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("control_status_leds", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}


extern "C" __EXPORT int control_status_leds_main(int argc, char *argv[])
{
	return ControlStatusLEDs::main(argc, argv);
}
