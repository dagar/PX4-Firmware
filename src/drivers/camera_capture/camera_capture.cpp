/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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

/**
 * @file camera_capture.cpp
 *
 * Online and offline geotagging from camera feedback
 *
 * @author Mohammed Kabir <kabir@uasys.io>
 */

#include "camera_capture.hpp"

#define commandParamToInt(n) static_cast<int>(n >= 0 ? n + 0.5f : n - 0.5f)

namespace camera_capture
{
CameraCapture *g_camera_capture{nullptr};
}

struct work_s CameraCapture::_work_publisher;

CameraCapture::CameraCapture() :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
	memset(&_work_publisher, 0, sizeof(_work_publisher));

	// Capture Parameters
	_p_strobe_delay = param_find("CAM_CAP_DELAY");
	param_get(_p_strobe_delay, &_strobe_delay);

	_p_camera_capture_mode = param_find("CAM_CAP_MODE");
	param_get(_p_camera_capture_mode, &_camera_capture_mode);

	_p_camera_capture_edge = param_find("CAM_CAP_EDGE");
	param_get(_p_camera_capture_edge, &_camera_capture_edge);
}

CameraCapture::~CameraCapture()
{
	camera_capture::g_camera_capture = nullptr;
}

void
CameraCapture::capture_callback(uint32_t chan_index, hrt_abstime edge_time, uint32_t edge_state, uint32_t overflow)
{
	_trigger.chan_index = chan_index;
	_trigger.edge_time = edge_time;
	_trigger.edge_state = edge_state;
	_trigger.overflow = overflow;

	work_queue(HPWORK, &_work_publisher, (worker_t)&CameraCapture::publish_trigger_trampoline, this, 0);
}

int
CameraCapture::gpio_interrupt_routine(int irq, void *context, void *arg)
{
	CameraCapture *dev = static_cast<CameraCapture *>(arg);

	dev->_trigger.chan_index = 0;
	dev->_trigger.edge_time = hrt_absolute_time();
	dev->_trigger.edge_state = 0;
	dev->_trigger.overflow = 0;

	work_queue(HPWORK, &_work_publisher, (worker_t)&CameraCapture::publish_trigger_trampoline, dev, 0);

	return PX4_OK;
}

void
CameraCapture::publish_trigger_trampoline(void *arg)
{
	CameraCapture *dev = static_cast<CameraCapture *>(arg);

	dev->publish_trigger();
}

void
CameraCapture::publish_trigger()
{
	bool publish = false;

	camera_trigger_s trigger{};

	// MODES 1 and 2 are not fully tested
	if (_camera_capture_mode == 0 || _gpio_capture) {
		trigger.timestamp = _trigger.edge_time - uint64_t(1000 * _strobe_delay);
		trigger.seq = _capture_seq++;
		_last_trig_time = trigger.timestamp;
		publish = true;

	} else if (_camera_capture_mode == 1) { // Get timestamp of mid-exposure (active high)
		if (_trigger.edge_state == 1) {
			_last_trig_begin_time = _trigger.edge_time - uint64_t(1000 * _strobe_delay);

		} else if (_trigger.edge_state == 0 && _last_trig_begin_time > 0) {
			trigger.timestamp = _trigger.edge_time - ((_trigger.edge_time - _last_trig_begin_time) / 2);
			trigger.seq = _capture_seq++;
			_last_exposure_time = _trigger.edge_time - _last_trig_begin_time;
			_last_trig_time = trigger.timestamp;
			publish = true;
			_capture_seq++;
		}

	} else { // Get timestamp of mid-exposure (active low)
		if (_trigger.edge_state == 0) {
			_last_trig_begin_time = _trigger.edge_time - uint64_t(1000 * _strobe_delay);

		} else if (_trigger.edge_state == 1 && _last_trig_begin_time > 0) {
			trigger.timestamp = _trigger.edge_time - ((_trigger.edge_time - _last_trig_begin_time) / 2);
			trigger.seq = _capture_seq++;
			_last_exposure_time = _trigger.edge_time - _last_trig_begin_time;
			_last_trig_time = trigger.timestamp;
			publish = true;
		}

	}

	trigger.feedback = true;
	_capture_overflows = _trigger.overflow;

	if (!publish) {
		return;
	}

	_trigger_pub.publish(trigger);
}

void
CameraCapture::capture_trampoline(void *context, uint32_t chan_index, hrt_abstime edge_time, uint32_t edge_state,
				  uint32_t overflow)
{
	camera_capture::g_camera_capture->capture_callback(chan_index, edge_time, edge_state, overflow);
}

void CameraCapture::Run()
{
	// Command handling
	vehicle_command_s cmd{};

	if (_command_sub.update(&cmd)) {

		// TODO : this should eventuallly be a capture control command
		if (cmd.command == vehicle_command_s::VEHICLE_CMD_DO_TRIGGER_CONTROL) {

			// Enable/disable signal capture
			if (commandParamToInt(cmd.param1) == 1) {
				set_capture_control(true);

			} else if (commandParamToInt(cmd.param1) == 0) {
				set_capture_control(false);
			}

			// Reset capture sequence
			if (commandParamToInt(cmd.param2) == 1) {
				reset_statistics(true);
			}

			// Acknowledge the command
			vehicle_command_ack_s command_ack{};
			command_ack.command = cmd.command;
			command_ack.result = (uint8_t)vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED;
			command_ack.target_system = cmd.source_system;
			command_ack.target_component = cmd.source_component;
			command_ack.timestamp = hrt_absolute_time();
			_command_ack_pub.publish(command_ack);
		}
	}
}

void CameraCapture::set_capture_control(bool enabled)
{
// a board can define BOARD_CAPTURE_GPIO to use a separate capture pin
#if defined(BOARD_CAPTURE_GPIO)

	px4_arch_gpiosetevent(BOARD_CAPTURE_GPIO, true, false, true, &CameraCapture::gpio_interrupt_routine, this);
	_capture_enabled = enabled;
	_gpio_capture = true;
	reset_statistics(false);

#else

	input_capture_config_t conf{};
	conf.channel = 5; // FMU chan 6
	conf.filter = 0;

	if (_camera_capture_mode == 0) {
		conf.edge = _camera_capture_edge ? Rising : Falling;

	} else {
		conf.edge = Both;
	}

	conf.callback = nullptr;
	conf.context = nullptr;

	if (enabled) {

		conf.callback = &CameraCapture::capture_trampoline;
		conf.context = this;

		unsigned int capture_count = 0;

		if (::ioctl(fd, INPUT_CAP_GET_COUNT, (unsigned long)&capture_count) != 0) {
			PX4_INFO("Not in a capture mode");

			if (up_input_capture_set(5, Rising, 0, NULL, NULL) == 0) {
				PX4_INFO("Mode changed to 4PWM2CAP");

			} else {
				PX4_ERR("Mode NOT changed to 4PWM2CAP!");
				goto err_out;
			}
		}
	}

	// FMU chan 6
	if (up_input_capture_set_callback(5, &CameraCapture::capture_trampoline, this) == 0) {
		_capture_enabled = enabled;
		_gpio_capture = false;

	} else {
		PX4_ERR("Unable to set capture callback for chan %u\n", conf.channel);
		_capture_enabled = false;
		goto err_out;
	}

	reset_statistics(false);

err_out:
	::close(fd);
#endif
}

void CameraCapture::reset_statistics(bool reset_seq)
{
	if (reset_seq) {
		_capture_seq = 0;
	}

	_last_trig_begin_time = 0;
	_last_exposure_time = 0;
	_last_trig_time = 0;
	_capture_overflows = 0;
}

int CameraCapture::start()
{
	// run every 100 ms (10 Hz)
	ScheduleOnInterval(100000, 10000);

	return PX4_OK;
}

void CameraCapture::stop()
{
	ScheduleClear();

	work_cancel(HPWORK, &_work_publisher);

	if (camera_capture::g_camera_capture != nullptr) {
		delete (camera_capture::g_camera_capture);
	}
}

int CameraCapture::print_status()
{
	PX4_INFO("Capture enabled : %s", _capture_enabled ? "YES" : "NO");
	PX4_INFO("Frame sequence : %u", _capture_seq);

	if (_last_trig_time != 0) {
		PX4_INFO("Last trigger timestamp : %" PRIu64 " (%i ms ago)", _last_trig_time,
			 (int)(hrt_elapsed_time(&_last_trig_time) / 1000));

	} else {
		PX4_INFO("No trigger yet");
	}

	if (_camera_capture_mode != 0) {
		PX4_INFO("Last exposure time : %0.2f ms", double(_last_exposure_time) / 1000.0);
	}

	PX4_INFO("Number of overflows : %u", _capture_overflows);

	return 0;
}

int CameraCapture::custom_command(int argc, char *argv[])
{
	PortMode new_mode = PORT_MODE_UNSET;
	const char *verb = argv[0];

	/* start the FMU if not running */
	if (!is_running()) {
		int ret = CameraCapture::task_spawn(argc, argv);

		if (ret) {
			return ret;
		}
	}

	return print_usage("unknown command");
}

int CameraCapture::task_spawn(int argc, char *argv[])
{
	CameraCapture *instance = new CameraCapture();

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

static int usage()
{
	PX4_INFO("usage: camera_capture {start|stop|on|off|reset|status}\n");
	return 1;
}

extern "C" __EXPORT int camera_capture_main(int argc, char *argv[])
{
	if (argc < 2) {
		return usage();
	}

	if (!strcmp(argv[1], "start")) {

		if (camera_capture::g_camera_capture != nullptr) {
			PX4_WARN("already running");
			return 0;
		}

		camera_capture::g_camera_capture = new CameraCapture();

		if (camera_capture::g_camera_capture == nullptr) {
			PX4_WARN("alloc failed");
			return 1;
		}

		if (!camera_capture::g_camera_capture->start()) {
			return 0;

		} else {
			return 1;
		}

	}

	if (camera_capture::g_camera_capture == nullptr) {
		PX4_WARN("not running");
		return 1;

	} else if (!strcmp(argv[1], "stop")) {
		camera_capture::g_camera_capture->stop();

	} else if (!strcmp(argv[1], "status")) {
		camera_capture::g_camera_capture->status();

	} else if (!strcmp(argv[1], "on")) {
		camera_capture::g_camera_capture->set_capture_control(true);

	} else if (!strcmp(argv[1], "off")) {
		camera_capture::g_camera_capture->set_capture_control(false);

	} else if (!strcmp(argv[1], "reset")) {
		camera_capture::g_camera_capture->set_capture_control(false);
		camera_capture::g_camera_capture->reset_statistics(true);

	} else {
		return usage();
	}

	return 0;
}

int InputCapture::print_usage(const char *reason)
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

The module is configured via mode_* commands. This defines which of the first N pins the driver should occupy.
By using mode_pwm4 for example, pins 5 and 6 can be used by the camera trigger driver or by a PWM rangefinder
driver. Alternatively, pwm_out can be started in one of the capture modes, and then drivers can register a capture
callback with ioctl calls.

### Implementation
By default the module runs on a work queue with a callback on the uORB actuator_controls topic.

### Examples
It is typically started with:
$ pwm_out mode_pwm
To drive all available pins.

Capture input (rising and falling edges) and print on the console: start pwm_out in one of the capture modes:
$ pwm_out mode_pwm3cap1
This will enable capturing on the 4th pin. Then do:
$ pwm_out test

Use the `pwm` command for further configurations (PWM rate, levels, ...), and the `mixer` command to load
mixer files.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("input_capture", "driver");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start the task (without any mode set, use any of the mode_* cmds)");

	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int pwm_out_main(int argc, char *argv[])
{
	return InputCapture::main(argc, argv);
}
