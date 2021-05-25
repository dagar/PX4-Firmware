/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

#include "AFBRS50.hpp"

AFBRS50::AFBRS50()
{
}

AFBRS50::~AFBRS50()
{
	orb_unadvertise(_distance_sensor_topic);
}

int AFBRS50::custom_command(int argc, char *argv[])
{
	return print_usage("Unrecognized command.");
}

AFBRS50 *AFBRS50::instantiate(int argc, char *argv[])
{
	AFBRS50 *AFBRS50 = new AFBRS50();
	return AFBRS50;
}

int AFBRS50::init()
{
	return PX4_OK;
}

int AFBRS50::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Ultrasonic range finder driver that handles the communication with the device and publishes the distance via uORB.

### Implementation
This driver is implented as a NuttX task. This Implementation was chosen due to the need for polling on a message
via UART, which is not supported in the work_queue. This driver continuously takes range measurements while it is
running. A simple algorithm to detect false readings is implemented at the driver levelin an attemptto improve
the quality of data that is being published. The driver will not publish data at all if it deems the sensor data
to be invalid or unstable.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("afbrs50", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("distance_sensor");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND("status");
	PRINT_MODULE_USAGE_COMMAND("stop");
	PRINT_MODULE_USAGE_COMMAND("help");

	return PX4_OK;
}

void AFBRS50::run()
{
	int ret = init();

	if(ret != PX4_OK) {
		PX4_INFO("Could not initialize device settings. Exiting.");
		return;
	}

	struct distance_sensor_s report = {};
	_distance_sensor_topic = orb_advertise(ORB_ID(distance_sensor), &report);

	if (_distance_sensor_topic == nullptr) {
		PX4_WARN("Advertise failed.");
		return;
	}

	_start_loop = hrt_absolute_time();

	while (!should_exit()) {

		// Control rate.
		uint64_t loop_time = hrt_absolute_time() - _start_loop;
		uint32_t sleep_time = (loop_time > POLL_RATE_US) ? 0 : POLL_RATE_US - loop_time;
		px4_usleep(sleep_time);

		_start_loop = hrt_absolute_time();
	}

	PX4_INFO("Exiting.");
}

int AFBRS50::task_spawn(int argc, char *argv[])
{
	px4_main_t entry_point = (px4_main_t)&run_trampoline;
	int stack_size = 1256;

	int task_id = px4_task_spawn_cmd("afbrs50", SCHED_DEFAULT,
					 SCHED_PRIORITY_SLOW_DRIVER, stack_size,
					 entry_point, (char *const *)argv);

	if (task_id < 0) {
		task_id = -1;
		return -errno;
	}

	_task_id = task_id;

	return PX4_OK;
}

void AFBRS50::uORB_publish_results(const float object_distance)
{
	struct distance_sensor_s report = {};
	report.timestamp = hrt_absolute_time();
	report.device_id = _device_id.devid;
	report.type = distance_sensor_s::MAV_DISTANCE_SENSOR_LASER;
	report.orientation = distance_sensor_s::ROTATION_DOWNWARD_FACING;
	report.current_distance = object_distance;
	report.min_distance = MIN_DETECTABLE_DISTANCE;
	report.max_distance = MAX_DETECTABLE_DISTANCE;
	report.signal_quality = 0;

	bool data_is_valid = false;
	static uint8_t good_data_counter = 0;

	// If we are within our MIN and MAX thresholds, continue.
	if (object_distance > MIN_DETECTABLE_DISTANCE && object_distance < MAX_DETECTABLE_DISTANCE) {

		// Height cannot change by more than MAX_SAMPLE_DEVIATION between measurements.
		bool sample_deviation_valid = (report.current_distance < _previous_valid_report_distance + MAX_SAMPLE_DEVIATION)
					      && (report.current_distance > _previous_valid_report_distance - MAX_SAMPLE_DEVIATION);

		// Must have NUM_SAMPLES_CONSISTENT valid samples to be publishing.
		if (sample_deviation_valid) {
			good_data_counter++;

			if (good_data_counter > NUM_SAMPLES_CONSISTENT - 1) {
				good_data_counter = NUM_SAMPLES_CONSISTENT;
				data_is_valid = true;

			} else {
				// Have not gotten NUM_SAMPLES_CONSISTENT consistently valid samples.
				data_is_valid = false;
			}

		} else if (good_data_counter > 0) {
			good_data_counter--;

		} else {
			// Reset our quality of data estimate after NUM_SAMPLES_CONSISTENT invalid samples.
			_previous_valid_report_distance = _previous_report_distance;
		}

		_previous_report_distance = report.current_distance;
	}

	if (data_is_valid) {
		report.signal_quality = 1;
		_previous_valid_report_distance = report.current_distance;
		orb_publish(ORB_ID(distance_sensor), _distance_sensor_topic, &report);
	}
}

extern "C" __EXPORT int afbrs50_main(int argc, char *argv[])
{
	return AFBRS50::main(argc, argv);
}
