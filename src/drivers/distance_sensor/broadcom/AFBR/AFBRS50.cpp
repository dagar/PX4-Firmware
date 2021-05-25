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

#include <fcntl.h>
#include <stdlib.h>
#include <string.h>

#include <lib/drivers/device/Device.hpp>

AFBRS50::AFBRS50(uint8_t device_orientation):
	ScheduledWorkItem(MODULE_NAME, px4::ins_instance_to_wq(0)),
	_px4_rangefinder(0, device_orientation)
{
	device::Device::DeviceId device_id;
	device_id.devid_s.bus_type = device::Device::DeviceBusType::DeviceBusType_SPI;

	uint8_t bus_num = 0;

	if (bus_num < 10) {
		device_id.devid_s.bus = bus_num;
	}

	_px4_rangefinder.set_device_id(device_id.devid);
	_px4_rangefinder.set_device_type(DRV_DIST_DEVTYPE_AFBRS50);

	_px4_rangefinder.set_max_distance(AFBRS50_MAX_DISTANCE);
	_px4_rangefinder.set_min_distance(AFBRS50_MIN_DISTANCE);
	_px4_rangefinder.set_fov(AFBRS50_FIELD_OF_VIEW);
}

AFBRS50::~AFBRS50()
{
	stop();

	perf_free(_comms_error);
	perf_free(_sample_perf);
}

int
AFBRS50::collect()
{
	perf_begin(_sample_perf);

	//const int buffer_size = sizeof(_buffer);
	//const int message_size = sizeof(reading_msg);

	//int bytes_read = ::read(_file_descriptor, _buffer + _buffer_len, buffer_size - _buffer_len);

	//if (bytes_read < 1) {
		// Trigger a new measurement.
		return measure();
	//}

	//_buffer_len += bytes_read;

	//if (_buffer_len < message_size) {
		// Return on next scheduled cycle to collect remaining data.
	//	return PX4_OK;
	//}

	// NOTE: little-endian support only.
	uint16_t distance_mm = 0;
	float distance_m = static_cast<float>(distance_mm) / 1000.0f;

	// @TODO - implement a meaningful signal quality value.
	int8_t signal_quality = -1;

	_px4_rangefinder.update(_measurement_time, distance_m, signal_quality);

	perf_end(_sample_perf);

	// Trigger the next measurement.
	return measure();
}

int
AFBRS50::init()
{

	hrt_abstime time_now = hrt_absolute_time();

	const hrt_abstime timeout_usec = time_now + 500000_us; // 0.5sec

	while (time_now < timeout_usec) {
		if (measure() == PX4_OK) {
			px4_usleep(AFBRS50_MEASURE_INTERVAL);

			if (collect() == PX4_OK) {
				// The file descriptor can only be accessed by the process that opened it,
				// so closing here allows the port to be opened from scheduled work queue.
				stop();
				return PX4_OK;
			}
		}

		px4_usleep(1000);
		time_now = hrt_absolute_time();
	}

	PX4_ERR("No readings from AFBRS50");
	return PX4_ERROR;
}

int
AFBRS50::measure()
{
	_measurement_time = hrt_absolute_time();
	return PX4_OK;
}

void
AFBRS50::print_info()
{
	perf_print_counter(_comms_error);
	perf_print_counter(_sample_perf);
}

void
AFBRS50::Run()
{
	collect();
}

void
AFBRS50::start()
{
	// Schedule the driver at regular intervals.
	ScheduleOnInterval(AFBRS50_MEASURE_INTERVAL, AFBRS50_MEASURE_INTERVAL);
}

void
AFBRS50::stop()
{
	// Clear the work queue schedule.
	ScheduleClear();
}
