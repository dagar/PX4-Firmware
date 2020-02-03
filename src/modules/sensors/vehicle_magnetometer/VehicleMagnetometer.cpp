/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#include "VehicleMagnetometer.hpp"

#include <px4_platform_common/log.h>
#include <lib/ecl/geo/geo.h>

using namespace matrix;
using namespace time_literals;
using math::radians;

static constexpr uint32_t SENSOR_TIMEOUT{300_ms};

VehicleMagnetometer::VehicleMagnetometer() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::att_pos_ctrl)
{
	_voter.set_timeout(SENSOR_TIMEOUT);
}

VehicleMagnetometer::~VehicleMagnetometer()
{
	Stop();

	perf_free(_cycle_perf);
}

bool VehicleMagnetometer::Start()
{
	ScheduleNow();

	return true;
}

void VehicleMagnetometer::Stop()
{
	Deinit();

	// clear all registered callbacks
	for (auto &sub : _sensor_sub) {
		sub.unregisterCallback();
	}
}

void VehicleMagnetometer::ParametersUpdate(bool force)
{
	// Check if parameters have changed
	if (_params_sub.updated() || force) {
		// clear update
		parameter_update_s param_update;
		_params_sub.copy(&param_update);

		updateParams();
	}
}

void VehicleMagnetometer::Run()
{
	perf_begin(_cycle_perf);

	bool updated[MAX_SENSOR_COUNT] {};

	for (int uorb_index = 0; uorb_index < MAX_SENSOR_COUNT; uorb_index++) {

		if (!_advertised[uorb_index] && _sensor_sub[uorb_index].advertised()) {
			_advertised[uorb_index] = true;

			if (uorb_index > 0) {
				/* the first always exists, but for each further sensor, add a new validator */
				if (!_voter.add_new_validator()) {
					PX4_ERR("failed to add validator for sensor_mag:%i", uorb_index);
				}
			}
		}

		updated[uorb_index] = _sensor_sub[uorb_index].updated();

		if (updated[uorb_index]) {
			if (_sensor_sub[uorb_index].copy(&_last_data[uorb_index])) {

				if (_priority[uorb_index] == 0) {
					// set initial priority
					_priority[uorb_index] = _sensor_sub[uorb_index].get_priority();
				}

				const sensor_mag_s &mag = _last_data[uorb_index];

				const Vector3f vect = _rotation[uorb_index] * Vector3f{mag.x, mag.y, mag.z};

				float magnetometer_ga[3];
				vect.copyTo(magnetometer_ga);

				_voter.put(uorb_index, mag.timestamp, magnetometer_ga, mag.error_count, _priority[uorb_index]);
			}
		}
	}

	int best_index = -1;
	_voter.get_best(hrt_absolute_time(), &best_index);

	if (best_index >= 0) {
		if (_selected_sensor_sub_index != best_index) {

			// clear all registered callbacks
			for (auto &sub : _sensor_sub) {
				sub.unregisterCallback();
			}

			_selected_sensor_sub_index = best_index;
			_selected_sensor_device_id = _last_data[best_index].device_id;

			_sensor_sub[_selected_sensor_sub_index].registerCallback();
		}

	} else {
		_selected_sensor_sub_index = -1;
	}

	if ((_selected_sensor_sub_index >= 0) && updated[_selected_sensor_sub_index]) {

		ParametersUpdate();

		const sensor_mag_s &mag = _last_data[_selected_sensor_sub_index];

		// populate vehicle_magnetometer with primary mag and publish
		vehicle_magnetometer_s out{};
		out.timestamp_sample = mag.timestamp; // TODO: mag.timestamp_sample;
		out.mag_device_id = mag.device_id;

		Vector3f data{mag.x, mag.y, mag.z}; // TODO: rotate, calibrate, etc
		data.copyTo(out.magnetometer_ga);

		out.timestamp = hrt_absolute_time();
		_vehicle_magnetometer_pub.publish(out);
	}

	// check failover and report
	if (_last_failover_count != _voter.failover_count()) {
		uint32_t flags = _voter.failover_state();
		int failover_index = _voter.failover_index();

		if (flags == DataValidator::ERROR_FLAG_NO_ERROR) {
			if (failover_index != -1) {
				// we switched due to a non-critical reason. No need to panic.
				PX4_INFO("sensor_mag switch from #%i", failover_index);
			}

		} else {
			if (failover_index != -1) {
				mavlink_log_emergency(&_mavlink_log_pub, "sensor_mag:#%i failed: %s%s%s%s%s!, reconfiguring priorities",
						      failover_index,
						      ((flags & DataValidator::ERROR_FLAG_NO_DATA) ? " OFF" : ""),
						      ((flags & DataValidator::ERROR_FLAG_STALE_DATA) ? " STALE" : ""),
						      ((flags & DataValidator::ERROR_FLAG_TIMEOUT) ? " TIMEOUT" : ""),
						      ((flags & DataValidator::ERROR_FLAG_HIGH_ERRCOUNT) ? " ERR CNT" : ""),
						      ((flags & DataValidator::ERROR_FLAG_HIGH_ERRDENSITY) ? " ERR DNST" : ""));

				// reduce priority of failed sensor to the minimum
				_priority[failover_index] = ORB_PRIO_MIN;
			}
		}
	}

	// reschedule timeout
	ScheduleDelayed(100_ms);

	perf_end(_cycle_perf);
}

void VehicleMagnetometer::PrintStatus()
{
	PX4_INFO("selected magnetometer: %d (%d)", _selected_sensor_device_id, _selected_sensor_sub_index);
	perf_print_counter(_cycle_perf);
	_voter.print();
}
