/****************************************************************************
 *
 *   Copyright (c) 2020-2021 PX4 Development Team. All rights reserved.
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

#include "VehicleAirData.hpp"

#include <px4_platform_common/log.h>
#include <lib/ecl/geo/geo.h>

namespace sensors
{

using namespace matrix;

static constexpr uint32_t SENSOR_TIMEOUT{300_ms};

VehicleAirData::VehicleAirData() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	_voter.set_timeout(SENSOR_TIMEOUT);

	ParametersUpdate(true);
}

VehicleAirData::~VehicleAirData()
{
	Stop();
	perf_free(_cycle_perf);
}

bool VehicleAirData::Start()
{
	ScheduleNow();
	return true;
}

void VehicleAirData::Stop()
{
	Deinit();

	// clear all registered callbacks
	for (auto &sub : _sensor_sub) {
		sub.unregisterCallback();
	}
}

void VehicleAirData::ParametersUpdate(bool force)
{
	// Check if parameters have changed
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		updateParams();

		// update priority
		for (int uorb_index = 0; uorb_index < MAX_SENSOR_COUNT; uorb_index++) {
			const int32_t priority_old = _calibration[uorb_index].priority();
			_calibration[uorb_index].ParametersUpdate();
			const int32_t priority_new = _calibration[uorb_index].priority();

			if (priority_old != priority_new) {
				if (_priority[uorb_index] == priority_old) {
					_priority[uorb_index] = priority_new;

				} else {
					// change relative priority to incorporate any sensor faults
					int priority_change = priority_new - priority_old;
					_priority[uorb_index] = math::constrain(_priority[uorb_index] + priority_change, 1, 100);
				}
			}
		}
	}
}

void VehicleAirData::Run()
{
	perf_begin(_cycle_perf);

	ParametersUpdate();

	bool updated[MAX_SENSOR_COUNT] {};

	for (int uorb_index = 0; uorb_index < MAX_SENSOR_COUNT; uorb_index++) {

		if (!_calibration[uorb_index].enabled()) {
			continue;
		}

		if (!_advertised[uorb_index]) {
			// use data's timestamp to throttle advertisement checks
			if (hrt_elapsed_time(&_last_publication_timestamp[uorb_index]) > 1_s) {
				if (_sensor_sub[uorb_index].advertised()) {
					if (uorb_index > 0) {
						/* the first always exists, but for each further sensor, add a new validator */
						if (!_voter.add_new_validator()) {
							PX4_ERR("failed to add validator for %s %i", "BARO", uorb_index);
						}
					}

					_advertised[uorb_index] = true;

					// advertise outputs in order if publishing all
					if (!_param_sens_baro_mode.get()) {
						for (int instance = 0; instance < uorb_index; instance++) {
							_vehicle_air_data_pub[instance].advertise();
						}
					}

					if (_selected_sensor_sub_index < 0) {
						_sensor_sub[uorb_index].registerCallback();
					}

				} else {
					_last_publication_timestamp[uorb_index] = hrt_absolute_time();
				}
			}
		}

		if (_advertised[uorb_index]) {
			sensor_baro_s report;

			while (_sensor_sub[uorb_index].update(&report)) {
				updated[uorb_index] = true;

				if (_calibration[uorb_index].device_id() != report.device_id) {
					_calibration[uorb_index].set_device_id(report.device_id);
					_priority[uorb_index] = _calibration[uorb_index].priority();
				}

				if (_calibration[uorb_index].enabled()) {
					// millibar to Pa
					const float raw_pressure_pascals = report.pressure * 100.f;

					// pressure corrected with offset (if available)
					const float corrected = _calibration[uorb_index].Correct(raw_pressure_pascals);

					float vect[3] {corrected, report.temperature, 0.f};
					_voter.put(uorb_index, report.timestamp, vect, report.error_count, _priority[uorb_index]);

					_timestamp_sample_sum[uorb_index] += report.timestamp_sample;
					_data_sum[uorb_index] += corrected;
					_temperature_sum[uorb_index] += report.temperature;
					_data_sum_count[uorb_index]++;

					_last_data[uorb_index] = corrected;
				}
			}

			// add calibration slot
			if (!_calibration[uorb_index].calibrated()) {
				_calibration[uorb_index].set_calibration_index(uorb_index);
				_calibration[uorb_index].ParametersSave();
			}
		}
	}

	// check for the current best sensor
	int best_index = 0;
	_voter.get_best(hrt_absolute_time(), &best_index);

	if (best_index >= 0) {
		if (_selected_sensor_sub_index != best_index) {
			// clear all registered callbacks
			for (auto &sub : _sensor_sub) {
				sub.unregisterCallback();
			}

			if (_param_sens_baro_mode.get() == 1) {
				if (_selected_sensor_sub_index >= 0) {
					PX4_INFO("%s switch from #%u -> #%d", "BARO", _selected_sensor_sub_index, best_index);
				}
			}

			_selected_sensor_sub_index = best_index;
			_sensor_sub[_selected_sensor_sub_index].registerCallback();
		}
	}

	calculateInconsistency();

	// Publish
	if (_param_sens_baro_mode.get()) {
		// publish only best
		if ((_selected_sensor_sub_index >= 0)
		    && (_voter.get_sensor_state(_selected_sensor_sub_index) == DataValidator::ERROR_FLAG_NO_ERROR)
		    && updated[_selected_sensor_sub_index]) {

			Publish(_selected_sensor_sub_index);
		}

	} else {
		// publish all
		for (int uorb_index = 0; uorb_index < MAX_SENSOR_COUNT; uorb_index++) {
			// publish all sensors as separate instances
			if (updated[uorb_index] && _calibration[uorb_index].enabled() && (_calibration[uorb_index].device_id() != 0)) {
				Publish(uorb_index, true);
			}
		}
	}


	// check failover and report
	if (_param_sens_baro_mode.get()) {
		if (_last_failover_count != _voter.failover_count()) {
			uint32_t flags = _voter.failover_state();
			int failover_index = _voter.failover_index();

			if (flags != DataValidator::ERROR_FLAG_NO_ERROR) {
				if (failover_index != -1) {
					const hrt_abstime now = hrt_absolute_time();

					if (now - _last_error_message > 3_s) {
						mavlink_log_emergency(&_mavlink_log_pub, "%s #%i failed: %s%s%s%s%s!",
								      "BARO",
								      failover_index,
								      ((flags & DataValidator::ERROR_FLAG_NO_DATA) ? " OFF" : ""),
								      ((flags & DataValidator::ERROR_FLAG_STALE_DATA) ? " STALE" : ""),
								      ((flags & DataValidator::ERROR_FLAG_TIMEOUT) ? " TIMEOUT" : ""),
								      ((flags & DataValidator::ERROR_FLAG_HIGH_ERRCOUNT) ? " ERR CNT" : ""),
								      ((flags & DataValidator::ERROR_FLAG_HIGH_ERRDENSITY) ? " ERR DNST" : ""));
						_last_error_message = now;
					}

					// reduce priority of failed sensor to the minimum
					_priority[failover_index] = 1;
				}
			}

			_last_failover_count = _voter.failover_count();
		}
	}

	// reschedule timeout
	ScheduleDelayed(100_ms);

	perf_end(_cycle_perf);
}

void VehicleAirData::Publish(uint8_t instance, bool multi)
{
	if ((_param_sens_baro_rate.get() > 0) && ((_last_publication_timestamp[instance] == 0) ||
			(hrt_elapsed_time(&_last_publication_timestamp[instance]) >= (1e6f / _param_sens_baro_rate.get())))) {

		// populate vehicle_air_data with primary baro and publish
		vehicle_air_data_s out{};
		out.timestamp_sample = _timestamp_sample_sum[instance] / _data_sum_count[instance];
		out.device_id = _calibration[instance].device_id();
		out.air_temperature_celcius = _temperature_sum[instance] / _data_sum_count[instance];

		// Convert from millibar to Pa and apply temperature compensation
		out.barometric_pressure_pa = _data_sum[instance] / _data_sum_count[instance];

		// calculate altitude using the hypsometric equation
		static constexpr float T1 = 15.f - CONSTANTS_ABSOLUTE_NULL_CELSIUS; // temperature at base height in Kelvin
		static constexpr float a = -6.5f / 1000.f; // temperature gradient in degrees per metre

		// current pressure at MSL in kPa (QNH in hPa)
		const float p1 = _param_sens_baro_qnh.get() * 0.1f;

		// measured pressure in kPa
		const float p = out.barometric_pressure_pa * 0.001f;

		/*
		 * Solve:
		 *
		 *     /        -(aR / g)     \
		 *    | (p / p1)          . T1 | - T1
		 *     \                      /
		 * h = -------------------------------  + h1
		 *                   a
		 */
		out.barometric_altitude_m = (((powf((p / p1), (-(a * CONSTANTS_AIR_GAS_CONST) / CONSTANTS_ONE_G))) * T1) - T1) / a;

		// calculate air density
		// estimate air density assuming typical 20degC ambient temperature
		// TODO: use air temperature if available (differential pressure sensors)
		static constexpr float pressure_to_density = 1.f / (CONSTANTS_AIR_GAS_CONST * (2.f - CONSTANTS_ABSOLUTE_NULL_CELSIUS));

		out.rho = pressure_to_density * out.barometric_pressure_pa;

		out.timestamp = hrt_absolute_time();

		if (multi) {
			_vehicle_air_data_pub[instance].publish(out);

		} else {
			// otherwise only ever publish the first instance
			_vehicle_air_data_pub[0].publish(out);
		}

		_last_publication_timestamp[instance] = out.timestamp;

		// reset
		_timestamp_sample_sum[instance] = 0;
		_data_sum[instance] = 0;
		_temperature_sum[instance] = 0;
		_data_sum_count[instance] = 0;
	}
}

void VehicleAirData::calculateInconsistency()
{
	if (_selected_sensor_sub_index >= 0) {

		float mean{};
		float all[MAX_SENSOR_COUNT] {};
		uint8_t sensor_count = 0;

		for (int i = 0; i < MAX_SENSOR_COUNT; i++) {
			if ((_calibration[i].device_id() != 0) && _calibration[i].enabled()) {
				sensor_count++;
				all[i] = _last_data[i];
				mean += all[i];
			}
		}

		if (sensor_count > 0) {
			mean /= sensor_count;

			for (int i = 0; i < MAX_SENSOR_COUNT; i++) {
				if ((_calibration[i].device_id() != 0) && _calibration[i].enabled()) {
					_diff[i] = 0.95f * _diff[i] + 0.05f * (all[i] - mean);
				}
			}
		}

		sensors_status_s status{};
		status.device_id_primary = _calibration[_selected_sensor_sub_index].device_id();

		for (int i = 0; i < MAX_SENSOR_COUNT; i++) {
			if ((_calibration[i].device_id() != 0) && _calibration[i].enabled()) {
				status.device_ids[i] = _calibration[i].device_id();
				status.inconsistency[i] = _diff[i];
				status.healthy[i] = (_voter.get_sensor_state(i) == DataValidator::ERROR_FLAG_NO_ERROR);
				status.calibrated[i] = _calibration[i].calibrated();

			} else {
				status.inconsistency[i] = NAN;
			}
		}

		status.timestamp = hrt_absolute_time();
		_sensors_status_pub.publish(status);
	}
}

void VehicleAirData::PrintStatus()
{
	if (_selected_sensor_sub_index >= 0) {
		PX4_INFO("selected barometer: %d (%d)", _calibration[_selected_sensor_sub_index].device_id(),
			 _selected_sensor_sub_index);
	}

	_voter.print();

	for (int i = 0; i < MAX_SENSOR_COUNT; i++) {
		if (_advertised[i] && (_priority[i] > 0)) {
			_calibration[i].PrintStatus();
		}
	}
}

}; // namespace sensors
