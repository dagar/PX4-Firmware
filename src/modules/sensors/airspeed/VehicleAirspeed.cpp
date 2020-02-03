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

#include "VehicleAirspeed.hpp"

#include <px4_platform_common/log.h>
#include <lib/ecl/geo/geo.h>

using namespace matrix;
using namespace time_literals;
using math::radians;

static constexpr uint32_t SENSOR_TIMEOUT{300_ms};

/**
 * HACK - true temperature is much less than indicated temperature in baro,
 * subtract 5 degrees in an attempt to account for the electrical upheating of the PCB
 */
static constexpr float PCB_TEMP_ESTIMATE_DEG{5.0f};

VehicleAirspeed::VehicleAirspeed() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::att_pos_ctrl)
{
	_voter.set_timeout(SENSOR_TIMEOUT);
	_voter.set_equal_value_threshold(100);
}

VehicleAirspeed::~VehicleAirspeed()
{
	Stop();

	perf_free(_cycle_perf);
}

bool VehicleAirspeed::Start()
{
	ScheduleNow();

	return true;
}

void VehicleAirspeed::Stop()
{
	Deinit();

	// clear all registered callbacks
	for (auto &sub : _sensor_sub) {
		sub.unregisterCallback();
	}
}

void VehicleAirspeed::ParametersUpdate(bool force)
{
	// Check if parameters have changed
	if (_params_sub.updated() || force) {
		// clear update
		parameter_update_s param_update;
		_params_sub.copy(&param_update);

		updateParams();

		/* update airspeed scale */
		int fd = px4_open(AIRSPEED0_DEVICE_PATH, 0);

		/* this sensor is optional, abort without error */
		if (fd >= 0) {
			struct airspeed_scale airscale = {
				_param_sens_dpres_off.get(),
				1.0f,
			};

			if (OK != px4_ioctl(fd, AIRSPEEDIOCSSCALE, (long unsigned int)&airscale)) {
				warn("WARNING: failed to set scale / offsets for airspeed sensor");
			}

			px4_close(fd);
		}
	}
}

void VehicleAirspeed::Run()
{
	perf_begin(_cycle_perf);

	bool updated[MAX_SENSOR_COUNT] {};

	for (int uorb_index = 0; uorb_index < MAX_SENSOR_COUNT; uorb_index++) {

		if (!_advertised[uorb_index] && _sensor_sub[uorb_index].advertised()) {
			_advertised[uorb_index] = true;

			if (uorb_index > 0) {
				/* the first always exists, but for each further sensor, add a new validator */
				if (!_voter.add_new_validator()) {
					PX4_ERR("failed to add validator for differential_pressure:%i", uorb_index);
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

				float vect[3] = {_last_data[uorb_index].differential_pressure_raw_pa, _last_data[uorb_index].differential_pressure_filtered_pa, _last_data[uorb_index].temperature};
				_voter.put(uorb_index, _last_data[uorb_index].timestamp, vect, _last_data[uorb_index].error_count,
					   _priority[uorb_index]);
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

		const differential_pressure_s &diff_pres = _last_data[_selected_sensor_sub_index];

		vehicle_air_data_s air_data{};
		_vehicle_air_data_sub.copy(&air_data);

		float air_temperature_celsius = (diff_pres.temperature > -300.0f) ? diff_pres.temperature :
						(air_data.baro_temp_celcius - PCB_TEMP_ESTIMATE_DEG);

		airspeed_s airspeed{};
		airspeed.timestamp_sample = diff_pres.timestamp;

		enum AIRSPEED_SENSOR_MODEL smodel {};

		switch ((diff_pres.device_id >> 16) & 0xFF) {
		case DRV_DIFF_PRESS_DEVTYPE_SDP31:

		/* fallthrough */
		case DRV_DIFF_PRESS_DEVTYPE_SDP32:

		/* fallthrough */
		case DRV_DIFF_PRESS_DEVTYPE_SDP33:
			/* fallthrough */
			smodel = AIRSPEED_SENSOR_MODEL_SDP3X;
			break;

		default:
			smodel = AIRSPEED_SENSOR_MODEL_MEMBRANE;
			break;
		}

		/* don't risk to feed negative airspeed into the system */
		airspeed.indicated_airspeed_m_s = calc_IAS_corrected((enum AIRSPEED_COMPENSATION_MODEL)
						  _param_cal_air_cmodel.get(),
						  smodel, _param_cal_air_tubelen.get(), _param_cal_air_tubed_mm.get(),
						  diff_pres.differential_pressure_filtered_pa, air_data.baro_pressure_pa,
						  air_temperature_celsius);

		airspeed.true_airspeed_m_s = calc_TAS_from_EAS(airspeed.indicated_airspeed_m_s, air_data.baro_pressure_pa,
					     air_temperature_celsius); // assume that EAS = IAS as we don't have an EAS-scale here

		airspeed.air_temperature_celsius = air_temperature_celsius;

		// populate airspeed with primary and publish
		if (PX4_ISFINITE(airspeed.indicated_airspeed_m_s) && PX4_ISFINITE(airspeed.true_airspeed_m_s)) {
			airspeed.timestamp = hrt_absolute_time();
			_airspeed_pub.publish(airspeed);
		}
	}

	// check failover and report
	if (_last_failover_count != _voter.failover_count()) {
		uint32_t flags = _voter.failover_state();
		int failover_index = _voter.failover_index();

		if (flags == DataValidator::ERROR_FLAG_NO_ERROR) {
			if (failover_index != -1) {
				// we switched due to a non-critical reason. No need to panic.
				PX4_INFO("differential_pressure switch from #%i", failover_index);
			}

		} else {
			if (failover_index != -1) {
				mavlink_log_emergency(&_mavlink_log_pub, "differential_pressure:#%i failed: %s%s%s%s%s!, reconfiguring priorities",
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

void VehicleAirspeed::PrintStatus()
{
	PX4_INFO("selected differential pressure: %d (%d)", _selected_sensor_device_id, _selected_sensor_sub_index);
	perf_print_counter(_cycle_perf);
	_voter.print();
}
