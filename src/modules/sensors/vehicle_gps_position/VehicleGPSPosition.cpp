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

#include "VehicleGPSPosition.hpp"

#include <px4_platform_common/log.h>
#include <lib/geo/geo.h>
#include <lib/mathlib/mathlib.h>

namespace sensors
{
VehicleGPSPosition::VehicleGPSPosition() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	for (auto &ts : _timesync) {
		ts.set_source(timesync_status_s::SOURCE_GPS);
	}
}

VehicleGPSPosition::~VehicleGPSPosition()
{
	Stop();
	perf_free(_cycle_perf);
}

bool VehicleGPSPosition::Start()
{
	// force initial updates
	ParametersUpdate(true);

	ScheduleNow();

	return true;
}

void VehicleGPSPosition::Stop()
{
	Deinit();

	// clear all registered callbacks
	for (auto &sub : _sensor_gps_sub) {
		sub.unregisterCallback();
	}
}

void VehicleGPSPosition::ParametersUpdate(bool force)
{
	// Check if parameters have changed
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		updateParams();

		if (_param_sens_gps_mask.get() == 0) {
			_sensor_gps_sub[0].registerCallback();

		} else {
			for (auto &sub : _sensor_gps_sub) {
				sub.registerCallback();
			}
		}

		_gps_blending.setBlendingUseSpeedAccuracy(_param_sens_gps_mask.get() & BLEND_MASK_USE_SPD_ACC);
		_gps_blending.setBlendingUseHPosAccuracy(_param_sens_gps_mask.get() & BLEND_MASK_USE_HPOS_ACC);
		_gps_blending.setBlendingUseVPosAccuracy(_param_sens_gps_mask.get() & BLEND_MASK_USE_VPOS_ACC);
		_gps_blending.setBlendingTimeConstant(_param_sens_gps_tau.get());
		_gps_blending.setPrimaryInstance(_param_sens_gps_prime.get());
	}
}

void VehicleGPSPosition::Run()
{
	perf_begin(_cycle_perf);
	ParametersUpdate();

	// GPS blending
	ScheduleDelayed(500_ms); // backup schedule

	// Check all GPS instance
	bool any_gps_updated = false;
	bool gps_updated = false;

	for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
		gps_updated = _sensor_gps_sub[i].updated();

		sensor_gps_s gps_data;

		if (gps_updated) {
			any_gps_updated = true;

			if (_sensor_gps_sub[i].copy(&gps_data)) {

				const bool sync_converged = _timesync[i].sync_converged();

				if (gps_data.time_utc_usec > _gps_utc_time_us[i]) {
					_timesync[i].update(gps_data.timestamp_sample, gps_data.time_utc_usec);

				} else {
					_timesync[i].reset_filter();
				}

				_gps_utc_time_us[i] = gps_data.time_utc_usec;

				if (!sync_converged && _timesync[i].sync_converged()) {
					struct timespec ts {};
					px4_clock_gettime(CLOCK_REALTIME, &ts);

					// convert to date time
					char buf[80];
					struct tm date_time;
					time_t utc_time_sec_current = ts.tv_sec + (ts.tv_nsec / 1e9);
					localtime_r(&utc_time_sec_current, &date_time);
					strftime(buf, sizeof(buf), "%a %Y-%m-%d %H:%M:%S %Z", &date_time);
					PX4_INFO("Current system time: %s", buf);
					uint64_t ts_abstime_current_us = ts.tv_sec * 1e6 + ts.tv_nsec / 1000;

					uint64_t ts_abstime_us = hrt_absolute_time();

					if (_timesync[i].convert_local_to_remote(ts_abstime_us)) {
						if (llabs(ts_abstime_current_us - ts_abstime_us) > 10_ms) {
							ts.tv_sec = ts_abstime_us / 1000000;
							ts_abstime_us -= ts.tv_sec * 1000000;
							ts.tv_nsec = ts_abstime_us * 1000; // microseconds -> nanoseconds

							// TODO: only update if the difference exceeds threshold
							if (px4_clock_settime(CLOCK_REALTIME, &ts) == 0) {
								// convert to date time
								time_t utc_time_sec = ts.tv_sec + (ts.tv_nsec / 1e9);
								localtime_r(&utc_time_sec, &date_time);
								strftime(buf, sizeof(buf), "%a %Y-%m-%d %H:%M:%S %Z", &date_time);

								PX4_INFO("Updated System time: %s", buf);
							}
						}
					}
				}

				// TODO: once GPS time_utc_usec offset relative to timestamp_sample is estimated should we use it as the sample timestamp?
				// uint64_t gps_time = gps_data.time_utc_usec;

				// if (_timesync[i].adjust_stamp(gps_time)) {
				// 	gps_data.timestamp_sample = gps_time;
				// }

				_gps_blending.setGpsData(gps_data, i);
			}

			if (!_sensor_gps_sub[i].registered()) {
				_sensor_gps_sub[i].registerCallback();
			}
		}
	}

	if (any_gps_updated) {
		_gps_blending.update(hrt_absolute_time());

		if (_gps_blending.isNewOutputDataAvailable()) {
			Publish(_gps_blending.getOutputGpsData(), _gps_blending.getSelectedGps());
		}
	}

	perf_end(_cycle_perf);
}

void VehicleGPSPosition::Publish(const sensor_gps_s &gps, uint8_t selected)
{
	vehicle_gps_position_s gps_output{};

	gps_output.timestamp_sample = gps.timestamp_sample;

	gps_output.time_utc_usec = gps.time_utc_usec;

	uint64_t time_utc_usec = gps.time_utc_usec;
	_timesync[0].convert_remote_to_local(time_utc_usec);
	gps_output.timestamp_sample2 = time_utc_usec;

	gps_output.lat = gps.lat;
	gps_output.lon = gps.lon;
	gps_output.alt = gps.alt;
	gps_output.alt_ellipsoid = gps.alt_ellipsoid;
	gps_output.s_variance_m_s = gps.s_variance_m_s;
	gps_output.c_variance_rad = gps.c_variance_rad;
	gps_output.eph = gps.eph;
	gps_output.epv = gps.epv;
	gps_output.hdop = gps.hdop;
	gps_output.vdop = gps.vdop;
	gps_output.pdop = sqrtf(gps.hdop * gps.hdop + gps.vdop * gps.vdop);
	gps_output.noise_per_ms = gps.noise_per_ms;
	gps_output.jamming_indicator = gps.jamming_indicator;
	gps_output.jamming_state = gps.jamming_state;
	gps_output.vel_m_s = gps.vel_m_s;
	gps_output.vel_n_m_s = gps.vel_n_m_s;
	gps_output.vel_e_m_s = gps.vel_e_m_s;
	gps_output.vel_d_m_s = gps.vel_d_m_s;
	gps_output.cog_rad = gps.cog_rad;
	gps_output.heading = gps.heading;
	gps_output.heading_offset = gps.heading_offset;
	gps_output.fix_type = gps.fix_type;
	gps_output.vel_ned_valid = gps.vel_ned_valid;
	gps_output.satellites_used = gps.satellites_used;

	gps_output.selected = selected;
	gps_output.timestamp = hrt_absolute_time();
	_vehicle_gps_position_pub.publish(gps_output);
}

void VehicleGPSPosition::PrintStatus()
{
	//PX4_INFO("selected GPS: %d", _gps_select_index);

	for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
		if (_timesync[i].sync_converged()) {

		}
	}

}

}; // namespace sensors
