/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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

#include "../PreFlightCheck.hpp"

#include <drivers/drv_hrt.h>
#include <HealthFlags.h>
#include <px4_defines.h>
#include <systemlib/mavlink_log.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/sensor_baro.h>
#include <uORB/topics/sensors_status.h>

using namespace time_literals;

static constexpr unsigned max_optional_baro_count = 4;
static constexpr unsigned max_mandatory_baro_count = 1;

bool PreFlightCheck::baroCheck(orb_advert_t *mavlink_log_pub, vehicle_status_s &status, const bool report_fail)
{
	int32_t sys_has_baro = 1;
	param_get(param_find("SYS_HAS_BARO"), &sys_has_baro);

	bool baro_fail_reported = false;

	/* check all sensors, but fail only for mandatory ones */
	for (unsigned i = 0; i < max_optional_baro_count; i++) {
		const bool required = (i < max_mandatory_baro_count) && (sys_has_baro == 1);
		const bool report_fail = (report_fail && !baro_fail_reported);

		int32_t device_id = -1;

		const bool exists = (orb_exists(ORB_ID(sensor_baro), i) == PX4_OK);
		bool valid = false;

		if (exists) {
			uORB::SubscriptionData<sensor_baro_s> baro{ORB_ID(sensor_baro), i};

			valid = (baro.get().device_id != 0) && (baro.get().timestamp != 0);

			if (!valid) {
				if (report_fail) {
					mavlink_log_critical(mavlink_log_pub, "Preflight Fail: no valid data from Baro #%u", i);
				}
			}

		} else {
			if (!optional && report_fail) {
				mavlink_log_critical(mavlink_log_pub, "Preflight Fail: Baro Sensor #%u missing", i);
			}
		}
	}


	// Get sensors status data if available and exit with a fail recorded if not
	uORB::SubscriptionData<sensors_status_s> sensors_status_baro_sub{ORB_ID(sensors_status_baro)};
	const sensors_status_s &sensors_status_baro = sensors_status_baro_sub.get();

	// Use the difference between IMU's to detect a bad calibration.
	// If a single IMU is fitted, the value being checked will be zero so this check will always pass.
	for (unsigned i = 0; i < (sizeof(sensors_status_baro.inconsistency) / sizeof(sensors_status_baro.inconsistency[0])); i++) {
		if (sensors_status_baro.device_ids[i] != 0) {
			if (sensors_status_baro.device_ids[i] == sensors_status_baro.device_id_primary) {
				set_health_flags_healthy(subsystem_info_s::SUBSYSTEM_TYPE_ABSPRESSURE, sensors_status_baro.healthy[i], status);
			}

			//const float inconsistency = accel.inconsistency[i];
		}
	}

	return valid;
}
