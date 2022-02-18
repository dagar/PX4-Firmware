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

#include "../PreFlightCheck.hpp"

#include <HealthFlags.h>
#include <math.h>
#include <lib/parameters/param.h>
#include <systemlib/mavlink_log.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/sensors_status_gps.h>

using namespace time_literals;

bool PreFlightCheck::gpsCheck(orb_advert_t *mavlink_log_pub, vehicle_status_s &vehicle_status, const bool optional,
			      const bool report_fail)
{
	bool success = true; // start with a pass and change to a fail if any test fails

	int32_t arm_without_gps = 0;
	param_get(param_find("COM_ARM_WO_GPS"), &arm_without_gps);

	bool gps_success = true;
	bool gps_present = true;

	// Get estimator status data if available and exit with a fail recorded if not
	uORB::SubscriptionData<sensors_status_gps_s> sensors_status_gps_sub{ORB_ID(sensors_status_gps)};
	const sensors_status_gps_s &status = sensors_status_gps_sub.get();

	if (status.timestamp == 0) {
		success = false;
		goto out;
	}

	for (int i = 0; i < 2; i++) {
		if (report_fail) {
			// Only report the first failure to avoid spamming
			const char *message = nullptr;

			if (status.status_fix[i]) {
				message = "Preflight%s: GPS%d fix too low";

			} else if (status.status_nsats[i]) {
				message = "Preflight%s: GPS%d not enough Satellites";

			} else if (status.status_pdop[i]) {
				message = "Preflight%s: GPS%d PDOP too high";

			} else if (status.status_hacc[i]) {
				message = "Preflight%s: GPS%d Horizontal Pos Error too high";

			} else if (status.status_vacc[i]) {
				message = "Preflight%s: GPS%d Vertical Pos Error too high";

			} else if (status.status_sacc[i]) {
				message = "Preflight%s: GPS%d Speed Accuracy too low";

			} else if (status.status_hdrift[i]) {
				message = "Preflight%s: GPS%d Horizontal Pos Drift too high";

			} else if (status.status_vdrift[i]) {
				message = "Preflight%s: GPS%d Vertical Pos Drift too high";

			} else if (status.status_hspeed[i]) {
				message = "Preflight%s: GPS%d Hor Speed Drift too high";

			} else if (status.status_vspeed[i]) {
				message = "Preflight%s: GPS%d Vert Speed Drift too high";

			} else if (status.healthy[i]) {
				// if we land here there was a new flag added and the code not updated. Show a generic message.
				message = "Preflight%s: Poor GPS%d Quality";
			}

			if (message) {
				if (!arm_without_gps) {
					mavlink_log_critical(mavlink_log_pub, message, " Fail", i);

				} else {
					mavlink_log_warning(mavlink_log_pub, message, "", i);
				}
			}
		}
	}

	gps_success = false;

	if (!arm_without_gps) {
		success = false;
		goto out;
	}

out:
	set_health_flags(subsystem_info_s::SUBSYSTEM_TYPE_GPS, gps_present, !arm_without_gps, gps_success, vehicle_status);
	return success;
}
