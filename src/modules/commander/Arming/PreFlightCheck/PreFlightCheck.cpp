/****************************************************************************
 *
 *   Copyright (c) 2019-2020 PX4 Development Team. All rights reserved.
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
 * @file PreFlightCheck.cpp
 */

#include "PreFlightCheck.hpp"

#include <drivers/drv_hrt.h>
#include <HealthFlags.h>
#include <lib/parameters/param.h>
#include <systemlib/mavlink_log.h>
#include <uORB/Subscription.hpp>

using namespace time_literals;

bool PreFlightCheck::preflightCheck(orb_advert_t *mavlink_log_pub, vehicle_status_s &status,
				    vehicle_status_flags_s &status_flags, const bool checkGNSS, bool reportFailures, const bool prearm,
				    const hrt_abstime &time_since_boot)
{
	reportFailures = (reportFailures && status_flags.condition_system_hotplug_timeout
			  && !status_flags.condition_calibration_enabled);

	bool failed = false;

	failed = failed || !airframeCheck(mavlink_log_pub, status);
	failed = failed || !magnetometerCheck(mavlink_log_pub, status, reportFailures);
	failed = failed || !accelerometerCheck(mavlink_log_pub, status, reportFailures);
	failed = failed || !gyroCheck(mavlink_log_pub, status, reportFailures);
	failed = failed || !baroCheck(mavlink_log_pub, status, reportFailures);
	failed = failed || !imuConsistencyCheck(mavlink_log_pub, status, reportFailures);
	failed = failed || !failureDetectorCheck(mavlink_log_pub, status, reportFailures, prearm);
	failed = failed || !manualControlCheck(mavlink_log_pub, reportFailures);
	failed = failed || !cpuResourceCheck(mavlink_log_pub, reportFailures);

	/* ---- AIRSPEED ---- */
	/* Perform airspeed check only if circuit breaker is not engaged and it's not a rotary wing */
	if (!status_flags.circuit_breaker_engaged_airspd_check &&
	    ((status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING) || status.is_vtol)) {

		int32_t fw_airspeed_mode = 0;
		param_get(param_find("FW_ARSP_MODE"), &fw_airspeed_mode);

		if (fw_airspeed_mode == 0) {
			if (!airspeedCheck(mavlink_log_pub, status, optional, reportFailures, prearm)) {
				failed = true;
			}
		}
	}

	/* ---- RC CALIBRATION ---- */
	if (status.rc_input_mode == vehicle_status_s::RC_IN_MODE_DEFAULT) {
		if (rcCalibrationCheck(mavlink_log_pub, reportFailures, status.is_vtol) != OK) {
			if (reportFailures) {
				mavlink_log_critical(mavlink_log_pub, "RC calibration check failed");
			}

			failed = true;

			set_health_flags(subsystem_info_s::SUBSYSTEM_TYPE_RCRECEIVER, status_flags.rc_signal_found_once, true, false, status);
			status_flags.rc_calibration_valid = false;

		} else {
			// The calibration is fine, but only set the overall health state to true if the signal is not currently lost
			status_flags.rc_calibration_valid = true;
			set_health_flags(subsystem_info_s::SUBSYSTEM_TYPE_RCRECEIVER, status_flags.rc_signal_found_once, true,
					 !status.rc_signal_lost, status);
		}
	}

	/* ---- SYSTEM POWER ---- */
	if (status_flags.condition_power_input_valid && !status_flags.circuit_breaker_engaged_power_check) {
		if (!powerCheck(mavlink_log_pub, status, reportFailures, prearm)) {
			failed = true;
		}
	}

	/* ---- Navigation EKF ---- */
	// only check EKF2 data if EKF2 is selected as the estimator and GNSS checking is enabled
	int32_t estimator_type = -1;

	if (status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING && !status.is_vtol) {
		param_get(param_find("SYS_MC_EST_GROUP"), &estimator_type);

	} else {
		// EKF2 is currently the only supported option for FW & VTOL
		estimator_type = 2;
	}

	if (estimator_type == 2) {
		// don't report ekf failures for the first 10 seconds to allow time for the filter to start
		bool report_ekf_fail = (time_since_boot > 10_s);

		if (!ekf2Check(mavlink_log_pub, status, false, reportFailures && report_ekf_fail, checkGNSS)) {
			failed = true;
		}

		if (!ekf2CheckStates(mavlink_log_pub, reportFailures && report_ekf_fail)) {
			failed = true;
		}
	}

	/* Report status */
	return !failed;
}
