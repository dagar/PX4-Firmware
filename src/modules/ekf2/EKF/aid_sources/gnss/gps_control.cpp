/****************************************************************************
 *
 *   Copyright (c) 2021-2022 PX4 Development Team. All rights reserved.
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
 * @file gps_control.cpp
 * Control functions for ekf GNSS fusion
 */

#include "ekf.h"
#include <mathlib/mathlib.h>

void Ekf::controlGpsFusion(const imuSample &imu_delayed)
{
	if (!_gps_buffer || (_params.gnss_ctrl == 0)) {
		stopGpsFusion();
		return;
	}

	_gps_intermittent = !isNewestSampleRecent(_time_last_gps_buffer_push, 2 * GNSS_MAX_INTERVAL);

	// check for arrival of new sensor data at the fusion time horizon
	const bool gps_data_ready = _gps_buffer->pop_first_older_than(imu_delayed.time_us, &_gps_sample_delayed);
	_gps_data_ready = gps_data_ready;

	if (gps_data_ready) {
		const gnssSample &gnss_sample = _gps_sample_delayed;

		if (runGnssChecks(gnss_sample) && isTimedOut(_last_gps_fail_us, (uint64_t)_min_gps_health_time_us / 2)) {
			if (isTimedOut(_last_gps_fail_us, (uint64_t)_min_gps_health_time_us)) {
				// First time checks are passing, latching.
				_gps_checks_passed = true;
			}

			collect_gps(gnss_sample);

		} else {
			// Skip this sample
			_gps_data_ready = false;

			if (_control_status.flags.gps && isTimedOut(_last_gps_pass_us, _params.reset_timeout_max)) {
				stopGpsFusion();
				_warning_events.flags.gps_quality_poor = true;
				ECL_WARN("GPS quality poor - stopping use");
			}
		}

		bool reset = false;



		const bool is_inflight_nav_failure = _control_status.flags.in_air
						     && isTimedOut(_time_last_hor_vel_fuse, _params.reset_timeout_max)
						     && isTimedOut(_time_last_hor_pos_fuse, _params.reset_timeout_max)
						     && (_time_last_hor_vel_fuse > _time_last_on_ground_us)
						     && (_time_last_hor_pos_fuse > _time_last_on_ground_us);

		reset = is_inflight_nav_failure;

		bool common_starting_conditions_passing = _control_status.flags.tilt_align && _gps_checks_passed;

#if defined(CONFIG_EKF2_GNSS_YAW)
		controlGpsYawFusion(gnss_sample);
#endif // CONFIG_EKF2_GNSS_YAW

		controlGnssHorizontalVelocityFusion(imu_delayed, gnss_sample, common_starting_conditions_passing, reset);
		controlGnssVerticalVelocityFusion(gnss_sample, common_starting_conditions_passing, reset);

		controlGnssHorizontalPositionFusion(gnss_sample, common_starting_conditions_passing, reset);
		controlGnssHeightFusion(gnss_sample);



	} else if (_control_status.flags.gps) {
		if (!isNewestSampleRecent(_time_last_gps_buffer_push, _params.reset_timeout_max)) {
			stopGpsFusion();
			_warning_events.flags.gps_data_stopped = true;
			ECL_WARN("GPS data stopped");
		}
	}
}

#if defined(CONFIG_EKF2_GNSS_YAW)
void Ekf::controlGpsYawFusion(const gnssSample &gps_sample)
{
	if (!(_params.gnss_ctrl & static_cast<int32_t>(GnssCtrl::YAW))
	    || _control_status.flags.gps_yaw_fault) {

		stopGpsYawFusion();
		return;
	}

	const bool is_new_data_available = PX4_ISFINITE(gps_sample.yaw);

	if (is_new_data_available) {

		updateGpsYaw(gps_sample);

		const bool continuing_conditions_passing = _control_status.flags.tilt_align;

		const bool is_gps_yaw_data_intermittent = !isNewestSampleRecent(_time_last_gps_yaw_buffer_push,
				2 * GNSS_YAW_MAX_INTERVAL);

		const bool starting_conditions_passing = continuing_conditions_passing
				&& _gps_checks_passed
				&& !is_gps_yaw_data_intermittent
				&& !_gps_intermittent;

		if (_control_status.flags.gps_yaw) {
			if (continuing_conditions_passing) {

				fuseGpsYaw(gps_sample.yaw_offset);

				const bool is_fusion_failing = isTimedOut(_aid_src_gnss_yaw.time_last_fuse, _params.reset_timeout_max);

				if (is_fusion_failing) {
					stopGpsYawFusion();

					// Before takeoff, we do not want to continue to rely on the current heading
					// if we had to stop the fusion
					if (!_control_status.flags.in_air) {
						ECL_INFO("clearing yaw alignment");
						_control_status.flags.yaw_align = false;
					}
				}

			} else {
				// Stop GPS yaw fusion but do not declare it faulty
				stopGpsYawFusion();
			}

		} else {
			if (starting_conditions_passing) {
				// Try to activate GPS yaw fusion
				const bool not_using_ne_aiding = !_control_status.flags.gps && !_control_status.flags.aux_gpos;

				if (!_control_status.flags.in_air
				    || !_control_status.flags.yaw_align
				    || not_using_ne_aiding) {

					// Reset before starting the fusion
					if (resetYawToGps(gps_sample.yaw, gps_sample.yaw_offset)) {

						resetAidSourceStatusZeroInnovation(_aid_src_gnss_yaw);

						_control_status.flags.gps_yaw = true;
						_control_status.flags.yaw_align = true;
					}

				} else if (!_aid_src_gnss_yaw.innovation_rejected) {
					// Do not force a reset but wait for the consistency check to pass
					_control_status.flags.gps_yaw = true;
					fuseGpsYaw(gps_sample.yaw_offset);
				}

				if (_control_status.flags.gps_yaw) {
					ECL_INFO("starting GPS yaw fusion");
				}
			}
		}

	} else if (_control_status.flags.gps_yaw
		   && !isNewestSampleRecent(_time_last_gps_yaw_buffer_push, _params.reset_timeout_max)) {

		// No yaw data in the message anymore. Stop until it comes back.
		stopGpsYawFusion();
	}
}

void Ekf::stopGpsYawFusion()
{
	if (_control_status.flags.gps_yaw) {

		_control_status.flags.gps_yaw = false;

		ECL_INFO("stopping GPS yaw fusion");
	}
}
#endif // CONFIG_EKF2_GNSS_YAW

void Ekf::stopGpsFusion()
{
	if (_control_status.flags.gps) {
		ECL_INFO("stopping GPS position and velocity fusion");

		_last_gps_fail_us = 0;
		_last_gps_pass_us = 0;

		_control_status.flags.gps = false;
	}

	stopGpsHgtFusion();
#if defined(CONFIG_EKF2_GNSS_YAW)
	stopGpsYawFusion();
#endif // CONFIG_EKF2_GNSS_YAW

	_yawEstimator.reset();
}
