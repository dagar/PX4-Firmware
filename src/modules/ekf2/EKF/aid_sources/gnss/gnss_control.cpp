/****************************************************************************
 *
 *   Copyright (c) 2021-2024 PX4 Development Team. All rights reserved.
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

#include "ekf.h"
#include <mathlib/mathlib.h>

void Ekf::controlGpsFusion(const imuSample &imu_delayed)
{
	if (!_gps_buffer || (_params.gnss_ctrl == 0)) {
		stopGpsFusion();
		return;
	}

	_gps_hgt_b_est.predict(_dt_ekf_avg);

	// yaw estimator
	if (!gyro_bias_inhibited()) {
		_yawEstimator.setGyroBias(getGyroBias(), _control_status.flags.vehicle_at_rest);
	}

	// run EKF-GSF yaw estimator once per imu_delayed update
	_yawEstimator.predict(imu_delayed.delta_ang, imu_delayed.delta_ang_dt,
			      imu_delayed.delta_vel, imu_delayed.delta_vel_dt,
			      (_control_status.flags.in_air && !_control_status.flags.vehicle_at_rest));

	// check for arrival of new sensor data at the fusion time horizon
	if (_gps_buffer && _gps_buffer->pop_first_older_than(imu_delayed.time_us, &_gps_sample_delayed)) {
		const gnssSample &gnss_sample = _gps_sample_delayed;


		bool reset = false;

		if (runGnssChecks(gnss_sample) && isTimedOut(_last_gps_fail_us, (uint64_t)_min_gps_health_time_us / 2)) {
			if (isTimedOut(_last_gps_fail_us, (uint64_t)_min_gps_health_time_us)) {
				// First time checks are passing, latching.
				_gps_checks_passed = true;
			}

			collect_gps(gnss_sample);

			updateGNSSYawEstimator(gnss_sample);

		} else {
			if (_control_status.flags.gps && isTimedOut(_last_gps_pass_us, _params.reset_timeout_max)) {
				_warning_events.flags.gps_quality_poor = true;
				ECL_WARN("GNSS quality poor - stopping use");
				stopGpsFusion();
			}
		}

		if (isYawFailure()
		    && isTimedOut(_time_last_hor_vel_fuse, _params.EKFGSF_reset_delay)
		    && (_time_last_hor_vel_fuse > _time_last_on_ground_us)) {

			if (tryYawEmergencyReset()) {
				//ECL_WARN("yaw failure %s, resetting", AID_SRC_NAME);
				reset = true;
			}
		}

		const bool is_inflight_nav_failure = _control_status.flags.in_air
						     && isTimedOut(_time_last_hor_vel_fuse, _params.reset_timeout_max)
						     && isTimedOut(_time_last_hor_pos_fuse, _params.reset_timeout_max)
						     && (_time_last_hor_vel_fuse > _time_last_on_ground_us)
						     && (_time_last_hor_pos_fuse > _time_last_on_ground_us);


		reset = is_inflight_nav_failure;

		bool common_starting_conditions_passing = _gps_checks_passed
				&& _control_status.flags.tilt_align
				&& !isNewestSampleRecent(_time_last_gps_buffer_push, 2 * GNSS_MAX_INTERVAL);

#if defined(CONFIG_EKF2_GNSS_YAW)
		controlGNSSYawFusion(gnss_sample);
#endif // CONFIG_EKF2_GNSS_YAW

		controlGNSSHorizontalVelocityFusion(gnss_sample, common_starting_conditions_passing, reset);
		controlGNSSVerticalVelocityFusion(gnss_sample, common_starting_conditions_passing, reset);

		controlGNSSHorizontalPositionFusion(gnss_sample, common_starting_conditions_passing, reset);
		controlGNSSHeightFusion(gnss_sample);

	} else if (_control_status.flags.gps) {
		if (!isNewestSampleRecent(_time_last_gps_buffer_push, _params.reset_timeout_max)) {
			stopGpsFusion();
			_warning_events.flags.gps_data_stopped = true;
			ECL_WARN("GPS data stopped");
		}
	}
}

void Ekf::stopGpsFusion()
{
	if (_control_status.flags.gps) {
		ECL_INFO("stopping GNSS fusion");

		_last_gps_fail_us = 0;
		_last_gps_pass_us = 0;

		stopGpsHgtFusion();

		_control_status.flags.gps = false;
		_control_status.flags.gps_hgt = false;
		_control_status.flags.gnss_vel_xy = false;
		_control_status.flags.gnss_vel_z = false;
		_control_status.flags.gps_yaw = false;

		_yawEstimator.reset();
	}
}
