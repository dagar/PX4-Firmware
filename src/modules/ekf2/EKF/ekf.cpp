/****************************************************************************
 *
 *   Copyright (c) 2015 Estimation and Control Library (ECL). All rights reserved.
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
 * 3. Neither the name ECL nor the names of its contributors may be
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
 * @file ekf.cpp
 * Core functions for ekf attitude and position estimator.
 *
 * @author Roman Bast <bapstroman@gmail.com>
 * @author Paul Riseborough <p_riseborough@live.com.au>
 */

#include "ekf.h"

#include <mathlib/mathlib.h>

bool Ekf::init(uint64_t timestamp)
{
	bool ret = initialise_interface(timestamp);
	reset();
	return ret;
}

void Ekf::reset()
{
	ECL_INFO("reset");

	_ekf24.reset();

#if defined(CONFIG_EKF2_RANGE_FINDER)
	_range_sensor.setPitchOffset(_params.rng_sens_pitch);
	_range_sensor.setCosMaxTilt(_params.range_cos_max_tilt);
	_range_sensor.setQualityHysteresis(_params.range_valid_quality_s);
#endif // CONFIG_EKF2_RANGE_FINDER

	_control_status.value = 0;
	_control_status_prev.value = 0;

	_control_status.flags.in_air = true;
	_control_status_prev.flags.in_air = true;

	_fault_status.value = 0;
	_innov_check_fail_status.value = 0;

	resetGpsDriftCheckFilters();

	_output_predictor.reset();

	// Ekf private fields
	_time_last_horizontal_aiding = 0;
	_time_last_v_pos_aiding = 0;
	_time_last_v_vel_aiding = 0;

	_time_last_hor_pos_fuse = 0;
	_time_last_hgt_fuse = 0;
	_time_last_hor_vel_fuse = 0;
	_time_last_ver_vel_fuse = 0;
	_time_last_heading_fuse = 0;
	_time_last_zero_velocity_fuse = 0;

	_last_known_pos.setZero();

	_time_acc_bias_check = 0;

	_gps_checks_passed = false;
	_gps_alt_ref = NAN;

	_baro_counter = 0;
	_mag_counter = 0;

	_time_bad_vert_accel = 0;
	_time_good_vert_accel = 0;
	_clip_counter = 0;

	resetEstimatorAidStatus(_aid_src_baro_hgt);
#if defined(CONFIG_EKF2_AIRSPEED)
	resetEstimatorAidStatus(_aid_src_airspeed);
#endif // CONFIG_EKF2_AIRSPEED
#if defined(CONFIG_EKF2_SIDESLIP)
	resetEstimatorAidStatus(_aid_src_sideslip);
#endif // CONFIG_EKF2_SIDESLIP

	resetEstimatorAidStatus(_aid_src_fake_pos);
	resetEstimatorAidStatus(_aid_src_fake_hgt);

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	resetEstimatorAidStatus(_aid_src_ev_hgt);
	resetEstimatorAidStatus(_aid_src_ev_pos);
	resetEstimatorAidStatus(_aid_src_ev_vel);
	resetEstimatorAidStatus(_aid_src_ev_yaw);
#endif // CONFIG_EKF2_EXTERNAL_VISION

	resetEstimatorAidStatus(_aid_src_gnss_hgt);
	resetEstimatorAidStatus(_aid_src_gnss_pos);
	resetEstimatorAidStatus(_aid_src_gnss_vel);

#if defined(CONFIG_EKF2_GNSS_YAW)
	resetEstimatorAidStatus(_aid_src_gnss_yaw);
#endif // CONFIG_EKF2_GNSS_YAW

	resetEstimatorAidStatus(_aid_src_mag_heading);
	resetEstimatorAidStatus(_aid_src_mag);

#if defined(CONFIG_EKF2_AUXVEL)
	resetEstimatorAidStatus(_aid_src_aux_vel);
#endif // CONFIG_EKF2_AUXVEL

#if defined(CONFIG_EKF2_OPTICAL_FLOW)
	resetEstimatorAidStatus(_aid_src_optical_flow);
	resetEstimatorAidStatus(_aid_src_terrain_optical_flow);
#endif // CONFIG_EKF2_OPTICAL_FLOW

#if defined(CONFIG_EKF2_RANGE_FINDER)
	resetEstimatorAidStatus(_aid_src_rng_hgt);
#endif // CONFIG_EKF2_RANGE_FINDER
}

bool Ekf::update()
{
	if (!_filter_initialised) {
		_filter_initialised = initialiseFilter();

		if (!_filter_initialised) {
			return false;
		}
	}

	// Only run the filter if IMU data in the buffer has been updated
	if (_imu_updated) {
		_imu_updated = false;

		// get the oldest IMU data from the buffer
		// TODO: explicitly pop at desired time horizon
		const imuSample imu_sample_delayed = _imu_buffer.get_oldest();

		if (_ekf24.update(imu_sample_delayed)) {

		}

		// control fusion of observation data
		controlFusionModes(imu_sample_delayed);

#if defined(CONFIG_EKF2_RANGE_FINDER)
		// run a separate filter for terrain estimation
		runTerrainEstimator(imu_sample_delayed);
#endif // CONFIG_EKF2_RANGE_FINDER

		_output_predictor.correctOutputStates(imu_sample_delayed.time_us, _state.quat_nominal, _state.vel, _state.pos, _state.gyro_bias, _state.accel_bias);

		return true;
	}

	return false;
}

bool Ekf::initialiseFilter()
{
	// Filter accel for tilt initialization
	const imuSample &imu_init = _imu_buffer.get_newest();

	_ekf24.initialiseFilter(imu_init);

#if defined(CONFIG_EKF2_RANGE_FINDER)
	// Initialise the terrain estimator
	initHagl();
#endif // CONFIG_EKF2_RANGE_FINDER

	// reset the output predictor state history to match the EKF initial values
	_output_predictor.alignOutputFilter(_ekf24.getStateAtFusionHorizonAsVector().quat_nominal, _ekf24.getStateAtFusionHorizonAsVector().vel, _ekf24.getStateAtFusionHorizonAsVector().pos);

	return true;
}
