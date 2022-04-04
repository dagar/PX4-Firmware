/****************************************************************************
 *
 *   Copyright (c) 2013-2022 PX4 Development Team. All rights reserved.
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

/*
 * @file LandDetector.cpp
 *
 * @author Johan Jansen <jnsn.johan@gmail.com>
 * @author Julian Oes <julian@oes.ch>
 */

#include "LandDetector.h"

using namespace time_literals;
using matrix::Vector3f;

namespace land_detector
{

LandDetector::LandDetector() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	for (auto &lpf : _imu_accel_lpf) {
		lpf.setAlpha(0.1f);
	}

	for (auto &lpf : _imu_gyro_lpf) {
		lpf.setAlpha(0.1f);
	}

	_land_detected.ground_contact = true;
	_land_detected.maybe_landed = true;
	_land_detected.landed = true;
	_land_detected.in_ground_effect = true;
	_land_detected.in_descend = false;
	_land_detected.has_low_throttle = false;
	_land_detected.vertical_movement = false;
	_land_detected.horizontal_movement = false;
	_land_detected.close_to_ground_or_skipped_check = true;
	_land_detected.at_rest = true;
}

LandDetector::~LandDetector()
{
	perf_free(_cycle_perf);
}

void LandDetector::start()
{
	ScheduleDelayed(50_ms);
	_vehicle_local_position_sub.registerCallback();
}

void LandDetector::Run()
{
	// push backup schedule
	ScheduleDelayed(50_ms);

	perf_begin(_cycle_perf);

	if (_parameter_update_sub.updated() || (_land_detected.timestamp == 0)) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		updateParams();
		_update_params();

		_total_flight_time = static_cast<uint64_t>(_param_total_flight_time_high.get()) << 32;
		_total_flight_time |= static_cast<uint32_t>(_param_total_flight_time_low.get());
	}

	actuator_armed_s actuator_armed;

	if (_actuator_armed_sub.update(&actuator_armed)) {
		_armed = actuator_armed.armed;
	}

	vehicle_acceleration_s vehicle_acceleration;

	if (_vehicle_acceleration_sub.update(&vehicle_acceleration)) {
		_acceleration = Vector3f{vehicle_acceleration.xyz};
	}

	vehicle_angular_velocity_s vehicle_angular_velocity{};

	if (_vehicle_angular_velocity_sub.update(&vehicle_angular_velocity)) {
		_angular_velocity = Vector3f{vehicle_angular_velocity.xyz};

		if (false && _at_rest) {
			if (_angular_velocity.longerThan(math::radians(3.f))) {
				_angular_velocity.print();

				if (vehicle_angular_velocity.timestamp_sample > _time_last_move_detect_us) {
					_time_last_move_detect_us = vehicle_angular_velocity.timestamp_sample;
				}
			}
		}
	}

	_vehicle_local_position_sub.update(&_vehicle_local_position);
	_vehicle_status_sub.update(&_vehicle_status);

	_update_topics();

	if (!_dist_bottom_is_observable) {
		// we consider the distance to the ground observable if the system is using a range sensor
		_dist_bottom_is_observable = _vehicle_local_position.dist_bottom_sensor_bitfield &
					     vehicle_local_position_s::DIST_BOTTOM_SENSOR_RANGE;
	}

	// Increase land detection time if not close to ground
	if (_dist_bottom_is_observable && !_vehicle_local_position.dist_bottom_valid) {
		_set_hysteresis_factor(3);

	} else {
		_set_hysteresis_factor(1);
	}

	const hrt_abstime now_us = hrt_absolute_time();

	_freefall_hysteresis.set_state_and_update(_get_freefall_state(), now_us);
	_ground_contact_hysteresis.set_state_and_update(_get_ground_contact_state(), now_us);
	_maybe_landed_hysteresis.set_state_and_update(_get_maybe_landed_state(), now_us);
	_landed_hysteresis.set_state_and_update(_get_landed_state(), now_us);
	_ground_effect_hysteresis.set_state_and_update(_get_ground_effect_state(), now_us);

	const bool freefallDetected = _freefall_hysteresis.get_state();
	const bool ground_contactDetected = _ground_contact_hysteresis.get_state();
	const bool maybe_landedDetected = _maybe_landed_hysteresis.get_state();
	const bool landDetected = _landed_hysteresis.get_state();
	const bool in_ground_effect = _ground_effect_hysteresis.get_state();

	UpdateVehicleAtRest(now_us);

	// publish at 1 Hz, very first time, or when the result has changed
	if ((hrt_elapsed_time(&_land_detected.timestamp) >= 1_s) ||
	    (_land_detected.landed != landDetected) ||
	    (_land_detected.freefall != freefallDetected) ||
	    (_land_detected.maybe_landed != maybe_landedDetected) ||
	    (_land_detected.ground_contact != ground_contactDetected) ||
	    (_land_detected.in_ground_effect != in_ground_effect) ||
	    (_land_detected.at_rest != _at_rest)) {

		if (!landDetected && _land_detected.landed && _takeoff_time == 0) { /* only set take off time once, until disarming */
			// We did take off
			_takeoff_time = now_us;
		}

		if (!_land_detected.at_rest && _at_rest) {
			PX4_INFO("at rest");

		} else if (_land_detected.at_rest && !_at_rest) {
			PX4_INFO("moving");
		}

		_land_detected.landed = landDetected;
		_land_detected.freefall = freefallDetected;
		_land_detected.maybe_landed = maybe_landedDetected;
		_land_detected.ground_contact = ground_contactDetected;
		_land_detected.in_ground_effect = in_ground_effect;
		_land_detected.in_descend = _get_in_descend();
		_land_detected.has_low_throttle = _get_has_low_throttle();
		_land_detected.horizontal_movement = _get_horizontal_movement();
		_land_detected.vertical_movement = _get_vertical_movement();
		_land_detected.close_to_ground_or_skipped_check = _get_close_to_ground_or_skipped_check();
		_land_detected.at_rest = _at_rest;
		_land_detected.timestamp = hrt_absolute_time();
		_vehicle_land_detected_pub.publish(_land_detected);
	}

	// set the flight time when disarming (not necessarily when landed, because all param changes should
	// happen on the same event and it's better to set/save params while not in armed state)
	if (_takeoff_time != 0 && !_armed && _previous_armed_state) {
		_total_flight_time += now_us - _takeoff_time;
		_takeoff_time = 0;

		uint32_t flight_time = (_total_flight_time >> 32) & 0xffffffff;

		_param_total_flight_time_high.set(flight_time);
		_param_total_flight_time_high.commit_no_notification();

		flight_time = _total_flight_time & 0xffffffff;

		_param_total_flight_time_low.set(flight_time);
		_param_total_flight_time_low.commit_no_notification();
	}

	_previous_armed_state = _armed;

	perf_end(_cycle_perf);

	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
	}
}

void LandDetector::UpdateVehicleAtRest(const hrt_abstime &now_us)
{
	bool moving = false;

	static constexpr uint64_t VEHICLE_IMU_TIMEOUT_US = 20_ms;

	for (int i = 0; i < _vehicle_imu_subs.size(); i++) {
		vehicle_imu_s vehicle_imu;

		if (_vehicle_imu_subs[i].update(&vehicle_imu) && (now_us < vehicle_imu.timestamp_sample + VEHICLE_IMU_TIMEOUT_US)) {

			// filtered gyro exceeding threshold
			static constexpr float GYRO_NORM_MAX = math::radians(2.f); // 2 degrees/second
			float gyro_dt_inv = 1.e6f / vehicle_imu.delta_angle_dt;
			_imu_gyro_lpf[i].update(Vector3f(vehicle_imu.delta_angle) * gyro_dt_inv);

			if (false && !moving) {
				if (_imu_gyro_lpf[i].getState().longerThan(GYRO_NORM_MAX)) {
					if (vehicle_imu.timestamp_sample > _time_last_move_detect_us) {
						PX4_INFO_RAW("imu gyro %d", i);
						_imu_gyro_lpf[i].getState().print();
						_time_last_move_detect_us = vehicle_imu.timestamp_sample;
						moving = true;
					}
				}
			}


			// accel

			float accel_dt_inv = 1.e6f / vehicle_imu.delta_velocity_dt;
			Vector3f accel_vec = Vector3f(Vector3f(vehicle_imu.delta_velocity) * accel_dt_inv).unit();

			if (_imu_accel_calibration_count[i] == vehicle_imu.accel_calibration_count) {
				const Vector3f accel_prev = _imu_accel_lpf[i].getState();
				_imu_accel_lpf[i].update(accel_vec);

				const Vector3f &accel_curr = _imu_accel_lpf[i].getState();

				if (false && !moving) {
					static constexpr float ACCEL_NORM_MAX = 0.1f;

					if (Vector3f(accel_curr - accel_prev).longerThan(ACCEL_NORM_MAX)) {
						if (vehicle_imu.timestamp_sample > _time_last_move_detect_us) {

							PX4_INFO_RAW("imu accel %d", i);
							accel_prev.print();
							accel_curr.print();
							Vector3f(accel_curr - accel_prev).print();

							_time_last_move_detect_us = vehicle_imu.timestamp_sample;
							moving = true;
						}
					}
				}

				if (false && !moving && _imu_accel_last_still_set[i]
				    && Vector3f(accel_curr - _imu_accel_last_still[i]).longerThan(0.01f)) {

					//float angle_error = AxisAnglef(Quatf(current_mag, primary_mag)).angle();

					PX4_INFO_RAW("imu accel last still %d", i);
					_imu_accel_last_still[i].print();
					accel_curr.print();
					Vector3f(accel_curr - _imu_accel_last_still[i]).print();

					if (vehicle_imu.timestamp_sample > _time_last_move_detect_us) {
						_time_last_move_detect_us = vehicle_imu.timestamp_sample;
						moving = true;
					}
				}

				if ((moving || !_at_rest) && _imu_accel_last_still_set[i]) {
					// clear last still accel
					_imu_accel_last_still_set[i] = false;

				} else if (!moving && _at_rest && !_imu_accel_last_still_set[i]) {
					bool still = false;

					if (_time_last_move_detect_us != 0) {
						if (now_us > _time_last_move_detect_us + 1_s) {
							still = true;
						}

					} else {
						if (now_us > _time_last_move_detect_us + 5_s) {
							still = true;
						}
					}

					if (still) {
						// save filtered accel when first at reset
						_imu_accel_last_still[i] = accel_curr;
						_imu_accel_last_still_set[i] = true;
					}
				}

			} else {
				_imu_accel_calibration_count[i] = vehicle_imu.accel_calibration_count;
				_imu_accel_last_still_set[i] = false;

				_imu_accel_lpf[i].reset(accel_vec);
			}


		}
	}

	for (auto &vehicle_imu_status_sub : _vehicle_imu_status_subs) {
		vehicle_imu_status_s imu_status;

		if (!moving && vehicle_imu_status_sub.update(&imu_status) && (now_us < imu_status.timestamp + VEHICLE_IMU_TIMEOUT_US)) {
			static constexpr float GYRO_VIBE_METRIC_MAX = 0.02f; // gyro_vibration_metric * dt * 4.0e4f > is_moving_scaler)
			static constexpr float ACCEL_VIBE_METRIC_MAX = 1.2f; // accel_vibration_metric * dt * 2.1e2f > is_moving_scaler

			if ((imu_status.gyro_vibration_metric > GYRO_VIBE_METRIC_MAX)
			    || (imu_status.accel_vibration_metric > ACCEL_VIBE_METRIC_MAX)
			    || !imu_status.still) {

				if (imu_status.timestamp > _time_last_move_detect_us) {
					PX4_INFO("imu status, still: %d gyro: %.6f > %.6f, accel: %.6f > %.6f", imu_status.still,
						 (double)imu_status.gyro_vibration_metric, (double)GYRO_VIBE_METRIC_MAX,
						 (double)imu_status.accel_vibration_metric, (double)ACCEL_VIBE_METRIC_MAX
						);
					_time_last_move_detect_us = imu_status.timestamp;
					moving = true;
				}
			}
		}
	}

	if (moving) {
		_at_rest = false;

	} else {
		if (now_us > _time_last_move_detect_us + 1_s) {
			_at_rest = true;
		}
	}

	if (_at_rest) {
		_time_last_still_detect_us = now_us;
	}
}

} // namespace land_detector
