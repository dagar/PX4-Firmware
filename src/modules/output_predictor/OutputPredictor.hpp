/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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

#pragma once

#include "Integrator.hpp"
#include "RingBuffer.hpp"

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <lib/sensor_calibration/Accelerometer.hpp>
#include <lib/sensor_calibration/Gyroscope.hpp>
#include <lib/matrix/matrix/math.hpp>
#include <lib/perf/perf_counter.h>
#include <lib/mathlib/math/Limits.hpp>
#include <lib/matrix/matrix/math.hpp>
#include <lib/mathlib/math/filter/LowPassFilter2p.hpp>
#include <lib/mathlib/math/filter/NotchFilter.hpp>
#include <lib/geo/geo.h>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/control_state.h>
#include <uORB/topics/estimator_sensor_bias.h>
#include <uORB/topics/estimator_selector_status.h>
#include <uORB/topics/estimator_states.h>
#include <uORB/topics/estimator_status.h>
#include <uORB/topics/output_predictor_status.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/sensor_selection.h>
#include <uORB/topics/vehicle_acceleration.h>
#include <uORB/topics/vehicle_angular_acceleration.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_odometry.h>

using namespace matrix;
using namespace time_literals;

class OutputPredictor : public ModuleBase<OutputPredictor>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	OutputPredictor();
	~OutputPredictor() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

	void PrintStatus();

private:

	static constexpr int MAX_SENSOR_COUNT = 4;

	void Run() override;

	void CheckFiltersAccel();
	void CheckFiltersGyro();
	void SensorBiasUpdate(bool force);
	bool SensorSelectionUpdate(bool force = false);

	template<typename T>
	static constexpr T sq(T in) { return in * in; }

	struct outputSample {
		Quatf quat_nominal;	///< nominal quaternion describing vehicle attitude
		Vector3f velocity_body;
		Vector3f velocity_NED;	///< NED velocity estimate in earth frame (m/sec)
		Vector3f position;	///< NED position estimate in earth frame (m/sec)
		float vertical_velocity;       ///< Vertical velocity calculated using alternative algorithm (m/sec)
		float vertical_velocity_integ; ///< Integral of vertical velocity (m)
		uint64_t time_us;	///< timestamp of the measurement (uSec)
	};

	/*
	* Implement a strapdown INS algorithm using the latest IMU data at the current time horizon.
	* Buffer the INS states and calculate the difference with the EKF states at the delayed fusion time horizon.
	* Calculate delta angle, delta velocity and velocity corrections from the differences and apply them at the
	* current time horizon so that the INS states track the EKF states at the delayed fusion time horizon.
	* The inspiration for using a complementary filter to correct for time delays in the EKF
	* is based on the work by A Khosravian:
	* “Recursive Attitude Estimation in the Presence of Multi-rate and Multi-delay Vector Measurements”
	* A Khosravian, J Trumpf, R Mahony, T Hamel, Australian National University
	*/
	void alignOutputFilter(const Quatf &quat_nominal, const Vector3f &vel, const Vector3f &pos);
	void CalculateQuaternionOutput(const hrt_abstime &timestamp_sample, const Vector3f &imu_delta_angle, const float dt);
	void CalculateOutputStates(const hrt_abstime &timestamp_sample, const Vector3f &imu_delta_velocity, const float dt);
	void CorrectOutputStates(const hrt_abstime &imu_timestamp_sample, const hrt_abstime &timestamp_estimate, const Quatf &q,
				 const Vector3f &velocity, const Vector3f &position);

	// get the derivative of the vertical position of the body frame origin in local NED earth frame
	float getVerticalPositionDerivative() const { return _output_new.vertical_velocity - _vel_imu_rel_body_ned(2); }

	uORB::Publication<control_state_s>                _control_state_pub{ORB_ID::control_state};
	uORB::Publication<output_predictor_status_s>      _output_predictor_status_pub{ORB_ID::output_predictor_status};
	uORB::Publication<vehicle_acceleration_s>         _vehicle_acceleration_pub{ORB_ID::vehicle_acceleration};
	uORB::Publication<vehicle_angular_acceleration_s> _vehicle_angular_acceleration_pub{ORB_ID::vehicle_angular_acceleration};
	uORB::Publication<vehicle_angular_velocity_s>     _vehicle_angular_velocity_pub{ORB_ID::vehicle_angular_velocity};
	uORB::Publication<vehicle_attitude_s>             _vehicle_attitude_pub{ORB_ID::vehicle_attitude};
	uORB::Publication<vehicle_local_position_s>       _vehicle_local_position_pub{ORB_ID::vehicle_local_position};
	uORB::Publication<vehicle_global_position_s>      _vehicle_global_position_pub{ORB_ID::vehicle_global_position};

	uORB::Subscription _estimator_selector_status_sub{ORB_ID::estimator_selector_status};
	uORB::Subscription _estimator_sensor_bias_sub{ORB_ID::estimator_sensor_bias};
	uORB::Subscription _estimator_states_sub{ORB_ID::estimator_states};
	uORB::Subscription _estimator_status_sub{ORB_ID::estimator_status};
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};

	uORB::SubscriptionCallbackWorkItem _sensor_accel_sub{this, ORB_ID(sensor_accel)};
	uORB::SubscriptionCallbackWorkItem _sensor_gyro_sub{this, ORB_ID(sensor_gyro)};
	uORB::SubscriptionCallbackWorkItem _sensor_selection_sub{this, ORB_ID(sensor_selection)};

	hrt_abstime _timestamp_sample_last_accel{0};
	hrt_abstime _timestamp_sample_last_gyro{0};

	Integrator       _accel_integrator{};
	IntegratorConing _gyro_integrator{};

	RingBuffer<outputSample, 100> _output_buffer;

	float _dt_imu_avg{0.01f};	// average imu update period in s
	float _dt_ekf_avg{0.01}; ///< average update rate of the ekf

	// output predictor states
	Vector3f _delta_angle_corr{};      ///< delta angle correction vector (rad)
	Vector3f _velocity_error_integ{};         ///< integral of velocity tracking error (m)
	Vector3f _velocity_body_error_integ{};         ///< integral of velocity tracking error (m)
	Vector3f _position_error_integ{};         ///< integral of position tracking error (m.s)

	outputSample _output_new{};               // filter output on the non-delayed time horizon

	Vector3f _vel_imu_rel_body_ned{};           // velocity of IMU relative to body origin in NED earth frame

	static constexpr const float kInitialRateHz{1000.0f}; /**< sensor update rate used for initialization */

	// angular velocity filters
	math::LowPassFilter2p<matrix::Vector3f> _lp_filter_angular_velocity{kInitialRateHz, 30.0f};
	math::NotchFilter<matrix::Vector3f> _notch_filter_angular_velocity{};

	// angular acceleration filter
	math::LowPassFilter2p<matrix::Vector3f> _lp_filter_angular_acceleration{kInitialRateHz, 30.0f};

	math::LowPassFilter2p<matrix::Vector3f> _lp_filter_linear_acceleration{kInitialRateHz, 30.0f};
	math::LowPassFilter2p<matrix::Vector3f> _lp_filter_velocity_body{kInitialRateHz, 30.0f};
	math::LowPassFilter2p<matrix::Vector3f> _lp_filter_velocity_ned{kInitialRateHz, 30.0f};

	matrix::Vector3f _linear_acceleration_prev{0.f, 0.f, 0.f};

	hrt_abstime _last_angular_velocity_publish{0};

	float _filter_sample_rate_accel{kInitialRateHz};
	float _filter_sample_rate_gyro{kInitialRateHz};

	float _interval_sum_accel{0.f};
	float _interval_sum_gyro{0.f};

	float _interval_count_accel{0.f};
	float _interval_count_gyro{0.f};

	uint8_t _required_sample_updates_accel{0}; /**< number or sensor publications required for configured rate */
	uint8_t _required_sample_updates_gyro{0}; /**< number or sensor publications required for configured rate */

	float _update_rate_hz_accel{kInitialRateHz}; /**< current rate-controller loop update rate in [Hz] */
	float _update_rate_hz_gyro{kInitialRateHz}; /**< current rate-controller loop update rate in [Hz] */


	uint32_t _selected_device_id_accel{0};
	uint32_t _selected_device_id_gyro{0};

	uint8_t _selected_sub_index_accel{0};
	uint8_t _selected_sub_index_gyro{0};

	bool _output_predictor_aligned{false};

	// parameters
	Vector3f _imu_pos_body;			///< xyz position of IMU in body frame (m)

	// output complementary filter tuning
	float _param_vel_tau{0.25f}; ///< velocity state correction time constant (1/sec)
	float _param_pos_tau{0.25f}; ///< position state correction time constant (1/sec)

	calibration::Accelerometer _calibration_accel{};
	calibration::Gyroscope     _calibration_gyro{};

	matrix::Vector3f _bias_accel{0.f, 0.f, 0.f};
	matrix::Vector3f _bias_gyro{0.f, 0.f, 0.f};

	matrix::Vector3f _angular_acceleration_prev{0.f, 0.f, 0.f};
	matrix::Vector3f _angular_velocity_prev{0.f, 0.f, 0.f};
	hrt_abstime _timestamp_sample_prev_gyro{0};

	perf_counter_t	_loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": update")};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::IMU_ACCEL_CUTOFF>) _param_imu_accel_cutoff,

		(ParamFloat<px4::params::IMU_GYRO_CUTOFF>) _param_imu_gyro_cutoff,
		(ParamFloat<px4::params::IMU_GYRO_NF_FREQ>) _param_imu_gyro_nf_freq,
		(ParamFloat<px4::params::IMU_GYRO_NF_BW>) _param_imu_gyro_nf_bw,
		(ParamInt<px4::params::IMU_GYRO_RATEMAX>) _param_imu_gyro_ratemax,
		(ParamFloat<px4::params::IMU_DGYRO_CUTOFF>) _param_imu_dgyro_cutoff
	)

	// TODO:
	//  - start by consuming estimator_states and side-by-side comparison with vehicle_attitude, vehicle_local_position, etc
	//  - filter accel
	//  - filter velocity
	//  - filter attitude?
	//  - parameterize IMU position offsets to body
	//  -  track per IMU?
	//        - full buffer of each IMU
	//        - per IMU output tracking error?
	//  -  track per estimator?
	//       - per estimator delayed time horizon quaternion, velocity, position

	// publish body frame rates, accel, velocity
	// publish local NED frame accel, velocity, position
	//   control_state?
	//  - if local position origin valid then also publish global_position

	// IDEA: return acceleration at the body origin
	//       - https://github.com/PX4/PX4-ECL/issues/621

	// - The acceleration returned is at the sensor frame.
	// - The only way to return acceleration at the body origin would be to use angular accelerations
	//   which is problematic due to the issues associated with differentiating rate gyro noise.
	// - This was tried in the past using the downsampled data available to the EKF but the resulting
	//   data product was noisy when engine vibration was present.
	// - It may be possible if we do the differentiation and filtering in the sensor driver on the high rate data before downsampling.
	// - "rate acceleration"
	// The compensation was done by low pass filtering the acceleration measurement and subtracting the position offset times the angular acceleration (obtained by taking the derivative of the filtered gyro signal).


	// - this module could maintain output tracking error for all IMUs?
};
