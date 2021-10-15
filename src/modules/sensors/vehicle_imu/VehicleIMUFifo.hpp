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

#pragma once

#include "Integrator.hpp"

#include <lib/mathlib/math/Limits.hpp>
#include <lib/mathlib/math/WelfordMean.hpp>
#include <lib/matrix/matrix/math.hpp>
#include <lib/perf/perf_counter.h>
#include <lib/sensor_calibration/Accelerometer.hpp>
#include <lib/sensor_calibration/Gyroscope.hpp>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_imu_fifo.h>
#include <uORB/topics/vehicle_imu.h>
#include <uORB/topics/vehicle_imu_status.h>

using namespace time_literals;

namespace sensors
{

class VehicleIMUFifo : public ModuleParams, public px4::ScheduledWorkItem
{
public:
	VehicleIMUFifo() = delete;
	VehicleIMUFifo(int instance, uint32_t device_id, const px4::wq_config_t &config);

	~VehicleIMUFifo() override;

	bool Start();
	void Stop();

	void PrintStatus();

private:

	void ParametersUpdate(bool force = false);
	bool Publish(const matrix::Vector3f &delta_angle, uint16_t delta_angle_dt,
		     const matrix::Vector3f &delta_velocity, uint16_t delta_velocity_dt);
	void Run() override;

	void UpdateIntegratorConfiguration();
	void UpdateAccelVibrationMetrics(const matrix::Vector3f &acceleration);
	void UpdateGyroVibrationMetrics(const matrix::Vector3f &angular_velocity);

	uORB::PublicationMulti<vehicle_imu_s> _vehicle_imu_pub{ORB_ID(vehicle_imu)};
	uORB::PublicationMulti<vehicle_imu_status_s> _vehicle_imu_status_pub{ORB_ID(vehicle_imu_status)};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::SubscriptionCallbackWorkItem _sensor_imu_fifo_sub{this, ORB_ID(sensor_imu_fifo)};

	calibration::Accelerometer _accel_calibration{};
	calibration::Gyroscope _gyro_calibration{};

	IntegratorConing _gyro_integrator{};

	matrix::Vector3f _accel_integral{};
	int16_t _last_accel_sample[3] {};
	float _accel_scale{0.f};

	uint32_t _imu_integration_interval_us{5000};

	hrt_abstime _timestamp_sample_last{0};
	hrt_abstime _timestamp_last{0};

	math::WelfordMean<matrix::Vector2f> _interval_mean{};

	math::WelfordMean<matrix::Vector2f> _update_latency_mean{};

	float _interval_best_variance{(float)INFINITY};

	float _interval_us{NAN};
	int _interval_samples{1};

	unsigned _last_generation{0};

	matrix::Vector3f _accel_sum{};
	matrix::Vector3f _gyro_sum{};
	int _accel_sum_count{0};
	int _gyro_sum_count{0};

	float _temperature_sum{0};
	int _temperature_sum_count{0};

	matrix::Vector3f _acceleration_prev{};     // acceleration from the previous IMU measurement for vibration metrics
	matrix::Vector3f _angular_velocity_prev{}; // angular velocity from the previous IMU measurement for vibration metrics

	vehicle_imu_status_s _status{};

	uint8_t _delta_velocity_clipping{0};

	hrt_abstime _last_clipping_notify_time{0};
	uint64_t _last_clipping_notify_total_count{0};
	orb_advert_t _mavlink_log_pub{nullptr};

	uint32_t _backup_schedule_timeout_us{20000};

	bool _update_integrator_config{true};
	bool _intervals_configured{false};
	bool _publish_status{false};

	const uint8_t _instance;

	perf_counter_t _imu_generation_gap_perf{perf_alloc(PC_COUNT, MODULE_NAME": imu data gap")};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::IMU_INTEG_RATE>) _param_imu_integ_rate,
		(ParamInt<px4::params::IMU_GYRO_RATEMAX>) _param_imu_gyro_ratemax
	)
};

} // namespace sensors
