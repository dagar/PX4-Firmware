/****************************************************************************
 *
 *   Copyright (c) 2019-2023 PX4 Development Team. All rights reserved.
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

#include "IMU.hpp"
#include "VehicleAcceleration.hpp"
#include "VehicleAngularVelocity.hpp"

#include <include/containers/Bitset.hpp>

#include <lib/mathlib/math/Limits.hpp>

#include <lib/matrix/matrix/math.hpp>
#include <lib/mathlib/math/filter/AlphaFilter.hpp>
#include <lib/mathlib/math/filter/LowPassFilter2p.hpp>
#include <lib/mathlib/math/filter/NotchFilter.hpp>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

// publications
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/sensors_status_imu.h>
#include <uORB/topics/sensor_selection.h>

// subscriptions
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/topics/estimator_selector_status.h>
#include <uORB/topics/estimator_sensor_bias.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/sensor_imu_fifo.h>

using namespace time_literals;

namespace sensors
{

class VehicleIMU : public ModuleParams, public px4::ScheduledWorkItem
{
public:
	VehicleIMU();
	~VehicleIMU() override;

	void PrintStatus();
	bool Start();
	void Stop();

private:
	void Run() override;

	void ParametersUpdate(bool force = false);

	bool PublishImu(sensors::IMU &imu);

	void SensorBiasUpdate();
	bool SensorSelectionUpdate(const hrt_abstime &time_now_us, bool force = false);
	void UpdateSensorImuFifo(uint8_t sensor_instance);
	void UpdateSensorAccel(uint8_t sensor_instance);
	void UpdateSensorGyro(uint8_t sensor_instance);

	// return the square of two floating point numbers
	static constexpr float sq(float var) { return var * var; }

	static constexpr int MAX_SENSOR_COUNT = 4;

	uORB::Subscription _estimator_selector_status_sub{ORB_ID(estimator_selector_status)};
	uORB::SubscriptionMultiArray<estimator_sensor_bias_s> _estimator_sensor_bias_subs{ORB_ID::estimator_sensor_bias};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::SubscriptionCallbackWorkItem _sensor_selection_sub{this, ORB_ID(sensor_selection)};

	struct SensorSubscription {
		SensorSubscription(px4::WorkItem *work_item, ORB_ID orb_id, uint8_t instance) :
			sub(work_item, orb_id, instance)
		{}

		~SensorSubscription() = default;

		uORB::SubscriptionCallbackWorkItem sub;
		unsigned last_generation{0};
	};

	SensorSubscription _sensor_imu_fifo_subs[MAX_SENSOR_COUNT] {
		{this, ORB_ID::sensor_imu_fifo, 0},
		{this, ORB_ID::sensor_imu_fifo, 1},
		{this, ORB_ID::sensor_imu_fifo, 2},
		{this, ORB_ID::sensor_imu_fifo, 3}
	};

	SensorSubscription _sensor_accel_subs[MAX_SENSOR_COUNT] {
		{this, ORB_ID::sensor_accel, 0},
		{this, ORB_ID::sensor_accel, 1},
		{this, ORB_ID::sensor_accel, 2},
		{this, ORB_ID::sensor_accel, 3}
	};

	IMU _imus[MAX_SENSOR_COUNT] {};




	// TODO:
	uORB::Publication<sensor_combined_s>    _sensor_combined_pub{ORB_ID(sensor_combined)}; // legacy
	uORB::Publication<sensor_selection_s>   _sensor_selection_pub{ORB_ID(sensor_selection)};
	uORB::Publication<sensors_status_imu_s> _sensors_status_imu_pub{ORB_ID(sensors_status_imu)};

	VehicleAcceleration _vehicle_acceleration{};
	VehicleAngularVelocity _vehicle_angular_velocity{};

	uint32_t _selected_accel_device_id{0};
	uint32_t _selected_gyro_device_id{0};

	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": IMU update")};
	perf_counter_t _selection_changed_perf{perf_alloc(PC_COUNT, MODULE_NAME": IMU selection changed")};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::IMU_INTEG_RATE>) _param_imu_integ_rate,
		(ParamInt<px4::params::IMU_GYRO_RATEMAX>) _param_imu_gyro_ratemax
	)
};

} // namespace sensors
