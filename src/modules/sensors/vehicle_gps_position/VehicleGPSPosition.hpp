/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#include <lib/mathlib/math/Limits.hpp>
#include <lib/matrix/matrix/math.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/sensors_status_gps.h>
#include <uORB/topics/vehicle_gps_position.h>

#include "gps_blending.hpp"

using namespace time_literals;

namespace sensors
{
class VehicleGPSPosition : public ModuleParams, public px4::ScheduledWorkItem
{
public:

	VehicleGPSPosition();
	~VehicleGPSPosition() override;

	bool Start();
	void Stop();

	void PrintStatus();

private:
	void Run() override;

	void ParametersUpdate(bool force = false);
	void Publish(const sensor_gps_s &gps, uint8_t selected);

	void resetGpsDriftCheckFilters();

	// defines used to specify the mask position for use of different accuracy metrics in the GPS blending algorithm
	static constexpr uint8_t BLEND_MASK_USE_SPD_ACC  = 1;
	static constexpr uint8_t BLEND_MASK_USE_HPOS_ACC = 2;
	static constexpr uint8_t BLEND_MASK_USE_VPOS_ACC = 4;

	// define max number of GPS receivers supported
	static constexpr int GPS_MAX_RECEIVERS = 2;
	static_assert(GPS_MAX_RECEIVERS == GpsBlending::GPS_MAX_RECEIVERS_BLEND,
		      "GPS_MAX_RECEIVERS must match to GPS_MAX_RECEIVERS_BLEND");

	uORB::Publication<sensors_status_gps_s> _sensors_status_gps_pub{ORB_ID(sensors_status_gps)};
	uORB::Publication<vehicle_gps_position_s> _vehicle_gps_position_pub{ORB_ID(vehicle_gps_position)};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::SubscriptionCallbackWorkItem _sensor_gps_sub[GPS_MAX_RECEIVERS] {	/**< sensor data subscription */
		{this, ORB_ID(sensor_gps), 0},
		{this, ORB_ID(sensor_gps), 1},
	};

	bool _landed{true};
	bool _at_rest{false};

	sensors_status_gps_s _sensors_status_gps{};

	// variables used for the GPS quality checks
	hrt_abstime _timestamp_prev[GPS_MAX_RECEIVERS] {};
	MapProjection _gps_pos_prev[GPS_MAX_RECEIVERS] {};  // Contains WGS-84 position latitude and longitude (radians) of the previous GPS message
	float _gps_alt_prev[GPS_MAX_RECEIVERS] {};	// height from the previous GPS message (m)
	matrix::Vector3f _gps_pos_deriv_filt[GPS_MAX_RECEIVERS] {};	///< GPS NED position derivative (m/sec)
	matrix::Vector2f _gps_velNE_filt[GPS_MAX_RECEIVERS] {};	///< filtered GPS North and East velocity (m/sec)

	float _gps_drift_metrics[GPS_MAX_RECEIVERS][3] {};	// Array containing GPS drift metrics
	// [0] Horizontal position drift rate (m/s)
	// [1] Vertical position drift rate (m/s)
	// [2] Filtered horizontal velocity (m/s)

	// GPS pre-flight check bit locations
	static constexpr uint32_t MASK_GPS_NSATS  = (1 << 0);
	static constexpr uint32_t MASK_GPS_PDOP   = (1 << 1);
	static constexpr uint32_t MASK_GPS_HACC   = (1 << 2);
	static constexpr uint32_t MASK_GPS_VACC   = (1 << 3);
	static constexpr uint32_t MASK_GPS_SACC   = (1 << 4);
	static constexpr uint32_t MASK_GPS_HDRIFT = (1 << 5);
	static constexpr uint32_t MASK_GPS_VDRIFT = (1 << 6);
	static constexpr uint32_t MASK_GPS_HSPD   = (1 << 7);
	static constexpr uint32_t MASK_GPS_VSPD   = (1 << 8);

	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};

	GpsBlending _gps_blending;

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::SENS_GPS_MASK>) _param_sens_gps_mask,
		(ParamFloat<px4::params::SENS_GPS_TAU>) _param_sens_gps_tau,
		(ParamInt<px4::params::SENS_GPS_PRIME>) _param_sens_gps_prime,

		(ParamFloat<px4::params::SENS_GPS_RQ_EPH>) _param_sens_gps_rq_eph,
		(ParamFloat<px4::params::SENS_GPS_RQ_EPV>) _param_sens_gps_rq_epv,
		(ParamFloat<px4::params::SENS_GPS_RQ_SACC>) _param_sens_gps_rq_sacc,
		(ParamInt<px4::params::SENS_GPS_RQ_NSAT>) _param_sens_gps_rq_nsat,
		(ParamFloat<px4::params::SENS_GPS_RQ_PDOP>) _param_sens_gps_rq_pdop,
		(ParamFloat<px4::params::SENS_GPS_RQ_HDRF>) _param_sens_gps_rq_hdrf,
		(ParamFloat<px4::params::SENS_GPS_RQ_VDRF>) _param_sens_gps_rq_vdrf,

		(ParamInt<px4::params::SENS_GPS_CHECKS>) _param_sens_gps_checks
	)
};
}; // namespace sensors
