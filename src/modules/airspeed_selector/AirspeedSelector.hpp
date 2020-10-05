/****************************************************************************
 *
 *   Copyright (c) 2018-2020 PX4 Development Team. All rights reserved.
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

#include <drivers/drv_hrt.h>
#include <ecl/airdata/WindEstimator.hpp>
#include <matrix/math.hpp>
#include <parameters/param.h>
#include <perf/perf_counter.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <lib/airspeed/airspeed.h>
#include <AirspeedValidator.hpp>
#include <systemlib/mavlink_log.h>

#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/airspeed_validated.h>
#include <uORB/topics/estimator_status.h>
#include <uORB/topics/mavlink_log.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/vehicle_acceleration.h>
#include <uORB/topics/vehicle_air_data.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vtol_vehicle_status.h>
#include <uORB/topics/wind_estimate.h>

using namespace time_literals;

static constexpr uint32_t SCHEDULE_INTERVAL{100_ms};	/**< The schedule interval in usec (10 Hz) */

using matrix::Dcmf;
using matrix::Quatf;
using matrix::Vector2f;
using matrix::Vector3f;

class AirspeedSelector : public ModuleBase<AirspeedSelector>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	AirspeedSelector();
	~AirspeedSelector() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

private:
	void Run() override;

	static constexpr int MAX_NUM_AIRSPEED_SENSORS = 3; /**< Support max 3 airspeed sensors */
	enum airspeed_index {
		DISABLED_INDEX = -1,
		GROUND_MINUS_WIND_INDEX,
		FIRST_SENSOR_INDEX,
		SECOND_SENSOR_INDEX,
		THIRD_SENSOR_INDEX
	};

	uORB::Publication<airspeed_validated_s> _airspeed_validated_pub {ORB_ID(airspeed_validated)};			/**< airspeed validated topic*/
	uORB::PublicationMulti<wind_estimate_s> _wind_est_pub[MAX_NUM_AIRSPEED_SENSORS + 1] {{ORB_ID(wind_estimate)}, {ORB_ID(wind_estimate)}, {ORB_ID(wind_estimate)}, {ORB_ID(wind_estimate)}}; /**< wind estimate topic (for each airspeed validator + purely sideslip fusion) */
	orb_advert_t 	_mavlink_log_pub {nullptr}; 						/**< mavlink log topic*/

	uORB::Subscription _estimator_status_sub{ORB_ID(estimator_status)};
	uORB::Subscription _param_sub{ORB_ID(parameter_update)};
	uORB::Subscription _vehicle_acceleration_sub{ORB_ID(vehicle_acceleration)};
	uORB::Subscription _vehicle_air_data_sub{ORB_ID(vehicle_air_data)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _vtol_vehicle_status_sub{ORB_ID(vtol_vehicle_status)};
	uORB::SubscriptionMultiArray<airspeed_s, MAX_NUM_AIRSPEED_SENSORS> _airspeed_subs{ORB_ID::airspeed};

	airspeed_validator_update_data _initial_input_data{};

	WindEstimator	_wind_estimator_sideslip; /**< wind estimator instance only fusing sideslip */
	wind_estimate_s _wind_estimate_sideslip {}; /**< wind estimate message for wind estimator instance only fusing sideslip */

	bool _armed{false};
	bool _landed{true};
	bool _fixed_wing{true};

	int32_t _number_of_airspeed_sensors{0}; /**<  number of airspeed sensors in use (detected during initialization)*/
	int32_t _prev_number_of_airspeed_sensors{0}; /**<  number of airspeed sensors in previous loop (to detect a new added sensor)*/
	AirspeedValidator _airspeed_validator[MAX_NUM_AIRSPEED_SENSORS] {}; /**< airspeedValidator instances (one for each sensor) */

	hrt_abstime _time_now_usec{0};
	int _valid_airspeed_index{-2}; /**< index of currently chosen (valid) airspeed sensor */
	int _prev_airspeed_index{-2}; /**< previously chosen airspeed sensor index */
	bool _initialized{false}; /**< module initialized*/
	bool _vehicle_local_position_valid{false}; /**< local position (from GPS) valid */
	bool _in_takeoff_situation{true}; /**< in takeoff situation (defined as not yet stall speed reached) */
	float _ground_minus_wind_TAS{0.0f}; /**< true airspeed from groundspeed minus windspeed */
	float _ground_minus_wind_CAS{0.0f}; /**< calibrated airspeed from groundspeed minus windspeed */

	bool _scale_estimation_previously_on{false}; /**< scale_estimation was on in the last cycle */

	perf_counter_t _perf_elapsed{perf_alloc(PC_ELAPSED, MODULE_NAME": elapsed")};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::ASPD_W_P_NOISE>) _param_west_w_p_noise,
		(ParamFloat<px4::params::ASPD_SC_P_NOISE>) _param_west_sc_p_noise,
		(ParamFloat<px4::params::ASPD_TAS_NOISE>) _param_west_tas_noise,
		(ParamFloat<px4::params::ASPD_BETA_NOISE>) _param_west_beta_noise,
		(ParamInt<px4::params::ASPD_TAS_GATE>) _param_west_tas_gate,
		(ParamInt<px4::params::ASPD_BETA_GATE>) _param_west_beta_gate,
		(ParamInt<px4::params::ASPD_SCALE_EST>) _param_west_scale_estimation_on,
		(ParamFloat<px4::params::ASPD_SCALE>) _param_west_airspeed_scale,
		(ParamInt<px4::params::ASPD_PRIMARY>) _param_airspeed_primary_index,
		(ParamInt<px4::params::ASPD_DO_CHECKS>) _param_airspeed_checks_on,
		(ParamInt<px4::params::ASPD_FALLBACK>) _param_airspeed_fallback,

		(ParamFloat<px4::params::ASPD_FS_INNOV>) _tas_innov_threshold, /**< innovation check threshold */
		(ParamFloat<px4::params::ASPD_FS_INTEG>) _tas_innov_integ_threshold, /**< innovation check integrator threshold */
		(ParamInt<px4::params::ASPD_FS_T1>) _checks_fail_delay, /**< delay to declare airspeed invalid */
		(ParamInt<px4::params::ASPD_FS_T2>) _checks_clear_delay, /**<  delay to declare airspeed valid again */
		(ParamFloat<px4::params::ASPD_STALL>) _airspeed_stall /**<  stall speed*/
	)

	void 		init(); 	/**< initialization of the airspeed validator instances */
	void 		check_for_connected_airspeed_sensors(); /**< check for airspeed sensors (airspeed topics) and get _number_of_airspeed_sensors */
	void		update_params(); /**< update parameters */
	void 		update_wind_estimator_sideslip(const hrt_abstime &timestamp, const matrix::Quatf &q,
			const matrix::Vector3f &velocity); /**< update the wind estimator instance only fusing sideslip */
	void		update_ground_minus_wind_airspeed(float baro_pressure_pa, float baro_temp_celcius,
			const matrix::Vector3f &velocity); /**< update airspeed estimate based on groundspeed minus windspeed */
	void 		select_airspeed_and_publish(); /**< select airspeed sensor (or groundspeed-windspeed) */

};
