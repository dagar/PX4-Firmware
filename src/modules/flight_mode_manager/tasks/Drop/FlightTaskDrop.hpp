/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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
 * @file FlightTaskDrop.hpp
 *
 */

#pragma once

#include "FlightTask.hpp"

#include "Sticks.hpp"
#include "StickAccelerationXY.hpp"
#include "StickYaw.hpp"

#include <lib/geo/geo.h>
#include <lib/mathlib/math/filter/AlphaFilter.hpp>
#include <lib/motion_planning/PositionSmoothing.hpp>

#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/failsafe_flags.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_local_position.h>

#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/vehicle_torque_setpoint.h>

#include <uORB/topics/offboard_control_mode.h>

#include <uORB/Publication.hpp>

class FlightTaskDrop : public FlightTask
{
public:
	FlightTaskDrop() = default;
	virtual ~FlightTaskDrop() = default;

	bool activate(const trajectory_setpoint_s &last_setpoint) override;
	void reActivate() override;

	bool updateInitialize() override;
	bool update() override;

private:

	/** Reset position or velocity setpoints in case of EKF reset event */
	void _ekfResetHandlerPositionXY(const matrix::Vector2f &delta_xy) override;
	void _ekfResetHandlerVelocityXY(const matrix::Vector2f &delta_vxy) override;
	void _ekfResetHandlerPositionZ(float delta_z) override;
	void _ekfResetHandlerVelocityZ(float delta_vz) override;
	void _ekfResetHandlerHeading(float delta_psi) override;

	void updateParams() override; /**< See ModuleParam class */

	uORB::Publication<vehicle_torque_setpoint_s> _vehicle_torque_setpoint_pub{ORB_ID(vehicle_torque_setpoint)};
	uORB::Publication<vehicle_thrust_setpoint_s> _vehicle_thrust_setpoint_pub{ORB_ID(vehicle_thrust_setpoint)};
	uORB::PublicationData<vehicle_rates_setpoint_s>     _vehicle_rates_setpoint_pub{ORB_ID(vehicle_rates_setpoint)};
	uORB::PublicationData<vehicle_attitude_setpoint_s>  _vehicle_attitude_setpoint_pub{ORB_ID(vehicle_attitude_setpoint)};

	uORB::Publication<offboard_control_mode_s> _offboard_control_mode_pub{ORB_ID(offboard_control_mode)};

	uORB::SubscriptionData<vehicle_angular_velocity_s> _vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
	uORB::SubscriptionData<vehicle_attitude_s> _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};

	uORB::SubscriptionData<actuator_armed_s> _actuator_armed_sub{ORB_ID(actuator_armed)};

	uORB::SubscriptionData<home_position_s> _home_position_sub{ORB_ID(home_position)};
	uORB::SubscriptionData<vehicle_status_s> _vehicle_status_sub{ORB_ID(vehicle_status)};

	uORB::SubscriptionData<failsafe_flags_s> _failsafe_flags_sub{ORB_ID(failsafe_flags)};
	uORB::SubscriptionData<vehicle_local_position_s> _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};

	uORB::SubscriptionData<trajectory_setpoint_s> _offboard_trajectory_setpoint_sub{ORB_ID(offboard_trajectory_setpoint)};

	enum class DropState {
		UNKNOWN                             = 0,

		DISARMED                            = 1,
		ARMING                              = 2,
		WAITING_FOR_DROP_DETECT             = 3,
		RATE_CONTROL_ENABLED                = 4,
		ATTITUDE_CONTROL_ENABLED            = 5,
		HEIGHT_RATE_CONTROL_ENABLED         = 6,
		VELOCITY_CONTROL_ENABLED            = 7,
		POSITION_CONTROL_ENABLED            = 8,
		FLYING                              = 9,
	};

	DropState _state{DropState::UNKNOWN};
	DropState _state_prev{DropState::UNKNOWN};


	static constexpr float kFilterTimeConstant = 0.01f; // Tc 0.01 ~= 15.9 Hz cutoff

	AlphaFilter<matrix::Vector2f> _velocity_xy_lpf{kFilterTimeConstant};
	AlphaFilter<float> _velocity_z_lpf{kFilterTimeConstant};

	AlphaFilter<matrix::Vector3f> _acceleration_lpf{kFilterTimeConstant};

	orb_advert_t _mavlink_log_pub{nullptr};
	hrt_abstime _state_last_transition_time{0};

	matrix::Vector2f _lock_position_xy{NAN, NAN}; /**< if no valid triplet is received, lock positition to current position */
	bool _yaw_lock{false}; /**< if within acceptance radius, lock yaw to current yaw */

	MapProjection _reference_position{}; /**< Class used to project lat/lon setpoint into local frame. */
	float _reference_altitude{NAN}; /**< Altitude relative to ground. */
	hrt_abstime _time_stamp_reference{0}; /**< time stamp when last reference update occured. */

	PositionSmoothing _position_smoothing{};
	Vector3f _unsmoothed_velocity_setpoint{};


	Vector3f _offboard_pos_sp_last{NAN, NAN, NAN};
	Vector3f _offboard_vel_sp_last{NAN, NAN, NAN};
	uint64_t _offboard_time_stamp_last{0};

	DEFINE_PARAMETERS_CUSTOM_PARENT(FlightTask,
					(ParamFloat<px4::params::MPC_DROP_LAUN_T>) _param_mpc_drop_laun_t,
					(ParamFloat<px4::params::MPC_DROP_VZ_THR>) _param_mpc_drop_vz_thr,
					(ParamFloat<px4::params::MPC_DROP_AZ_THR>) _param_mpc_drop_az_thr,
					(ParamFloat<px4::params::MPC_DROP_AZ_MAX>) _param_mpc_drop_az_max,
					(ParamFloat<px4::params::MPC_DROP_HOLD_T>) _param_mpc_drop_hold_t,
					(ParamFloat<px4::params::MPC_DROP_ANGVEL>) _param_mpc_drop_angvel,

					(ParamFloat<px4::params::MPC_THR_HOVER>)    _param_mpc_thr_hover,

					(ParamFloat<px4::params::MPC_XY_CRUISE>) _param_mpc_xy_cruise,
					(ParamFloat<px4::params::NAV_MC_ALT_RAD>)
					_param_nav_mc_alt_rad, //vertical acceptance radius at which waypoints are updated
					(ParamInt<px4::params::MPC_YAW_MODE>) _param_mpc_yaw_mode, // defines how heading is executed,
					(ParamInt<px4::params::COM_OBS_AVOID>) _param_com_obs_avoid, // obstacle avoidance active
					(ParamFloat<px4::params::MPC_YAWRAUTO_MAX>) _param_mpc_yawrauto_max,
					(ParamFloat<px4::params::MIS_YAW_ERR>) _param_mis_yaw_err, // yaw-error threshold
					(ParamFloat<px4::params::MPC_ACC_HOR>) _param_mpc_acc_hor, // acceleration in flight
					(ParamFloat<px4::params::MPC_ACC_UP_MAX>) _param_mpc_acc_up_max,
					(ParamFloat<px4::params::MPC_ACC_DOWN_MAX>) _param_mpc_acc_down_max,
					(ParamFloat<px4::params::MPC_JERK_AUTO>) _param_mpc_jerk_auto,
					(ParamFloat<px4::params::MPC_XY_TRAJ_P>) _param_mpc_xy_traj_p,
					(ParamFloat<px4::params::MPC_XY_ERR_MAX>) _param_mpc_xy_err_max,
					(ParamFloat<px4::params::MPC_LAND_SPEED>) _param_mpc_land_speed,
					(ParamFloat<px4::params::MPC_LAND_CRWL>) _param_mpc_land_crawl_speed,
					(ParamInt<px4::params::MPC_LAND_RC_HELP>) _param_mpc_land_rc_help,
					(ParamFloat<px4::params::MPC_LAND_RADIUS>) _param_mpc_land_radius,
					(ParamFloat<px4::params::MPC_LAND_ALT1>)
					_param_mpc_land_alt1, // altitude at which we start ramping down speed
					(ParamFloat<px4::params::MPC_LAND_ALT2>)
					_param_mpc_land_alt2, // altitude at which we descend at land speed
					(ParamFloat<px4::params::MPC_LAND_ALT3>)
					_param_mpc_land_alt3, // altitude where we switch to crawl speed, if LIDAR available
					(ParamFloat<px4::params::MPC_Z_V_AUTO_UP>) _param_mpc_z_v_auto_up,
					(ParamFloat<px4::params::MPC_Z_V_AUTO_DN>) _param_mpc_z_v_auto_dn,
					(ParamFloat<px4::params::MPC_TKO_SPEED>) _param_mpc_tko_speed,
					(ParamFloat<px4::params::MPC_TKO_RAMP_T>)
					_param_mpc_tko_ramp_t // time constant for smooth takeoff ramp


				       );

};
