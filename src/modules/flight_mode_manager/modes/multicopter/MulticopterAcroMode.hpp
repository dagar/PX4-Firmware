
#pragma once

#include "Mode.hpp"

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>

#include <uORB/SubscriptionCallback.hpp>

#include <uORB/topics/manual_control_setpoint.h>

class FixedwingManualMode
{
public:
	FixedwingManualMode() :
		Mode("Manual", vehicle_status_s::VEHICLE_TYPE_FIXED_WING, vehicle_status_s::NAVIGATION_STATE_MANUAL)
	{

	}

private:

	bool init()
	{
		// init flaps, spoilers, etc?
	}

	void run()
	{
		// generate the rate setpoint from sticks
		manual_control_setpoint_s manual_control_setpoint;

		if (_manual_control_setpoint_sub.update(&manual_control_setpoint)) {
			// manual rates control - ACRO mode
			const Vector3f man_rate_sp{
				math::superexpo(manual_control_setpoint.y, _param_mc_acro_expo.get(), _param_mc_acro_supexpo.get()),
				math::superexpo(-manual_control_setpoint.x, _param_mc_acro_expo.get(), _param_mc_acro_supexpo.get()),
				math::superexpo(manual_control_setpoint.r, _param_mc_acro_expo_y.get(), _param_mc_acro_supexpoy.get())};

			_rates_setpoint = man_rate_sp.emult(_acro_rate_max);
			_thrust_setpoint(2) = -math::constrain(manual_control_setpoint.z, 0.0f, 1.0f);
			_thrust_setpoint(0) = _thrust_setpoint(1) = 0.f;

			// publish rate setpoint
			vehicle_rates_setpoint.roll = _rates_setpoint(0);
			vehicle_rates_setpoint.pitch = _rates_setpoint(1);
			vehicle_rates_setpoint.yaw = _rates_setpoint(2);
			_thrust_setpoint.copyTo(vehicle_rates_setpoint.thrust_body);
			vehicle_rates_setpoint.timestamp = hrt_absolute_time();

			_vehicle_rates_setpoint_pub.publish(vehicle_rates_setpoint);
		}


	}



	// nav_state vehicle_status_s::NAVIGATION_STATE_MANUAL
	// vehicle_type vehicle_status_s::VEHICLE_TYPE_FIXED_WING


	// requirements
	// manual_control_setpoint



	uORB::Publication<actuator_controls_s> _actuator_controls_0_pub;
	uORB::Publication<vehicle_thrust_setpoint_s> _vehicle_thrust_setpoint_pub{ORB_ID(vehicle_thrust_setpoint)};
	uORB::Publication<vehicle_torque_setpoint_s> _vehicle_torque_setpoint_pub{ORB_ID(vehicle_torque_setpoint)};

	// uORB::SubscriptionCallbackWorkItem _att_sub{this, ORB_ID(vehicle_attitude)};
	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::MPC_MAN_Y_MAX>) _param_mpc_man_y_max,			/**< scaling factor from stick to yaw rate */

		(ParamFloat<px4::params::MC_ACRO_R_MAX>) _param_mc_acro_r_max,
		(ParamFloat<px4::params::MC_ACRO_P_MAX>) _param_mc_acro_p_max,
		(ParamFloat<px4::params::MC_ACRO_Y_MAX>) _param_mc_acro_y_max,

		(ParamFloat<px4::params::MC_ACRO_EXPO>) _param_mc_acro_expo,			/**< expo stick curve shape (roll & pitch) */
		(ParamFloat<px4::params::MC_ACRO_EXPO_Y>) _param_mc_acro_expo_y,		/**< expo stick curve shape (yaw) */
		(ParamFloat<px4::params::MC_ACRO_SUPEXPO>) _param_mc_acro_supexpo,		/**< superexpo stick curve shape (roll & pitch) */
		(ParamFloat<px4::params::MC_ACRO_SUPEXPOY>) _param_mc_acro_supexpoy,		/**< superexpo stick curve shape (yaw) */
	)

};
