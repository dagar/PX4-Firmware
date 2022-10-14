
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
		manual_control_setpoint_s man;

		if (_manual_control_setpoint_sub.update(&man)) {
			// manual/direct control
			actuator_controls_s act{};
			act.timestamp_sample = man.timestamp;
			act.control[actuator_controls_s::INDEX_ROLL]  =  man.y * _param_fw_man_r_sc.get() + _param_trim_roll.get();
			act.control[actuator_controls_s::INDEX_PITCH] = -man.x * _param_fw_man_p_sc.get() + _param_trim_pitch.get();
			act.control[actuator_controls_s::INDEX_YAW]   =  man.r * _param_fw_man_y_sc.get() + _param_trim_yaw.get();
			act.control[actuator_controls_s::INDEX_THROTTLE] = math::constrain(man.z, 0.f, 1.f);


			// Add feed-forward from roll control output to yaw control output
			// This can be used to counteract the adverse yaw effect when rolling the plane
			// att.control[actuator_controls_s::INDEX_YAW] += _param_fw_rll_to_yaw_ff.get() * constrain(att.control[actuator_controls_s::INDEX_ROLL], -1.f, 1.f);

			// att.control[actuator_controls_s::INDEX_FLAPS] = _flaps_setpoint_with_slewrate.getState();
			// att.control[actuator_controls_s::INDEX_SPOILERS] = _spoiler_setpoint_with_slewrate.getState();
			// att.control[actuator_controls_s::INDEX_AIRBRAKES] = 0.f;
			// att.control[actuator_controls_s::INDEX_LANDING_GEAR] = man.aux3;


			act.timestamp = hrt_absolute_time();
			_actuator_controls_0_pub.publish(act);
		}


		//

		// if (!_vehicle_status.is_vtol) {
		// 	publishTorqueSetpoint(angular_velocity.timestamp_sample);
		// 	publishThrustSetpoint(angular_velocity.timestamp_sample);
		// }


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

		(ParamFloat<px4::params::FW_MAN_P_MAX>) _param_fw_man_p_max,
		(ParamFloat<px4::params::FW_MAN_P_SC>) _param_fw_man_p_sc,
		(ParamFloat<px4::params::FW_MAN_R_MAX>) _param_fw_man_r_max,
		(ParamFloat<px4::params::FW_MAN_R_SC>) _param_fw_man_r_sc,
		(ParamFloat<px4::params::FW_MAN_Y_SC>) _param_fw_man_y_sc

	)

};
