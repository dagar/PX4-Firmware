
#pragma once

#include "Mode.hpp"

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>

#include <uORB/SubscriptionCallback.hpp>

#include <uORB/topics/manual_control_setpoint.h>

class FixedwingStabilizeMode
{
public:
	FixedwingStabilizeMode() = default;

private:

	bool init()
	{

		// hand off previous nav_state and vehicle_control_mode?
	}

	void run()
	{
		manual_control_setpoint_s man;

		if (_manual_control_setpoint_sub.update(&man)) {


			_att_sp.roll_body = man.y * radians(_param_fw_man_r_max.get());

			_att_sp.pitch_body = -man.x * radians(_param_fw_man_p_max.get()) + radians(_param_fw_psp_off.get());
			_att_sp.pitch_body = constrain(_att_sp.pitch_body, -radians(_param_fw_man_p_max.get()),
						       radians(_param_fw_man_p_max.get()));

			_att_sp.yaw_body = yaw_body; // yaw is not controlled, so set setpoint to current yaw
			_att_sp.thrust_body[0] = math::constrain(man.z, 0.0f, 1.0f);

			Quatf q(Eulerf(_att_sp.roll_body, _att_sp.pitch_body, _att_sp.yaw_body));
			q.copyTo(_att_sp.q_d);

			_att_sp.timestamp = hrt_absolute_time();

			_attitude_sp_pub.publish(_att_sp);
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
