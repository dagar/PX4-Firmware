
#include <functional>
#include <memory>

#include "std_msgs/msg/string.hpp"

#include "px4/msg/actuator_controls.hpp"
#include "px4/msg/battery_status.hpp"
#include "px4/msg/landing_gear.hpp"
#include "px4/msg/manual_control_setpoint.hpp"
#include "px4/msg/multirotor_motor_limits.hpp"
#include "px4/msg/parameter_update.hpp"
#include "px4/msg/rate_ctrl_status.hpp"
#include "px4/msg/vehicle_angular_acceleration.hpp"
#include "px4/msg/vehicle_angular_velocity.hpp"
#include "px4/msg/vehicle_control_mode.hpp"
#include "px4/msg/vehicle_land_detected.hpp"
#include "px4/msg/vehicle_rates_setpoint.hpp"
#include "px4/msg/vehicle_status.hpp"

#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>

#include <px4_platform_common/Node.hpp>
#include <px4_platform_common/module_params.h>

#include <drivers/drv_hrt.h> // TODO: ros time

// modules start/stop/status (px4_add_module)
//   work_quue
//   px4_task_create?
// pub/sub
// parameters
// time

using namespace std::chrono_literals;

using std::placeholders::_1;

class MulticopterRateControl : public px4::Node, public ModuleParams
{
public:
	MulticopterRateControl() :
		Node("mc_rate_control"),
		ModuleParams(nullptr)
	{
		RCLCPP_INFO(get_logger(), "constructing %lu", hrt_absolute_time());

		timer_ = this->create_wall_timer(500ms, std::bind(&MulticopterRateControl::run, this));

		rcl_interfaces::msg::ParameterDescriptor descriptor;
		descriptor.name = "MC_ROLLRATE_P";
		descriptor.description = "Roll rate P gain";
		descriptor.read_only = false;
		descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
		//descriptor.dynamic_typing = false;
		//descriptor.additional_constraints = "Supported values: [jpeg, png]"; // TODO: enum

		this->declare_parameter<float>("MC_ROLLRATE_P", 0.15f, descriptor);
	}

private:

	void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
	{
		RCLCPP_INFO(get_logger(), "I heard: '%s'", msg->data.c_str());
	}

	void run()
	{
		this->get_parameter("MC_ROLLRATE_P", _param_mc_rollrate_p);

		//RCLCPP_INFO(get_logger(), "running: %lu", hrt_absolute_time());

		vehicle_status_s vehicle_status;

		if (_vehicle_status_sub.update(&vehicle_status)) {
			RCLCPP_INFO(get_logger(), "vehicle_status armed: %d", vehicle_status.arming_state);
		}
	}

	rclcpp::TimerBase::SharedPtr timer_;

	uORB::Subscription2<px4::msg::BatteryStatus> _battery_status_sub{this, ORB_ID::battery_status};
	uORB::Subscription2<px4::msg::LandingGear> _landing_gear_sub{this, ORB_ID::landing_gear};
	uORB::Subscription2<px4::msg::ManualControlSetpoint> _manual_control_setpoint_sub{this, ORB_ID::manual_control_setpoint};
	uORB::Subscription2<px4::msg::MultirotorMotorLimits> _motor_limits_sub{this, ORB_ID::multirotor_motor_limits};
	uORB::Subscription2<px4::msg::VehicleControlMode> _v_control_mode_sub{this, ORB_ID::vehicle_control_mode};
	uORB::Subscription2<px4::msg::VehicleRatesSetpoint> _v_rates_sp_sub{this, ORB_ID::vehicle_rates_setpoint};
	uORB::Subscription2<px4::msg::VehicleAngularAcceleration> _vehicle_angular_acceleration_sub{this, ORB_ID::vehicle_angular_acceleration};
	uORB::Subscription2<px4::msg::VehicleAngularVelocity> _vehicle_angular_velocity_sub{this, ORB_ID::vehicle_angular_velocity};
	uORB::Subscription2<px4::msg::VehicleLandDetected> _vehicle_land_detected_sub{this, ORB_ID::vehicle_land_detected};
	uORB::Subscription2<px4::msg::VehicleStatus> _vehicle_status_sub{this, ORB_ID::vehicle_status};

	uORB::Publication<px4::msg::ActuatorControls>      _actuators_0_pub{this, ORB_ID::actuator_controls};
	uORB::Publication<px4::msg::RateCtrlStatus>       _controller_status_pub{this, ORB_ID::rate_ctrl_status};	/**< controller status publication */
	uORB::Publication<px4::msg::VehicleRatesSetpoint> _v_rates_sp_pub{this, ORB_ID::vehicle_rates_setpoint};			/**< rate setpoint publication */


	float _param_mc_rollrate_p{0.f};
};

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<MulticopterRateControl>());
	rclcpp::shutdown();
	return 0;
}
