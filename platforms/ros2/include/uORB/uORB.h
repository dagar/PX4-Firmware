
#pragma once

#include "px4/msg/vehicle_status.hpp"

enum class ORB_ID {
	INVALID = 0,

	actuator_controls,
	battery_status,
	landing_gear,
	manual_control_setpoint,
	multirotor_motor_limits,
	rate_ctrl_status,
	vehicle_control_mode,
	vehicle_angular_acceleration,
	vehicle_angular_velocity,
	vehicle_land_detected,
	vehicle_rates_setpoint,
	vehicle_status,  // px4::msg::VehicleStatus vehicle_status
};


using vehicle_status_s = px4::msg::VehicleStatus;

using orb_id_t = void *;

// Notes
//  - ORB_ID map to msg T (compile time)
//  - How to check if updated or track generation?

static orb_id_t get_orb_meta(ORB_ID orb_id)
{
	(void)orb_id;
	return nullptr;
}

static const char *get_topic(ORB_ID orb_id)
{
	switch (orb_id) {
	case ORB_ID::actuator_controls: return "actuator_controls";

	case ORB_ID::battery_status: return "battery_status";

	case ORB_ID::landing_gear: return "landing_gear";

	case ORB_ID::manual_control_setpoint: return "manual_control_setpoint";

	case ORB_ID::multirotor_motor_limits: return "multirotor_motor_limits";

	case ORB_ID::rate_ctrl_status: return "rate_ctrl_status";

	case ORB_ID::vehicle_control_mode: return "vehicle_control_mode";

	case ORB_ID::vehicle_angular_acceleration: return "vehicle_angular_acceleration";

	case ORB_ID::vehicle_angular_velocity: return "vehicle_angular_velocity";

	case ORB_ID::vehicle_land_detected: return "vehicle_land_detected";

	case ORB_ID::vehicle_rates_setpoint: return "vehicle_rates_setpoint";

	case ORB_ID::vehicle_status: return "vehicle_status";

	default:
		return nullptr;
	}

	return nullptr;
}

// const orb_metadata *meta
