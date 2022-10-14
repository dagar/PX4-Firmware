
#pragma once

class PX4Mode
{
public:
	PX4Mode(const char *name, uint8_t nav_state, uint8_t vehicle_type) :
		_name(name),
		_nav_state(nav_state),
		_vehicle_type(vehicle_type)
	{

	}

protected:
	uint8_t _nav_state; // vehicle_status_s::NAVIGATION_STATE_MANUAL
	uint8_t _vehicle_type; // vehicle_status_s::VEHICLE_TYPE_FIXED_WING

	const char *_name;

	//vehicle_control_mode_s control_mode{ .flag_control_manual_enabled = true; }

	// scheduling topic
	// scheduling rate (min, max)
};

// lifecycle?
//   activate
//   run
//   shutdown

// ready to run
//  default implementation based on checking static properties
//  optional runtime checks?


// TODO:
//  based on control mode commander listens for appropriate topic updates


// match vehicle_type, match nav_state, then check other property? eg MPC_POS_MODE
