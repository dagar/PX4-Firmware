
float32 roll	    # body angular rates in NED frame
float32 pitch	    # body angular rates in NED frame
float32 yaw			# body angular rates in NED frame
float32 thrust	    # thrust normalized to 0..1

# TODO: rethink these
bool disable_mc_yaw_control			# control yaw for mc (used for vtol weather-vane mode)
float32 landing_gear

# TOPICS vehicle_rates_setpoint mc_virtual_rates_setpoint fw_virtual_rates_setpoint
