
uint64 timestamp	# time since system start (microseconds)

uint32 device_id	# unique device ID for the sensor that does not change between power cycles

uint32 integral_dt	# integration time (microseconds)

float32[3] delta_angle		# delta angle in the NED frame in rad/s over the integration time frame (integral_dt)
float32[3] delta_velocity	# delta velocity in the NED frame in m/s over the integration time frame (integral_dt)
