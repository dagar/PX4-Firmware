
uint64 timestamp	# time since system start (microseconds)

uint32 device_id	# unique device ID for the sensor that does not change between power cycles

uint64 error_count

uint16 dt		# integration time (microseconds)
float32[3] delta_angle	# delta angle in the NED board axis in rad/s over the integration time frame (integral_dt)

float32 temperature	# temperature in degrees celsius
