uint64 timestamp	# time since system start (microseconds)

uint32 device_id	# unique device ID for the sensor that does not change between power cycles

uint64 timestamp_sample	# time since system start (microseconds)

uint64 error_count

uint32 integral_dt	# integration time (microseconds)
uint8 integral_samples	# number of samples integrated
float32 x_integral	# delta angle in the NED X board axis in rad/s over the integration time frame (integral_dt)
float32 y_integral	# delta angle in the NED Y board axis in rad/s over the integration time frame (integral_dt)
float32 z_integral	# delta angle in the NED Z board axis in rad/s over the integration time frame (integral_dt)
uint8 integral_clip_count	# total clip count per integration period on any axis

float32 temperature	# temperature in degrees celsius
