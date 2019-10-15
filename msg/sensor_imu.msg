uint64 timestamp		# time since system start (microseconds)

uint32 device_id		# unique device ID for the sensor that does not change between power cycles

uint64 timestamp_sample		# time since system start (microseconds)

uint16 delta			# integration time (microseconds)
uint8 samples			# number of samples integrated

float32[3] delta_angle		# delta angle in the NED frame in rad/s over the integration time frame (dt)
float32[3] delta_velocity	# delta velocity in the NED frame in m/s over the integration time frame (dt)

uint8 delta_angle_clip_count	# total delta angle clip count during the integration period across all axes
uint8 delta_velocity_clip_count	# total delta velocity clip count during the integration period across all axes
