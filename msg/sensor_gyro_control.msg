uint32 device_id	# unique device ID for the sensor that does not change between power cycles

uint64 timestamp	# time since system start (microseconds)

uint64 timestamp_sample	# time the raw data was sampled (microseconds)

float32 x		# filtered angular velocity in the NED X board axis in rad/s
float32 y		# filtered angular velocity in the NED Y board axis in rad/s
float32 z		# filtered angular velocity in the NED Z board axis in rad/s
