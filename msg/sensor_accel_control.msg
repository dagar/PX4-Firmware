uint32 device_id	# unique device ID for the sensor that does not change between power cycles

uint64 timestamp	# time since system start (microseconds)
uint64 timestamp_sample	# time since system start (microseconds)

float32[3] acceleration	# filtered acceleration in the NED X board axis in m/s^2
