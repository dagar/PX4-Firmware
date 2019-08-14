uint32 device_id	# unique device ID for the sensor that does not change between power cycles

uint64 timestamp	# time since system start (microseconds)

uint64 timestamp_sample	# time the raw data was sampled (microseconds)

float32 scaling		# scaling from raw to rad/s
int16 x_raw
int16 y_raw
int16 z_raw
