uint64 timestamp	# time since system start (microseconds)
uint32 device_id	# unique device ID for the sensor that does not change between power cycles

uint64 error_count

float32 temperature

# clipping per axis?
uint64[3] clipping

uint16 sample_rate
uint16 measure_rate

float32 full_scale_range

float32 vibration_metric	# high frequency vibration level in the IMU delta velocity data (m/s)
