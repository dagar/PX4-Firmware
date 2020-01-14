# IMU readings in SI-unit form.

uint64 timestamp		# time since system start (microseconds)

uint64 timestamp_sample		# sample time

uint32 accel_device_id		# unique device ID for the sensor that does not change between power cycles
uint32 gyro_device_id		# unique device ID for the sensor that does not change between power cycles

float32[3] delta_angle		# average angular rate measured in the XYZ body frame in rad/s over the last gyro sampling period
float32[3] delta_velocity	# average value acceleration measured in the XYZ body frame in m/s/s over the last accelerometer sampling period

uint16 dt			# integration period in us

uint8 integral_samples		# number of samples integrated
uint8 integral_clip_count	# total clip count per integration period on any axis
