# Optical flow in XYZ body frame in SI units.

uint64 timestamp		# time since system start (microseconds)
uint64 timestamp_sample

uint8 sensor_id			# id of the sensor emitting the flow value

float32[2] pixel_flow_integral  # accumulated optical flow in radians where a positive value is produced by a RH rotation about the X and Y body axis

float32[3] delta_angle          # accumulated gyro value in radians where a positive value is produced by a RH rotation about the X body axis. Set to NaN if flow sensor does not have 3-axis gyro data.

uint16 dt	# accumulation timespan in microseconds

uint8 quality	# Average of quality of accumulated frames, 0: bad quality, 255: maximum quality

float32 max_flow_rate # Magnitude of maximum angular which the optical flow sensor can measure reliably
float32 min_ground_distance # Minimum distance from ground at which the optical flow sensor operates reliably
float32 max_ground_distance # Maximum distance from ground at which the optical flow sensor operates reliably
