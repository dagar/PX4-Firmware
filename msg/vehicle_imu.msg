# IMU readings in SI-unit form.
# These fields are scaled and offset-compensated where possible and do not
# change with board revisions and sensor updates.

uint64 timestamp				# time since system start (microseconds)

float32[3] delta_angle			# delta angle measured in the XYZ body frame in rad/s over the last gyro sampling period
float32[3] delta_velocity		# delta velocity measured in the XYZ body frame in m/s/s over the last accelerometer sampling period

uint16 dt				# integration period in us
