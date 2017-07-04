#
# Sensor in-run biases in SI-unit form
#

uint32 accel_device_id		# unique device ID for the selected accelerometers
float32 accel_x_bias	# X accelerometer in-run bias in body frame (m/s^2, x forward)
float32 accel_y_bias	# Y accelerometer in-run bias in body frame (m/s^2, y right)
float32 accel_z_bias	# Z accelerometer in-run bias in body frame (m/s^2, z down)

uint32 gyro_device_id		# unique device ID for the selected rate gyros
float32 gyro_x_bias		# X gyroscope in-run bias in body frame (rad/s, x forward)
float32 gyro_y_bias		# Y gyroscope in-run bias in body frame (rad/s, y right)
float32 gyro_z_bias		# Z gyroscope in-run bias in body frame (rad/s, z down)

uint32 mag_device_id		# unique device ID for the selected magnetometer
float32 mag_x_bias		# X magnetometer in-run bias in body frame (Gauss, x forward)
float32 mag_y_bias		# Y magnetometer in-run bias in body frame (Gauss, y right)
float32 mag_z_bias		# Z magnetometer in-run bias in body frame (Gauss, z down)
