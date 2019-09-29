#
# Gyroscope corrections in SI-unit form
#

uint64 timestamp		# time since system start (microseconds)

uint32 device_id		# unique device ID for the sensor that does not change between power cycles
uint8 parameter_index
uint8 orb_index

# Corrections for gyro angular rate outputs where corrected_rate = raw_rate * gyro_scale + gyro_offset
# Note the corrections are in the sensor frame and must be applied before the sensor data is rotated into body frame
# corrections for uORB instance 0
float32[3] offset	# gyro XYZ offsets in the sensor frame in rad/s
float32[3] scale	# gyro XYZ scale factors in the sensor frame

