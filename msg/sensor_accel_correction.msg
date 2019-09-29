#
# Accelerometer corrections in SI-unit form
#

uint64 timestamp		# time since system start (microseconds)

uint32 device_id		# unique device ID for the sensor that does not change between power cycles
uint8 parameter_index
uint8 orb_index

# Corrections for acceleromter acceleration outputs where corrected_accel = raw_accel * accel_scale + accel_offset
# Note the corrections are in the sensor frame and must be applied before the sensor data is rotated into body frame
# corrections for uORB instance 0
float32[3] offset	# accelerometer XYZ offsets in the sensor frame in m/s/s
float32[3] scale	# accelerometer XYZ scale factors in the sensor frame

