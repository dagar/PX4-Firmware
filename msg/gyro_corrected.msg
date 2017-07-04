#
# Gyro readings and in-run biases in SI-unit form. Sensor readings are compensated for static offsets,
# scale errors, in-run bias and thermal drift (if thermal compensation is enabled and available).
#

float32 x		# Bias corrected angular velocity about X body axis (roll) (in rad/s)
float32 y		# Bias corrected angular velocity about Y body axis (pitch) (in rad/s)
float32 z		# Bias corrected angular velocity about Z body axis (yaw) (in rad/s)
