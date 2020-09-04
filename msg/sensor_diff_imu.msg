#
# IMU sensor difference relative to the primary sensor.
# These will be all be zero if the vehicle only has one sensor.
# The value for the primary sensor will be zero.
#
uint64 timestamp # time since system start (microseconds)
uint32[3] accel_device_ids
float32[3] accel_inconsistency_m_s_s # magnitude of maximum acceleration difference between each IMU instance and the primary in (m/s/s).
uint32[3] gyro_device_ids
float32[3] gyro_inconsistency_rad_s # magnitude of maximum angular rate difference between each IMU instance and the primary in (rad/s).
uint8 MAX_SENSORS = 3
