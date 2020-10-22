#
# Sensor check metrics. This will be zero for a sensor that's primary or unpopulated.
#
uint64 timestamp # time since system start (microseconds)

uint32 device_id_primary       # current primary accel device id for reference

uint32[4] device_ids
float32[4] inconsistency # magnitude of sensor difference between each instance and primary
bool[4] healthy
bool[4] calibrated

# TOPICS sensors_status sensors_status_accel sensors_status_baro sensors_status_gyro
# TOPICS sensors_status_gps sensors_status_mag
