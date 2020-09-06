#
# Sensor check metrics.
#
uint64 timestamp # time since system start (microseconds)

uint32 device_id_primary       # current primary device id for reference
uint32[4] device_ids
float32[4] angle_diff          # maximum angle between magnetometer instance field vectors in radians.
float32[4] inconsistency       # maximum angle between magnetometer instance field vectors in radians.
bool[4] healthy                # sensor healthy
