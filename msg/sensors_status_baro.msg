#
# Sensor check metrics.
#
uint64 timestamp # time since system start (microseconds)

uint32 device_id_primary       # current primary device id for reference
uint32[3] device_ids
float32[3] inconsistency       # maximum angle between magnetometer instance field vectors in radians.
bool[3] healthy                # sensor healthy
