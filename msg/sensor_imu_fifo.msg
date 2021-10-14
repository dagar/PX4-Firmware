uint64 timestamp          # time since system start (microseconds)
uint64 timestamp_sample

uint32 device_id          # unique device ID for the sensor that does not change between power cycles

float32 dt                # delta time between samples (microseconds)
float32 scale

uint8 samples             # number of valid samples

int16[16] x               # angular velocity in the FRD board frame X-axis in rad/s
int16[16] y               # angular velocity in the FRD board frame Y-axis in rad/s
int16[16] z               # angular velocity in the FRD board frame Z-axis in rad/s

float32 temperature       # temperature in degrees Celsius

uint32 error_count

uint8 ORB_QUEUE_LENGTH = 8
