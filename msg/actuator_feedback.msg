uint64 timestamp                   # time since system start (microseconds)
uint64 timestamp_sample            # the timestamp the data this control response is based on was sampled

uint8 NUM_ACTUATORS = 8

char[8] prefix

float32[8] actuator

uint8 function

float32[8] min
float32[8] max

float32[8] current

float32[8] setpoint
float32[8] setpoint_adjusted # battery scale, thrust, etc
