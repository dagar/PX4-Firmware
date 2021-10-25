uint64 timestamp                   # time since system start (microseconds)
uint64 timestamp_sample            # the timestamp the data this control response is based on was sampled

uint8 NUM_ACTUATORS = 8

char[8] prefix

float32[8] actuator


uint8 FUNCTION_DISABLED     = 0
uint8 FUNCTION_CONSTANT_MIN = 1
uint8 FUNCTION_CONSTANT_MAX = 2

uint8 FUNCTION_MOTOR1       = 101
uint8 FUNCTION_MOTOR2       = 102
uint8 FUNCTION_MOTOR3       = 103
uint8 FUNCTION_MOTOR4       = 104
uint8 FUNCTION_MOTOR5       = 105
uint8 FUNCTION_MOTOR6       = 106
uint8 FUNCTION_MOTOR7       = 107
uint8 FUNCTION_MOTOR8       = 108

uint8 FUNCTION_SERVO1       = 201
uint8 FUNCTION_SERVO2       = 202
uint8 FUNCTION_SERVO3       = 203
uint8 FUNCTION_SERVO4       = 204
uint8 FUNCTION_SERVO5       = 205
uint8 FUNCTION_SERVO6       = 206
uint8 FUNCTION_SERVO7       = 207
uint8 FUNCTION_SERVO8       = 208


uint8 function

float32[8] min
float32[8] max

float32[8] current

float32[8] setpoint
float32[8] setpoint_adjusted # battery scale thrust etc


bool[8] enabled
bool[8] healthy
