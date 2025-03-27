# Local setpoint constraints in NED frame
# setting something to NaN means that no limit is provided

uint64 timestamp # time since system start (microseconds)

float32 speed_up # in meters/sec
float32 speed_down # in meters/sec

bool want_takeoff # tell the controller to initiate takeoff when idling (ignored during flight)


uint8 DROP_STATE_UNKNOWN                             = 0
uint8 DROP_STATE_DISARMED                            = 1
uint8 DROP_STATE_ARMING                              = 2
uint8 DROP_STATE_WAITING_FOR_DROP_DETECT             = 3
uint8 DROP_STATE_RATE_CONTROL_ENABLED                = 4
uint8 DROP_STATE_ATTITUDE_CONTROL_ENABLED            = 5
uint8 DROP_STATE_HEIGHT_RATE_CONTROL_ENABLED         = 6
uint8 DROP_STATE_VELOCITY_CONTROL_ENABLED            = 7
uint8 DROP_STATE_POSITION_CONTROL_ENABLED            = 8
uint8 DROP_STATE_FLYING                              = 9

uint8 drop_state
