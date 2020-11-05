
uint64 timestamp        # time since system start (microseconds)
uint64 timestamp_sample # timestamp of the data sample on which this message is based (microseconds)

float32[4] q                    # Quaternion rotation from XYZ body frame to NED earth frame.
float32[4] delta_q_reset        # Amount by which quaternion has changed during last reset
uint8 quat_reset_counter        # Quaternion reset counter

float32[3] angular_velocity     # Bias corrected angular velocity about X, Y, Z body axis in rad/s
float32[3] angular_acceleration # Bias corrected angular acceleration about X, Y, Z body axis in rad/s

float32[3] linear_acceleration  # Bias corrected body frame acceleration without gravity (metres/sec^2)

float32[3] velocity_body        # velocity in body frame, (metres/sec)
float32[3] velocity_ned         # velocity in NED frame, (metres/sec)

float32[3] position             # position in NED frame, (metres)

# Position reset delta
float32[2] delta_xy
uint8 xy_reset_counter

float32 delta_z
uint8 z_reset_counter


bool velocity_valid
bool position_valid
