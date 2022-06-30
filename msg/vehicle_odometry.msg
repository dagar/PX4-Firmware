# Vehicle odometry data. Fits ROS REP 147 for aerial vehicles
uint64 timestamp		# time since system start (microseconds)
uint64 timestamp_sample

# Position and linear velocity frame of reference constants
uint8 FRAME_UNKNOWN     = 0
uint8 LOCAL_FRAME_NED   = 1 # NED earth-fixed frame
uint8 LOCAL_FRAME_FRD   = 2 # FRD earth-fixed frame, arbitrary heading reference
uint8 BODY_FRAME_FRD    = 3 # FRD body-fixed frame

# Position and linear velocity local frame of reference
uint8 local_frame

float32 position # Position in meters. Frame of reference defined by local_frame. NaN if invalid/unknown

# Orientation quaternion. First value NaN if invalid/unknown
float32[4] q			# Quaternion rotation from FRD body frame to refernce frame
float32[4] q_offset		# Quaternion rotation from odometry reference frame to navigation frame

# Row-major representation of 3x3 position cross-covariance matrix URT.
# Order: x, y, z
uint8 POSITION_COVARIANCE_X_VARIANCE = 0
uint8 POSITION_COVARIANCE_Y_VARIANCE = 3
uint8 POSITION_COVARIANCE_Z_VARIANCE = 5
float32[6] position_covariance

# Row-major representation of 3x3 orientation cross-covariance matrix URT.
# Order: roll, pitch, yaw
uint8 ORIENTATION_COVARIANCE_ROLL_VARIANCE  = 0
uint8 ORIENTATION_COVARIANCE_PITCH_VARIANCE = 3
uint8 ORIENTATION_COVARIANCE_YAW_VARIANCE   = 5
float32[6] orientation_covariance

# Reference frame of the velocity data
uint8 velocity_frame

float32 velocity         # Velocity in meters/sec. Frame of reference defined by velocity_frame variable. NaN if invalid/unknown

float32 angular_velocity # Angular velocity in body-fixed frame (rad/s). NaN if invalid/unknown

# Row-major representation of 3x3 velocity cross-covariance matrix URT.
# Order: vx, vy, vz
uint8 VELOCITY_COVARIANCE_VX_VARIANCE        = 0
uint8 VELOCITY_COVARIANCE_VY_VARIANCE        = 3
uint8 VELOCITY_COVARIANCE_VZ_VARIANCE        = 5
float32[6] velocity_covariance

uint8 reset_counter

# TOPICS vehicle_odometry vehicle_mocap_odometry vehicle_visual_odometry
# TOPICS estimator_odometry estimator_visual_odometry_aligned
