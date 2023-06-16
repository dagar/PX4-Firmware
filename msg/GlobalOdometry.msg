# Placeholder for px4 message /fmu/in/vehicle_global_odometry
# Vehicle odometry data. Fits ROS REP 147 for aerial vehicles
uint64 timestamp		# publication timestamp
uint64 timestamp_sample #  timestamp for when the data is valid for.

# uint8 POSE_FRAME_UNKNOWN = 0
# uint8 POSE_FRAME_NED     = 1 # NED earth-fixed frame
# uint8 POSE_FRAME_FRD     = 2 # FRD world-fixed frame, arbitrary heading reference
# uint8 pose_frame            # Position and orientation frame of reference
float64 latitude  # degrees
float64 longitude # degrees
#float64 altitude  # meters (above MSL)
float64 altitude_agl # meters (above ground level)
float64 positional_uncertainty # meters


bool valid # consume or not.

# float32[3] position         # Position in meters. Frame of reference defined by local_frame. NaN if invalid/unknown
#float32[4] q                # Local level NED to FRD body frame
float64 heading # radians +/- pi
bool heading_valid # false

# TODO: add these once we come up with the math.
# float32[3] position_variance
# float32[3] orientation_variance

########################
# Logging only right now.
int32 number_of_features_img1
int32 number_of_features_img2
int32 number_of_matches_threshold1
int32 number_of_matches_threshold2
int32 number_of_matches_threshold3
float32 fit_reprojection_error
float32[9] fit_homography_matrix # raw fit homography matrix
float32[3] fit_ned               #

# end logging section
#####################

# TOPICS global_odometry
