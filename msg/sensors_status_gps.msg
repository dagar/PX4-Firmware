#
# Sensor check metrics. This will be zero for a sensor that's primary or unpopulated.
#
uint64 timestamp # time since system start (microseconds)

uint32 device_id_primary       # current primary accel device id for reference

uint32[4] device_ids
float32[4] inconsistency # magnitude of sensor difference between each instance and primary
bool[4] healthy
bool[4] calibrated



float32 hpos_drift_rate		# Horizontal position rate magnitude checked using EKF2_REQ_HDRIFT (m/s)
float32 vpos_drift_rate		# Vertical position rate magnitude checked using EKF2_REQ_VDRIFT (m/s)

float32 hspd			# Filtered horizontal velocity magnitude checked using EKF2_REQ_HDRIFT (m/s)

bool blocked			# true when drift calculation is blocked due to IMU movement check controlled by EKF2_MOVE_TEST



uint16 gps_check_fail_flags     # Bitmask to indicate status of GPS checks - see definition below

# bits are true when corresponding test has failed
uint8 GPS_CHECK_FAIL_GPS_FIX = 0		# 0 : insufficient fix type (no 3D solution)
uint8 GPS_CHECK_FAIL_MIN_SAT_COUNT = 1		# 1 : minimum required sat count fail
uint8 GPS_CHECK_FAIL_MIN_PDOP = 2		# 2 : minimum required PDOP fail
uint8 GPS_CHECK_FAIL_MAX_HORZ_ERR = 3		# 3 : maximum allowed horizontal position error fail
uint8 GPS_CHECK_FAIL_MAX_VERT_ERR = 4		# 4 : maximum allowed vertical position error fail
uint8 GPS_CHECK_FAIL_MAX_SPD_ERR = 5		# 5 : maximum allowed speed error fail
uint8 GPS_CHECK_FAIL_MAX_HORZ_DRIFT = 6		# 6 : maximum allowed horizontal position drift fail - requires stationary vehicle
uint8 GPS_CHECK_FAIL_MAX_VERT_DRIFT = 7		# 7 : maximum allowed vertical position drift fail - requires stationary vehicle
uint8 GPS_CHECK_FAIL_MAX_HORZ_SPD_ERR = 8	# 8 : maximum allowed horizontal speed fail - requires stationary vehicle
uint8 GPS_CHECK_FAIL_MAX_VERT_SPD_ERR = 9	# 9 : maximum allowed vertical velocity discrepancy fail
