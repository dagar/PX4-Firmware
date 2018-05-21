float32[24] states		# Internal filter states
float32 n_states		# Number of states effectively used

float32[3] vibe			# IMU vibration metrics in the following array locations
# 0 : Gyro delta angle coning metric = filtered length of (delta_angle x prev_delta_angle)
# 1 : Gyro high frequency vibe = filtered length of (delta_angle - prev_delta_angle)
# 2 : Accel high frequency vibe = filtered length of (delta_velocity - prev_delta_velocity)

uint8 health_flags		# Bitmask to indicate sensor health states (vel, pos, hgt)
uint8 timeout_flags		# Bitmask to indicate timeout flags (vel, pos, hgt)

float32[24] covariances	# Diagonal Elements of Covariance Matrix

float32 mag_test_ratio # ratio of the largest magnetometer innovation component to the innovation test limit
float32 vel_test_ratio # ratio of the largest velocity innovation component to the innovation test limit
float32 pos_test_ratio # ratio of the largest horizontal position innovation component to the innovation test limit
float32 hgt_test_ratio # ratio of the vertical position innovation to the innovation test limit
float32 tas_test_ratio # ratio of the true airspeed innovation to the innovation test limit
float32 hagl_test_ratio # ratio of the height above ground innovation to the innovation test limit
float32 beta_test_ratio # ratio of the synthetic sideslip innovation to the innovation test limit

float32 time_slip # cumulative amount of time in seconds that the EKF inertial calculation has slipped relative to system time
