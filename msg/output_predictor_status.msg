uint64 timestamp		# time since system start (microseconds)
uint64 timestamp_sample         # the timestamp of the raw data (microseconds)

float32 angle_tracking_error     # output predictor angular tracking error magnitudes (rad)

float32[3] velocity_tracking_error  # output predictor velocity tracking error magnitudes (m/s)
float32[3] velocity_body_tracking_error  # output predictor velocity tracking error magnitudes (m/s)

float32[3] position_tracking_error  # output predictor position tracking error magnitudes (m)
