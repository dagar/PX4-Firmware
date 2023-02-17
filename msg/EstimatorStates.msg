uint64 timestamp		# time since system start (microseconds)
uint64 timestamp_sample         # the timestamp of the raw data (microseconds)

float32[24] states		# Internal filter states
uint8 n_states		# Number of states effectively used

float32[24] covariances	# Diagonal Elements of Covariance Matrix

# resets
float32[24] reset_delta
uint16 reset_counter		# Incremented by 1 for each reset event
