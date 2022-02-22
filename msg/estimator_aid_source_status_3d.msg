uint64 timestamp			# time since system start (microseconds)
uint64 timestamp_sample			# the timestamp of the raw data (microseconds)

uint8 estimator_instance

uint32 device_id

uint64 time_last_measurement

uint64[3] time_last_fuse

float32[3] innov
float32[3] innov_var
float32[3] test_ratio

bool[3] filter_control_status_fuse       # true when measurements are being fused
bool[3] fault_status_numerical_error     # true if fusion has encountered a numerical error
bool[3] innovation_fault_status_rejected # true if the observation has been rejected

uint8 num_resets_available

# measurement?
float32[3] measurement_adjusted
float32[3] measurement_variance

# TOPICS estimator_aid_source_status_3d estimator_status_ev_pos estimator_status_ev_vel
