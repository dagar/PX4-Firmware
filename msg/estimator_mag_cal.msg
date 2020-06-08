uint64 timestamp		# time since system start (microseconds)

float32[3] sample		# magnetic field in the NED board axis in Gauss

float32[3] expected_field

float32[3] calibrated_sample


float32 declination
float32 inclination
float32 mag_strength

float32[3] bias_estimate
float32[9] scale_estimate
float32[3] bias_estimate_variance
float32[9] scale_estimate_variance
