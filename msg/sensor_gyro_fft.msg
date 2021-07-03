uint64 timestamp          # time since system start (microseconds)
uint64 timestamp_sample

uint32 device_id          # unique device ID for the sensor that does not change between power cycles

float32 sensor_sample_rate_hz
float32 resolution_hz

float32[4] peak_frequencies_x # x axis peak frequencies
float32[4] peak_frequencies_y # y axis peak frequencies
float32[4] peak_frequencies_z # z axis peak frequencies

float32[4] peak_magnitude_x   # x axis peak frequencies magnitude
float32[4] peak_magnitude_y   # y axis peak frequencies magnitude
float32[4] peak_magnitude_z   # z axis peak frequencies magnitude

float32[4] peak_magnitude_x_low   # x axis peak frequencies magnitude
float32[4] peak_magnitude_y_low   # y axis peak frequencies magnitude
float32[4] peak_magnitude_z_low   # z axis peak frequencies magnitude

float32[4] peak_magnitude_x_up   # x axis peak frequencies magnitude
float32[4] peak_magnitude_y_up   # y axis peak frequencies magnitude
float32[4] peak_magnitude_z_up   # z axis peak frequencies magnitude

float32[4] peak_snr_x # x axis peak SNR
float32[4] peak_snr_y # y axis peak SNR
float32[4] peak_snr_z # z axis peak SNR

float32[3] total_energy
