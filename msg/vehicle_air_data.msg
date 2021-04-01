
uint64 timestamp                # time since system start (microseconds)
uint64 timestamp_sample         # the timestamp of the raw data (microseconds)

uint32 device_id                # unique device ID for the selected barometer

float32 barometric_altitude_m   # Altitude above MSL calculated from temperature compensated baro sensor data using an ISA corrected for sea level pressure SENS_BARO_QNH.
float32 barometric_pressure_pa  # Absolute pressure in Pascals

float32 air_temperature_celcius # Temperature in degrees Celsius

float32 rho                     # air density

uint8 calibration_count         # Calibration changed counter. Monotonically increases whenever calibration changes.
