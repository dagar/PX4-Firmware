uint64 timestamp		# time since system start (microseconds)

float32 voltage5v_v		# peripheral 5V rail voltage
float32 voltage3v3_v		# Sensor 3V3 rail voltage

bool v3v3_valid                 # Sensor 3V3 rail voltage was read.

bool usb_connected
bool usb_valid

uint8 brick_valid_flags		# brick bits power is good when bit 1

bool servo_valid		# servo power is good when true

bool periph_5v_oc		# peripheral overcurrent when true
bool hipower_5v_oc		# hi power peripheral overcurrent when true
