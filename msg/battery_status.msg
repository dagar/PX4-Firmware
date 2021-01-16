uint64 timestamp			# time since system start (microseconds)

int8 id                                 # ID number of a battery. Should be unique and consistent for the lifetime of a vehicle. 1-indexed.

float32 voltage_v                       # Battery voltage in volts, 0 if unknown
float32 voltage_filtered_v              # Battery voltage in volts, filtered, 0 if unknown

float32 current_a                       # Battery current in amperes, -1 if unknown
float32 current_filtered_a              # Battery current in amperes, filtered, 0 if unknown
float32 discharged_mah                  # Discharged amount in mAh, -1 if unknown

float32 remaining			# From 1 to 0, -1 if unknown

float32 scale				# Power scaling factor, >= 1, or -1 if unknown

int8 cell_count                         # Number of cells

bool connected				# Whether or not a battery is connected, based on a voltage threshold

uint8 BATTERY_SOURCE_POWER_MODULE = 0
uint8 BATTERY_SOURCE_EXTERNAL     = 1
uint8 BATTERY_SOURCE_ESCS         = 2
uint8 source                            # Battery source

uint16 capacity                         # actual capacity of the battery

uint8 BATTERY_WARNING_NONE      = 0	# no battery low voltage warning active
uint8 BATTERY_WARNING_LOW       = 1	# warning of low voltage
uint8 BATTERY_WARNING_CRITICAL  = 2	# critical voltage, return / abort immediately
uint8 BATTERY_WARNING_EMERGENCY = 3	# immediate landing required
uint8 BATTERY_WARNING_FAILED    = 4	# the battery has failed completely
uint8 warning				# current battery warning

uint8 MAX_INSTANCES = 4
