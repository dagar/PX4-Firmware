uint64 timestamp                 # time since system start (microseconds)

uint64 timestamp_sample          # the timestamp of the raw data (microseconds)

uint8 SOURCE_RC = 1			# radio control
uint8 SOURCE_MAVLINK_0 = 2		# mavlink instance 0
uint8 SOURCE_MAVLINK_1 = 3		# mavlink instance 1
uint8 SOURCE_MAVLINK_2 = 4		# mavlink instance 2
uint8 SOURCE_MAVLINK_3 = 5		# mavlink instance 4

uint8 data_source		 # where this input is coming from

uint8 BUTTON_POS_NONE   = 0      # button is not mapped
uint8 BUTTON_POS_ON     = 1      # button activated (value = 1)
uint8 BUTTON_POS_OFF    = 2      # button not activated (value = -1)

uint8[16] button

uint8 MAX_BUTTONS = 16

uint32 button_changes            # number of button changes
