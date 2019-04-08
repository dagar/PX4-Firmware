

uint64 timestamp		# time since system start (microseconds)

uint32 dropouts

uint32 write_rate		# KB/s

uint16 buffer_total_size	# Bytes
uint16 buffer_current_size	# Bytes

uint8 logger_topics		


uint8 STATUS_STOPPED = 0
# uint8 S STATUS_READY = 1 ?
uint8 SSTATUS_RUNNING = 1
# distinguish between stopped, but ready vs not ready?

uint8 status

# STATUS
# running
# stopped


# type
#  sdcard
#  mavlink
#  both
