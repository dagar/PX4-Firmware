# A logging message, output with PX4_{WARN,ERR,INFO}

uint64 timestamp		# time since system start (microseconds)

uint8 LOG_LEVEL_DEBUG = 0
uint8 LOG_LEVEL_INFO  = 1
uint8 LOG_LEVEL_WARN  = 2
uint8 LOG_LEVEL_ERROR = 3
uint8 LOG_LEVEL_PANIC = 4

uint8 severity # log level (same as in the linux kernel, starting with 0)

char[127] text

uint8 ORB_QUEUE_LENGTH = 8

# TOPICS log_message mavlink_log
