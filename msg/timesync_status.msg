uint64 timestamp			# time since system start (microseconds)

uint8 SOURCE_UNKNOWN          = 0
uint8 SOURCE_PROTOCOL_MAVLINK = 1
uint8 SOURCE_PROTOCOL_RTPS    = 2
uint8 SOURCE_GPS              = 3
uint8 source                            # timesync source

int64 observed_offset			# raw time offset directly observed from this timesync packet (microseconds)
int64 estimated_offset			# smoothed time offset between companion system and PX4 (microseconds)

uint64 local_time
uint64 local_time_to_remote
uint64 remote_time
uint64 remote_time_to_local

int32 sequence
