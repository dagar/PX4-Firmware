uint64 timestamp	# time since system start (microseconds)

uint8 DATAMAN_ID_UNKNOWN = 0
uint8 DATAMAN_ID_0       = 1
uint8 DATAMAN_ID_1       = 2

uint8 dataman_id	# there are two storage places in the data manager (dataman): DATAMAN_ID_0 or DATAMAN_ID_1

uint16 count		# count of the mission items stored in the dataman
int32 current_seq	# default -1, start at the one changed latest
