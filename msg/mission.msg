uint64 timestamp	# time since system start (microseconds)

uint8 dataman_id	# there are two offboard storage places in the dataman (DM_KEY_WAYPOINTS_OFFBOARD_0 & DM_KEY_WAYPOINTS_OFFBOARD_1)

uint16 count		# count of the missions stored in the dataman
int32 current_seq	# default -1, start at the one changed latest
