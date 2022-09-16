
#pragma once


#include <uORB/Subscription.hpp>


typedef bool (*ucdr_serialize)(const timesync_s &topic, ucdrBuffer &buf, int64_t time_offset = 0);

// dynamic area of DataWriters based on configuration


class DataWriter
{
public:
	// uint32_t topic_size = ucdr_topic_size_vehicle_control_mode();
	// serialize_func_ptr
	DataWriter(ORB_ID orb_id, const char *data_type_name, uint16_t topic_size, serialize_func_ptr) :
		_sub(orb_id),
		_topic_size(topic_size),
		_data_type_name(data_type_name),
		_serialize_func_ptr(serialize_func_ptr)
	{

	}

private:

	uORB::Subscription _sub;

	uxrObjectId _datawriter_id;

	uint16_t _topic_size{0};

	char *_data_type_name{nullptr};

	serialize_func_ptr

	bool (*ucdr_serialize)(const timesync_s &topic, ucdrBuffer &buf, int64_t time_offset = 0) ;

	uint16_t _max_update_ms{1000};
};

