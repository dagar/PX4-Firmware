#pragma once

#include <uORB/ucdr/vehicle_control_mode.h>


// INPUT
//  topic_name  (eg /fmu/out/vehicle_control_mode)
//  dataType    (eg px4_msgs::msg::dds_::VehicleControlMode_)
//  ORB_ID      (eg ORB_ID(vehicle_control_mode))
//  integer for object_id (use ORB_ID enum?)
//  uint32_t topic_size

//  missing
//   uint32_t topic_size = ucdr_topic_size_vehicle_control_mode();
//   ucdr_serialize_vehicle_control_mode(data, ub);


// TODO: how to handle multiple instances? topic naming, etc

// TODO: with and without subscriptioninterval?

// TODO: best effort or not

// max rate in milliseconds (stored to a small integer)

// automatically use reliable if queue depth


// default to all topics at 1 Hz


#define TOPIC_NAME_SIZE 128

static bool generate_topic_name(char *topic, const char *client_namespace, const char *direction, const char *name)
{
	int ret;

	if (client_namespace == nullptr) {
		ret = snprintf(topic, TOPIC_NAME_SIZE, "rt/fmu/%s/%s", direction, name);
		return (ret > 0 && ret < TOPIC_NAME_SIZE);

	} else {
		ret = snprintf(topic, TOPIC_NAME_SIZE, "rt/%s/fmu/%s/%s", client_namespace, direction, name);
		return (ret > 0 && ret < TOPIC_NAME_SIZE);
	}
}


class DataWrite
{
public:
	// uint32_t topic_size = ucdr_topic_size_vehicle_control_mode();
	// serialize_func_ptr
	DataWrite(ORB_ID orb_id, const char *data_type_name, uint16_t topic_size, serialize_func_ptr) :
		_sub(orb_id),
		_topic_size(topic_size),
		_data_type_name(data_type_name),
		_serialize_func_ptr(serialize_func_ptr)
	{

	}

	int update(uxrSession *session, uxrStreamId stream_id)
	{
		vehicle_control_mode_s data;

		if (_sub.update(&data)) {
			ucdrBuffer ub;
			uxr_prepare_output_stream(session, stream_id, _datawriter_id, &ub, _topic_size);

			// serialize_func_ptr
			ucdr_serialize_vehicle_control_mode(data, ub);

			// TODO: fill up the MTU and then flush, which reduces the packet overhead
			uxr_flash_output_streams(session);
			//return num_payload_sent += topic_size;
			return topic_size;
		}

		return 0;
	}

	bool init(uxrSession *session, uxrStreamId stream_id, uxrObjectId participant_id, const char *client_namespace)
	{
		// TODO: don't create until topic is advertised

		char *topic_name = _sub.get_topic()->o_name;
		// _data_type_name
		uint16_t id = _sub.orb_id() + _sub.get_instance();

		uxrObjectId topic_id = uxr_object_id(id, UXR_TOPIC_ID);

		char topic[TOPIC_NAME_SIZE];

		if (!generate_topic_name(topic, client_namespace, "out", topic_name)) {
			PX4_ERR("topic path too long");
			return false;
		}

		char data_type[TOPIC_NAME_SIZE] {"px4_msgs::msg::dds_::@(topic_pascal)_"}; // "px4_msgs::msg::dds_::@(topic_pascal)_"
		uint16_t topic_req = uxr_buffer_create_topic_bin(session, stream_id, topic_id, participant_id, topic, data_type,
				     UXR_REPLACE);


		uxrObjectId publisher_id = uxr_object_id(id, UXR_PUBLISHER_ID);
		const char *publisher_xml = "";
		uint16_t publisher_req = uxr_buffer_create_publisher_xml(session, stream_id, publisher_id, participant_id,
					 publisher_xml, UXR_REPLACE);


		uxrQoS_t qos{
			.durability = UXR_DURABILITY_TRANSIENT_LOCAL,
			.reliability = UXR_RELIABILITY_RELIABLE,
			.history = UXR_HISTORY_KEEP_LAST,
			.depth = 0
		};
		_datawriter_id = uxr_object_id(id, UXR_DATAWRITER_ID);
		uint16_t datawriter_req = uxr_buffer_create_datawriter_bin(session, stream_id, _datawriter_id, publisher_id, topic_id,
					  qos, UXR_REPLACE);

		// Send create entities message and wait its status
		uint16_t requests[3] {topic_req, publisher_req, datawriter_req};
		uint8_t status[3];

		if (!uxr_run_session_until_all_status(session, 1000, requests, status, 3)) {
			PX4_ERR("create entities failed: %s, topic: %i publisher: %i datawriter: %i",
				"/fmu/out/vehicle_control_mode",
				status[0], status[1], status[2]);
			return false;
		}

		return true;
	}

private:
	uORB::Subscription _sub;

	uxrObjectId _datawriter_id;

	uint16_t _topic_size{0};

	char *_data_type_name{nullptr};

	serialize_func_ptr

	uint16_t _max_update_ms{1000};
};


// ORB_ID orb_id_enum = ORB_ID::timesync;
// const char* topic_name = "timesync";
// const char* dds_type_name = "px4_msgs::msg::dds_::Timesync_";

// timesync_s data;
// uint32_t topic_size = ucdr_topic_size_timesync();

// ucdrBuffer ub;
// uxr_prepare_output_stream(session, publish_stream_id, data_writer, &ub, topic_size);

// if (ucdr_buffer_remaining(&buf) < topic_size) {
// 	return false;
// }

// uint8_t raw_buffer[1024];

// if (sub.update(raw_buffer)) {
// 	if (data_writer.id == UXR_INVALID_ID) {
// 		// data writer not created yet
// 		create_data_writer(session, create_stream_id, participant_id, orb_id_enum, client_namespace, topic_name, dds_type_name, data_writer);
// 	}

// 	ucdr_serialize_timesync(raw_buffer, ub, time_offset_us);

// 	// TODO: fill up the MTU and then flush, which reduces the packet overhead
// 	uxr_flash_output_streams(session);
// 	num_payload_sent += topic_size;
// }



// updateDataWriter(uORB::Subscription& sub, ucdr_topic_size_timesync, ucdr_serialize_timesync)
