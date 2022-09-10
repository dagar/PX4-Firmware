
#include <uORB/ucdr/sensor_optical_flow.h>


session = session_;
uxr_set_topic_callback(session, on_topic_update, this);


{
	uxrObjectId subscriber_id = uxr_object_id(5 + 1, UXR_SUBSCRIBER_ID);
	const char *subscriber_xml = "";
	uint16_t subscriber_req = uxr_buffer_create_subscriber_xml(session, stream_id, subscriber_id, participant_id,
				  subscriber_xml, UXR_REPLACE);

	uxrObjectId topic_id = uxr_object_id(1000 + 5, UXR_TOPIC_ID);

	const char *topic_xml =
		"<dds>"
		"<topic>"
		"<name>rt/fmu/in/sensor_optical_flow</name>"
		"<dataType>px4_msgs::msg::dds_::SensorOpticalFlow_</dataType>"
		"</topic>"
		"</dds>";

	uint16_t topic_req = uxr_buffer_create_topic_xml(session, stream_id, topic_id, participant_id, topic_xml, UXR_REPLACE);

	uxrObjectId datareader_id = uxr_object_id(5 + 1, UXR_DATAREADER_ID);

	const char *datareader_xml =
		"<dds>"
		"<data_reader>"
		"<topic>"
		"<kind>NO_KEY</kind>"
		"<name>rt/fmu/in/sensor_optical_flow</name>"
		"<dataType>px4_msgs::msg::dds_::SensorOpticalFlow_</dataType>"
		"</topic>"
		"</data_reader>"
		"</dds>";

	uint16_t datareader_req = uxr_buffer_create_datareader_xml(session, stream_id, datareader_id, subscriber_id,
				  datareader_xml, UXR_REPLACE);
	uint16_t requests[3] {topic_req, subscriber_req, datareader_req};
	uint8_t status[3];

	if (!uxr_run_session_until_all_status(session, 1000, requests, status, 3)) {
		PX4_ERR("create entities failed: %s %i %i %i",
			"/fmu/in/sensor_optical_flow",
			status[0], status[1], status[2]);
		return false;
	}

	uxrDeliveryControl delivery_control{};
	delivery_control.max_samples = UXR_MAX_SAMPLES_UNLIMITED;
	uxr_buffer_request_data(session, stream_id, datareader_id, input_stream, &delivery_control);

}



uORB::Publication<sensor_optical_flow_s> sensor_optical_flow_pub{ORB_ID(sensor_optical_flow)};




void on_topic_update(uxrSession *session, uxrObjectId object_id, uint16_t request_id, uxrStreamId stream_id,
		     struct ucdrBuffer *ub, uint16_t length, void *args)
{
	RcvTopicsPubs *pubs = (RcvTopicsPubs *)args;
	pubs->num_payload_received += length;

	switch (object_id.id) {
	case 5+1: {
			sensor_optical_flow_s data;

			if (ucdr_deserialize_sensor_optical_flow(*ub, data)) {
				pubs->sensor_optical_flow_pub.publish(data);
			}
		}
		break;
	}
}
