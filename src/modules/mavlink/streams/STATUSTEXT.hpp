
#include <uORB/topics/log_message.h>

class MavlinkStreamStatustext : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamStatustext(mavlink); }

	static constexpr const char *get_name_static() { return "STATUSTEXT"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_STATUSTEXT; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _mavlink_log_sub.updated() ? (MAVLINK_MSG_ID_STATUSTEXT_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES) : 0;
	}

private:
	explicit MavlinkStreamStatustext(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _mavlink_log_sub{ORB_ID(mavlink_log)};
	uint16_t _id{0};

	bool send() override
	{
		log_message_s mavlink_log;

		if ((_mavlink->get_free_tx_buf() >= get_size()) && _mavlink_log_sub.update(&mavlink_log)) {

			static constexpr size_t MAX_CHUNK_SIZE = sizeof(msg.text);
			const size_t text_length = strlen(mavlink_log.text);

			mavlink_statustext_t msg;
			msg.severity = mavlink_log.severity;
			msg.id = _id++;

			size_t copied = 0;
			uint8_t chunk_seq = 0;

			while (copied < text_length) {

				memcpy(msg.text, &mavlink_log.text, math::min(text_length, MAX_CHUNK_SIZE));

				// pad with zeros
				memset(&msg.text[0] + chunk_size, 0, MAX_CHUNK_SIZE - chunk_size);

				msg.chunk_seq = chunk_seq++;
				mavlink_msg_statustext_send_struct(_mavlink->get_channel(), &msg);

				msg.chunk_seq++;
			}

			return true;
		}

		return false;
	}
};
