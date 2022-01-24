/****************************************************************************
 *
 * Copyright (c) 2017 Proyectos y Sistemas de Mantenimiento SL (eProsima).
 * Copyright (c) 2018-2021 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <microRTPS_transport.h>
#include <microRTPS_client.h>

#include <inttypes.h>
#include <cstdio>
#include <ctime>
#include <pthread.h>

#include <ucdr/microcdr.h>
#include <px4_time.h>
#include <uORB/uORB.h>

#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/onboard_computer_status.h>
#include <uORB_microcdr/topics/onboard_computer_status.h>
#include <uORB/topics/debug_key_value.h>
#include <uORB_microcdr/topics/debug_key_value.h>
#include <uORB/topics/debug_vect.h>
#include <uORB_microcdr/topics/debug_vect.h>
#include <uORB/topics/debug_array.h>
#include <uORB_microcdr/topics/debug_array.h>
#include <uORB/topics/trajectory_bezier.h>
#include <uORB_microcdr/topics/trajectory_bezier.h>
#include <uORB/topics/offboard_control_mode.h>
#include <uORB_microcdr/topics/offboard_control_mode.h>
#include <uORB/topics/vehicle_trajectory_waypoint.h>
#include <uORB_microcdr/topics/vehicle_trajectory_waypoint.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB_microcdr/topics/sensor_combined.h>
#include <uORB/topics/position_setpoint.h>
#include <uORB_microcdr/topics/position_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB_microcdr/topics/vehicle_status.h>
#include <uORB/topics/vehicle_odometry.h>
#include <uORB_microcdr/topics/vehicle_odometry.h>
#include <uORB/topics/trajectory_waypoint.h>
#include <uORB_microcdr/topics/trajectory_waypoint.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB_microcdr/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB_microcdr/topics/vehicle_control_mode.h>
#include <uORB/topics/timesync.h>
#include <uORB_microcdr/topics/timesync.h>
#include <uORB/topics/collision_constraints.h>
#include <uORB_microcdr/topics/collision_constraints.h>
#include <uORB/topics/vehicle_trajectory_bezier.h>
#include <uORB_microcdr/topics/vehicle_trajectory_bezier.h>
#include <uORB/topics/optical_flow.h>
#include <uORB_microcdr/topics/optical_flow.h>
#include <uORB/topics/debug_value.h>
#include <uORB_microcdr/topics/debug_value.h>
#include <uORB/topics/telemetry_status.h>
#include <uORB_microcdr/topics/telemetry_status.h>
#include <uORB/topics/timesync_status.h>
#include <uORB_microcdr/topics/timesync_status.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB_microcdr/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB_microcdr/topics/vehicle_command.h>

using namespace time_literals;

// Publishers for received messages
struct RcvTopicsPubs {
	uORB::Publication <debug_array_s> debug_array_pub{ORB_ID(debug_array)};
	uORB::Publication <debug_key_value_s> debug_key_value_pub{ORB_ID(debug_key_value)};
	uORB::Publication <debug_value_s> debug_value_pub{ORB_ID(debug_value)};
	uORB::Publication <debug_vect_s> debug_vect_pub{ORB_ID(debug_vect)};
	uORB::Publication <offboard_control_mode_s> offboard_control_mode_pub{ORB_ID(offboard_control_mode)};
	uORB::Publication <optical_flow_s> optical_flow_pub{ORB_ID(optical_flow)};
	uORB::Publication <position_setpoint_s> position_setpoint_pub{ORB_ID(position_setpoint)};
	uORB::Publication <position_setpoint_triplet_s> position_setpoint_triplet_pub{ORB_ID(position_setpoint_triplet)};
	uORB::Publication <telemetry_status_s> telemetry_status_pub{ORB_ID(telemetry_status)};
	uORB::Publication <timesync_s> timesync_pub{ORB_ID(timesync)};
	uORB::Publication <vehicle_command_s> vehicle_command_pub{ORB_ID(vehicle_command)};
	uORB::Publication <vehicle_local_position_setpoint_s> vehicle_local_position_setpoint_pub{ORB_ID(vehicle_local_position_setpoint)};
	uORB::Publication <vehicle_trajectory_waypoint_s> vehicle_trajectory_waypoint_pub{ORB_ID(vehicle_trajectory_waypoint)};
	uORB::Publication <onboard_computer_status_s> onboard_computer_status_pub{ORB_ID(onboard_computer_status)};
	uORB::Publication <trajectory_bezier_s> trajectory_bezier_pub{ORB_ID(trajectory_bezier)};
	uORB::Publication <vehicle_trajectory_bezier_s> vehicle_trajectory_bezier_pub{ORB_ID(vehicle_trajectory_bezier)};
	uORB::Publication <vehicle_local_position_setpoint_s> trajectory_setpoint_pub{ORB_ID(trajectory_setpoint)};
	uORB::Publication <vehicle_odometry_s> vehicle_mocap_odometry_pub{ORB_ID(vehicle_mocap_odometry)};
	uORB::Publication <vehicle_odometry_s> vehicle_visual_odometry_pub{ORB_ID(vehicle_visual_odometry)};
};

// Subscribers for messages to send
struct SendTopicsSubs {
	uORB::Subscription timesync_sub{ORB_ID(timesync)};
	uORB::Subscription trajectory_waypoint_sub{ORB_ID(trajectory_waypoint)};
	uORB::Subscription vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription vehicle_odometry_sub{ORB_ID(vehicle_odometry)};
	uORB::Subscription vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription collision_constraints_sub{ORB_ID(collision_constraints)};
	uORB::Subscription timesync_status_sub{ORB_ID(timesync_status)};
	uORB::Subscription sensor_combined_sub{ORB_ID(sensor_combined)};
	uORB::Subscription vehicle_trajectory_waypoint_desired_sub{ORB_ID(vehicle_trajectory_waypoint_desired)};
};

struct SendThreadArgs {
	const uint32_t &datarate;
	uint64_t &total_sent;
	uint64_t &sent_last_sec;
	uint64_t &sent;
	int &sent_loop;
	SendThreadArgs(const uint32_t &datarate_, uint64_t &total_sent_,
                    uint64_t &sent_last_sec_, uint64_t &sent_, int &sent_loop_)
		: datarate(datarate_),
		  total_sent(total_sent_),
		  sent_last_sec(sent_last_sec_),
		  sent(sent_),
		  sent_loop(sent_loop_) {}
};

void *send(void *args)
{
	char data_buffer[BUFFER_SIZE]{};
	int read{0};
	uint32_t length{0};
	size_t header_length{0};
	uint8_t last_msg_seq{0};
	uint8_t last_remote_msg_seq{0};

	struct SendThreadArgs *data = reinterpret_cast<struct SendThreadArgs *>(args);
	SendTopicsSubs *subs = new SendTopicsSubs();

	float bandwidth_mult{0};
	float tx_interval{1.f};
	uint64_t tx_last_sec_read{0};
	hrt_abstime last_stats_update{0};

	// ucdrBuffer to serialize using the user defined buffer
	ucdrBuffer writer;
	header_length = transport_node->get_header_length();
	ucdr_init_buffer(&writer, reinterpret_cast<uint8_t *>(&data_buffer[header_length]), BUFFER_SIZE - header_length);

	while (!_should_exit_task) {
		{
			timesync_s timesync_data;

			if (subs->timesync_sub.update(&timesync_data))
			{
				if (timesync_data.seq != last_remote_msg_seq && timesync_data.tc1 == 0) {
					last_remote_msg_seq = timesync_data.seq;

					timesync_data.timestamp = hrt_absolute_time();
					timesync_data.seq = last_msg_seq;
					timesync_data.tc1 = hrt_absolute_time() * 1000ULL;
					timesync_data.ts1 = timesync_data.ts1;

					last_msg_seq++;
					// copy raw data into local buffer. Payload is shifted by header length to make room for header
					serialize_timesync(&writer, &timesync_data, &data_buffer[header_length], &length);

					if (0 < (read = transport_node->write(static_cast<char>(10), data_buffer, length))) {
						data->total_sent += read;
						tx_last_sec_read += read;
						++data->sent;
					}

				}

			}
		}
		{
			trajectory_waypoint_s trajectory_waypoint_data;

			if (subs->trajectory_waypoint_sub.update(&trajectory_waypoint_data))
			{
					// copy raw data into local buffer. Payload is shifted by header length to make room for header
					serialize_trajectory_waypoint(&writer, &trajectory_waypoint_data, &data_buffer[header_length], &length);

					if (0 < (read = transport_node->write(static_cast<char>(11), data_buffer, length))) {
						data->total_sent += read;
						tx_last_sec_read += read;
						++data->sent;
					}

			}
		}
		{
			vehicle_control_mode_s vehicle_control_mode_data;

			if (subs->vehicle_control_mode_sub.update(&vehicle_control_mode_data))
			{
					// copy raw data into local buffer. Payload is shifted by header length to make room for header
					serialize_vehicle_control_mode(&writer, &vehicle_control_mode_data, &data_buffer[header_length], &length);

					if (0 < (read = transport_node->write(static_cast<char>(13), data_buffer, length))) {
						data->total_sent += read;
						tx_last_sec_read += read;
						++data->sent;
					}

			}
		}
		{
			vehicle_odometry_s vehicle_odometry_data;

			if (subs->vehicle_odometry_sub.update(&vehicle_odometry_data))
			{
					// copy raw data into local buffer. Payload is shifted by header length to make room for header
					serialize_vehicle_odometry(&writer, &vehicle_odometry_data, &data_buffer[header_length], &length);

					if (0 < (read = transport_node->write(static_cast<char>(16), data_buffer, length))) {
						data->total_sent += read;
						tx_last_sec_read += read;
						++data->sent;
					}

			}
		}
		{
			vehicle_status_s vehicle_status_data;

			if (subs->vehicle_status_sub.update(&vehicle_status_data))
			{
					// copy raw data into local buffer. Payload is shifted by header length to make room for header
					serialize_vehicle_status(&writer, &vehicle_status_data, &data_buffer[header_length], &length);

					if (0 < (read = transport_node->write(static_cast<char>(19), data_buffer, length))) {
						data->total_sent += read;
						tx_last_sec_read += read;
						++data->sent;
					}

			}
		}
		{
			collision_constraints_s collision_constraints_data;

			if (subs->collision_constraints_sub.update(&collision_constraints_data))
			{
					// copy raw data into local buffer. Payload is shifted by header length to make room for header
					serialize_collision_constraints(&writer, &collision_constraints_data, &data_buffer[header_length], &length);

					if (0 < (read = transport_node->write(static_cast<char>(22), data_buffer, length))) {
						data->total_sent += read;
						tx_last_sec_read += read;
						++data->sent;
					}

			}
		}
		{
			timesync_status_s timesync_status_data;

			if (subs->timesync_status_sub.update(&timesync_status_data))
			{
					// copy raw data into local buffer. Payload is shifted by header length to make room for header
					serialize_timesync_status(&writer, &timesync_status_data, &data_buffer[header_length], &length);

					if (0 < (read = transport_node->write(static_cast<char>(26), data_buffer, length))) {
						data->total_sent += read;
						tx_last_sec_read += read;
						++data->sent;
					}

			}
		}
		{
			sensor_combined_s sensor_combined_data;

			if (subs->sensor_combined_sub.update(&sensor_combined_data))
			{
					// copy raw data into local buffer. Payload is shifted by header length to make room for header
					serialize_sensor_combined(&writer, &sensor_combined_data, &data_buffer[header_length], &length);

					if (0 < (read = transport_node->write(static_cast<char>(27), data_buffer, length))) {
						data->total_sent += read;
						tx_last_sec_read += read;
						++data->sent;
					}

			}
		}
		{
			vehicle_trajectory_waypoint_s vehicle_trajectory_waypoint_desired_data;

			if (subs->vehicle_trajectory_waypoint_desired_sub.update(&vehicle_trajectory_waypoint_desired_data))
			{
					// copy raw data into local buffer. Payload is shifted by header length to make room for header
					serialize_vehicle_trajectory_waypoint(&writer, &vehicle_trajectory_waypoint_desired_data, &data_buffer[header_length], &length);

					if (0 < (read = transport_node->write(static_cast<char>(21), data_buffer, length))) {
						data->total_sent += read;
						tx_last_sec_read += read;
						++data->sent;
					}

			}
		}

		if (hrt_absolute_time() - last_stats_update >= 1_s) {
			data->sent_last_sec = tx_last_sec_read;
			if (data->datarate > 0) {
				bandwidth_mult = static_cast<float>(data->datarate) / static_cast<float>(tx_last_sec_read);
				// Apply a low-pass filter to determine the new TX interval
				tx_interval += 0.5f * (tx_interval / bandwidth_mult - tx_interval);
				// Clamp the interval between 1 and 1000 ms
				tx_interval = math::constrain(tx_interval, MIN_TX_INTERVAL_US, MAX_TX_INTERVAL_US);
			}
			tx_last_sec_read = 0;
			last_stats_update = hrt_absolute_time();
		}

		px4_usleep(tx_interval);

		++data->sent_loop;
	}

	delete(data);
	delete(subs);

	return nullptr;
}

static int launch_send_thread(pthread_t &sender_thread, struct SendThreadArgs &args)
{
	pthread_attr_t sender_thread_attr;
	pthread_attr_init(&sender_thread_attr);
	pthread_attr_setstacksize(&sender_thread_attr, PX4_STACK_ADJUSTED(2250));
	struct sched_param param;
	(void)pthread_attr_getschedparam(&sender_thread_attr, &param);
	param.sched_priority = SCHED_PRIORITY_DEFAULT;
	(void)pthread_attr_setschedparam(&sender_thread_attr, &param);
	int rc = pthread_create(&sender_thread, &sender_thread_attr, &send, (void *)&args);
	if (rc != 0) {
		errno = rc;
		PX4_ERR("Could not create send thread (%d)", errno);
		return -1;
	}
	rc = pthread_setname_np(sender_thread, "urtpsclient_snd");
	if (pthread_setname_np(sender_thread, "urtpsclient_snd")) {
		errno = rc;
		PX4_ERR("Could not set pthread name for the send thread (%d)", errno);
	}
	pthread_attr_destroy(&sender_thread_attr);

	return 0;
}

void micrortps_start_topics(const uint32_t &datarate, struct timespec &begin, uint64_t &total_rcvd,
			    uint64_t &total_sent, uint64_t &sent_last_sec,
			    uint64_t &rcvd_last_sec, uint64_t &received, uint64_t &sent, int &rcvd_loop, int &sent_loop)
{
	px4_clock_gettime(CLOCK_REALTIME, &begin);
	_should_exit_task = false;

	char data_buffer[BUFFER_SIZE]{};
	int read{0};
	uint8_t topic_ID{255};

	uint64_t rx_last_sec_read{0};
	hrt_abstime last_stats_update{0};

	RcvTopicsPubs *pubs = new RcvTopicsPubs();

	// Set the main task name to 'urtpsclient_rcv' in case there is
	// data to receive
	px4_prctl(PR_SET_NAME, "urtpsclient_rcv", px4_getpid());

	// ucdrBuffer to deserialize using the user defined buffer
	ucdrBuffer reader;
	ucdr_init_buffer(&reader, reinterpret_cast<uint8_t *>(data_buffer), BUFFER_SIZE);

	// var struct to be updated on the thread
	SendThreadArgs *sender_thread_args = new SendThreadArgs(datarate, total_sent, sent_last_sec, sent, sent_loop);

	// create a thread for sending data
	pthread_t sender_thread;
	launch_send_thread(sender_thread, (*sender_thread_args));

	while (!_should_exit_task) {
		while (0 < (read = transport_node->read(&topic_ID, data_buffer, BUFFER_SIZE))) {
			total_rcvd += read;
			rx_last_sec_read += read;

			uint64_t read_time = hrt_absolute_time();

			switch (topic_ID) {
			case 1: {
				debug_array_s debug_array_data;
				deserialize_debug_array(&reader, &debug_array_data, data_buffer);

				if (debug_array_data.timestamp > read_time) {
					// don't allow timestamps from the future
					debug_array_data.timestamp = read_time;
				}

				pubs->debug_array_pub.publish(debug_array_data);
				++received;
			}
			break;
			case 2: {
				debug_key_value_s debug_key_value_data;
				deserialize_debug_key_value(&reader, &debug_key_value_data, data_buffer);

				if (debug_key_value_data.timestamp > read_time) {
					// don't allow timestamps from the future
					debug_key_value_data.timestamp = read_time;
				}

				pubs->debug_key_value_pub.publish(debug_key_value_data);
				++received;
			}
			break;
			case 3: {
				debug_value_s debug_value_data;
				deserialize_debug_value(&reader, &debug_value_data, data_buffer);

				if (debug_value_data.timestamp > read_time) {
					// don't allow timestamps from the future
					debug_value_data.timestamp = read_time;
				}

				pubs->debug_value_pub.publish(debug_value_data);
				++received;
			}
			break;
			case 4: {
				debug_vect_s debug_vect_data;
				deserialize_debug_vect(&reader, &debug_vect_data, data_buffer);

				if (debug_vect_data.timestamp > read_time) {
					// don't allow timestamps from the future
					debug_vect_data.timestamp = read_time;
				}

				pubs->debug_vect_pub.publish(debug_vect_data);
				++received;
			}
			break;
			case 5: {
				offboard_control_mode_s offboard_control_mode_data;
				deserialize_offboard_control_mode(&reader, &offboard_control_mode_data, data_buffer);

				if (offboard_control_mode_data.timestamp > read_time) {
					// don't allow timestamps from the future
					offboard_control_mode_data.timestamp = read_time;
				}

				pubs->offboard_control_mode_pub.publish(offboard_control_mode_data);
				++received;
			}
			break;
			case 6: {
				optical_flow_s optical_flow_data;
				deserialize_optical_flow(&reader, &optical_flow_data, data_buffer);

				if (optical_flow_data.timestamp > read_time) {
					// don't allow timestamps from the future
					optical_flow_data.timestamp = read_time;
				}

				pubs->optical_flow_pub.publish(optical_flow_data);
				++received;
			}
			break;
			case 7: {
				position_setpoint_s position_setpoint_data;
				deserialize_position_setpoint(&reader, &position_setpoint_data, data_buffer);

				if (position_setpoint_data.timestamp > read_time) {
					// don't allow timestamps from the future
					position_setpoint_data.timestamp = read_time;
				}

				pubs->position_setpoint_pub.publish(position_setpoint_data);
				++received;
			}
			break;
			case 8: {
				position_setpoint_triplet_s position_setpoint_triplet_data;
				deserialize_position_setpoint_triplet(&reader, &position_setpoint_triplet_data, data_buffer);

				if (position_setpoint_triplet_data.timestamp > read_time) {
					// don't allow timestamps from the future
					position_setpoint_triplet_data.timestamp = read_time;
				}

				pubs->position_setpoint_triplet_pub.publish(position_setpoint_triplet_data);
				++received;
			}
			break;
			case 9: {
				telemetry_status_s telemetry_status_data;
				deserialize_telemetry_status(&reader, &telemetry_status_data, data_buffer);

				if (telemetry_status_data.timestamp > read_time) {
					// don't allow timestamps from the future
					telemetry_status_data.timestamp = read_time;
				}

				pubs->telemetry_status_pub.publish(telemetry_status_data);
				++received;
			}
			break;
			case 10: {
				timesync_s timesync_data;
				deserialize_timesync(&reader, &timesync_data, data_buffer);

				if (timesync_data.timestamp > read_time) {
					// don't allow timestamps from the future
					timesync_data.timestamp = read_time;
				}

				pubs->timesync_pub.publish(timesync_data);
				++received;
			}
			break;
			case 12: {
				vehicle_command_s vehicle_command_data;
				deserialize_vehicle_command(&reader, &vehicle_command_data, data_buffer);

				if (vehicle_command_data.timestamp > read_time) {
					// don't allow timestamps from the future
					vehicle_command_data.timestamp = read_time;
				}

				pubs->vehicle_command_pub.publish(vehicle_command_data);
				++received;
			}
			break;
			case 14: {
				vehicle_local_position_setpoint_s vehicle_local_position_setpoint_data;
				deserialize_vehicle_local_position_setpoint(&reader, &vehicle_local_position_setpoint_data, data_buffer);

				if (vehicle_local_position_setpoint_data.timestamp > read_time) {
					// don't allow timestamps from the future
					vehicle_local_position_setpoint_data.timestamp = read_time;
				}

				pubs->vehicle_local_position_setpoint_pub.publish(vehicle_local_position_setpoint_data);
				++received;
			}
			break;
			case 20: {
				vehicle_trajectory_waypoint_s vehicle_trajectory_waypoint_data;
				deserialize_vehicle_trajectory_waypoint(&reader, &vehicle_trajectory_waypoint_data, data_buffer);

				if (vehicle_trajectory_waypoint_data.timestamp > read_time) {
					// don't allow timestamps from the future
					vehicle_trajectory_waypoint_data.timestamp = read_time;
				}

				pubs->vehicle_trajectory_waypoint_pub.publish(vehicle_trajectory_waypoint_data);
				++received;
			}
			break;
			case 23: {
				onboard_computer_status_s onboard_computer_status_data;
				deserialize_onboard_computer_status(&reader, &onboard_computer_status_data, data_buffer);

				if (onboard_computer_status_data.timestamp > read_time) {
					// don't allow timestamps from the future
					onboard_computer_status_data.timestamp = read_time;
				}

				pubs->onboard_computer_status_pub.publish(onboard_computer_status_data);
				++received;
			}
			break;
			case 24: {
				trajectory_bezier_s trajectory_bezier_data;
				deserialize_trajectory_bezier(&reader, &trajectory_bezier_data, data_buffer);

				if (trajectory_bezier_data.timestamp > read_time) {
					// don't allow timestamps from the future
					trajectory_bezier_data.timestamp = read_time;
				}

				pubs->trajectory_bezier_pub.publish(trajectory_bezier_data);
				++received;
			}
			break;
			case 25: {
				vehicle_trajectory_bezier_s vehicle_trajectory_bezier_data;
				deserialize_vehicle_trajectory_bezier(&reader, &vehicle_trajectory_bezier_data, data_buffer);

				if (vehicle_trajectory_bezier_data.timestamp > read_time) {
					// don't allow timestamps from the future
					vehicle_trajectory_bezier_data.timestamp = read_time;
				}

				pubs->vehicle_trajectory_bezier_pub.publish(vehicle_trajectory_bezier_data);
				++received;
			}
			break;
			case 15: {
				vehicle_local_position_setpoint_s trajectory_setpoint_data;
				deserialize_vehicle_local_position_setpoint(&reader, &trajectory_setpoint_data, data_buffer);

				if (trajectory_setpoint_data.timestamp > read_time) {
					// don't allow timestamps from the future
					trajectory_setpoint_data.timestamp = read_time;
				}

				pubs->trajectory_setpoint_pub.publish(trajectory_setpoint_data);
				++received;
			}
			break;
			case 17: {
				vehicle_odometry_s vehicle_mocap_odometry_data;
				deserialize_vehicle_odometry(&reader, &vehicle_mocap_odometry_data, data_buffer);

				if (vehicle_mocap_odometry_data.timestamp > read_time) {
					// don't allow timestamps from the future
					vehicle_mocap_odometry_data.timestamp = read_time;
				}

				pubs->vehicle_mocap_odometry_pub.publish(vehicle_mocap_odometry_data);
				++received;
			}
			break;
			case 18: {
				vehicle_odometry_s vehicle_visual_odometry_data;
				deserialize_vehicle_odometry(&reader, &vehicle_visual_odometry_data, data_buffer);

				if (vehicle_visual_odometry_data.timestamp > read_time) {
					// don't allow timestamps from the future
					vehicle_visual_odometry_data.timestamp = read_time;
				}

				pubs->vehicle_visual_odometry_pub.publish(vehicle_visual_odometry_data);
				++received;
			}
			break;
			default:
				PX4_WARN("Unexpected topic ID '%hhu' to getMsg. Please make sure the client is capable of parsing the message associated to the topic ID '%hhu'",
					 topic_ID, topic_ID);
				break;
			}
		}

		if (hrt_absolute_time() - last_stats_update >= 1_s) {
			rcvd_last_sec = rx_last_sec_read;
			rx_last_sec_read = 0;
			last_stats_update = hrt_absolute_time();
		}

		// loop forever if informed loop number is negative
		if (_options.loops >= 0 && rcvd_loop >= _options.loops) { break; }

		px4_usleep(_options.sleep_us);
		++rcvd_loop;
	}

	_should_exit_task = true;
	pthread_join(sender_thread, nullptr);
	delete(pubs);
}
