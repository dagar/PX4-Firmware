/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/console_buffer.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/getopt.h>

#include <uORB/Subscription.hpp>
#include <uORB/topics/uORBTopics.hpp>

#include <stdio.h>
#include <string.h>

static void	usage();

extern "C" {
	__EXPORT int orbtopic_main(int argc, char *argv[]);
}



static int print_fields(const orb_metadata *meta, uint8_t *buffer)
{
	// constexpr char __orb_vehicle_status_fields[] = "uint64_t timestamp;uint64_t nav_state_timestamp;uint32_t onboard_control_sensors_present;uint32_t onboard_control_sensors_enabled;uint32_t onboard_control_sensors_health;uint8_t nav_state;uint8_t arming_state;uint8_t hil_state;bool failsafe;uint8_t system_type;uint8_t system_id;uint8_t component_id;uint8_t vehicle_type;bool is_vtol;bool is_vtol_tailsitter;bool vtol_fw_permanent_stab;bool in_transition_mode;bool in_transition_to_fw;bool rc_signal_lost;uint8_t rc_input_mode;bool data_link_lost;uint8_t data_link_lost_counter;bool high_latency_data_link_lost;bool engine_failure;bool mission_failure;uint8_t failure_detector_status;uint8_t latest_arming_reason;uint8_t latest_disarming_reason;uint8_t[5] _padding0;";


	const size_t fields_len = strlen(meta->o_fields);

	// find ; and get substring?
	size_t semicolons[128] {};
	size_t semicolon_count = 0;

	for (size_t char_index = 0; char_index < fields_len; char_index++) {
		if (meta->o_fields[char_index] == ';') {
			semicolons[semicolon_count] = char_index;
			semicolon_count++;
		}
	}

	if (semicolon_count > 0)  {
		PX4_INFO_RAW("TOPIC: %s\n", meta->o_name);

		size_t buffer_offset = 0;

		size_t previous_semicolon = 0;

		for (size_t substring = 0; substring <= semicolon_count; substring++) {

			if (semicolons[substring] == 0) {
				break;
			}

			const char *str_start = meta->o_fields + previous_semicolon + ((substring == 0) ? 0 : 1);
			size_t str_len = semicolons[substring] - previous_semicolon - ((substring == 0) ? 0 : 1);
			char str[80] {};
			strncpy(str, str_start, str_len);

			// now split on space and find the type
			char field_type_str[20] {};
			char field_name_str[60] {};

			for (size_t char_index = 0; char_index < str_len; char_index++) {
				if (str[char_index] == ' ') {
					strncpy(field_type_str, str, char_index);
					strncpy(field_name_str, str + char_index + 1, str_len - char_index - 1);

					size_t field_size = 0;

					if (strcmp(field_type_str, "bool") == 0) {
						field_size = 1;
						bool field = 0;
						memcpy(&field, &buffer[buffer_offset], field_size);
						printf("\t%s: %s\n", field_name_str, field ? "True" : "False");

					} else if (strcmp(field_type_str, "uint8_t") == 0) {
						field_size = 1;
						uint8_t field = 0;
						memcpy(&field, &buffer[buffer_offset], field_size);
						printf("\t%s: %lu\n", field_name_str, field);

					} else if (strcmp(field_type_str, "uint16_t") == 0) {
						field_size = 2;
						uint16_t field = 0;
						memcpy(&field, &buffer[buffer_offset], field_size);
						printf("\t%s: %lu\n", field_name_str, field);

					} else if (strcmp(field_type_str, "uint32_t") == 0) {
						field_size = 4;
						uint32_t field = 0;
						memcpy(&field, &buffer[buffer_offset], field_size);
						printf("\t%s: %lu\n", field_name_str, field);

					} else if (strcmp(field_type_str, "uint64_t") == 0) {
						field_size = 8;
						uint64_t field = 0;
						memcpy(&field, &buffer[buffer_offset], field_size);
						printf("\t%s: %llu\n", field_name_str, field);

					} else {

						printf("The substring is: \"%s\" - [%d, %d] length: %d\n", str, previous_semicolon, semicolons[substring], str_len);
					}

					//printf("%s %s: FIELD_VALUE %d %d\n", field_type_str, field_name_str, buffer_offset, field_size);

					buffer_offset += field_size;


				}
			}


			previous_semicolon = semicolons[substring];
		}
	}

	return 0;
}

static int orb_echo(const char *topic_name)
{
	PX4_INFO("echo topic: %s", topic_name);


	const orb_metadata *const *topics = orb_get_topics();

	for (size_t i = 0; i < orb_topics_count(); i++) {
		if (strcmp(topic_name, topics[i]->o_name) == 0) {
			PX4_INFO("found in orb topics: %s", topics[i]->o_name);

			if (orb_exists(topics[i], 0) == PX4_OK) {
				// copy latest
				uORB::Subscription sub{topics[i]};
				uint8_t buffer[topics[i]->o_size] {};
				sub.copy(buffer);

				print_fields(topics[i], buffer);
			}


		}
	}

	return 0;
}

int orbtopic_main(int argc, char *argv[])
{
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "f", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'f':
			//follow = true;
			break;

		default:
			usage();
			return -1;
			break;
		}
	}

	if (myoptind >= argc) {
		usage();
		return 1;
	}

	// bw <topic-name>
	//  Display the bandwidth used by a topic.

	// echo <topic-name>
	//  Display messages published to a topic.
	//   -n COUNT

	// hz <topic-name>
	//  Display the publishing rate of a topic. The rate reported is by default the average rate over the entire time rostopic has been running.

	// info <topic-name>

	// list
	//  Display a list of current topics.

	// top?



	// uorb_topics_list
	//   orb_get_topics() - array of orb_metadata
	//   get_orb_meta(ORB_ID id)

	if (argc >= 2) {
		if (!strcmp(argv[1], "echo")) {
			if (argc >= 3) {
				return orb_echo(argv[2]);
			}
		}
	}

	return 0;
}

static void usage()
{

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("orbtopic", "system");
	PRINT_MODULE_USAGE_PARAM_FLAG('f', "Follow: wait for new messages", true);

}
