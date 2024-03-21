/****************************************************************************
 *
 *   Copyright (c) 2016-2024 PX4 Development Team. All rights reserved.
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

/**
 * @file Replay.cpp
 * This module reads messages from an ULog file and publishes them.
 * It sets the parameters from the log file and handles user-defined
 * parameter overrides.
 *
 * @author Beat Kueng
*/

#include <drivers/drv_hrt.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/time.h>
#include <px4_platform_common/shutdown.h>
#include <lib/parameters/param.h>
#include <uORB/uORBMessageFields.hpp>

#include <cstring>
#include <float.h>
#include <fstream>
#include <iostream>
#include <math.h>
#include <time.h>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <string>

#include <logger/messages.h>

#include "Replay.hpp"

#define PARAMS_OVERRIDE_FILE PX4_ROOTFSDIR "/replay_params.txt"
#define DYNAMIC_PARAMS_OVERRIDE_FILE PX4_ROOTFSDIR "/replay_params_dynamic.txt"

using namespace std;
using namespace time_literals;

namespace px4
{

char *Replay::_replay_file = nullptr;

Replay::~Replay()
{
	for (size_t i = 0; i < _subscriptions.size(); ++i) {
		delete (_subscriptions[i]);
	}

	_subscriptions.clear();
}

void
Replay::setupReplayFile(const char *file_name)
{
	if (_replay_file) {
		free(_replay_file);
	}

	_replay_file = strdup(file_name);
}

void
Replay::setParameter(const string &parameter_name, const double parameter_value)
{
	param_t handle = param_find(parameter_name.c_str());
	param_type_t param_format = param_type(handle);

	if (param_format == PARAM_TYPE_INT32) {
		int32_t orig_value = 0;
		param_get(handle, &orig_value);

		int32_t value = (int32_t)parameter_value;

		if (orig_value != value) {
			PX4_WARN("Setting %s (INT32) %d -> %d", param_name(handle), orig_value, value);
		}

		param_set(handle, (const void *)&value);

	} else if (param_format == PARAM_TYPE_FLOAT) {
		float orig_value = 0;
		param_get(handle, &orig_value);

		float value = (float)parameter_value;

		if (fabsf(orig_value - value) > FLT_EPSILON) {
			PX4_WARN("Setting %s (FLOAT) %.3f -> %.3f", param_name(handle), (double)orig_value, (double)value);
		}

		param_set(handle, (const void *)&value);
	}
}

void
Replay::setUserParams(const char *filename)
{
	string line;
	string pname;
	string value_string;
	ifstream myfile(filename);

	if (!myfile.is_open()) {
		return;
	}

	PX4_INFO("Applying override params from %s...", filename);

	while (!myfile.eof()) {
		getline(myfile, line);

		if (line.empty() || line[0] == '#') {
			continue;
		}

		istringstream mystrstream(line);
		mystrstream >> pname;
		mystrstream >> value_string;

		_overridden_params.insert(pname);

		double param_value_double = stod(value_string);

		setParameter(pname, param_value_double);
	}
}

void
Replay::readDynamicParams(const char *filename)
{
	_dynamic_parameter_schedule.clear();

	string line;
	string param_name;
	string value_string;
	string time_string;
	ifstream myfile(filename);

	if (!myfile.is_open()) {
		return;
	}

	PX4_INFO("Reading dynamic params from %s...", filename);

	while (!myfile.eof()) {
		getline(myfile, line);

		if (line.empty() || line[0] == '#') {
			continue;
		}

		istringstream mystrstream(line);
		mystrstream >> param_name;
		mystrstream >> value_string;
		mystrstream >> time_string;

		_dynamic_parameters.insert(param_name);

		double param_value = stod(value_string);
		uint64_t change_timestamp = (uint64_t)(stod(time_string) * 1e6);

		// Construct and store parameter change event
		ParameterChangeEvent change_event = {change_timestamp, param_name, param_value};
		_dynamic_parameter_schedule.push_back(change_event);
	}

	// Sort by event time
	sort(_dynamic_parameter_schedule.begin(), _dynamic_parameter_schedule.end());

	_next_param_change = 0;
}

bool
Replay::readFileHeader(std::ifstream &file)
{
	file.seekg(0);
	ulog_file_header_s msg_header;
	file.read((char *)&msg_header, sizeof(msg_header));

	if (!file) {
		return false;
	}

	_file_start_time = msg_header.timestamp;
	//verify it's an ULog file
	char magic[8];
	magic[0] = 'U';
	magic[1] = 'L';
	magic[2] = 'o';
	magic[3] = 'g';
	magic[4] = 0x01;
	magic[5] = 0x12;
	magic[6] = 0x35;
	return memcmp(magic, msg_header.magic, 7) == 0;
}

bool
Replay::readFileDefinitions(std::ifstream &file)
{
	PX4_INFO("Applying params from ULog file...");

	ulog_message_header_s message_header;
	file.seekg(sizeof(ulog_file_header_s));

	while (true) {
		file.read((char *)&message_header, ULOG_MSG_HEADER_LEN);

		if (!file) {
			return false;
		}

		switch (message_header.msg_type) {
		case (int)ULogMessageType::FLAG_BITS:
			if (!readFlagBits(file, message_header.msg_size)) {
				return false;
			}

			break;

		case (int)ULogMessageType::FORMAT:
			if (!readFormat(file, message_header.msg_size)) {
				return false;
			}

			break;

		case (int)ULogMessageType::PARAMETER:
			if (!readAndApplyParameter(file, message_header.msg_size)) {
				return false;
			}

			break;

		case (int)ULogMessageType::ADD_LOGGED_MSG:
			_data_section_start = file.tellg() - (streamoff)ULOG_MSG_HEADER_LEN;
			return true;

		case (int)ULogMessageType::INFO: //skip
		case (int)ULogMessageType::INFO_MULTIPLE: //skip
		case (int)ULogMessageType::PARAMETER_DEFAULT:
			file.seekg(message_header.msg_size, ios::cur);
			break;

		default:
			PX4_ERR("unknown log definition type %i, size %i (offset %i)",
				(int)message_header.msg_type, (int)message_header.msg_size, (int)file.tellg());
			file.seekg(message_header.msg_size, ios::cur);
			break;
		}
	}

	return true;
}

bool
Replay::readFlagBits(std::ifstream &file, uint16_t msg_size)
{
	if (msg_size != 40) {
		PX4_ERR("unsupported message length for FLAG_BITS message (%i)", msg_size);
		return false;
	}

	_read_buffer.reserve(msg_size);
	uint8_t *message = (uint8_t *)_read_buffer.data();
	file.read((char *)message, msg_size);
	//uint8_t *compat_flags = message;
	uint8_t *incompat_flags = message + 8;

	// handle & validate the flags
	bool contains_appended_data = incompat_flags[0] & ULOG_INCOMPAT_FLAG0_DATA_APPENDED_MASK;
	bool has_unknown_incompat_bits = false;

	if (incompat_flags[0] & ~0x1) {
		has_unknown_incompat_bits = true;
	}

	for (int i = 1; i < 8; ++i) {
		if (incompat_flags[i]) {
			has_unknown_incompat_bits = true;
		}
	}

	if (has_unknown_incompat_bits) {
		PX4_ERR("Log contains unknown incompat bits set. Refusing to parse");
		return false;
	}

	if (contains_appended_data) {
		uint64_t appended_offsets[3];
		memcpy(appended_offsets, message + 16, sizeof(appended_offsets));

		if (appended_offsets[0] > 0) {
			// the appended data is currently only used for hardfault dumps, so it's safe to ignore it.
			PX4_INFO("Log contains appended data. Replay will ignore this data");
			_read_until_file_position = appended_offsets[0];
		}
	}

	return true;
}

bool
Replay::readFormat(std::ifstream &file, uint16_t msg_size)
{
	_read_buffer.reserve(msg_size + 1);
	char *format = (char *)_read_buffer.data();
	file.read(format, msg_size);
	format[msg_size] = 0;

	if (!file) {
		return false;
	}

	string str_format(format);
	size_t pos = str_format.find(':');

	if (pos == string::npos) {
		return false;
	}

	string name = str_format.substr(0, pos);
	string fields = str_format.substr(pos + 1);
	_file_formats[name] = fields;

	return true;
}

string Replay::getOrbFields(const orb_metadata *meta)
{
	char format[3000];
	char buffer[2048];
	uORB::MessageFormatReader format_reader(buffer, sizeof(buffer));

	if (!format_reader.readUntilFormat(meta->o_id)) {
		PX4_ERR("failed to find format for topic %s", meta->o_name);
		return "";
	}

	int field_length = 0;
	int format_length = 0;

	while (format_reader.readNextField(field_length)) {
		format_length += snprintf(format + format_length, sizeof(buffer) - format_length - 1, "%s;", buffer);
	}

	if (uORB::MessageFormatReader::expandMessageFormat(format, format_length, sizeof(format)) < 0) {
		PX4_ERR("failed to expand message format for %s", meta->o_name);
		return "";
	}

	return format;
}

bool
Replay::readAndAddSubscription(std::ifstream &file, uint16_t msg_size)
{
	_read_buffer.reserve(msg_size + 1);
	uint8_t *message = _read_buffer.data();
	streampos this_message_pos = file.tellg() - (streamoff)ULOG_MSG_HEADER_LEN;
	file.read((char *)message, msg_size);
	message[msg_size] = 0;

	if (!file) {
		return false;
	}

	if (file.tellg() <= _subscription_file_pos) { //already read this subscription
		return true;
	}

	_subscription_file_pos = file.tellg();

	uint8_t multi_id = *(uint8_t *)message;
	uint16_t msg_id = ((uint16_t)message[1]) | (((uint16_t)message[2]) << 8);
	string topic_name((char *)message + 3);
	const orb_metadata *orb_meta = findTopic(topic_name);

	if (!orb_meta) {
		PX4_WARN("Topic %s not found internally. Will ignore it", topic_name.c_str());
		return true;
	}


	// check the format: the field definitions must match
	// FIXME: this should check recursively, all used nested types
	string file_format = _file_formats[topic_name];

	const string orb_fields = getOrbFields(orb_meta);

	if (file_format != orb_fields) {
		PX4_ERR("Formats for %s don't match. Will ignore it.", topic_name.c_str());
		PX4_WARN(" Internal format:");
		size_t start = 0;

		for (size_t i = 0; i < orb_fields.length(); ++i) {
			if (orb_fields[i] == ';') {
				std::string field(orb_fields.substr(start, i - start));

				if (file_format.find(field) != std::string::npos) {
					PX4_WARN(" - %s", field.c_str());

				} else {
					PX4_ERR(" - %s", field.c_str());
				}

				start = i + 1;
			}
		}

		PX4_WARN(" File format    : %s", file_format.c_str());
		start = 0;

		for (size_t i = 0; i < file_format.length(); ++i) {
			if (file_format[i] == ';') {
				std::string field(file_format.substr(start, i - start));

				if (orb_fields.find(field) != std::string::npos) {
					PX4_WARN(" - %s", field.c_str());

				} else {
					PX4_ERR(" - %s", field.c_str());
				}

				start = i + 1;
			}
		}

		return true; // not a fatal error
	}

	Subscription *subscription = new Subscription();
	subscription->orb_meta = orb_meta;
	subscription->multi_id = multi_id;

	//find the timestamp offset
	int field_size;
	bool timestamp_found = findFieldOffset(orb_fields, "timestamp", subscription->timestamp_offset, field_size);

	if (!timestamp_found) {
		delete subscription;
		return true;
	}

	if (field_size != 8) {
		PX4_ERR("Unsupported timestamp with size %i, ignoring the topic %s", field_size, orb_meta->o_name);
		delete subscription;
		return true;
	}

	//find first data message (and the timestamp)
	streampos cur_pos = file.tellg();
	subscription->next_read_pos = this_message_pos; //this will be skipped

	if (!nextDataMessage(file, *subscription, msg_id)) {
		delete subscription;
		return false;
	}

	file.seekg(cur_pos);

	if (!subscription->orb_meta) {
		//no message found. This is not a fatal error
		delete subscription;
		return true;
	}

	PX4_DEBUG("adding subscription for %s (msg_id %i)", subscription->orb_meta->o_name, msg_id);

	//add subscription
	if (_subscriptions.size() <= msg_id) {
		_subscriptions.resize(msg_id + 1);
	}

	_subscriptions[msg_id] = subscription;

	onSubscriptionAdded(*_subscriptions[msg_id], msg_id);

	return true;
}

bool
Replay::findFieldOffset(const string &format, const string &field_name, int &offset, int &field_size)
{
	size_t prev_field_end = 0;
	size_t field_end = format.find(';');
	offset = 0;
	field_size = 0;

	while (field_end != string::npos) {
		size_t space_pos = format.find(' ', prev_field_end);

		if (space_pos != string::npos) {
			string type_name_full = format.substr(prev_field_end, space_pos - prev_field_end);
			string cur_field_name = format.substr(space_pos + 1, field_end - space_pos - 1);

			if (cur_field_name == field_name) {
				field_size = sizeOfFullType(type_name_full);
				return true;

			} else {
				offset += sizeOfFullType(type_name_full);
			}
		}

		prev_field_end = field_end + 1;
		field_end = format.find(';', prev_field_end);
	}

	return false;
}

bool
Replay::readAndHandleAdditionalMessages(std::ifstream &file, std::streampos end_position)
{
	ulog_message_header_s message_header;

	while (file.tellg() < end_position) {
		file.read((char *)&message_header, ULOG_MSG_HEADER_LEN);

		if (!file) {
			return false;
		}

		switch (message_header.msg_type) {
		case (int)ULogMessageType::PARAMETER:
			if (!readAndApplyParameter(file, message_header.msg_size)) {
				return false;
			}

			break;

		case (int)ULogMessageType::DROPOUT:
			readDropout(file, message_header.msg_size);
			break;

		default: //skip all others
			file.seekg(message_header.msg_size, ios::cur);
			break;
		}
	}

	return true;
}

bool
Replay::readAndApplyParameter(std::ifstream &file, uint16_t msg_size)
{
	_read_buffer.reserve(msg_size);
	uint8_t *message = (uint8_t *)_read_buffer.data();
	file.read((char *)message, msg_size);

	if (!file) {
		return false;
	}

	uint8_t key_len = message[0];
	string key((char *)message + 1, key_len);

	size_t pos = key.find(' ');

	if (pos == string::npos) {
		return false;
	}

	string type = key.substr(0, pos);
	string param_name = key.substr(pos + 1);

	if (_overridden_params.find(param_name) != _overridden_params.end() ||
	    _dynamic_parameters.find(param_name) != _dynamic_parameters.end()) {
		//this parameter is overridden, so don't apply it
		return true;
	}

	if (type != "int32_t" && type != "float") {
		PX4_WARN("unknown parameter type %s, name %s (ignoring it)", type.c_str(), param_name.c_str());
		return true;
	}

	param_t handle = param_find(param_name.c_str());

	if (handle != PARAM_INVALID) {
		param_set(handle, (const void *)(message + 1 + key_len));
	}

	return true;
}

bool
Replay::readDropout(std::ifstream &file, uint16_t msg_size)
{
	uint16_t duration;
	file.read((char *)&duration, sizeof(duration));

	PX4_ERR("Dropout in replayed log, %i ms", (int)duration);
	return file.good();
}

bool
Replay::nextDataMessage(std::ifstream &file, Subscription &subscription, int msg_id)
{
	ulog_message_header_s message_header;
	file.seekg(subscription.next_read_pos);
	//ignore the first message (it's data we already read)
	file.read((char *)&message_header, ULOG_MSG_HEADER_LEN);

	if (file) {
		file.seekg(message_header.msg_size, ios::cur);
	}

	uint16_t file_msg_id;
	bool done = false;

	while (file && !done) {
		streampos cur_pos = file.tellg();
		file.read((char *)&message_header, ULOG_MSG_HEADER_LEN);

		if (!file) {
			break;
		}

		if (((streamoff)cur_pos) + ULOG_MSG_HEADER_LEN + message_header.msg_size > _read_until_file_position) {
			file.setstate(std::ios::eofbit);
			break;
		}

		switch (message_header.msg_type) {
		case (int)ULogMessageType::ADD_LOGGED_MSG:
			readAndAddSubscription(file, message_header.msg_size);
			break;

		case (int)ULogMessageType::DATA:
			file.read((char *)&file_msg_id, sizeof(file_msg_id));

			if (file) {
				if (msg_id == file_msg_id) {
					if (message_header.msg_size == subscription.orb_meta->o_size_no_padding + 2) {
						subscription.next_read_pos = cur_pos;
						file.seekg(subscription.timestamp_offset, ios::cur);
						file.read((char *)&subscription.next_timestamp, sizeof(subscription.next_timestamp));
						done = true;

					} else { //sanity check failed!
						PX4_ERR("data message %s has wrong size %i (expected %i). Skipping",
							subscription.orb_meta->o_name, message_header.msg_size,
							subscription.orb_meta->o_size_no_padding + 2);
						file.seekg(message_header.msg_size - sizeof(file_msg_id), ios::cur);
					}

				} else { //not the one we are looking for
					file.seekg(message_header.msg_size - sizeof(file_msg_id), ios::cur);
				}
			}

			break;

		case (int)ULogMessageType::REMOVE_LOGGED_MSG: //skip these
		case (int)ULogMessageType::PARAMETER:
		case (int)ULogMessageType::DROPOUT:
		case (int)ULogMessageType::INFO:
		case (int)ULogMessageType::INFO_MULTIPLE:
		case (int)ULogMessageType::SYNC:
		case (int)ULogMessageType::LOGGING:
		case (int)ULogMessageType::PARAMETER_DEFAULT:
			file.seekg(message_header.msg_size, ios::cur);
			break;

		default:
			//this really should not happen
			PX4_ERR("unknown log message type %i, size %i (offset %i)",
				(int)message_header.msg_type, (int)message_header.msg_size, (int)file.tellg());
			file.seekg(message_header.msg_size, ios::cur);
			break;
		}
	}

	if (file.eof()) { //no more data messages for this subscription
		subscription.orb_meta = nullptr;
		file.clear();

	} else {
		if (subscription.orb_meta) {
			subscription.topic_name = (char *)subscription.orb_meta->o_name;
		}

	}

	return file.good();
}

const orb_metadata *
Replay::findTopic(const std::string &name)
{
	const orb_metadata *const *topics = orb_get_topics();

	for (size_t i = 0; i < orb_topics_count(); i++) {
		if (name == topics[i]->o_name) {
			return topics[i];
		}
	}

	return nullptr;
}

std::string
Replay::extractArraySize(const std::string &type_name_full, int &array_size)
{
	size_t start_pos = type_name_full.find('[');
	size_t end_pos = type_name_full.find(']');

	if (start_pos == string::npos || end_pos == string::npos) {
		array_size = 1;
		return type_name_full;
	}

	array_size = atoi(type_name_full.substr(start_pos + 1, end_pos - start_pos - 1).c_str());
	return type_name_full.substr(0, start_pos);
}

size_t
Replay::sizeOfType(const std::string &type_name)
{
	if (type_name == "int8_t" || type_name == "uint8_t") {
		return 1;

	} else if (type_name == "int16_t" || type_name == "uint16_t") {
		return 2;

	} else if (type_name == "int32_t" || type_name == "uint32_t") {
		return 4;

	} else if (type_name == "int64_t" || type_name == "uint64_t") {
		return 8;

	} else if (type_name == "float") {
		return 4;

	} else if (type_name == "double") {
		return 8;

	} else if (type_name == "char" || type_name == "bool") {
		return 1;
	}

	const orb_metadata *orb_meta = findTopic(type_name);

	if (orb_meta) {
		return orb_meta->o_size;
	}

	PX4_ERR("unknown type: %s", type_name.c_str());
	return 0;
}

size_t
Replay::sizeOfFullType(const std::string &type_name_full)
{
	int array_size;
	string type_name = extractArraySize(type_name_full, array_size);
	return sizeOfType(type_name) * array_size;
}

bool
Replay::readDefinitionsAndApplyParams(std::ifstream &file)
{
	// log reader currently assumes little endian
	int num = 1;

	if (*(char *)&num != 1) {
		PX4_ERR("Replay only works on little endian!");
		return false;
	}

	if (!file.is_open()) {
		PX4_ERR("Failed to open replay file");
		return false;
	}

	if (!readFileHeader(file)) {
		PX4_ERR("Failed to read file header. Not a valid ULog file");
		return false;
	}

	//initialize the formats and apply the parameters from the log file
	if (!readFileDefinitions(file)) {
		PX4_ERR("Failed to read ULog definitions section. Broken file?");
		return false;
	}

	setUserParams(PARAMS_OVERRIDE_FILE);
	readDynamicParams(DYNAMIC_PARAMS_OVERRIDE_FILE);
	return true;
}

void
Replay::run()
{
	uint64_t timestamp_last = 0;

	// time starts at 0
	{
		struct timespec ts {};
		abstime_to_ts(&ts, timestamp_last);
		px4_clock_settime(CLOCK_MONOTONIC, &ts);
	}

	ifstream replay_file(_replay_file, ios::in | ios::binary);

	if (!readDefinitionsAndApplyParams(replay_file)) {
		return;
	}

	_speed_factor = 1.f;
	const char *speedup = getenv("PX4_SIM_SPEED_FACTOR");

	if (speedup) {
		_speed_factor = atof(speedup);
	}

	PX4_INFO("Replay in progress...");

	ulog_message_header_s message_header;
	replay_file.seekg(_data_section_start);

	//we know the next message must be an ADD_LOGGED_MSG
	replay_file.read((char *)&message_header, ULOG_MSG_HEADER_LEN);

	if (!readAndAddSubscription(replay_file, message_header.msg_size)) {
		PX4_ERR("Failed to read subscription");
		return;
	}

	uint32_t nr_published_messages = 0;
	streampos last_additional_message_pos = _data_section_start;

	int imu_publish_count = 0;
	bool imu_published = false;

	hrt_abstime last_update = 0;

	while (!should_exit() && replay_file) {
		// Find the next message to publish. Messages from different subscriptions don't need
		// to be in chronological order, so we need to check all subscriptions
		uint64_t next_file_time = 0;
		int next_msg_id = -1;
		bool first_time = true;

		for (size_t i = 0; i < _subscriptions.size(); ++i) {
			const Subscription *subscription = _subscriptions[i];

			if (!subscription) {
				continue;
			}

			if (subscription->orb_meta && !subscription->ignored) {
				if (first_time || subscription->next_timestamp < next_file_time) {
					first_time = false;
					next_msg_id = (int)i;
					next_file_time = subscription->next_timestamp;
				}
			}
		}

		if (next_msg_id == -1) {
			break; // no active subscription anymore. We're done.
		}

		Subscription &sub = *_subscriptions[next_msg_id];

		if (next_file_time == 0 || next_file_time < _file_start_time) {
			// someone didn't set the timestamp properly. Consider the message invalid
			PX4_ERR("%s:%d missing timestamp", sub.orb_meta->o_name, sub.multi_id);
			nextDataMessage(replay_file, sub, next_msg_id);
			continue;
		}

		if (findTopic(_publisher_rule, sub.orb_meta->o_name)) {
			PX4_DEBUG("not allowing publish topic %s", sub.orb_meta->o_name);
			//return (orb_advert_t)_Instance;
		}

		// handle additional messages between last and next published data
		replay_file.seekg(last_additional_message_pos);
		streampos next_additional_message_pos = sub.next_read_pos;
		readAndHandleAdditionalMessages(replay_file, next_additional_message_pos);
		last_additional_message_pos = next_additional_message_pos;

		// Perform scheduled parameter changes
		while (_next_param_change < _dynamic_parameter_schedule.size() &&
		       _dynamic_parameter_schedule[_next_param_change].timestamp <= next_file_time) {
			const auto param_change = _dynamic_parameter_schedule[_next_param_change];
			PX4_WARN("Performing param change scheduled for t=%.3lf at t=%.3lf.",
				 (double)param_change.timestamp / 1.e6,
				 (double)next_file_time / 1.e6);
			setParameter(param_change.parameter_name, param_change.parameter_value);
			_next_param_change++;
		}

		// Perform scheduled parameter changes
		while (_next_param_change < _dynamic_parameter_schedule.size() &&
		       _dynamic_parameter_schedule[_next_param_change].timestamp <= next_file_time) {

			const auto param_change = _dynamic_parameter_schedule[_next_param_change];
			PX4_WARN("Performing param change scheduled for t=%.3lf at t=%.3lf.",
				 (double)param_change.timestamp / 1.e6,
				 (double)next_file_time / 1.e6);

			setParameter(param_change.parameter_name, param_change.parameter_value);
			_next_param_change++;
		}



		bool publish_msg = true;

		const uint64_t publish_timestamp = sub.next_timestamp;


		// TODO: ekf2 replay
		const auto whitelist = {
			"airspeed",
			"airspeed_validated",
			"landing_target_pose",
			"parameter_update",
			"sensor_airflow",
			"sensor_combined",
			"sensor_selection",
			"vehicle_air_data",
			"vehicle_command",
			"vehicle_gps_position",
			"vehicle_land_detected",
			"vehicle_magnetometer",
			"vehicle_optical_flow",
			"vehicle_status",
			"vehicle_visual_odometry",
			"distance_sensor",
			"vehicle_imu",

			"sensor_accel",
			"sensor_gyro",
		};

		if (sub.orb_meta) {

			bool topic_whitelisted = false;

			for (auto &topic : whitelist) {
				if (strcmp(sub.orb_meta->o_name, topic) == 0) {
					topic_whitelisted = true;
					break;
				}
			}

			if (!topic_whitelisted) {

				// skip
				PX4_DEBUG("%lu [%lu] (+%lu us) skipping %s:%d", hrt_absolute_time(), publish_timestamp,
					  publish_timestamp - timestamp_last,
					  sub.orb_meta->o_name, sub.multi_id);

				//continue;

				publish_msg = false;
			}
		}

		if (publish_msg) {
			// adjust the lockstep time to the publication time
			struct timespec ts {};
			abstime_to_ts(&ts, publish_timestamp);
			px4_clock_settime(CLOCK_MONOTONIC, &ts);

			if (_replay_start_time == 0) {
				_replay_start_time = hrt_absolute_time();
			}

			// It's time to publish
			readTopicDataToBuffer(sub, replay_file);

			if (publish_timestamp >= timestamp_last) {
				PX4_DEBUG("%lu [%lu] (+%lu us) publishing %s:%d", hrt_absolute_time(), publish_timestamp,
					  publish_timestamp - timestamp_last,
					  sub.orb_meta->o_name, sub.multi_id);

			} else {
				PX4_ERR("%s:%d bad timestamp %lu (last timestamp %lu)", sub.orb_meta->o_name, sub.multi_id, publish_timestamp,
					timestamp_last);
			}

			if (publishTopic(sub, _read_buffer.data())) {
				++nr_published_messages;
			}

			timestamp_last = publish_timestamp;


			// Wait for other modules, such as logger or ekf2
			if (imu_published) {
				px4_lockstep_wait_for_components();

			} else {
				if (sub.orb_meta) {
					if (strcmp(sub.orb_meta->o_name, "vehicle_imu") == 0) {
						imu_publish_count++;

						if (imu_publish_count > 1000) {
							//imu_published = true;
						}
					}
				}

				//system_usleep(1);
			}

			if (hrt_elapsed_time(&last_update) >= 3_s) {
				PX4_INFO("[%.6fs] %d messages published", timestamp_last * 1e-6, nr_published_messages);
				last_update = timestamp_last;
			}
		}

		nextDataMessage(replay_file, sub, next_msg_id);

		// TODO: output status (eg. every sec), including total duration...
	}

	for (auto &sub : _subscriptions) {
		if (!sub) {
			continue;
		}

		if (sub && (sub->publication_counter > 0 || sub->error_counter > 0)) {

			if (sub->topic_name) {
				//PX4_INFO("%s: %i, %i", sub->orb_meta->o_name, sub->publication_counter, sub->error_counter);
				PX4_INFO("%s: %i, %i", sub->topic_name, sub->publication_counter, sub->error_counter);

			} else {
				PX4_WARN("%s: %i, %i", "HUH?", sub->publication_counter, sub->error_counter);
			}
		}

		if (sub->orb_advert) {
			orb_unadvertise(sub->orb_advert);
			sub->orb_advert = nullptr;
		}
	}

	if (!should_exit()) {
		PX4_INFO("Replay done (published %u msgs, %.3lf s)", nr_published_messages,
			 (double)hrt_elapsed_time(&_replay_start_time) / 1.e6);
	}

	if (!should_exit()) {
		replay_file.close();
		px4_shutdown_request();
		// we need to ensure the shutdown logic gets updated and eventually triggers shutdown
		hrt_abstime t = hrt_absolute_time();

		for (int i = 0; i < 1000; ++i) {
			struct timespec ts;
			abstime_to_ts(&ts, t);
			px4_clock_settime(CLOCK_MONOTONIC, &ts);
			t += 10_ms;
		}
	}
}

void
Replay::readTopicDataToBuffer(const Subscription &sub, std::ifstream &replay_file)
{
	const size_t msg_read_size = sub.orb_meta->o_size_no_padding;
	const size_t msg_write_size = sub.orb_meta->o_size;
	_read_buffer.reserve(msg_write_size);
	replay_file.seekg(sub.next_read_pos + (streamoff)(ULOG_MSG_HEADER_LEN + 2)); //skip header & msg id
	replay_file.read((char *)_read_buffer.data(), msg_read_size);
}

bool
Replay::publishTopic(Subscription &sub, void *data)
{
	bool published = false;

	if (sub.orb_advert) {
		orb_publish(sub.orb_meta, sub.orb_advert, data);
		published = true;

	} else {
		if (sub.multi_id == 0) {
			sub.orb_advert = orb_advertise(sub.orb_meta, data);
			published = true;

		} else {
			// make sure the other instances are advertised already so that we get the correct instance
			bool advertised = false;

			for (const auto &subscription : _subscriptions) {
				if (!subscription) {
					continue;
				}

				if (subscription->orb_meta) {
					if (strcmp(sub.orb_meta->o_name, subscription->orb_meta->o_name) == 0 &&
					    subscription->orb_advert && subscription->multi_id == sub.multi_id - 1) {
						advertised = true;
					}
				}
			}

			if (advertised) {
				int instance;
				sub.orb_advert = orb_advertise_multi(sub.orb_meta, data, &instance);
				published = true;
			}
		}
	}

	if (published) {
		++sub.publication_counter;
	}

	return published;
}

bool Replay::startsWith(const char *pre, const char *str)
{
	size_t lenpre = strlen(pre);
	size_t lenstr = strlen(str);

	if (lenstr < lenpre) {
		return false;
	}

	return (strncmp(pre, str, lenpre) == 0);
}

bool Replay::findTopic(const PublisherRule &rule, const char *topic_name)
{
	const char **topics_ptr = rule.topics;

	while (topics_ptr && *topics_ptr) {
		if (strcmp(*topics_ptr, topic_name) == 0) {
			return true;
		}

		++topics_ptr;
	}

	return false;
}

void Replay::strTrim(const char **str)
{
	while (**str == ' ' || **str == '\t') { ++(*str); }
}

int Replay::readPublisherRulesFromFile(const char *file_name, PublisherRule &rule)
{
	static const int line_len = 1024;
	int ret = PX4_OK;
	char *line = new char[line_len];

	if (!line) {
		return -ENOMEM;
	}

	FILE *fp = fopen(file_name, "r");

	if (fp == NULL) {
		delete[](line);
		return -errno;
	}

	const char *restrict_topics_str = "restrict_topics:";

	rule.topics = nullptr;

	while (fgets(line, line_len, fp) && ret == PX4_OK) {

		if (strlen(line) < 2 || line[0] == '#') {
			continue;
		}

		if (startsWith(restrict_topics_str, line)) {
			// read topics list
			char *start = line + strlen(restrict_topics_str);
			strTrim((const char **)&start);
			char *topics = strdup(start);
			int topic_len = 0;
			int num_topics = 0;

			for (int i = 0; topics[i]; ++i) {
				if (topics[i] == ',' || topics[i] == '\n') {
					if (topic_len > 0) {
						topics[i] = 0;
						++num_topics;
					}

					topic_len = 0;

				} else {
					++topic_len;
				}
			}

			if (num_topics > 0) {
				rule.topics = new const char *[num_topics + 1];
				int topic = 0;
				strTrim((const char **)&topics);
				rule.topics[topic++] = topics;

				while (topic < num_topics) {
					if (*topics == 0) {
						++topics;
						strTrim((const char **)&topics);
						rule.topics[topic++] = topics;

					} else {
						++topics;
					}
				}

				rule.topics[num_topics] = nullptr;
			}

		} else {
			PX4_ERR("orb rules file: wrong format: %s", line);
			ret = -EINVAL;
		}
	}

	if (ret == PX4_OK && !rule.topics) {
		PX4_ERR("Wrong format in orb publisher rules file");
		ret = -EINVAL;
	}

	delete[](line);
	fclose(fp);
	return ret;
}

int
Replay::custom_command(int argc, char *argv[])
{
	if (!strcmp(argv[0], "tryapplyparams")) {
		return Replay::applyParams(true);
	}

	if (!strcmp(argv[0], "trystart")) {
		return Replay::task_spawn(argc, argv);
	}

	// ignore?

	return print_usage("unknown command");
}

int
Replay::task_spawn(int argc, char *argv[])
{
	// check if a log file was found
	if (!isSetup()) {
		if (argc > 0 && strncmp(argv[0], "try", 3) == 0) {
			return 0;
		}

		PX4_ERR("no log file given (via env variable %s)", replay::ENV_FILENAME);
		return -1;
	}

	_task_id = px4_task_spawn_cmd("replay",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_MAX - 5,
				      4000,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

int
Replay::applyParams(bool quiet)
{
	if (!isSetup()) {
		if (quiet) {
			return -1;
		}

		PX4_ERR("no log file given (via env variable %s)", replay::ENV_FILENAME);
		return -1;
	}

	int ret = 0;
	Replay *r = new Replay();

	if (r == nullptr) {
		PX4_ERR("alloc failed");
		return -ENOMEM;
	}

	ifstream replay_file(_replay_file, ios::in | ios::binary);

	if (!r->readDefinitionsAndApplyParams(replay_file)) {
		ret = -1;
	}

	delete r;

	return ret;
}

Replay *
Replay::instantiate(int argc, char *argv[])
{
	// check the replay mode
	//const char *replay_mode = getenv(replay::ENV_MODE);

	Replay *instance = new Replay();

	return instance;
}

int
Replay::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This module is used to replay ULog files.

There are 2 environment variables used for configuration: `replay_file`, which must be set to an ULog file name - it's
the log file to be replayed. The second is the mode, specified via `replay_mode`:
- `replay_mode=ekf2`: specific EKF2 replay mode. It can only be used with the ekf2 module, but allows the replay
  to run as fast as possible.
- Generic otherwise: this can be used to replay any module(s), but the replay will be done with the same speed as the
  log was recorded.

The module is typically used together with uORB publisher rules, to specify which messages should be replayed.
The replay module will just publish all messages that are found in the log. It also applies the parameters from
the log.

The replay procedure is documented on the [System-wide Replay](https://docs.px4.io/main/en/debug/system_wide_replay.html)
page.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("replay", "system");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start replay, using log file from ENV variable 'replay'");
	PRINT_MODULE_USAGE_COMMAND_DESCR("trystart", "Same as 'start', but silently exit if no log file given");
	PRINT_MODULE_USAGE_COMMAND_DESCR("tryapplyparams", "Try to apply the parameters from the log file");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

} //namespace px4
