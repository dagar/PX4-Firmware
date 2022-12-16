/****************************************************************************
 *
 *   Copyright (c) 2012-2022 PX4 Development Team. All rights reserved.
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
 * @file parameters.cpp
 *
 * Global parameter store.
 *
 * Note that it might make sense to convert this into a driver.  That would
 * offer some interesting options regarding state for e.g. ORB advertisements
 * and background parameter saving.
 */

#define PARAM_IMPLEMENTATION

#include "ParameterServer.hpp"
static ParameterServer *parameter_server {nullptr};

#include "param.h"

#include <uORB/SubscriptionBlocking.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/parameter_request.h>
#include <uORB/topics/parameter_value.h>

void param_init()
{
	if (parameter_server == nullptr) {
		px4_usleep(100'000);
		parameter_server = new ParameterServer();
	}
}

void param_notify_changes()
{
	if (parameter_server) {
		parameter_server->notifyChanges();
	}
}

param_t param_find(const char *name)
{
	if (parameter_server) {
		return parameter_server->findParameter(name, true);
	}

	return PARAM_INVALID;
}

param_t param_find_no_notification(const char *name)
{
	if (parameter_server) {
		return parameter_server->findParameter(name, false);
	}

	return PARAM_INVALID;
}

size_t param_size(param_t param)
{
	if (parameter_server) {
		return parameter_server->getParameterSize(param);
	}

	return 0;
}

unsigned param_count()
{
	if (parameter_server) {
		return parameter_server->count();
	}

	return 0;
}

unsigned param_count_used()
{
	if (parameter_server) {
		return parameter_server->countUsed();
	}

	return 0;
}

param_t param_for_index(unsigned index)
{
	if (parameter_server) {
		return parameter_server->forIndex(index);
	}

	return PARAM_INVALID;
}

param_t param_for_used_index(unsigned index)
{
	if (parameter_server) {
		return parameter_server->forUsedIndex(index);
	}

	return PARAM_INVALID;
}

int param_get_index(param_t param)
{
	if (parameter_server) {
		return parameter_server->getParameterIndex(param);
	}

	return -1;
}

int param_get_used_index(param_t param)
{
	if (parameter_server) {
		return parameter_server->getParameterUsedIndex(param);
	}

	return -1;
}

const char *param_name(param_t param)
{
	if (parameter_server) {
		return parameter_server->getParameterName(param);
	}

	return nullptr;
}

param_type_t param_type(param_t param)
{
	if (parameter_server) {
		return parameter_server->getParameterType(param);
	}

	return PARAM_TYPE_UNKNOWN;
}

bool param_is_volatile(param_t param)
{
	if (parameter_server) {
		return parameter_server->isParameterVolatile(param);
	}

	return false;
}


bool param_value_unsaved(param_t param)
{
	if (parameter_server) {
		return parameter_server->isParameterValueUnsaved(param);
	}

	return true;
}

int param_get(param_t param, void *val)
{
#if 0

	if (parameter_server) {
		return parameter_server->getParameterValue(param, val);
	}

	return -1;
#else
	uORB::Publication<parameter_request_s> param_request_pub {ORB_ID(parameter_request)};
	uORB::SubscriptionBlocking<parameter_value_s> param_value_sub{ORB_ID(parameter_value)};

	// publish request
	parameter_request_s request{};
	request.message_type = parameter_request_s::MESSAGE_TYPE_PARAM_REQUEST_READ;
	request.param_index = param;
	request.timestamp = hrt_absolute_time();
	param_request_pub.publish(request);

	// response
	while ((hrt_elapsed_time(&request.timestamp) < 50_ms) || param_value_sub.updated()) {
		const unsigned last_generation = param_value_sub.get_last_generation();
		parameter_value_s parameter_value{};

		if (param_value_sub.updateBlocking(parameter_value, 1'000)) {

			if (param_value_sub.get_last_generation() != last_generation + 1) {
				PX4_ERR("param_get: missed parameter_value, generation %d -> %d",
					last_generation, param_value_sub.get_last_generation());
			}

			if ((parameter_value.timestamp_requested == request.timestamp)
			    && (parameter_value.param_index == request.param_index)) {
				switch (parameter_value.type) {
				case parameter_request_s::TYPE_INT32: {
						int32_t v = parameter_value.int64_value;
						//fprintf(stderr, "param_get: ParameterValue: %s (%d)=%d\n", parameter_value.param_name, parameter_value.param_index, v);
						memcpy(val, &v, sizeof(int32_t));
						return 0;
					}
					break;

				case parameter_request_s::TYPE_FLOAT32: {
						float f = (float)parameter_value.float64_value;
						//fprintf(stderr, "param_get: ParameterValue: %s (%d)=%.3f\n", parameter_value.param_name, parameter_value.param_index, (double)f);
						memcpy(val, &f, sizeof(float));
						return 0;
					}
					break;

				}
			}
		}
	}

	PX4_ERR("param_get %d failed", param);
	return -1;
#endif
}

int param_get_default_value(param_t param, void *default_val)
{
	if (parameter_server) {
		parameter_server->getParameterDefaultValue(param, default_val);
	}

	return -1;
}

int param_get_system_default_value(param_t param, void *default_val)
{
	if (parameter_server) {
		parameter_server->getParameterSystemDefaultValue(param, default_val);
	}

	return -1;
}

bool param_value_is_default(param_t param)
{
	if (parameter_server) {
		parameter_server->isParameterValueDefault(param);
	}

	return true;
}

void param_control_autosave(bool enable)
{
	if (parameter_server) {
		parameter_server->controlAutosave(enable);
	}
}

int param_set(param_t param, const void *val)
{
	if (parameter_server) {
		return parameter_server->setParameter(param, val, false, true);
	}

	return -1;
}

int param_set_no_notification(param_t param, const void *val)
{
	if (parameter_server) {
		return parameter_server->setParameter(param, val, false, false);
	}

	return -1;
}

bool param_used(param_t param)
{
	if (parameter_server) {
		return parameter_server->isParameterUsed(param);
	}

	return false;
}

void param_set_used(param_t param)
{
	if (parameter_server) {
		return parameter_server->setParameterUsed(param);
	}
}

int param_set_default_value(param_t param, const void *val)
{
	if (parameter_server) {
		return parameter_server->setParameterDefaultValue(param, val);
	}

	return -1;
}

int param_reset(param_t param)
{
	if (parameter_server) {
		return parameter_server->resetParameter(param, true);
	}

	return -1;
}

int param_reset_no_notification(param_t param)
{
	if (parameter_server) {
		return parameter_server->resetParameter(param, false);
	}

	return -1;
}

void param_reset_all()
{
	if (parameter_server) {
		return parameter_server->resetAllParameters(true);
	}
}

void param_reset_excludes(const char *excludes[], int num_excludes)
{
	if (parameter_server) {
		return parameter_server->resetExcludes(excludes, num_excludes);
	}
}

void param_reset_specific(const char *resets[], int num_resets)
{
	if (parameter_server) {
		return parameter_server->resetSpecificParameter(resets, num_resets);
	}
}

int param_set_default_file(const char *filename)
{
	if (parameter_server) {
		return parameter_server->setDefaultFile(filename);
	}

	return -1;
}

const char *param_get_default_file()
{
	if (parameter_server) {
		return parameter_server->getDefaultFile();
	}

	return nullptr;
}

int param_set_backup_file(const char *filename)
{
	if (parameter_server) {
		return parameter_server->setBackupFile(filename);
	}

	return -1;
}

const char *param_get_backup_file()
{
	if (parameter_server) {
		return parameter_server->getBackupFile();
	}

	return nullptr;
}

int param_save_default()
{
	if (parameter_server) {
		parameter_server->autoSave(true);
		return 0;
	}

	return -1;
}

int param_load_default()
{
	if (parameter_server) {
		return parameter_server->loadDefault();
	}

	return -1;
}

int param_export(const char *filename, param_filter_func filter)
{
	if (parameter_server) {
		return parameter_server->exportToFile(filename, filter);
	}

	return -1;
}

int param_import(int fd)
{
	if (parameter_server) {
		return parameter_server->importFromFileDescriptor(fd);
	}

	return -1;
}

int param_load(int fd)
{
	if (parameter_server) {
		return parameter_server->loadFromFileDescriptor(fd);
	}

	return -1;
}

int param_dump(int fd)
{
	if (parameter_server) {
		return parameter_server->bsonDump(fd);
	}

	return -1;
}

void param_foreach(void (*func)(void *arg, param_t param), void *arg, bool only_changed, bool only_used)
{
	if (parameter_server) {
		parameter_server->forEachParameter(func, arg, only_changed, only_used);
	}
}

uint32_t param_hash_check()
{
	if (parameter_server) {
		parameter_server->hashCheck();
	}

	return 0;
}

void param_print_status()
{
	if (parameter_server) {
		parameter_server->printStatus();
	}
}
