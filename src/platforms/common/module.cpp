/****************************************************************************
 *
 * Copyright (C) 2017 PX4 Development Team. All rights reserved.
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
 * @file module.cpp
 * Implementation of the API declared in px4_module.h.
 */

#ifndef MODULE_NAME
#define MODULE_NAME "module"
#endif

#include <px4_module.h>
#include <px4_defines.h>
#include <px4_log.h>

pthread_mutex_t px4_modules_mutex = PTHREAD_MUTEX_INITIALIZER;

BlockingList<ModuleBaseInterface *> _px4_modules_list;

ModuleBaseInterface *get_module_instance(const char *name, int8_t instance)
{
	auto lg = _px4_modules_list.getLockGuard();

	// search list
	for (ModuleBaseInterface *module : _px4_modules_list) {
		const bool name_match = (strcmp(module->name(), name) == 0);
		const bool instance_match = (module->instance_id() == instance);

		if (name_match && instance_match) {
			return module;
		}
	}

	return nullptr;
}

bool module_running(const char *name, int8_t instance)
{
	// search list
	ModuleBaseInterface *module = get_module_instance(name, instance);

	if (module != nullptr) {
		return module->running();
	}

	return false;
}

/**
 * @brief Waits until object is initialized, (from the new thread). This can be called from task_spawn().
 * @return Returns 0 iff successful, -1 on timeout or otherwise.
 */
int module_wait_until_running(const char *name, int8_t instance)
{
	int i = 0;

	ModuleBaseInterface *object = nullptr;

	do {
		object = get_module_instance(name, instance);

		// Wait up to 1 s
		px4_usleep(2500);

	} while (!object && ++i < 400);

	if (i == 400) {
		PX4_ERR("Timed out while waiting for thread to start");
		return -1;
	}

	return 0;
}

int module_stop(const char *name, int8_t instance)
{
	int ret = 0;
	ModuleBaseInterface::lock_module();

	if (module_running(name, instance)) {

		ModuleBaseInterface *object = nullptr;
		unsigned int i = 0;

		do {
			// search for module again to request stop
			object = get_module_instance(name, instance);

			if (object != nullptr) {
				object->request_stop();

				ModuleBaseInterface::unlock_module();
				px4_usleep(20000); // 20 ms
				ModuleBaseInterface::lock_module();

				// search for module again to check status
				object = get_module_instance(name, instance);

				if (++i > 100 && (object != nullptr)) { // wait at most 2 sec

					// module didn't stop, remove from list then delete
					_px4_modules_list.remove(object);

					if (object->task_id() != task_id_is_work_queue) {
						// delete task
						px4_task_delete(object->task_id());
					}

					delete object;
					object = nullptr;

					ret = -1;
					break;
				}
			}
		} while (object != nullptr);
	}

	ModuleBaseInterface::unlock_module();
	return ret;
}

void module_exit_and_cleanup(const char *name, int8_t instance)
{
	// Take the lock here:
	// - if startup fails and we're faster than the parent thread, it will set
	//   _task_id and subsequently it will look like the task is running.
	// - deleting the object must take place inside the lock.
	ModuleBaseInterface::lock_module();

	ModuleBaseInterface *object = get_module_instance(name, instance);

	if (object) {
		_px4_modules_list.remove(object);
		delete object;
	}

	//_task_id = -1; // Signal a potentially waiting thread for the module to exit that it can continue.
	ModuleBaseInterface::unlock_module();
}

int module_status(const char *name, int8_t instance)
{
	int ret = -1;

	ModuleBaseInterface::lock_module();
	ModuleBaseInterface *object = get_module_instance(name, instance);

	if (module_running(name, instance) && object) {
		ret = object->print_status();

	} else {
		PX4_INFO("%s not running", name);
	}

	ModuleBaseInterface::unlock_module();

	return ret;
}

void modules_status_all()
{
	auto lg = _px4_modules_list.getLockGuard();

	// search list
	for (ModuleBaseInterface *module : _px4_modules_list) {
		if (module->instance_id() >= 0) {
			PX4_INFO("Running: %s: %d", module->name(), module->instance_id());

		} else {
			PX4_INFO("Running: %s", module->name());
		}

	}
}

void modules_stop_all()
{
	auto lg = _px4_modules_list.getLockGuard();

	// search list
	for (ModuleBaseInterface *module : _px4_modules_list) {
		PX4_INFO("Stopping: %s %d", module->name(), module->instance_id());
		module->request_stop();
	}
}

#ifndef __PX4_NUTTX

void PRINT_MODULE_DESCRIPTION(const char *description)
{
	// TODO: the output could be improved by:
	// - mark titles in bold (lines starting with ##)
	// - highlight commands (lines starting with $, or `cmd`)
	PX4_INFO_RAW("%s\n\n", description);
}

#endif /* __PX4_NUTTX */

void PRINT_MODULE_USAGE_NAME(const char *executable_name, const char *category)
{
	PX4_INFO_RAW("Usage: %s <command> [arguments...]\n", executable_name);
	PX4_INFO_RAW(" Commands:\n");
}

void PRINT_MODULE_USAGE_SUBCATEGORY(const char *subcategory)
{
	(void)subcategory;
}

void PRINT_MODULE_USAGE_NAME_SIMPLE(const char *executable_name, const char *category)
{
	PX4_INFO_RAW("Usage: %s [arguments...]\n", executable_name);
}

void PRINT_MODULE_USAGE_COMMAND_DESCR(const char *name, const char *description)
{
	if (description) {
		PX4_INFO_RAW("\n   %-13s %s\n", name, description);

	} else {
		PX4_INFO_RAW("\n   %s\n", name);
	}
}

void PRINT_MODULE_USAGE_PARAM_COMMENT(const char *comment)
{
	PX4_INFO_RAW("\n %s\n", comment);
}

void PRINT_MODULE_USAGE_PARAM_INT(char option_char, int default_val, int min_val, int max_val,
				  const char *description, bool is_optional)
{
	if (is_optional) {
		PX4_INFO_RAW("     [-%c <val>]  %s\n", option_char, description);

		if (default_val != -1) {
			PX4_INFO_RAW("                 default: %i\n", default_val);
		}

	} else {
		PX4_INFO_RAW("     -%c <val>    %s\n", option_char, description);
	}
}

void PRINT_MODULE_USAGE_PARAM_FLOAT(char option_char, float default_val, float min_val, float max_val,
				    const char *description, bool is_optional)
{
	if (is_optional) {
		PX4_INFO_RAW("     [-%c <val>]  %s\n", option_char, description);

		if (PX4_ISFINITE(default_val)) {
			PX4_INFO_RAW("                 default: %.1f\n", (double)default_val);
		}

	} else {
		PX4_INFO_RAW("     -%c <val>    %s\n", option_char, description);
	}
}

void PRINT_MODULE_USAGE_PARAM_FLAG(char option_char, const char *description, bool is_optional)
{
	if (is_optional) {
		PX4_INFO_RAW("     [-%c]        %s\n", option_char, description);

	} else {
		PX4_INFO_RAW("     -%c          %s\n", option_char, description);
	}
}

void PRINT_MODULE_USAGE_PARAM_STRING(char option_char, const char *default_val, const char *values,
				     const char *description, bool is_optional)
{
	if (is_optional) {
		PX4_INFO_RAW("     [-%c <val>]  %s\n", option_char, description);

	} else {
		PX4_INFO_RAW("     -%c <val>    %s\n", option_char, description);
	}

	if (values) {
		if (default_val) {
			PX4_INFO_RAW("                 values: %s, default: %s\n", values, default_val);

		} else {
			PX4_INFO_RAW("                 values: %s\n", values);
		}

	} else {
		if (default_val) {
			PX4_INFO_RAW("                 default: %s\n", default_val);
		}
	}
}


void PRINT_MODULE_USAGE_ARG(const char *values, const char *description, bool is_optional)
{
	if (is_optional) {
		PX4_INFO_RAW("     [%-9s] %s\n", values, description);

	} else {
		PX4_INFO_RAW("     %-11s %s\n", values, description);
	}
}

