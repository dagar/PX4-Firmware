/****************************************************************************
 *
 * Copyright (C) 2019 PX4 Development Team. All rights reserved.
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
 * @file driver.cpp
 * Implementation of the API declared in px4_driver.h.
 */

#ifndef MODULE_NAME
#define MODULE_NAME "drivers"
#endif

#include "DriverInterface.hpp"

#include <px4_defines.h>
#include <px4_log.h>

namespace device
{

pthread_mutex_t _px4_drivers_mutex = PTHREAD_MUTEX_INITIALIZER;
BlockingList<DriverInterface *> _px4_drivers_list;

static void lock_drivers() { pthread_mutex_lock(&_px4_drivers_mutex); }
static void unlock_drivers() { pthread_mutex_unlock(&_px4_drivers_mutex); }

DriverInterface *get_driver_instance(const char *name)
{
	auto lg = _px4_drivers_list.getLockGuard();

	// search list
	for (DriverInterface *driver : _px4_drivers_list) {
		const bool name_match = (strcmp(driver->name(), name) == 0);

		if (name_match) {
			return driver;
		}
	}

	return nullptr;
}

bool driver_running(const char *name)
{
	// search list
	DriverInterface *driver = get_driver_instance(name);

	if (driver != nullptr) {
		return true;
	}

	return false;
}

/**
 * @brief Waits until object is initialized, (from the new thread). This can be called from task_spawn().
 * @return Returns 0 iff successful, -1 on timeout or otherwise.
 */
int driver_wait_until_running(const char *name)
{
	int i = 0;

	DriverInterface *object = nullptr;

	do {
		object = get_driver_instance(name);

		// Wait up to 1 s
		px4_usleep(2500);

	} while (!object && ++i < 400);

	if (i == 400) {
		PX4_ERR("Timed out while waiting for thread to start");
		return -1;
	}

	return 0;
}

int driver_stop(const char *name)
{
	int ret = 0;
	lock_drivers();

	if (driver_running(name)) {

		DriverInterface *object = nullptr;
		unsigned int i = 0;

		do {
			// search for driver again to request stop
			object = get_driver_instance(name);

			if (object != nullptr) {
				object->Stop();

				unlock_drivers();
				px4_usleep(20000); // 20 ms
				lock_drivers();

				// search for driver again to check status
				object = get_driver_instance(name);

				if (++i > 100 && (object != nullptr)) { // wait at most 2 sec

					// driver didn't stop, remove from list then delete
					_px4_drivers_list.remove(object);

					delete object;
					object = nullptr;

					ret = -1;
					break;
				}
			}
		} while (object != nullptr);
	}

	unlock_drivers();
	return ret;
}

void driver_exit_and_cleanup(const char *name)
{
	// Take the lock here:
	// - deleting the object must take place inside the lock.
	lock_drivers();

	DriverInterface *object = get_driver_instance(name);

	if (object) {
		_px4_drivers_list.remove(object);
		delete object;
	}

	unlock_drivers();
}

int driver_status(const char *name)
{
	int ret = -1;

	lock_drivers();
	DriverInterface *object = get_driver_instance(name);

	if (driver_running(name) && object) {
		object->PrintStatus();
		ret = PX4_OK;

	} else {
		PX4_INFO("%s not running", name);
	}

	unlock_drivers();

	return ret;
}

void drivers_status_all()
{
	auto lg = _px4_drivers_list.getLockGuard();

	for (DriverInterface *driver : _px4_drivers_list) {
		PX4_INFO("Running: %s", driver->name());
	}
}

void drivers_stop_all()
{
	auto lg = _px4_drivers_list.getLockGuard();

	for (DriverInterface *driver : _px4_drivers_list) {
		PX4_INFO("Stopping: %s", driver->name());
		driver->Stop();
	}
}

} // namespace device
