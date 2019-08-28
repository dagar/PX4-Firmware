/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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

#pragma once

#include "Device.hpp"

#include <containers/BlockingList.hpp>
#include <px4_atomic.h>
#include <px4_time.h>

namespace device
{

class DriverInterface;
extern BlockingList<DriverInterface *> _px4_drivers_list;

class DriverInterface : public ListNode<DriverInterface *>
{
public:

	virtual ~DriverInterface()
	{
		Deinit();
	};

	// no copy, assignment, move, move assignment
	DriverInterface(const DriverInterface &) = delete;
	DriverInterface &operator=(const DriverInterface &) = delete;
	DriverInterface(DriverInterface &&) = delete;
	DriverInterface &operator=(DriverInterface &&) = delete;

	bool Init()
	{
		// TODO: check for and prevent duplicate instances

		if (true /* no dupes */) {
			// automatically add driver to list
			_px4_drivers_list.add(&_interface);
		}
	}

	bool Deinit()
	{
		_px4_drivers_list.remove(&_interface);
	}

	virtual bool Start() = 0;
	virtual bool Stop() = 0;
	virtual bool Reset() { return true; }
	virtual bool Probe() { return true; }
	virtual bool Test() { return true; }

	virtual void PrintStatus() {}

	const char *name() const { return _name; }

protected:

	explicit DriverInterface(Device interface) : _name(name)
	{


		// TODO: Device dedup
	};

	px4::atomic_bool _driver_should_stop{false};

protected:

	Device	_interface;

};


DriverInterface *get_driver_instance(const char *name);
bool driver_running(const char *name);
int driver_stop(const char *name);
int driver_status(const char *name);
void driver_exit_and_cleanup(const char *name);
int driver_wait_until_running(const char *name);

void drivers_status_all();
void drivers_stop_all();

} // namespace device
