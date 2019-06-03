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

#include "ScheduledWorkItem.hpp"

#include <drivers/drv_hrt.h>
#include <uORB/uORB.h>
#include <uORB/Subscription.hpp>

using namespace time_literals;

namespace px4
{

template <typename T>
class ORBActorWorkItem : public ScheduledWorkItem
{
public:

	ORBActorWorkItem(const wq_config_t &config, const orb_metadata *meta, uint8_t instance = 0) :
		ScheduledWorkItem(config),
		_meta(meta),
		_instance(instance),
		_subscription{meta, instance}
	{
		// try to register
		if (!RegisterORBCallBack()) {
			// otherwise, try again in 1 second
			ScheduleDelayed(1_s);
		}
	}

	virtual ~ORBActorWorkItem()
	{
		ScheduleClear();
		orb_unregister_work_callback(this, _meta, _instance);
	}

	bool ORBTopicChange(const orb_metadata *meta, uint8_t instance = 0)
	{
		if (orb_exists(meta, instance) == PX4_OK) {
			// first unregister existing
			orb_unregister_work_callback(this, _meta, _instance);

			// set new
			_meta = meta;
			_instance = instance;

			// register new
			return RegisterORBCallBack();
		}

		return false;
	}

	bool RegisterORBCallBack()
	{
		// try to register again
		int ret = orb_register_work_callback(this, _meta, _instance);

		if (ret == PX4_OK) {
			_registered = true;

		} else {
			_registered = false;
		}

		return _registered;
	}

	void Run() override
	{
		T data;
		bool updated = _subscription.update(&data);

		if (_registered && updated) {
			OnUpdate(data);

		} else {
			// try to register again
			if (!RegisterORBCallBack()) {
				// otherwise, try again in 1 second
				ScheduleDelayed(1_s);
			}
		}
	}

	virtual void OnUpdate(const T &data) = 0;

	virtual void OnTimeout() {}


	bool set_timeout(uint32_t timeout_us)
	{
		ScheduleClear();
		ScheduleDelayed(timeout_us);

		return true;
	}

	bool registered() const { return _registered; }

private:

	const orb_metadata	*_meta{nullptr};

	int			_instance{0};

	uORB::Subscription	_subscription;

	bool			_registered{false};

};

} // namespace px4
