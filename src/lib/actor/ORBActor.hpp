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

#include <px4_work_queue/WorkItem.hpp>

#include <uORB/SubscriptionCallback.hpp>

namespace px4
{

template<typename T>
class ORBActor : public WorkItem
{
public:

	ORBActor(const wq_config_t &config, const orb_metadata *meta, uint8_t instance = 0) :
		WorkItem(config),
		_subscription{this, meta, instance}
	{
	}

	virtual ~ORBActor()
	{
		Unregister();
	}

	bool Register()
	{
		return _subscription.register_work_item();
	}

	bool Unregister()
	{
		return _subscription.unregister_work_item();
	}

	void Run() override final
	{
		T data;

		if (_subscription.copy(data)) {
			OnUpdate(data);
		}
	}

	virtual void OnUpdate(const T &msg) = 0;


	bool change_orb_topic(orb_id_t new_topic, uint8_t new_instance = 0)
	{

	}

private:
	uORB::SubscriptionCallback<T>	_subscription;
};

} // namespace px4
