/****************************************************************************
 *
 *   Copyright (c) 2012-2017 PX4 Development Team. All rights reserved.
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
 * @file Subscription.cpp
 *
 */

#include "Subscription.hpp"

namespace uORB
{

SubscriptionBase::SubscriptionBase(const orb_metadata *meta, uint8_t interval, uint8_t instance) :
	_meta(meta),
	_interval(interval),
	_instance(instance)
{
	init();
}

bool SubscriptionBase::init()
{
	if (published()) {
		DeviceMaster *device_master = uORB::Manager::get_instance()->get_device_master();

		char path[orb_maxpath];
		const struct orb_metadata *meta = _meta;
		int instance = _instance;

		int ret = uORB::Utils::node_mkpath(path, meta, &instance);

		PX4_DEBUG("path: %s", path);

		if (ret != PX4_OK) {
			PX4_ERR("init failed %s", path);
		}

		_node = device_master->getDeviceNode(path);

		return (_node != nullptr);
	}

	return false;
}

bool SubscriptionBase::published()
{
	DeviceMaster *device_master = uORB::Manager::get_instance()->get_device_master();
	return device_master->published(_meta, _instance);
}

bool SubscriptionBase::update(uint64_t *time, void *data)
{
	if (update(data)) {
		/* data copied successfully */
		*time = _node->last_update();
		return true;
	}

	return false;
}

SubscriptionNode::SubscriptionNode(const struct orb_metadata *meta, unsigned interval, unsigned instance,
				   List<SubscriptionNode *> *list)
	: SubscriptionBase(meta, interval, instance)
{
	if (list != nullptr) {
		list->add(this);
	}
}

} // namespace uORB
