/****************************************************************************
 *
 *   Copyright (c) 2012-2019 PX4 Development Team. All rights reserved.
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
 * @file Subscription2.hpp
 *
 */

#pragma once

#include <uORB/uORB.h>
#include <uORB/topics/uORBTopics.hpp>

#include <px4_platform_common/defines.h>
#include <lib/mathlib/mathlib.h>

#include "uORBDeviceNode.hpp"
#include "uORBManager.hpp"
#include "uORBUtils.hpp"

namespace uORB
{

class Subscription2Base
{
public:

	bool valid() const { return _node != nullptr; }

	void unsubscribe()
	{
		if (_node != nullptr) {
			_node->remove_internal_subscriber();
		}

		_node = nullptr;
	}

	unsigned get_last_generation() const { return _last_generation; }

protected:
	DeviceNode *_node{nullptr};
	unsigned _last_generation{0}; /**< last generation the subscriber has seen */
}

template<ORB_ID OID, class T>
class Subscription2
{
public:
	Subscription2()
	{
		subscribe();
	}

	~Subscription2()
	{
		unsubscribe();
	}

	bool subscribe()
	{
		// check if already subscribed
		if (_node != nullptr) {
			return true;
		}

		DeviceMaster *device_master = uORB::Manager::get_instance()->get_device_master();

		if (device_master != nullptr) {

			if (!device_master->deviceNodeExists(OID, 0)) {
				return false;
			}

			uORB::DeviceNode *node = device_master->getDeviceNode(get_topic(), 0);

			if (node != nullptr) {
				_node = node;
				_node->add_internal_subscriber();

				// If there were any previous publications, allow the subscriber to read them
				const unsigned curr_gen = _node->published_message_count();
				const uint8_t q_size = _node->get_queue_size();

				if (q_size < curr_gen) {
					_last_generation = curr_gen - q_size;

				} else {
					_last_generation = 0;
				}

				return true;
			}
		}

		return false;
	}

	bool advertised()
	{
		if (valid()) {
			return _node->is_advertised();
		}

		// try to initialize
		if (subscribe()) {
			// check again if valid
			if (valid()) {
				return _node->is_advertised();
			}
		}

		return false;
	}

	/**
	 * Check if there is a new update.
	 * */
	bool updated() { return advertised() && (_node->published_message_count() != _last_generation); }

	/**
	 * Update the struct
	 * @param dst The uORB message struct we are updating.
	 */
	bool update(T &dst) { return updated() && _node->copy(&dst, _last_generation); }

	/**
	 * Copy the struct
	 * @param dst The uORB message struct we are updating.
	 */
	bool copy(T &dst) { return advertised() && _node->copy(&dst, _last_generation); }

	T get()
	{
		if (advertised()) {
			T data;
			_node->copy(&data, _last_generation);
			return data;
		}

		return T{};
	}


	ORB_PRIO get_priority() { return advertised() ? _node->get_priority() : ORB_PRIO_UNINITIALIZED; }
	orb_id_t get_topic() const { return get_orb_meta(OID); }

protected:

};

} // namespace uORB
