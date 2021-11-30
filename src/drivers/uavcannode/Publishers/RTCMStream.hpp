/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

#include "UavcanPublisherBase.hpp"

#include <uavcan/equipment/gnss/RTCMStream.hpp>

#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/gps_inject_data.h>

namespace uavcannode
{

class RTCMStreamPub :
	public UavcanPublisherBase,
	public uORB::SubscriptionCallbackWorkItem,
	private uavcan::Publisher<uavcan::equipment::gnss::RTCMStream>
{
public:
	RTCMStreamPub(px4::WorkItem *work_item, uavcan::INode &node) :
		UavcanPublisherBase(uavcan::equipment::gnss::RTCMStream::DefaultDataTypeID),
		uORB::SubscriptionCallbackWorkItem(work_item, ORB_ID(gps_inject_data)),
		uavcan::Publisher<uavcan::equipment::gnss::RTCMStream>(node)
	{
		this->setPriority(uavcan::TransferPriority::NumericallyMax);
	}

	void PrintInfo() override
	{
		if (uORB::SubscriptionCallbackWorkItem::advertised()) {
			printf("\t%s -> %s:%d\n",
			       uORB::SubscriptionCallbackWorkItem::get_topic()->o_name,
			       uavcan::equipment::gnss::RTCMStream::getDataTypeFullName(),
			       id());
		}
	}

	void BroadcastAnyUpdates() override
	{
		using uavcan::equipment::gnss::RTCMStream;

		// gps_inject_data -> uavcan::equipment::gnss::RTCMStream
		gps_inject_data_s inject_data;

		if (uORB::SubscriptionCallbackWorkItem::update(&inject_data)) {
			// Prevent republishing rtcm data we received from uavcan
			if (inject_data.device_id > uavcan::NodeID::Max) {
				RTCMStream rtcmstream{};
				rtcmstream.protocol_id = RTCMStream::PROTOCOL_ID_RTCM3;

				const size_t capacity = rtcmstream.data.capacity();
				size_t written = 0;
				int result = 0;

				while ((result >= 0) && written < inject_data.len) {
					size_t chunk_size = inject_data.len - written;

					if (chunk_size > capacity) {
						chunk_size = capacity;
					}

					for (size_t i = 0; i < chunk_size; ++i) {
						rtcmstream.data.push_back(inject_data.data[written]);
						written += 1;
					}

					result = uavcan::Publisher<uavcan::equipment::gnss::RTCMStream>::broadcast(rtcmstream);

					// ensure callback is registered
					uORB::SubscriptionCallbackWorkItem::registerCallback();

					rtcmstream.data.clear();
				}
			}
		}
	}
};
} // namespace uavcannode
