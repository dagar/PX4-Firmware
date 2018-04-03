/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
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
 * @file subsystem_info_pub.cpp
 *
 * Contains helper functions to efficiently publish the subsystem_info topic from various locations inside the code.
 *
 * @author Philipp Oettershagen (philipp.oettershagen@mavt.ethz.ch)
 */

#include "subsystem_info_pub.h"

int vehicle_status_sub = 0;
struct vehicle_status_s status = {};

/* Publishes the full state information for a specific subsystem type */
void publish_subsystem_info(orb_advert_t* pub_handle, uint64_t subsystem_type, bool present, bool enabled, bool ok)
{
	struct subsystem_info_s info = {};
	info.present = present;
	info.enabled = enabled;
	info.ok = ok;
	info.subsystem_type = subsystem_type;

	if (*pub_handle != nullptr) {
		orb_publish(ORB_ID(subsystem_info), *pub_handle, &info);
		PX4_INFO("publish_subsystem_info: type=%llu, handle=%d, pres=%u, enab=%u, ok=%u",subsystem_type,*pub_handle,present,enabled,ok);
	} else {
		*pub_handle = orb_advertise_queue(ORB_ID(subsystem_info), &info,subsystem_info_s::ORB_QUEUE_LENGTH);
		PX4_INFO("advertise_subsystem_info: type=%llu, handle=%d, pres=%u, enab=%u, ok=%u",subsystem_type,*pub_handle,present,enabled,ok);
	}
}

/* Leaves the present and enabled flags for a certain subsystem type unchanged, but changes the ok/healthy flag */
void publish_subsystem_info_healthy(orb_advert_t* pub_handle, uint64_t subsystem_type, bool ok)
{
	if(vehicle_status_sub==0) vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
	orb_copy(ORB_ID(vehicle_status), vehicle_status_sub, &status);

	bool present = status.onboard_control_sensors_present & (uint32_t) subsystem_type;
	bool enabled = status.onboard_control_sensors_enabled & (uint32_t) subsystem_type;

	//PX4_INFO("pub_subsys_info_healthy: type: %llu, present:%u, enabled:%u", subsystem_type, present,enabled);

	// Warning: In theory, if the subsystem_info queue size is >1 and depending on when exactly commander evaluates the subsystem_info
	// messages, there is a small risk that this function reads "old" present+enabled values from vehicle_status, then publishes a
	// subsystem_info message with those "old" values into the queue but before that (in the queue) there is already another
	// subsystem_info message that has not been applied yet that will change the present+enabled values. In that case, this function would
	// overwrite those values!
	publish_subsystem_info(pub_handle, subsystem_type, present, enabled, ok);

}
