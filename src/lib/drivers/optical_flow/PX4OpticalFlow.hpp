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

#include <drivers/drv_hrt.h>
#include <lib/cdev/CDev.hpp>
#include <lib/conversion/rotation.h>
#include <uORB/uORB.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/optical_flow.h>

class PX4OpticalFlow : public cdev::CDev
{

public:
	PX4OpticalFlow(const uint32_t device_id,
		       const uint8_t priority = ORB_PRIO_DEFAULT,
		       const uint8_t device_orientation = distance_sensor_s::ROTATION_DOWNWARD_FACING);
	~PX4OpticalFlow() override;

	void print_status();

	void set_device_type(uint8_t device_type);
	//void set_error_count(uint64_t error_count) { _distance_sensor_pub.get().error_count = error_count; }

	void set_device_id(const uint8_t device_id) { _distance_sensor_pub.get().id = device_id; };

	void set_max_distance(const float distance) { _distance_sensor_pub.get().max_distance = distance; }
	void set_min_distance(const float distance) { _distance_sensor_pub.get().min_distance = distance; }

	void set_orientation(const uint8_t device_orientation = distance_sensor_s::ROTATION_DOWNWARD_FACING);

	void update(const hrt_abstime timestamp, const float delta_x, const float delta_y, const int quality = -1);

private:

	uORB::PublicationMultiData<optial_flow_s> _optical_flow_pub;

	enum Rotation _yaw_rotation;

	int _class_device_instance{-1};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::SENS_FLOW_ROT>) _param_sens_flow_rot,
		(ParamFloat<px4::params::SENS_FLOW_MINHGT>) _param_sens_flow_minhgt,
		(ParamFloat<px4::params::SENS_FLOW_MAXHGT>) _param_sens_flow_maxhgt,
		(ParamFloat<px4::params::SENS_FLOW_MAXR>) _param_sens_flow_maxr
	)

};
