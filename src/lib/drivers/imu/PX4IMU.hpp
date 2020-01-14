/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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
#include <lib/conversion/rotation.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/sensor_imu.h>

#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>

class PX4IMU
{
public:
	PX4IMU(uint32_t device_id, uint8_t priority = ORB_PRIO_DEFAULT, enum Rotation rotation = ROTATION_NONE);
	~PX4IMU() = default;

	void set_error_count(uint64_t error_count);
	void set_sample_rate(uint16_t rate);
	void set_temperature(float temperature);
	void set_update_rate(uint16_t rate);

	PX4Accelerometer &accel() { return _accel; }
	PX4Gyroscope &gyro() { return _gyro; }

	void update(hrt_abstime timestamp_sample, matrix::Vector3f accel, matrix::Vector3f gyro);

	void print_status();

private:

	PX4Accelerometer _accel;
	PX4Gyroscope     _gyro;

	uORB::PublicationMulti<sensor_imu_s> _sensor_imu_pub;
};
