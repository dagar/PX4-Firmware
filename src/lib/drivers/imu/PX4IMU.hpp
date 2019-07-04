/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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
#include <drivers/device/accelerometer/PX4Accelerometer.hpp>
#include <drivers/device/gyroscope/PX4Gyroscope.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/sensor_imu.h>

class PX4IMU
{

public:
	PX4IMU(uint32_t device_id, uint8_t priority = ORB_PRIO_DEFAULT, enum Rotation rotation = ROTATION_NONE);
	~PX4IMU() override;

	void set_device_type(uint8_t devtype);
	void set_error_count(uint64_t error_count) { _sensor_pub.get().error_count = error_count; }
	void set_scale(float scale) { _sensor_pub.get().scaling = scale; }
	void set_temperature(float temperature) { _sensor_pub.get().temperature = temperature; }

	void set_sample_rate(unsigned rate);

	void updateAccelFIFO(sensor_accel_fifo_s &fifo);
	void updateGyroFIFO(sensor_accel_fifo_s &fifo);

	void print_status();

private:
	uORB::PublicationData<sensor_imu_s>		_imu_pub;

	PX4Accelerometer	_px4_accel;
	PX4Gyroscope		_px4_gyro;

};
