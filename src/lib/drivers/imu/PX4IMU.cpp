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


#include "PX4Accelerometer.hpp"

#include <lib/drivers/device/Device.hpp>

PX4IMU::PX4IMU(uint32_t device_id, uint8_t priority, enum Rotation rotation) :
	_imu_pub{ORB_ID(sensor_imu), priority}
{
}

PX4IMU::~PX4IMU()
{
}

void PX4IMU::updateFIFOAccel(sensor_accel_fifo_s &fifo)
{
	_px4_accel.updateFIFO(fifo);

	// integrated data (INS)
	{
		matrix::Vector3f integrated_value;
		uint32_t integral_dt = 0;

		for (int n = 0; n < fifo.samples; n++) {
			const hrt_abstime timestamp = fifo.timestamp_sample + fifo.dt * n;
			const matrix::Vector3f val{(float)fifo.x[n], (float)fifo.y[n], (float)fifo.z[n]};


		}
	}
}

void PX4IMU::updateFIFOGyro(sensor_gyro_fifo_s &fifo)
{
	_px4_gyro.updateFIFO(fifo);

	// integrated data (INS)
	{
		matrix::Vector3f integrated_value;
		uint32_t integral_dt = 0;

		for (int n = 0; n < fifo.samples; n++) {
			const hrt_abstime timestamp = fifo.timestamp_sample + fifo.dt * n;
			const matrix::Vector3f val{(float)fifo.x[n], (float)fifo.y[n], (float)fifo.z[n]};


		}
	}
}

void PX4IMU::print_status()
{
}
