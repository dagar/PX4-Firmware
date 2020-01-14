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


#include "PX4IMU.hpp"

#include <lib/drivers/device/Device.hpp>

using namespace time_literals;
using matrix::Vector3f;

PX4IMU::PX4IMU(uint32_t device_id, uint8_t priority, enum Rotation rotation) :
	_accel(device_id, priority, rotation),
	_gyro(device_id, priority, rotation),
	_sensor_imu_pub{ORB_ID(sensor_imu), priority}
{
}

void PX4IMU::set_error_count(uint64_t error_count)
{
	_accel.set_error_count(error_count);
	_gyro.set_error_count(error_count);
}

void PX4IMU::set_sample_rate(uint16_t rate)
{
	_accel.set_sample_rate(rate);
	_gyro.set_sample_rate(rate);
}

void PX4IMU::set_temperature(float temperature)
{
	_accel.set_temperature(temperature);
	_gyro.set_temperature(temperature);
}

void PX4IMU::set_update_rate(uint16_t rate)
{
	_accel.set_update_rate(rate);
	_gyro.set_update_rate(rate);
}

void PX4IMU::update(hrt_abstime timestamp_sample, Vector3f accel, Vector3f gyro)
{
	// TODO: control the integration reset
	_gyro.update(timestamp_sample, gyro(0), gyro(1), gyro(2));
	_accel.update(timestamp_sample, accel(0), accel(1), accel(2));

	if (_accel.integrator_updated() && _gyro.integrator_updated()) {

		const uint64_t accel_dt = _accel.integrator_dt();
		const uint64_t gyro_dt = _gyro.integrator_dt();

		if (accel_dt == gyro_dt) {
			sensor_imu_s sensor_imu{};
			sensor_imu.timestamp_sample = timestamp_sample;
			sensor_imu.device_id = _accel.device_id();

			_gyro.delta_angle().copyTo(sensor_imu.delta_angle);
			_accel.delta_velocity().copyTo(sensor_imu.delta_velocity);
			sensor_imu.dt = accel_dt;

			sensor_imu.timestamp = hrt_absolute_time();
			_sensor_imu_pub.publish(sensor_imu);

		} else {
			PX4_ERR("accel and gyro integrators out of sync, resetting");
			_accel.ResetIntegrator();
			_gyro.ResetIntegrator();
		}

	} else {
		if (_accel.integrator_updated() != _gyro.integrator_updated()) {
			PX4_ERR("accel and gyro integrators out of sync, resetting");
			_accel.ResetIntegrator();
			_gyro.ResetIntegrator();
		}
	}
}

void PX4IMU::print_status()
{
	_accel.print_status();
	_gyro.print_status();
}
