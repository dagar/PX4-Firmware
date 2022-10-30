/****************************************************************************
 *
 *   Copyright (c) 2019-2022 PX4 Development Team. All rights reserved.
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
 * Feeds Ekf with Gps yaw data
 */
#ifndef EKF_GPS_YAW_H
#define EKF_GPS_YAW_H

#include "sensor.h"

namespace sensor_simulator
{
namespace sensor
{

class GpsYaw: public Sensor
{
public:
	GpsYaw(std::shared_ptr<Ekf> ekf) : Sensor(ekf) {}
	~GpsYaw() = default;

	void setData(const gpsYawSample &gps_yaw) { _gps_yaw_data = gps_yaw; }

	void setYaw(const float yaw) { _gps_yaw_data.yaw = yaw; }
	void setYawOffset(const float yaw_offset) { _gps_yaw_data.yaw_offset = yaw_offset; }

	gpsYawSample getDefaultGpsData()
	{
		gpsYawSample gps_yaw_data{};
		gps_yaw_data.time_us = 0;
		gps_yaw_data.yaw = NAN;
		gps_yaw_data.yaw_offset = NAN;
		gps_yaw_data.yaw_accuracy = NAN;
		return gps_yaw_data;
	}

private:
	void send(uint64_t time) override
	{
		_gps_yaw_data.time_us = time;
		_ekf->setGpsYawData(_gps_yaw_data);
	}

	gpsYawSample _gps_yaw_data{};
};

} // namespace sensor
} // namespace sensor_simulator
#endif // EKF_GPS_YAW_H
