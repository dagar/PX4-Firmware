/****************************************************************************
 *
 *   Copyright (c) 2014, 2015 PX4 Development Team. All rights reserved.
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
 * @author Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <drivers/drv_hrt.h>
#include "baro.hpp"
#include <cmath>

const char *const UavcanBarometerBridge::NAME = "baro";

UavcanBarometerBridge::UavcanBarometerBridge(uavcan::INode &node) :
	UavcanSensorBridgeBase(node, "uavcan_baro"),
	_sub_air_pressure_data(node),
	_sub_air_temperature_data(node),
	_px4_barometer(get_device_id(), ORB_PRIO_HIGH)
{
	_px4_barometer.set_device_type(DRV_BARO_DEVTYPE_MS5611); // not correct in general
}

int
UavcanBarometerBridge::init()
{
	int res = _sub_air_pressure_data.start(AirPressureCbBinder(this, &UavcanBarometerBridge::air_pressure_sub_cb));

	if (res < 0) {
		PX4_ERR("failed to start uavcan sub: %d", res);
		return res;
	}

	res = _sub_air_temperature_data.start(AirTemperatureCbBinder(this, &UavcanBarometerBridge::air_temperature_sub_cb));

	if (res < 0) {
		PX4_ERR("failed to start uavcan sub: %d", res);
		return res;
	}

	return 0;
}

void
UavcanBarometerBridge::air_temperature_sub_cb(const
		uavcan::ReceivedDataStructure<uavcan::equipment::air_data::StaticTemperature> &msg)
{
	_px4_barometer.set_temperature(msg.static_temperature - 273.15F);	// Convert from Kelvin
}

void
UavcanBarometerBridge::air_pressure_sub_cb(const
		uavcan::ReceivedDataStructure<uavcan::equipment::air_data::StaticPressure> &msg)
{
	const float pressure = msg.static_pressure / 100.0F;  // Convert to millibar

	_px4_barometer.update(hrt_absolute_time(), pressure);
}
