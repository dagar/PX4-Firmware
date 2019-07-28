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

#include "mag.hpp"

#include <drivers/drv_hrt.h>
#include <systemlib/err.h>

const char *const UavcanMagnetometerBridge::NAME = "mag";

UavcanMagnetometerBridge::UavcanMagnetometerBridge(uavcan::INode &node) :
	UavcanSensorBridgeBase(node, "uavcan_mag"),
	_sub_mag(node),
	_sub_mag2(node),
	_px4_magnetometer(get_device_id(), ORB_PRIO_HIGH, ROTATION_NONE)
{
	_px4_magnetometer.set_device_type(DRV_MAG_DEVTYPE_HMC5883);	// not correct in general
	_px4_magnetometer.set_external(true);

	// fake scaling for "raw data"
	_px4_magnetometer.set_scale(1.0f / 1000.0f);
}

int
UavcanMagnetometerBridge::init()
{
	int res = _sub_mag.start(MagCbBinder(this, &UavcanMagnetometerBridge::mag_sub_cb));

	if (res < 0) {
		PX4_ERR("failed to start uavcan sub: %d", res);
		return res;
	}

	int res2 = _sub_mag2.start(Mag2CbBinder(this, &UavcanMagnetometerBridge::mag2_sub_cb));

	if (res2 < 0) {
		PX4_ERR("failed to start uavcan sub2: %d", res);
		return res;
	}

	return 0;
}

void
UavcanMagnetometerBridge::mag_sub_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::ahrs::MagneticFieldStrength>
				     &msg)
{
	// uavcan.equipment.ahrs.MagneticFieldStrength

	// parameters
	// uavcan.pubp-mag - broadcast period
	// uavcan.prio-mag - transfer priority
	// mag.variance
	// mag.scaling_coef
	// mag.pwron_slftst

	_px4_magnetometer.update(hrt_absolute_time(),
				 msg.magnetic_field_ga[0] * 1000,
				 msg.magnetic_field_ga[1] * 1000,
				 msg.magnetic_field_ga[2] * 1000);


	// check msg.getSrcNodeID().get()


	msg.getSrcNodeID().get();	// address
	msg.getIfaceIndex();		// UAVCAN bus
}

void
UavcanMagnetometerBridge::mag2_sub_cb(const
				      uavcan::ReceivedDataStructure<uavcan::equipment::ahrs::MagneticFieldStrength2> &msg)
{
	_px4_magnetometer.update(hrt_absolute_time(),
				 msg.magnetic_field_ga[0] * 1000,
				 msg.magnetic_field_ga[1] * 1000,
				 msg.magnetic_field_ga[2] * 1000);
}
