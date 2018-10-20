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


#include <drivers/device/integrator.h>
#include <drivers/drv_accel.h>
#include <drivers/drv_hrt.h>
#include <lib/cdev/CDev.hpp>
#include <lib/conversion/rotation.h>
#include <mathlib/math/filter/LowPassFilter2pVector3f.hpp>
#include <uORB/topics/sensor_accel.h>
#include <uORB/uORB.h>

class PX4Accelerometer : public cdev::CDev
{

public:
	PX4Accelerometer(uint32_t device_id, enum Rotation rotation, float scale);
	~PX4Accelerometer() override;

	int	init() override;
	int	ioctl(cdev::file_t *filp, int cmd, unsigned long arg) override;

	void configure_filter(float sample_freq, float cutoff_freq);

	void update(int16_t x, int16_t y, int16_t z);

	void update_temperature(float temperature) { _report.temperature = temperature; }

private:

	sensor_accel_s		_report{};

	matrix::Vector3f	_calibration_scale{1.0f, 1.0f, 1.0f};
	matrix::Vector3f	_calibration_offset{0.0f, 0.0f, 0.0f};

	orb_advert_t		_topic{nullptr};

	int			_orb_class_instance{-1};

	int			_class_device_instance{-1};

	const matrix::Dcmf	_rotation;

	math::LowPassFilter2pVector3f _filter{1000, 100};

	Integrator _integrator{4000};
};
