/****************************************************************************
 *
 *   Copyright (c) 2018-2020 PX4 Development Team. All rights reserved.
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


#include "PX4Gyroscope.hpp"

#include <lib/drivers/device/Device.hpp>

using namespace time_literals;
using matrix::Vector3f;

static inline int32_t sum(const int16_t samples[16], uint8_t len)
{
	int32_t sum = 0;

	for (int n = 0; n < len; n++) {
		sum += samples[n];
	}

	return sum;
}

static constexpr unsigned clipping(const int16_t samples[16], int16_t clip_limit, uint8_t len)
{
	unsigned clip_count = 0;

	for (int n = 0; n < len; n++) {
		if (abs(samples[n]) >= clip_limit) {
			clip_count++;
		}
	}

	return clip_count;
}

PX4Gyroscope::PX4Gyroscope(uint32_t device_id, uint8_t priority, enum Rotation rotation) :
	ModuleParams(nullptr),
	_sensor_pub{ORB_ID(sensor_gyro), priority},
	_sensor_fifo_pub{ORB_ID(sensor_gyro_fifo), priority},
	_sensor_status_pub{ORB_ID(sensor_gyro_status), priority},
	_device_id{device_id},
	_rotation{rotation}
{
	updateParams();
}

void PX4Gyroscope::set_device_type(uint8_t devtype)
{
	// current DeviceStructure
	union device::Device::DeviceId device_id;
	device_id.devid = _device_id;

	// update to new device type
	device_id.devid_s.devtype = devtype;

	// copy back
	_device_id = device_id.devid;
}

void PX4Gyroscope::update(const hrt_abstime &timestamp_sample, float x, float y, float z)
{
	// Apply rotation
	rotate_3f(_rotation, x, y, z);

	const Vector3f raw{x, y, z};
	const Vector3f val{raw * _scale};

	sensor_gyro_s report;
	report.timestamp_sample = timestamp_sample;
	report.device_id = _device_id;
	report.temperature = _temperature;
	report.x = val(0);
	report.y = val(1);
	report.z = val(2);
	report.timestamp = hrt_absolute_time();
	_sensor_pub.publish(report);

	// Clipping (check unscaled raw values)
	for (int i = 0; i < 3; i++) {
		if (fabsf(raw(i)) > _clip_limit) {
			_clipping[i]++;
		}
	}

	PublishStatus();
}

void PX4Gyroscope::updateFIFO(const FIFOSample &sample)
{
	const uint8_t N = sample.samples;
	const float dt = sample.dt;

	// clipping
	unsigned clip_count_x = clipping(sample.x, _clip_limit, N);
	unsigned clip_count_y = clipping(sample.y, _clip_limit, N);
	unsigned clip_count_z = clipping(sample.z, _clip_limit, N);

	_clipping[0] += clip_count_x;
	_clipping[1] += clip_count_y;
	_clipping[2] += clip_count_z;

	// publish sensor_gyro_fifo
	sensor_gyro_fifo_s fifo{};
	fifo.timestamp_sample = sample.timestamp_sample;
	fifo.device_id = _device_id;
	fifo.dt = dt;
	fifo.scale = _scale;
	fifo.samples = N;

	memcpy(fifo.x, sample.x, sizeof(sample.x[0]) * N);
	memcpy(fifo.y, sample.y, sizeof(sample.y[0]) * N);
	memcpy(fifo.z, sample.z, sizeof(sample.z[0]) * N);

	fifo.timestamp = hrt_absolute_time();
	_sensor_fifo_pub.publish(fifo);


	// publish sensor_gyro
	float x = (float)sum(sample.x, N) / (float)N;
	float y = (float)sum(sample.y, N) / (float)N;
	float z = (float)sum(sample.z, N) / (float)N;

	// Apply rotation
	rotate_3f(_rotation, x, y, z);
	const Vector3f val{(Vector3f{x, y, z} * _scale)};

	sensor_gyro_s report;
	report.timestamp_sample = sample.timestamp_sample;
	report.device_id = _device_id;
	report.temperature = _temperature;
	report.x = val(0);
	report.y = val(1);
	report.z = val(2);
	report.timestamp = hrt_absolute_time();
	_sensor_pub.publish(report);


	PublishStatus();
}

void PX4Gyroscope::PublishStatus()
{
	// publish sensor status
	if (hrt_elapsed_time(&_status_last_publish) >= 100_ms) {
		sensor_gyro_status_s status;

		status.device_id = _device_id;
		status.error_count = _error_count;
		status.full_scale_range = _range;
		status.rotation = _rotation;
		status.measure_rate_hz = _update_rate;
		status.temperature = _temperature;
		//status.vibration_metric = _vibration_metric;
		//status.coning_vibration = _coning_vibration;
		status.clipping[0] = _clipping[0];
		status.clipping[1] = _clipping[1];
		status.clipping[2] = _clipping[2];
		status.timestamp = hrt_absolute_time();
		_sensor_status_pub.publish(status);

		_status_last_publish = status.timestamp;
	}
}

void PX4Gyroscope::UpdateClipLimit()
{
	// 99.9% of potential max
	_clip_limit = fmaxf((_range / _scale) * 0.999f, INT16_MAX);
}

void PX4Gyroscope::print_status()
{
	char device_id_buffer[80] {};
	device::Device::device_id_print_buffer(device_id_buffer, sizeof(device_id_buffer), _device_id);
	PX4_INFO("device id: %d (%s)", _device_id, device_id_buffer);
	PX4_INFO("rotation: %d", _rotation);
}
