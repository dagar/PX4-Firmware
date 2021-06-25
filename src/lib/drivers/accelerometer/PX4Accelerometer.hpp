/****************************************************************************
 *
 *   Copyright (c) 2018-2021 PX4 Development Team. All rights reserved.
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
#include <lib/ecl/geo/geo.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/sensor_accel_fifo.h>

class PX4Accelerometer
{
public:
	PX4Accelerometer(uint32_t device_id, enum Rotation rotation = ROTATION_NONE);
	~PX4Accelerometer();

	uint32_t get_device_id() const { return _device_id; }

	int32_t get_max_rate_hz() const { return math::constrain(_imu_gyro_rate_max, static_cast<int32_t>(100), static_cast<int32_t>(4000)); }

	void set_device_id(uint32_t device_id) { _device_id = device_id; }
	void set_device_type(uint8_t devtype);
	void set_error_count(uint32_t error_count) { _error_count = error_count; }
	void increase_error_count() { _error_count++; }
	void set_range(float range) { _range = range; UpdateClipLimit(); }
	void set_scale(float scale);
	void set_temperature(float temperature) { _temperature = temperature; }

	void update(const hrt_abstime &timestamp_sample, float x, float y, float z)
	{
		// Apply rotation (before scaling)
		rotate_3f(_rotation, x, y, z);

		// publish
		sensor_accel_s report;

		report.timestamp_sample = timestamp_sample;
		report.device_id = _device_id;
		report.temperature = _temperature;
		report.error_count = _error_count;
		report.x = x * _scale;
		report.y = y * _scale;
		report.z = z * _scale;
		report.clip_counter[0] = (fabsf(x) >= _clip_limit);
		report.clip_counter[1] = (fabsf(y) >= _clip_limit);
		report.clip_counter[2] = (fabsf(z) >= _clip_limit);
		report.samples = 1;
		report.timestamp = hrt_absolute_time();

		_sensor_pub.publish(report);
	}

	void updateFIFO(sensor_accel_fifo_s &sample)
	{
		// rotate all raw samples and publish fifo
		const uint8_t N = sample.samples;

		for (int n = 0; n < N; n++) {
			rotate_3i(_rotation, sample.x[n], sample.y[n], sample.z[n]);
		}

		sample.device_id = _device_id;
		sample.scale = _scale;
		sample.timestamp = hrt_absolute_time();
		_sensor_fifo_pub.publish(sample);

		// trapezoidal integration (equally spaced, scaled by dt later)
		const matrix::Vector3f integral{
			(0.5f * (_last_sample[0] + sample.x[N - 1]) + sum(sample.x, N - 1)),
			(0.5f * (_last_sample[1] + sample.y[N - 1]) + sum(sample.y, N - 1)),
			(0.5f * (_last_sample[2] + sample.z[N - 1]) + sum(sample.z, N - 1)),
		};

		_last_sample[0] = sample.x[N - 1];
		_last_sample[1] = sample.y[N - 1];
		_last_sample[2] = sample.z[N - 1];

		const float scale = _scale / static_cast<float>(N);

		// publish
		sensor_accel_s report;
		report.timestamp_sample = sample.timestamp_sample;
		report.device_id = _device_id;
		report.temperature = _temperature;
		report.error_count = _error_count;
		report.x = integral(0) * scale;
		report.y = integral(1) * scale;
		report.z = integral(2) * scale;
		report.clip_counter[0] = clipping(sample.x, _clip_limit, N);
		report.clip_counter[1] = clipping(sample.y, _clip_limit, N);
		report.clip_counter[2] = clipping(sample.z, _clip_limit, N);
		report.samples = N;
		report.timestamp = hrt_absolute_time();

		_sensor_pub.publish(report);
	}

	int get_instance() { return _sensor_pub.get_instance(); };

private:
	void UpdateClipLimit();

	static constexpr int32_t sum(const int16_t samples[], uint8_t len)
	{
		int32_t sum = 0;

		for (int n = 0; n < len; n++) {
			sum += samples[n];
		}

		return sum;
	}

	static constexpr uint8_t clipping(const int16_t samples[], int16_t clip_limit, uint8_t len)
	{
		unsigned clip_count = 0;

		for (int n = 0; n < len; n++) {
			if (abs(samples[n]) >= clip_limit) {
				clip_count++;
			}
		}

		return clip_count;
	}

	uORB::PublicationMulti<sensor_accel_s> _sensor_pub{ORB_ID(sensor_accel)};
	uORB::PublicationMulti<sensor_accel_fifo_s> _sensor_fifo_pub{ORB_ID(sensor_accel_fifo)};

	uint32_t _device_id{0};
	const enum Rotation _rotation;

	int32_t _imu_gyro_rate_max{0}; // match gyro max rate

	float _range{16 * CONSTANTS_ONE_G};
	float _scale{1.f};
	float _clip_limit{_range / _scale};
	float _temperature{NAN};
	uint32_t _error_count{0};
	int16_t _last_sample[3] {};
};
