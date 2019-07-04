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

using namespace time_literals;

PX4Accelerometer::PX4Accelerometer(uint32_t device_id, uint8_t priority, enum Rotation rotation) :
	CDev(nullptr),
	ModuleParams(nullptr),
	_sensor_pub{ORB_ID(sensor_accel), priority},
	_sensor_control_pub{ORB_ID(sensor_accel_control), priority},
	_sensor_status_pub{ORB_ID(sensor_accel_status), priority},
	_rotation{rotation}
{
	_class_device_instance = register_class_devname(ACCEL_BASE_DEVICE_PATH);

	_sensor_pub.get().device_id = device_id;
	_sensor_pub.get().scaling = 1.0f;

	// set software low pass filter for controllers
	updateParams();
	configure_filter(_param_imu_accel_cutoff.get());
}

PX4Accelerometer::~PX4Accelerometer()
{
	if (_class_device_instance != -1) {
		unregister_class_devname(ACCEL_BASE_DEVICE_PATH, _class_device_instance);
	}
}

int
PX4Accelerometer::ioctl(cdev::file_t *filp, int cmd, unsigned long arg)
{
	switch (cmd) {
	case ACCELIOCSSCALE: {
			// Copy offsets and scale factors in
			accel_calibration_s cal{};
			memcpy(&cal, (accel_calibration_s *) arg, sizeof(cal));

			_calibration_offset = matrix::Vector3f{cal.x_offset, cal.y_offset, cal.z_offset};
			_calibration_scale = matrix::Vector3f{cal.x_scale, cal.y_scale, cal.z_scale};
		}

		return PX4_OK;

	case DEVIOCGDEVICEID:
		return _sensor_pub.get().device_id;

	default:
		return -ENOTTY;
	}
}

void
PX4Accelerometer::set_device_type(uint8_t devtype)
{
	// current DeviceStructure
	union device::Device::DeviceId device_id;
	device_id.devid = _sensor_pub.get().device_id;

	// update to new device type
	device_id.devid_s.devtype = devtype;

	// copy back to report
	_sensor_pub.get().device_id = device_id.devid;
}

void
PX4Accelerometer::set_sample_rate(unsigned rate)
{
	_sample_rate = rate;
	_filter.set_cutoff_frequency(_sample_rate, _filter.get_cutoff_freq());

	_filterArrayX.set_cutoff_frequency(_sample_rate, _filterArrayX.get_cutoff_freq());
	_filterArrayY.set_cutoff_frequency(_sample_rate, _filterArrayY.get_cutoff_freq());
	_filterArrayZ.set_cutoff_frequency(_sample_rate, _filterArrayZ.get_cutoff_freq());
}

void
PX4Accelerometer::update(hrt_abstime timestamp, float x, float y, float z)
{
	sensor_accel_s &report = _sensor_pub.get();
	report.timestamp = timestamp;

	// Apply rotation (before scaling)
	rotate_3f(_rotation, x, y, z);

	const matrix::Vector3f raw{x, y, z};

	// Apply range scale and the calibrating offset/scale
	const matrix::Vector3f val_calibrated{(((raw * report.scaling) - _calibration_offset).emult(_calibration_scale))};

	// Filtered values
	const matrix::Vector3f val_filtered{_filter.apply(val_calibrated)};

	// Integrated values
	matrix::Vector3f integrated_value;
	uint32_t integral_dt = 0;

	if (_integrator.put(timestamp, val_calibrated, integrated_value, integral_dt)) {

		// Raw values (ADC units 0 - 65535)
		report.x_raw = x;
		report.y_raw = y;
		report.z_raw = z;

		report.x = val_filtered(0);
		report.y = val_filtered(1);
		report.z = val_filtered(2);

		report.integral_dt = integral_dt;
		report.x_integral = integrated_value(0);
		report.y_integral = integrated_value(1);
		report.z_integral = integrated_value(2);

		poll_notify(POLLIN);
		_sensor_pub.update();
	}
}

void
PX4Accelerometer::updateFIFO(sensor_accel_fifo_s &fifo)
{
	// filtered data (control)
	float x = _filterArrayX.apply(fifo.x, fifo.samples);
	float y = _filterArrayY.apply(fifo.y, fifo.samples);
	float z = _filterArrayZ.apply(fifo.z, fifo.samples);

	// Apply rotation (before scaling)
	rotate_3f(_rotation, x, y, z);

	const matrix::Vector3f raw{x, y, z};

	// Apply range scale and the calibrating offset/scale
	const matrix::Vector3f val_calibrated{(((raw * _sensor_pub.get().scaling) - _calibration_offset).emult(_calibration_scale))};

	sensor_accel_control_s control{};
	control.timestamp_sample = fifo.timestamp_sample;
	control.device_id = _sensor_pub.get().device_id;
	val_calibrated.copyTo(control.acceleration);

	control.timestamp = hrt_absolute_time();
	_sensor_control_pub.publish(control);


	// status
	{
		sensor_accel_status_s status{};
		status.device_id = _sensor_pub.get().device_id;
		status.error_count = _sensor_pub.get().error_count;
		status.temperature = _sensor_pub.get().temperature;

		status.clipping[0] = clipping(fifo.x, fifo.samples);
		status.clipping[1] = clipping(fifo.y, fifo.samples);
		status.clipping[2] = clipping(fifo.z, fifo.samples);

		//vibrationMetrics(fifo);
		//status.vibration_metric = _vibration_metric;

		status.timestamp = hrt_absolute_time();
		_sensor_status_pub.publish(status);
	}


	// integrated data (INS)
	{
		// integrate x axis
		for (int n = 1; n < fifo.samples; n++) {
			_integrator_accum[0] += (fifo.x[n] - fifo.x[n - 1]) * fifo.dt; // ADC units and time in microseconds
		}

		// integrate y axis
		for (int n = 1; n < fifo.samples; n++) {
			_integrator_accum[1] += (fifo.y[n] - fifo.y[n - 1]) * fifo.dt; // ADC units and time in microseconds
		}

		// integrate z axis
		for (int n = 1; n < fifo.samples; n++) {
			_integrator_accum[2] += (fifo.z[n] - fifo.z[n - 1]) * fifo.dt; // ADC units and time in microseconds
		}


		const hrt_abstime integrator_dt = hrt_elapsed_time(&_integrator_reset);

		if (integrator_dt > 3500_us) {

			_integrator_reset = hrt_absolute_time();

			matrix::Vector3f integrator_accum{(float)_integrator_accum[0], (float)_integrator_accum[1], (float)_integrator_accum[2]};

			// publish
			rotate_3f(_rotation, integrator_accum(0), integrator_accum(1), integrator_accum(2));

			matrix::Vector3f val2 = integrator_accum * 1e-6f;	// microseconds -> seconds

			const matrix::Vector3f val_cal{(((val2 * _sensor_pub.get().scaling) - _calibration_offset).emult(_calibration_scale))};

			// legacy sensor_accel_s message
			sensor_accel_s &report = _sensor_pub.get();

			// Raw values (ADC units 0 - 65535)
			report.x_raw = fifo.x[0];
			report.y_raw = fifo.y[0];
			report.z_raw = fifo.z[0];

			report.x = val_calibrated(0);
			report.y = val_calibrated(1);
			report.z = val_calibrated(2);

			report.integral_dt = integrator_dt;
			report.x_integral = val_cal(0) * fifo.dt;
			report.y_integral = val_cal(1) * fifo.dt;
			report.z_integral = val_cal(2) * fifo.dt;

			report.timestamp = hrt_absolute_time();	// TODO: timestamp_sample
			_sensor_pub.update();

			_integrator_accum[0] = 0;
			_integrator_accum[1] = 0;
			_integrator_accum[2] = 0;
		}
	}
}

uint32_t
PX4Accelerometer::clipping(int16_t samples[], uint8_t num_samples)
{
	int16_t clip_limit = 16.0f * 9.81f * _sensor_pub.get().scaling;

	int clip_count = 0;

	for (int n = 0; n < num_samples; n++) {
		if (samples[n] > clip_limit) {
			clip_count++;
		}
	}

	return clip_count;
}

void
PX4Accelerometer::vibrationMetrics(sensor_accel_fifo_s &fifo)
{
	for (int n = 1; n < fifo.samples; n++) {

		// float delta_vel = (fifo.x[n] - fifo.x[n-1]) / fifo.dt;	// ADC units and time in microseconds
		matrix::Vector3f delta_velocity;

		// calculate a metric which indicates the amount of high frequency accelerometer vibration
		matrix::Vector3f temp = delta_velocity - _delta_velocity_prev;
		_delta_velocity_prev = delta_velocity;
		_vibration_metric = 0.99f * _vibration_metric + 0.01f * temp.norm();
	}
}

void
PX4Accelerometer::print_status()
{
	PX4_INFO(ACCEL_BASE_DEVICE_PATH " device instance: %d", _class_device_instance);
	PX4_INFO("sample rate: %d Hz", _sample_rate);
	PX4_INFO("filter cutoff: %.3f Hz", (double)_filter.get_cutoff_freq());

	PX4_INFO("calibration scale: %.5f %.5f %.5f", (double)_calibration_scale(0), (double)_calibration_scale(1),
		 (double)_calibration_scale(2));
	PX4_INFO("calibration offset: %.5f %.5f %.5f", (double)_calibration_offset(0), (double)_calibration_offset(1),
		 (double)_calibration_offset(2));

	print_message(_sensor_pub.get());
}
