/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

#include "VehicleIMUFifo.hpp"

#include <px4_platform_common/log.h>
#include <px4_platform_common/events.h>
#include <lib/systemlib/mavlink_log.h>

#include <float.h>

using namespace matrix;

using math::constrain;

namespace sensors
{

static constexpr uint8_t clipping(const int16_t samples[], int first_sample, int last_sample)
{
	unsigned clip_count = 0;

	for (int n = first_sample; n <= last_sample; n++) {
		if ((samples[n] == INT16_MIN) || (samples[n] == INT16_MAX)) {
			clip_count++;
		}
	}

	return clip_count;
}

static constexpr int32_t sum(const int16_t samples[], int first_sample, int last_sample)
{
	int32_t sum = 0;

	for (int n = first_sample; n <= last_sample; n++) {
		sum += samples[n];
	}

	return sum;
}

VehicleIMUFifo::VehicleIMUFifo(int instance, uint32_t device_id, const px4::wq_config_t &config) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, config),
	_instance(instance)
{
	_imu_integration_interval_us = 1e6f / _param_imu_integ_rate.get();

	for (uint8_t i = 0; i < ORB_MULTI_MAX_INSTANCES; i++) {
		// accel
		uORB::SubscriptionData<sensor_imu_fifo_s> sensor_imu_fifo_sub{ORB_ID(sensor_imu_fifo), i};

		if (sensor_imu_fifo_sub.get().device_id == device_id) {
			_sensor_imu_fifo_sub.ChangeInstance(i);
		}
	}

	_gyro_integrator.set_reset_interval(_imu_integration_interval_us);
	_gyro_integrator.set_reset_samples(sensor_imu_fifo_s::ORB_QUEUE_LENGTH);

#if defined(ENABLE_LOCKSTEP_SCHEDULER)
	// currently with lockstep every raw sample needs a corresponding vehicle_imu publication
	_sensor_imu_fifo_sub.set_required_updates(1);
#else
	// schedule conservatively until the actual accel & gyro rates are known
	_sensor_imu_fifo_sub.set_required_updates(sensor_imu_fifo_s::ORB_QUEUE_LENGTH / 2);
#endif

	// advertise immediately to ensure consistent ordering
	_vehicle_imu_pub.advertise();
	_vehicle_imu_status_pub.advertise();
}

VehicleIMUFifo::~VehicleIMUFifo()
{
	Stop();

	perf_free(_imu_generation_gap_perf);

	_vehicle_imu_pub.unadvertise();
	_vehicle_imu_status_pub.unadvertise();
}

bool VehicleIMUFifo::Start()
{
	// force initial updates
	ParametersUpdate(true);

	_sensor_imu_fifo_sub.registerCallback();

	ScheduleNow();
	return true;
}

void VehicleIMUFifo::Stop()
{
	// clear all registered callbacks
	_sensor_imu_fifo_sub.unregisterCallback();

	Deinit();
}

void VehicleIMUFifo::ParametersUpdate(bool force)
{
	// Check if parameters have changed
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		const auto imu_integ_rate_prev = _param_imu_integ_rate.get();

		updateParams();

		_accel_calibration.ParametersUpdate();
		_gyro_calibration.ParametersUpdate();

		// constrain IMU integration time 1-10 milliseconds (100-1000 Hz)
		int32_t imu_integration_rate_hz = constrain(_param_imu_integ_rate.get(),
						  (int32_t)100, math::max(_param_imu_gyro_ratemax.get(), (int32_t)1000));

		if (imu_integration_rate_hz != _param_imu_integ_rate.get()) {
			PX4_WARN("IMU_INTEG_RATE updated %" PRId32 " -> %" PRIu32, _param_imu_integ_rate.get(), imu_integration_rate_hz);
			_param_imu_integ_rate.set(imu_integration_rate_hz);
			_param_imu_integ_rate.commit_no_notification();
		}

		_imu_integration_interval_us = 1e6f / imu_integration_rate_hz;

		if (_param_imu_integ_rate.get() != imu_integ_rate_prev) {
			// force update
			_update_integrator_config = true;
		}
	}
}

void VehicleIMUFifo::Run()
{
	const hrt_abstime now_us = hrt_absolute_time();

	// backup schedule
	ScheduleDelayed(_backup_schedule_timeout_us);

	ParametersUpdate();

	if (!_accel_calibration.enabled() || !_gyro_calibration.enabled()) {
		return;
	}

	// reset data gap monitor
	_data_gap = false;

	sensor_imu_fifo_s sensor_imu_fifo;

	while (_sensor_imu_fifo_sub.update(&sensor_imu_fifo)) {

		bool consume_all = !_intervals_configured || _data_gap;

		// monitor scheduling latency and force catch up with latest gyro if falling behind
		if (_sensor_imu_fifo_sub.updated() && (_update_latency_mean.count() > 100)
		    && (_update_latency_mean.mean()(1) > _interval_us * 1e-6f)) {

			PX4_INFO("update mean sample latency: %.6f, publish latency %.6f",
				 (double)_update_latency_mean.mean()(0),
				 (double)_update_latency_mean.mean()(1));

			consume_all = true;
		}

		if (_sensor_imu_fifo_sub.get_last_generation() != _last_generation + 1) {
			perf_count(_imu_generation_gap_perf);

			// reset average sample measurement
			_interval_mean.reset();

		} else {
			// collect sample interval average for filters
			if (sensor_imu_fifo.timestamp_sample > _timestamp_sample_last) {
				if ((_timestamp_sample_last != 0) && (sensor_imu_fifo.samples > 0)) {
					float interval_us = sensor_imu_fifo.timestamp_sample - _timestamp_sample_last;
					_interval_mean.update(Vector2f{interval_us, (float)sensor_imu_fifo.samples});
				}

			} else {
				PX4_ERR("%d - accel %" PRIu32 " timestamp error timestamp_sample: %" PRIu64 ", previous timestamp_sample: %" PRIu64,
					_instance, sensor_imu_fifo.device_id, sensor_imu_fifo.timestamp_sample, _timestamp_sample_last);
			}

			if (sensor_imu_fifo.timestamp < sensor_imu_fifo.timestamp_sample) {
				PX4_ERR("%d - accel %" PRIu32 " timestamp (%" PRIu64 ") < timestamp_sample (%" PRIu64 ")",
					_instance, sensor_imu_fifo.device_id, sensor_imu_fifo.timestamp, sensor_imu_fifo.timestamp_sample);
			}

			const int interval_count = _interval_mean.count();
			const float interval_variance = _interval_mean.variance()(0);

			// check measured interval periodically
			if ((_interval_mean.valid() && (interval_count % 10 == 0))
			    && (!PX4_ISFINITE(_interval_best_variance)
				|| (interval_variance < _interval_best_variance)
				|| (interval_count > 1000))) {

				const float interval_mean = _interval_mean.mean()(0);
				const float interval_mean_fifo = interval_mean / _interval_mean.mean()(1);

				// update sample rate if previously invalid or changed
				const float interval_delta_us = fabsf(interval_mean - _interval_us);
				const float percent_changed = interval_delta_us / _interval_us;

				if (!PX4_ISFINITE(_interval_us) || (percent_changed > 0.001f)) {
					if (PX4_ISFINITE(interval_mean) && PX4_ISFINITE(interval_mean_fifo) && PX4_ISFINITE(interval_variance)) {
						// update integrator configuration if interval has changed by more than 10%
						if (interval_delta_us > 0.1f * _interval_us) {
							_update_integrator_config = true;
						}

						_interval_us = interval_mean;
						_interval_samples = _interval_mean.mean()(1);

						_interval_best_variance = interval_variance;

						_status.accel_rate_hz = 1e6f / interval_mean;
						_status.gyro_rate_hz = 1e6f / interval_mean;

						_status.accel_raw_rate_hz = 1e6f / interval_mean_fifo; // FIFO
						_status.gyro_raw_rate_hz = 1e6f / interval_mean_fifo; // FIFO

						_publish_status = true;

					} else {
						_interval_mean.reset();
					}
				}
			}

			if (interval_count > 10000) {
				// reset periodically to prevent numerical issues
				_interval_mean.reset();
			}
		}

		_last_generation = _sensor_imu_fifo_sub.get_last_generation();

		_accel_calibration.set_device_id(sensor_imu_fifo.device_id);
		_gyro_calibration.set_device_id(sensor_imu_fifo.device_id);

		if (sensor_imu_fifo.error_count != _status.accel_error_count) {
			_publish_status = true;
			_status.accel_error_count = sensor_imu_fifo.error_count;
			_status.gyro_error_count = sensor_imu_fifo.error_count;
		}

		const int N = sensor_imu_fifo.samples;

		_timestamp_last = sensor_imu_fifo.timestamp;
		_timestamp_sample_last = sensor_imu_fifo.timestamp_sample;

		_temperature_sum += sensor_imu_fifo.temperature;
		_temperature_sum_count++;

		if (fabsf(sensor_imu_fifo.accel_scale - _accel_scale) > FLT_EPSILON) {
			// rescale last sample on scale change
			float rescale = _accel_scale / sensor_imu_fifo.accel_scale;

			for (auto &s : _last_accel_sample) {
				s = roundf(s * rescale);
			}

			_accel_scale = sensor_imu_fifo.accel_scale;
		}

		const float dt_s = sensor_imu_fifo.dt * 1e-6f;

		int last_sample = N - 1;

		if (!consume_all && (_gyro_integrator.integrated_samples() + N > _gyro_integrator.get_reset_samples())) {
			// TODO: publish early and then integrate the rest?
			if (_gyro_integrator.get_reset_samples() > _gyro_integrator.integrated_samples()) {
				int required_samples = _gyro_integrator.get_reset_samples() - _gyro_integrator.integrated_samples();

				if (required_samples < sensor_imu_fifo_s::FIFO_SIZE / 2) {
					last_sample = N - required_samples;

					PX4_DEBUG("intgrated samples (%d) + N (%d) > reset samples (%d)   %" PRIu8 " - IMU: %" PRIu32 ", Req samples: %d",
						  _gyro_integrator.integrated_samples(), N, _gyro_integrator.get_reset_samples(),
						  _instance, _accel_calibration.device_id(), required_samples);
				}
			}
		}

		IntegrateAccel(sensor_imu_fifo.accel_x, sensor_imu_fifo.accel_y, sensor_imu_fifo.accel_z, 0, last_sample);
		IntegrateGyro(sensor_imu_fifo.gyro_x, sensor_imu_fifo.gyro_y, sensor_imu_fifo.gyro_z, 0, last_sample,
			      sensor_imu_fifo.gyro_scale, dt_s);

		// reconfigure integrators if calculated sensor intervals have changed
		if (_update_integrator_config || !_intervals_configured) {
			UpdateIntegratorConfiguration();
		}

		// publish if both accel & gyro integrators are ready
		bool publish = _intervals_configured && _gyro_integrator.integral_ready();

		if (consume_all && _sensor_imu_fifo_sub.updated()) {
			publish = false;
		}

		if (publish) {
			Vector3f delta_angle;
			uint16_t delta_angle_dt;

			if (_gyro_integrator.reset(delta_angle, delta_angle_dt)) {
				Vector3f delta_velocity{_accel_integral *_accel_scale * dt_s};
				_accel_integral.zero();
				uint16_t delta_velocity_dt = delta_angle_dt;

				if (Publish(delta_angle, delta_angle_dt, delta_velocity, delta_velocity_dt)) {
					// record gyro publication latency and integrated samples
					if (_update_latency_mean.count() > 10000) {
						// reset periodically to avoid numerical issues
						_update_latency_mean.reset();
					}

					const float time_run_s = now_us * 1e-6f;

					_update_latency_mean.update(Vector2f{time_run_s - _timestamp_sample_last * 1e-6f, time_run_s - _timestamp_last * 1e-6f});


					// integrate leftover samples
					if (last_sample < N - 1) {
						IntegrateAccel(sensor_imu_fifo.accel_x, sensor_imu_fifo.accel_y, sensor_imu_fifo.accel_z, last_sample + 1, N - 1);
						IntegrateGyro(sensor_imu_fifo.gyro_x, sensor_imu_fifo.gyro_y, sensor_imu_fifo.gyro_z, last_sample + 1, N - 1,
							      sensor_imu_fifo.gyro_scale, dt_s);

					}


					return;
				}
			}
		}
	}
}

void VehicleIMUFifo::IntegrateAccel(const int16_t x[], const int16_t y[], const int16_t z[],
				    int first_sample, int last_sample)
{
	// integrate accel
	// trapezoidal integration (equally spaced)
	_accel_integral(0) += (0.5f * (_last_accel_sample[0] + x[last_sample]) + sum(x, first_sample, last_sample - 1));
	_accel_integral(1) += (0.5f * (_last_accel_sample[1] + y[last_sample]) + sum(y, first_sample, last_sample - 1));
	_accel_integral(2) += (0.5f * (_last_accel_sample[2] + z[last_sample]) + sum(z, first_sample, last_sample - 1));

	_last_accel_sample[0] = x[last_sample];
	_last_accel_sample[1] = y[last_sample];
	_last_accel_sample[2] = z[last_sample];

	int clip_counter[3] {
		clipping(x, first_sample, last_sample),
		clipping(y, first_sample, last_sample),
		clipping(z, first_sample, last_sample),
	};

	if (clip_counter[0] > 0 || clip_counter[1] > 0 || clip_counter[2] > 0) {
		// rotate sensor clip counts into vehicle body frame
		const Vector3f clipping{_accel_calibration.rotation() *Vector3f{(float)clip_counter[0], (float)clip_counter[1], (float)clip_counter[2]}};

		// round to get reasonble clip counts per axis (after board rotation)
		const uint8_t clip_x = roundf(fabsf(clipping(0)));
		const uint8_t clip_y = roundf(fabsf(clipping(1)));
		const uint8_t clip_z = roundf(fabsf(clipping(2)));

		_status.accel_clipping[0] += clip_x;
		_status.accel_clipping[1] += clip_y;
		_status.accel_clipping[2] += clip_z;

		if (clip_x > 0) {
			_delta_velocity_clipping |= vehicle_imu_s::CLIPPING_X;
		}

		if (clip_y > 0) {
			_delta_velocity_clipping |= vehicle_imu_s::CLIPPING_Y;
		}

		if (clip_z > 0) {
			_delta_velocity_clipping |= vehicle_imu_s::CLIPPING_Z;
		}

		if (_accel_calibration.enabled() && (hrt_elapsed_time(&_last_clipping_notify_time) > 3_s)) {
			// start notifying the user periodically if there's significant continuous clipping
			const uint64_t clipping_total = _status.accel_clipping[0] + _status.accel_clipping[1] + _status.accel_clipping[2];

			if (clipping_total > _last_clipping_notify_total_count + 1000) {
				mavlink_log_critical(&_mavlink_log_pub, "Accel %" PRIu8 " clipping, not safe to fly!\t", _instance);
				/* EVENT
				 * @description Land now, and check the vehicle setup.
				 * Clipping can lead to fly-aways.
				 */
				events::send<uint8_t>(events::ID("vehicle_imu_accel_clipping"), events::Log::Critical,
						      "Accel {1} clipping, not safe to fly!", _instance);
				_last_clipping_notify_time = hrt_absolute_time();
				_last_clipping_notify_total_count = clipping_total;
			}
		}
	}
}

void VehicleIMUFifo::IntegrateGyro(const int16_t x[], const int16_t y[], const int16_t z[], int first_sample,
				   int last_sample, float scale, float dt_s)
{
	for (int n = first_sample; n <= last_sample; n++) {
		Vector3f gyro_raw{(float)x[n] *scale, (float)y[n] *scale, (float)z[n] *scale};
		_gyro_integrator.put(gyro_raw, dt_s);
	}
}

bool VehicleIMUFifo::Publish(const Vector3f &delta_angle, uint16_t delta_angle_dt, const Vector3f &delta_velocity,
			     uint16_t delta_velocity_dt)
{
	bool updated = false;

	if (_accel_calibration.enabled() && _gyro_calibration.enabled()) {

		// delta angle: apply offsets, scale, and board rotation
		_gyro_calibration.SensorCorrectionsUpdate();
		const float gyro_dt_inv = 1.e6f / delta_angle_dt;
		const Vector3f angular_velocity{_gyro_calibration.Correct(delta_angle * gyro_dt_inv)};
		_gyro_sum += angular_velocity;
		_gyro_sum_count++;
		UpdateGyroVibrationMetrics(angular_velocity);
		const Vector3f delta_angle_corrected{angular_velocity / gyro_dt_inv};

		// delta velocity: apply offsets, scale, and board rotation
		_accel_calibration.SensorCorrectionsUpdate();
		const float accel_dt_inv = 1.e6f / delta_velocity_dt;
		const Vector3f acceleration{_accel_calibration.Correct(delta_velocity * accel_dt_inv)};
		_accel_sum += acceleration;
		_accel_sum_count++;
		UpdateAccelVibrationMetrics(acceleration);
		const Vector3f delta_velocity_corrected{acceleration / accel_dt_inv};

		// vehicle_imu_status
		//  publish before vehicle_imu so that error counts are available synchronously if needed
		if ((_accel_sum_count > 0) && (_gyro_sum_count > 0)
		    && (_publish_status || (hrt_elapsed_time(&_status.timestamp) >= 100_ms))) {

			_status.accel_device_id = _accel_calibration.device_id();
			_status.gyro_device_id = _gyro_calibration.device_id();

			// temperature
			float temperature = _temperature_sum / _temperature_sum_count;
			// reset
			_temperature_sum = 0.f;
			_temperature_sum_count = 0;


			// mean accel
			const Vector3f accel_mean{_accel_sum / _accel_sum_count};
			accel_mean.copyTo(_status.mean_accel);
			_status.temperature_accel = temperature;
			// reset
			_accel_sum.zero();
			_accel_sum_count = 0;


			// mean gyro
			const Vector3f gyro_mean{_gyro_sum / _gyro_sum_count};
			gyro_mean.copyTo(_status.mean_gyro);
			_status.temperature_gyro = temperature;
			// reset
			_gyro_sum.zero();
			_gyro_sum_count = 0;

			_status.timestamp = hrt_absolute_time();
			_vehicle_imu_status_pub.publish(_status);

			_publish_status = false;
		}

		// publish vehicle_imu
		vehicle_imu_s imu;
		imu.timestamp_sample = _timestamp_sample_last;
		imu.accel_device_id = _accel_calibration.device_id();
		imu.gyro_device_id = _gyro_calibration.device_id();
		delta_angle_corrected.copyTo(imu.delta_angle);
		delta_velocity_corrected.copyTo(imu.delta_velocity);
		imu.delta_angle_dt = delta_angle_dt;
		imu.delta_velocity_dt = delta_velocity_dt;
		imu.delta_velocity_clipping = _delta_velocity_clipping;
		imu.calibration_count = _accel_calibration.calibration_count() + _gyro_calibration.calibration_count();
		imu.timestamp = hrt_absolute_time();
		_vehicle_imu_pub.publish(imu);

		// reset clip counts
		_delta_velocity_clipping = 0;

		updated = true;
	}

	return updated;
}

void VehicleIMUFifo::UpdateIntegratorConfiguration()
{
	if (PX4_ISFINITE(_interval_us) && PX4_ISFINITE(_interval_us)) {

		// determine number of sensor samples that will get closest to the desired integration interval
		uint8_t gyro_integral_samples = math::max(1, (int)roundf(_imu_integration_interval_us / _interval_us));

		// if gyro samples exceeds queue depth, instead round to nearest even integer to improve scheduling options
		if (gyro_integral_samples > sensor_imu_fifo_s::ORB_QUEUE_LENGTH) {
			gyro_integral_samples = math::max(1, (int)roundf(_imu_integration_interval_us / _interval_us / 2) * 2);
		}

		//uint32_t integration_interval_us = roundf(gyro_integral_samples * _interval_us);

		_gyro_integrator.set_reset_interval(_imu_integration_interval_us);
		_gyro_integrator.set_reset_samples(gyro_integral_samples * _interval_samples);

		_backup_schedule_timeout_us = sensor_imu_fifo_s::ORB_QUEUE_LENGTH * _interval_us;

		// gyro: find largest integer multiple of gyro_integral_samples
		for (int n = sensor_imu_fifo_s::ORB_QUEUE_LENGTH; n > 0; n--) {
			if (gyro_integral_samples > sensor_imu_fifo_s::ORB_QUEUE_LENGTH) {
				gyro_integral_samples /= 2;
			}

			if (gyro_integral_samples % n == 0) {
				_sensor_imu_fifo_sub.set_required_updates(n);
				_sensor_imu_fifo_sub.registerCallback();

				_intervals_configured = true;
				_update_integrator_config = false;

				PX4_INFO("IMU FIFO (%" PRIu32 "), FIFO samples: %" PRIu8 ", interval: %.1f",
					 _accel_calibration.device_id(), gyro_integral_samples, (double)_interval_us);

				break;
			}
		}
	}
}

void VehicleIMUFifo::UpdateAccelVibrationMetrics(const Vector3f &acceleration)
{
	// Accel high frequency vibe = filtered length of (acceleration - acceleration_prev)
	_status.accel_vibration_metric = 0.99f * _status.accel_vibration_metric
					 + 0.01f * Vector3f(acceleration - _acceleration_prev).norm();

	_acceleration_prev = acceleration;
}

void VehicleIMUFifo::UpdateGyroVibrationMetrics(const Vector3f &angular_velocity)
{
	// Gyro high frequency vibe = filtered length of (angular_velocity - angular_velocity_prev)
	_status.gyro_vibration_metric = 0.99f * _status.gyro_vibration_metric
					+ 0.01f * Vector3f(angular_velocity - _angular_velocity_prev).norm();

	// Gyro delta angle coning metric = filtered length of (angular_velocity x angular_velocity_prev)
	const Vector3f coning_metric{angular_velocity % _angular_velocity_prev};
	_status.gyro_coning_vibration = 0.99f * _status.gyro_coning_vibration + 0.01f * coning_metric.norm();

	_angular_velocity_prev = angular_velocity;
}

void VehicleIMUFifo::PrintStatus()
{
	PX4_INFO("%" PRIu8 " - IMU: %" PRIu32 ", interval: %.1f us (SD %.1f us)",
		 _instance, _accel_calibration.device_id(), (double)_interval_us, (double)sqrtf(_interval_best_variance));

	PX4_INFO("gyro update mean sample latency: %.6f s, publish latency %.6f s, gyro interval %.6f s",
		 (double)_update_latency_mean.mean()(0),
		 (double)_update_latency_mean.mean()(1),
		 (double)(_interval_us * 1e-6f));

	perf_print_counter(_imu_generation_gap_perf);

	_accel_calibration.PrintStatus();
	_gyro_calibration.PrintStatus();
}

} // namespace sensors
