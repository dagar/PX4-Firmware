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

static constexpr int32_t sum(const int16_t samples[], uint8_t len)
{
	int32_t sum = 0;

	for (int n = 0; n < len; n++) {
		sum += samples[n];
	}

	return sum;
}

VehicleIMUFifo::VehicleIMUFifo(int instance, uint32_t accel_device_id, uint32_t gyro_device_id,
			       const px4::wq_config_t &config) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, config),
	_instance(instance)
{
	_imu_integration_interval_us = 1e6f / _param_imu_integ_rate.get();

	for (uint8_t i = 0; i < ORB_MULTI_MAX_INSTANCES; i++) {
		// accel
		uORB::SubscriptionData<sensor_accel_fifo_s> sensor_accel_fifo_sub{ORB_ID(sensor_accel_fifo), i};

		if (sensor_accel_fifo_sub.get().device_id == accel_device_id) {
			_sensor_accel_fifo_sub.ChangeInstance(i);
			_accel_scale = sensor_accel_fifo_sub.get().scale;
		}

		// gyro
		uORB::SubscriptionData<sensor_gyro_fifo_s> sensor_gyro_fifo_sub{ORB_ID(sensor_gyro_fifo), i};

		if (sensor_gyro_fifo_sub.get().device_id == gyro_device_id) {
			_sensor_gyro_fifo_sub.ChangeInstance(i);
			_gyro_scale = sensor_gyro_fifo_sub.get().scale;
		}
	}

	_accel_integrator.set_reset_interval(_imu_integration_interval_us);
	_accel_integrator.set_reset_samples(sensor_accel_fifo_s::ORB_QUEUE_LENGTH);

	_gyro_integrator.set_reset_interval(_imu_integration_interval_us);
	_gyro_integrator.set_reset_samples(sensor_gyro_fifo_s::ORB_QUEUE_LENGTH);

#if defined(ENABLE_LOCKSTEP_SCHEDULER)
	// currently with lockstep every raw sample needs a corresponding vehicle_imu publication
	_sensor_gyro_fifo_sub.set_required_updates(1);
#else
	// schedule conservatively until the actual accel & gyro rates are known
	_sensor_gyro_fifo_sub.set_required_updates(sensor_gyro_fifo_s::ORB_QUEUE_LENGTH / 2);
#endif

	// advertise immediately to ensure consistent ordering
	_vehicle_imu_pub.advertise();
	_vehicle_imu_status_pub.advertise();
}

VehicleIMUFifo::~VehicleIMUFifo()
{
	Stop();

	perf_free(_accel_generation_gap_perf);
	perf_free(_gyro_generation_gap_perf);

	_vehicle_imu_pub.unadvertise();
	_vehicle_imu_status_pub.unadvertise();
}

bool VehicleIMUFifo::Start()
{
	// force initial updates
	ParametersUpdate(true);

	_sensor_gyro_fifo_sub.registerCallback();

	ScheduleNow();
	return true;
}

void VehicleIMUFifo::Stop()
{
	// clear all registered callbacks
	_sensor_gyro_fifo_sub.unregisterCallback();

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

bool VehicleIMUFifo::AccelAvailable()
{
	return _sensor_accel_fifo_sub.updated();
}

bool VehicleIMUFifo::GyroAvailable()
{
	return _sensor_gyro_fifo_sub.updated();
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

	while (GyroAvailable() || AccelAvailable()) {
		bool updated = false;

		bool consume_all_gyro = !_intervals_configured || _data_gap;

		// monitor scheduling latency and force catch up with latest gyro if falling behind
		if (GyroAvailable() && (_gyro_update_latency_mean.count() > 100)
		    && (_gyro_update_latency_mean.mean()(1) > _gyro_interval_us * 1e-6f)) {

			PX4_DEBUG("gyro update mean sample latency: %.6f, publish latency %.6f",
				  (double)_gyro_update_latency_mean.mean()(0),
				  (double)_gyro_update_latency_mean.mean()(1));

			consume_all_gyro = true;
		}

		// update gyro until integrator ready and not falling behind
		if (!_gyro_integrator.integral_ready() || consume_all_gyro) {
			if (UpdateGyroFifo()) {
				updated = true;
			}
		}


		// update accel until integrator ready and caught up to gyro
		// TODO:
		while (AccelAvailable()
		       && (!_accel_integrator.integral_ready() || !_intervals_configured || _data_gap
			   || (_accel_timestamp_sample_last < (_gyro_timestamp_sample_last - 0.5f * _accel_interval_us)))) {

			if (UpdateAccelFifo()) {
				updated = true;
			}
		}

		// reconfigure integrators if calculated sensor intervals have changed
		if (_update_integrator_config || !_intervals_configured) {
			UpdateIntegratorConfiguration();
		}

		// check for additional updates and that we're fully caught up before publishing
		if ((consume_all_gyro || _data_gap) && GyroAvailable()) {
			continue;
		}

		// publish if both accel & gyro integrators are ready
		if (_intervals_configured && _accel_integrator.integral_ready() && _gyro_integrator.integral_ready()) {
			if (Publish()) {
				// record gyro publication latency and integrated samples
				if (_gyro_update_latency_mean.count() > 10000) {
					// reset periodically to avoid numerical issues
					_gyro_update_latency_mean.reset();
				}

				const float time_run_s = now_us * 1e-6f;

				_gyro_update_latency_mean.update(Vector2f{time_run_s - _gyro_timestamp_sample_last * 1e-6f, time_run_s - _gyro_timestamp_last * 1e-6f});

				return;
			}
		}

		// finish if there are no more updates, but didn't publish
		if (!updated) {
			return;
		}
	}
}

bool VehicleIMUFifo::UpdateAccelFifo()
{
	bool updated = false;

	// update accel, stopping once caught up to the last gyro sample
	sensor_accel_fifo_s accel_fifo;

	if (_sensor_accel_fifo_sub.update(&accel_fifo)) {
		if (_sensor_accel_fifo_sub.get_last_generation() != _accel_last_generation + 1) {
			_data_gap = true;
			perf_count(_accel_generation_gap_perf);

			// reset average sample measurement
			_accel_interval_mean.reset();

		} else {
			// collect sample interval average for filters
			if (accel_fifo.timestamp_sample > _accel_timestamp_sample_last) {
				if ((_accel_timestamp_sample_last != 0) && (accel_fifo.samples > 0)) {
					float interval_us = accel_fifo.timestamp_sample - _accel_timestamp_sample_last;
					_accel_interval_mean.update(Vector2f{interval_us, (float)accel_fifo.samples});
				}

			} else {
				PX4_ERR("%d - accel %" PRIu32 " timestamp error timestamp_sample: %" PRIu64 ", previous timestamp_sample: %" PRIu64,
					_instance, accel_fifo.device_id, accel_fifo.timestamp_sample, _accel_timestamp_sample_last);
			}

			if (accel_fifo.timestamp < accel_fifo.timestamp_sample) {
				PX4_ERR("%d - accel %" PRIu32 " timestamp (%" PRIu64 ") < timestamp_sample (%" PRIu64 ")",
					_instance, accel_fifo.device_id, accel_fifo.timestamp, accel_fifo.timestamp_sample);
			}

			const int interval_count = _accel_interval_mean.count();
			const float interval_variance = _accel_interval_mean.variance()(0);

			// check measured interval periodically
			if ((_accel_interval_mean.valid() && (interval_count % 10 == 0))
			    && (!PX4_ISFINITE(_accel_interval_best_variance)
				|| (interval_variance < _accel_interval_best_variance)
				|| (interval_count > 1000))) {

				const float interval_mean = _accel_interval_mean.mean()(0);
				const float interval_mean_fifo = interval_mean / _accel_interval_mean.mean()(1);

				// update sample rate if previously invalid or changed
				const float interval_delta_us = fabsf(interval_mean - _accel_interval_us);
				const float percent_changed = interval_delta_us / _accel_interval_us;

				if (!PX4_ISFINITE(_accel_interval_us) || (percent_changed > 0.001f)) {
					if (PX4_ISFINITE(interval_mean) && PX4_ISFINITE(interval_mean_fifo) && PX4_ISFINITE(interval_variance)) {
						// update integrator configuration if interval has changed by more than 10%
						if (interval_delta_us > 0.1f * _accel_interval_us) {
							_update_integrator_config = true;
						}

						_accel_interval_us = interval_mean;
						_accel_interval_samples = _accel_interval_mean.mean()(1);

						_accel_interval_best_variance = interval_variance;

						_status.accel_rate_hz = 1e6f / interval_mean;
						_status.accel_raw_rate_hz = 1e6f / interval_mean_fifo; // FIFO

						_publish_status = true;

					} else {
						_accel_interval_mean.reset();
					}
				}
			}

			if (interval_count > 10000) {
				// reset periodically to prevent numerical issues
				_accel_interval_mean.reset();
			}
		}

		_accel_last_generation = _sensor_accel_fifo_sub.get_last_generation();

		_accel_calibration.set_device_id(accel_fifo.device_id);

		if (accel_fifo.error_count != _status.accel_error_count) {
			_publish_status = true;
			_status.accel_error_count = accel_fifo.error_count;
		}

		const int N = accel_fifo.samples;
		const float dt_s = accel_fifo.dt * 1e-6f;

		_accel_timestamp_sample_last = accel_fifo.timestamp_sample;


		// TODO:
		//  is gyro ready to publish?
		//    yes
		//     - determine number of accel samples to integrate
		//     - integrate them all

		if (fabsf(_accel_scale - accel_fifo.scale) > 0.f) {
			// scale changed
			_accel_scale = accel_fifo.scale;
		}

		if (_gyro_integrator.integral_ready()) {

			_gyro_integrator.integral_dt();

			_accel_integrator.integral_dt();

			// N * accel_fifo.dt;


		} else {
			// trapezoidal integration (equally spaced)
			const float scale = _accel_scale / (float)N;
			_accel_integration(0) += (0.5f * (_accel_last_sample[0] + accel_fifo.x[N - 1]) + sum(accel_fifo.x, N - 1)) * scale;
			_accel_integration(1) += (0.5f * (_accel_last_sample[1] + accel_fifo.y[N - 1]) + sum(accel_fifo.y, N - 1)) * scale;
			_accel_integration(2) += (0.5f * (_accel_last_sample[2] + accel_fifo.z[N - 1]) + sum(accel_fifo.z, N - 1)) * scale;

			_accel_last_sample[0] = accel_fifo.x[N - 1];
			_accel_last_sample[1] = accel_fifo.y[N - 1];
			_accel_last_sample[2] = accel_fifo.z[N - 1];
		}






		for (int n = 0; n < N; n++) {
			// TODO: integrate raw data, handle scale change, and align with gyro perfectly
			Vector3f accel_raw{(float)accel_fifo.x[n] *accel_fifo.scale, (float)accel_fifo.y[n] *accel_fifo.scale, (float)accel_fifo.z[n] *accel_fifo.scale};

			_accel_integrator.put(accel_raw, dt_s);

			// TODO: move to publish?
			_accel_sum += accel_raw;
			_accel_temperature += accel_fifo.temperature;
			_accel_sum_count++;

			// break once caught up to gyro
			// const float interval_s = _accel_interval.update_interval_raw * 1e-6f;

			// if (_accel_integrator.integral_ready() && _gyro_integrator.integral_ready() && _intervals_configured
			//     && (_accel_integrator.integral_dt() >= (_gyro_integrator.integral_dt() - 0.5f * interval_s))) {

			// 	Publish();
			// }
		}

		// TODO: clipping
		// // clipping
		// uint8_t clip_count[3] {
		// 	clipping(sample.x, _clip_limit, N),
		// 	clipping(sample.y, _clip_limit, N),
		// 	clipping(sample.z, _clip_limit, N),
		// };

		// break once caught up to gyro
		// if (!_sensor_data_gap && _intervals_configured
		//     && (_last_timestamp_sample_accel >= (_last_timestamp_sample_gyro - 0.5f * _accel_interval.update_interval))) {

		// 	break;
		// }

		updated = true;
	}

	return updated;
}

bool VehicleIMUFifo::UpdateGyroFifo()
{
	bool updated = false;

	// integrate queued gyro
	sensor_gyro_fifo_s gyro_fifo;

	if (_sensor_gyro_fifo_sub.update(&gyro_fifo)) {
		if (_sensor_gyro_fifo_sub.get_last_generation() != _gyro_last_generation + 1) {
			_data_gap = true;
			perf_count(_gyro_generation_gap_perf);

			// reset average sample measurement
			_gyro_interval_mean.reset();

		} else {
			// collect sample interval average for filters
			if (gyro_fifo.timestamp_sample > _gyro_timestamp_sample_last) {
				if ((_gyro_timestamp_sample_last != 0) && (gyro_fifo.samples > 0)) {
					float interval_us = gyro_fifo.timestamp_sample - _gyro_timestamp_sample_last;
					_gyro_interval_mean.update(Vector2f{interval_us, (float)gyro_fifo.samples});
				}

			} else {
				PX4_ERR("%d - gyro %" PRIu32 " timestamp error timestamp_sample: %" PRIu64 ", previous timestamp_sample: %" PRIu64,
					_instance, gyro_fifo.device_id, gyro_fifo.timestamp_sample, _gyro_timestamp_sample_last);
			}

			if (gyro_fifo.timestamp < gyro_fifo.timestamp_sample) {
				PX4_ERR("%d - gyro %" PRIu32 " timestamp (%" PRIu64 ") < timestamp_sample (%" PRIu64 ")",
					_instance, gyro_fifo.device_id, gyro_fifo.timestamp, gyro_fifo.timestamp_sample);
			}

			const int interval_count = _gyro_interval_mean.count();
			const float interval_variance = _gyro_interval_mean.variance()(0);

			// check measured interval periodically
			if ((_gyro_interval_mean.valid() && (interval_count % 10 == 0))
			    && (!PX4_ISFINITE(_gyro_interval_best_variance)
				|| (interval_variance < _gyro_interval_best_variance)
				|| (interval_count > 1000))) {

				const float interval_mean = _gyro_interval_mean.mean()(0);
				const float interval_mean_fifo = interval_mean / _gyro_interval_mean.mean()(1);

				// update sample rate if previously invalid or changed
				const float interval_delta_us = fabsf(interval_mean - _gyro_interval_us);
				const float percent_changed = interval_delta_us / _gyro_interval_us;

				if (!PX4_ISFINITE(_gyro_interval_us) || (percent_changed > 0.001f)) {
					if (PX4_ISFINITE(interval_mean) && PX4_ISFINITE(interval_mean_fifo) && PX4_ISFINITE(interval_variance)) {
						// update integrator configuration if interval has changed by more than 10%
						if (interval_delta_us > 0.1f * _gyro_interval_us) {
							_update_integrator_config = true;
						}

						_gyro_interval_us = interval_mean;
						_gyro_interval_samples = _gyro_interval_mean.mean()(1);

						_gyro_interval_best_variance = interval_variance;

						_status.gyro_rate_hz = 1e6f / interval_mean;
						_status.gyro_raw_rate_hz = 1e6f / interval_mean_fifo; // FIFO

						_publish_status = true;

					} else {
						_gyro_interval_mean.reset();
					}
				}
			}

			if (interval_count > 10000) {
				// reset periodically to prevent numerical issues
				_gyro_interval_mean.reset();
			}
		}

		_gyro_last_generation = _sensor_gyro_fifo_sub.get_last_generation();
		_gyro_timestamp_last = gyro_fifo.timestamp;

		_gyro_calibration.set_device_id(gyro_fifo.device_id);

		if (gyro_fifo.error_count != _status.gyro_error_count) {
			_publish_status = true;
			_status.gyro_error_count = gyro_fifo.error_count;
		}

		const int N = gyro_fifo.samples;
		const float dt_s = gyro_fifo.dt * 1e-6f;

		for (int n = 0; n < N; n++) {
			Vector3f gyro_raw{(float)gyro_fifo.x[n] *gyro_fifo.scale, (float)gyro_fifo.y[n] *gyro_fifo.scale, (float)gyro_fifo.z[n] *gyro_fifo.scale};

			_gyro_integrator.put(gyro_raw, dt_s);

			_gyro_sum += gyro_raw;
			_gyro_temperature += gyro_fifo.temperature;
			_gyro_sum_count++;
		}

		_gyro_timestamp_sample_last = gyro_fifo.timestamp_sample;

		updated = true;
	}


	return updated;
}

bool VehicleIMUFifo::Publish()
{
	bool updated = false;

	vehicle_imu_s imu;
	Vector3f delta_angle;
	Vector3f delta_velocity;

	if (_accel_integrator.reset(delta_velocity, imu.delta_velocity_dt)
	    && _gyro_integrator.reset(delta_angle, imu.delta_angle_dt)) {

		if (_accel_calibration.enabled() && _gyro_calibration.enabled()) {

			// delta angle: apply offsets, scale, and board rotation
			_gyro_calibration.SensorCorrectionsUpdate();
			const float gyro_dt_inv = 1.e6f / imu.delta_angle_dt;
			const Vector3f angular_velocity{_gyro_calibration.Correct(delta_angle * gyro_dt_inv)};
			UpdateGyroVibrationMetrics(angular_velocity);
			const Vector3f delta_angle_corrected{angular_velocity / gyro_dt_inv};

			// delta velocity: apply offsets, scale, and board rotation
			_accel_calibration.SensorCorrectionsUpdate();
			const float accel_dt_inv = 1.e6f / imu.delta_velocity_dt;
			const Vector3f acceleration{_accel_calibration.Correct(delta_velocity * accel_dt_inv)};
			UpdateAccelVibrationMetrics(acceleration);
			const Vector3f delta_velocity_corrected{acceleration / accel_dt_inv};

			// vehicle_imu_status
			//  publish before vehicle_imu so that error counts are available synchronously if needed
			if ((_accel_sum_count > 0) && (_gyro_sum_count > 0)
			    && (_publish_status || (hrt_elapsed_time(&_status.timestamp) >= 100_ms))) {

				_status.accel_device_id = _accel_calibration.device_id();
				_status.gyro_device_id = _gyro_calibration.device_id();

				// mean accel
				const Vector3f accel_mean{_accel_calibration.Correct(_accel_sum / _accel_sum_count)};
				accel_mean.copyTo(_status.mean_accel);
				_status.temperature_accel = _accel_temperature / _accel_sum_count;
				_accel_sum.zero();
				_accel_temperature = 0;
				_accel_sum_count = 0;

				// mean gyro
				const Vector3f gyro_mean{_gyro_calibration.Correct(_gyro_sum / _gyro_sum_count)};
				gyro_mean.copyTo(_status.mean_gyro);
				_status.temperature_gyro = _gyro_temperature / _gyro_sum_count;
				_gyro_sum.zero();
				_gyro_temperature = 0;
				_gyro_sum_count = 0;

				_status.timestamp = hrt_absolute_time();
				_vehicle_imu_status_pub.publish(_status);

				_publish_status = false;
			}

			// publish vehicle_imu
			imu.timestamp_sample = _gyro_timestamp_sample_last;
			imu.accel_device_id = _accel_calibration.device_id();
			imu.gyro_device_id = _gyro_calibration.device_id();
			delta_angle_corrected.copyTo(imu.delta_angle);
			delta_velocity_corrected.copyTo(imu.delta_velocity);
			imu.delta_velocity_clipping = _delta_velocity_clipping;
			imu.calibration_count = _accel_calibration.calibration_count() + _gyro_calibration.calibration_count();
			imu.timestamp = hrt_absolute_time();
			_vehicle_imu_pub.publish(imu);

			// reset clip counts
			_delta_velocity_clipping = 0;

			updated = true;
		}
	}

	return updated;
}

void VehicleIMUFifo::UpdateIntegratorConfiguration()
{
	if (PX4_ISFINITE(_accel_interval_us) && PX4_ISFINITE(_gyro_interval_us)) {

		// determine number of sensor samples that will get closest to the desired integration interval
		uint8_t gyro_integral_samples = math::max(1, (int)roundf(_imu_integration_interval_us / _gyro_interval_us));

		// if gyro samples exceeds queue depth, instead round to nearest even integer to improve scheduling options
		if (gyro_integral_samples > sensor_gyro_fifo_s::ORB_QUEUE_LENGTH) {
			gyro_integral_samples = math::max(1, (int)roundf(_imu_integration_interval_us / _gyro_interval_us / 2) * 2);
		}

		uint32_t integration_interval_us = roundf(gyro_integral_samples * _gyro_interval_us);

		// accel follows gyro as closely as possible
		uint8_t accel_integral_samples = math::max(1, (int)roundf(integration_interval_us / _accel_interval_us));

		// let the gyro set the configuration and scheduling
		// relaxed minimum integration time required
		_accel_integrator.set_reset_interval(roundf((accel_integral_samples - 0.5f) * _accel_interval_us));
		_accel_integrator.set_reset_samples(accel_integral_samples * _accel_interval_samples);

		_gyro_integrator.set_reset_interval(roundf((gyro_integral_samples - 0.5f) * _gyro_interval_us));
		_gyro_integrator.set_reset_samples(gyro_integral_samples * _gyro_interval_samples);

		_backup_schedule_timeout_us = math::min(sensor_accel_fifo_s::ORB_QUEUE_LENGTH * _accel_interval_us,
							sensor_gyro_fifo_s::ORB_QUEUE_LENGTH * _gyro_interval_us);

		// gyro: find largest integer multiple of gyro_integral_samples
		for (int n = sensor_gyro_fifo_s::ORB_QUEUE_LENGTH; n > 0; n--) {
			if (gyro_integral_samples > sensor_gyro_fifo_s::ORB_QUEUE_LENGTH) {
				gyro_integral_samples /= 2;
			}

			if (gyro_integral_samples % n == 0) {
				_sensor_gyro_fifo_sub.set_required_updates(n);
				_sensor_gyro_fifo_sub.registerCallback();

				_intervals_configured = true;
				_update_integrator_config = false;

				PX4_INFO("accel FIFO (%" PRIu32 "), gyro FIFO (%" PRIu32 "), accel samples: %" PRIu8 ", gyro FIFO samples: %" PRIu8
					 ", accel interval: %.1f, gyro interval: %.1f sub samples: %d",
					 _accel_calibration.device_id(), _gyro_calibration.device_id(), accel_integral_samples, gyro_integral_samples,
					 (double)_accel_interval_us, (double)_gyro_interval_us, n);

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
	PX4_INFO("%" PRIu8 " - Accel: %" PRIu32 ", interval: %.1f us (SD %.1f us), Gyro: %" PRIu32
		 ", interval: %.1f us (SD %.1f us)",
		 _instance,
		 _accel_calibration.device_id(), (double)_accel_interval_us, (double)sqrtf(_accel_interval_best_variance),
		 _gyro_calibration.device_id(), (double)_gyro_interval_us, (double)sqrtf(_gyro_interval_best_variance));

	PX4_DEBUG("gyro update mean sample latency: %.6f s, publish latency %.6f s, gyro interval %.6f s",
		  (double)_gyro_update_latency_mean.mean()(0), (double)_gyro_update_latency_mean.mean()(1),
		  (double)(_gyro_interval_us * 1e-6f));

	perf_print_counter(_accel_generation_gap_perf);
	perf_print_counter(_gyro_generation_gap_perf);

	_accel_calibration.PrintStatus();
	_gyro_calibration.PrintStatus();
}

} // namespace sensors
