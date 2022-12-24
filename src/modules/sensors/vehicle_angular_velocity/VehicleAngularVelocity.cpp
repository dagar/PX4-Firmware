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

#include "VehicleAngularVelocity.hpp"

#include <px4_platform_common/log.h>

#include <uORB/topics/vehicle_imu_status.h>

using namespace matrix;

namespace sensors
{

static constexpr int clipping(const int16_t sample) { return (sample == INT16_MIN) || (sample == INT16_MAX); }

static constexpr int clipping(const int16_t samples[], int length)
{
	int clip_count = 0;

	for (int n = 0; n < length; n++) {
		if ((samples[n] == INT16_MIN) || (samples[n] == INT16_MAX)) {
			clip_count++;
		}
	}

	return clip_count;
}

VehicleAngularVelocity::VehicleAngularVelocity() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
{
	_vehicle_angular_velocity_pub.advertise();
}

VehicleAngularVelocity::~VehicleAngularVelocity()
{
	Stop();

	perf_free(_cycle_perf);
	perf_free(_filter_reset_perf);
	perf_free(_selection_changed_perf);

#if !defined(CONSTRAINED_FLASH)
	delete[] _dynamic_notch_filter_esc_rpm;
	perf_free(_dynamic_notch_filter_esc_rpm_disable_perf);
	perf_free(_dynamic_notch_filter_esc_rpm_init_perf);
	perf_free(_dynamic_notch_filter_esc_rpm_update_perf);

	perf_free(_dynamic_notch_filter_fft_disable_perf);
	perf_free(_dynamic_notch_filter_fft_update_perf);
#endif // CONSTRAINED_FLASH
}

bool VehicleAngularVelocity::Start()
{
	// force initial updates
	ParametersUpdate(true);

	// sensor_selection needed to change the active sensor if the primary stops updating
	if (!_sensor_selection_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	if (!SensorSelectionUpdate(true)) {
		ScheduleNow();
	}

	return true;
}

void VehicleAngularVelocity::Stop()
{
	// clear all registered callbacks
	_sensor_gyro_sub.unregisterCallback();
	_sensor_imu_fifo_sub.unregisterCallback();
	_sensor_selection_sub.unregisterCallback();

	Deinit();
}

bool VehicleAngularVelocity::UpdateSampleRate()
{
	float sample_rate_hz = NAN;
	float publish_rate_hz = NAN;

	for (uint8_t i = 0; i < MAX_SENSOR_COUNT; i++) {
		uORB::SubscriptionData<vehicle_imu_status_s> imu_status{ORB_ID(vehicle_imu_status), i};

		if (imu_status.get().gyro_device_id == _selected_gyro_device_id) {
			sample_rate_hz = imu_status.get().gyro_raw_rate_hz;
			publish_rate_hz = imu_status.get().gyro_rate_hz;
			break;
		}
	}

	// calculate sensor update rate
	if (PX4_ISFINITE(sample_rate_hz) && (sample_rate_hz > 10) && (sample_rate_hz < 10'000)
	    && PX4_ISFINITE(publish_rate_hz) && (publish_rate_hz > 0)
	   ) {
		// check if sample rate error is greater than 1%
		const bool sample_rate_changed = (fabsf(sample_rate_hz - _filter_sample_rate_hz) / sample_rate_hz) > 0.01f;

		if (_update_sample_rate || sample_rate_changed
		    || (_filter_sample_rate_hz <= FLT_EPSILON) || !PX4_ISFINITE(_filter_sample_rate_hz)) {

			PX4_DEBUG("updating sample rate: %.3f Hz -> %.3f Hz", (double)_filter_sample_rate_hz, (double)sample_rate_hz);

			if (sample_rate_changed || !PX4_ISFINITE(_filter_sample_rate_hz)) {
				_reset_filters = true;
			}

			_filter_sample_rate_hz = sample_rate_hz;
			_update_sample_rate = false;

			if (_param_imu_gyro_ratemax.get() > 0.f) {
				// determine number of sensor samples that will get closest to the desired rate
				const float configured_interval_us = 1e6f / _param_imu_gyro_ratemax.get();
				const float publish_interval_us = 1e6f / publish_rate_hz;

				const uint8_t samples = roundf(configured_interval_us / publish_interval_us);

				if (_fifo_available) {
					_sensor_imu_fifo_sub.set_required_updates(math::constrain(samples, (uint8_t)1, sensor_imu_fifo_s::ORB_QUEUE_LENGTH));

				} else {
					_sensor_gyro_sub.set_required_updates(math::constrain(samples, (uint8_t)1, sensor_gyro_s::ORB_QUEUE_LENGTH));
				}

				// publish interval (constrained 100 Hz - 8 kHz)
				_publish_interval_min_us = math::constrain((int)roundf(configured_interval_us - (publish_interval_us * 0.5f)), 125,
							   10000);

			} else {
				_sensor_gyro_sub.set_required_updates(1);
				_sensor_imu_fifo_sub.set_required_updates(1);
				_publish_interval_min_us = 0;
			}
		}
	}

	return PX4_ISFINITE(_filter_sample_rate_hz) && (_filter_sample_rate_hz > 0);
}

void VehicleAngularVelocity::ResetFilters(const hrt_abstime &time_now_us)
{
	if ((_filter_sample_rate_hz > 0) && PX4_ISFINITE(_filter_sample_rate_hz)) {

		// accel filters
		const Vector3f acceleration_uncalibrated{GetResetAcceleration()};

		PX4_INFO("reset acceleration [%.5f, %.5f, %.5f]", (double)acceleration_uncalibrated(0),
			 (double)acceleration_uncalibrated(1), (double)acceleration_uncalibrated(2));

		for (int axis = 0; axis < 3; axis++) {
			// acceleration low pass
			_accel_lp_filter[axis].set_cutoff_frequency(_filter_sample_rate_hz, _param_imu_accel_cutoff.get());
			_accel_lp_filter[axis].reset(acceleration_uncalibrated(axis));
		}

		// gyro filters
		const Vector3f angular_velocity_uncalibrated{GetResetAngularVelocity()};
		const Vector3f angular_acceleration_uncalibrated{GetResetAngularAcceleration()};

		for (int axis = 0; axis < 3; axis++) {
			// angular velocity low pass
			_gyro_lp_filter_velocity[axis].set_cutoff_frequency(_filter_sample_rate_hz, _param_imu_gyro_cutoff.get());
			_gyro_lp_filter_velocity[axis].reset(angular_velocity_uncalibrated(axis));

			// angular velocity notch 0
			_gyro_notch_filter0_velocity[axis].setParameters(_filter_sample_rate_hz, _param_imu_gyro_nf0_frq.get(),
					_param_imu_gyro_nf0_bw.get());
			_gyro_notch_filter0_velocity[axis].reset();

			// angular velocity notch 1
			_gyro_notch_filter1_velocity[axis].setParameters(_filter_sample_rate_hz, _param_imu_gyro_nf1_frq.get(),
					_param_imu_gyro_nf1_bw.get());
			_gyro_notch_filter1_velocity[axis].reset();

			// angular acceleration low pass
			if ((_param_imu_dgyro_cutoff.get() > 0.f)
			    && (_gyro_derivative_lp_filter[axis].setCutoffFreq(_filter_sample_rate_hz, _param_imu_dgyro_cutoff.get()))) {
				_gyro_derivative_lp_filter[axis].reset(angular_acceleration_uncalibrated(axis));

			} else {
				// disable filtering
				_gyro_derivative_lp_filter[axis].setAlpha(1.f);
			}
		}

		// force reset notch filters on any scale change
		UpdateDynamicNotchEscRpm(time_now_us, true);
		UpdateDynamicNotchFFT(time_now_us, true);

		_angular_velocity_raw_prev = angular_velocity_uncalibrated;

		_reset_filters = false;
		perf_count(_filter_reset_perf);
	}
}

void VehicleAngularVelocity::SensorBiasUpdate(bool force)
{
	// find corresponding estimated sensor bias
	if (_estimator_selector_status_sub.updated()) {
		estimator_selector_status_s estimator_selector_status;

		if (_estimator_selector_status_sub.copy(&estimator_selector_status)) {
			//_estimator_sensor_bias_sub.ChangeInstance(estimator_selector_status.primary_instance);
		}
	}

	if (_estimator_sensor_bias_subs.updated() || force) {
		for (auto &sub : _estimator_sensor_bias_subs) {
			estimator_sensor_bias_s bias;

			if (sub.copy(&bias)) {

				if ((bias.accel_device_id == _selected_accel_device_id) && bias.accel_bias_valid) {
					_accel_bias = Vector3f{bias.accel_bias};

				} else {
					//_accel_bias.zero();
				}

				if ((bias.gyro_device_id == _selected_gyro_device_id) && bias.gyro_bias_valid) {
					_gyro_bias = Vector3f{bias.gyro_bias};

				} else {
					//_gyro_bias.zero();
				}
			}
		}
	}
}

bool VehicleAngularVelocity::SensorSelectionUpdate(const hrt_abstime &time_now_us, bool force)
{
	if (_sensor_selection_sub.updated() || (_selected_gyro_device_id == 0) || force) {
		sensor_selection_s sensor_selection{};
		_sensor_selection_sub.copy(&sensor_selection);

		bool selected_device_id_valid = false;
		uint32_t device_id = sensor_selection.gyro_device_id;
		uint32_t device_id_first_valid_imu = 0;

		// use vehicle_imu_status to do basic sensor selection validation
		for (uint8_t i = 0; i < MAX_SENSOR_COUNT; i++) {
			uORB::SubscriptionData<vehicle_imu_status_s> imu_status{ORB_ID(vehicle_imu_status), i};

			if (imu_status.advertised()
			    && (imu_status.get().timestamp != 0) && (time_now_us < imu_status.get().timestamp + 1_s)
			    && (imu_status.get().gyro_device_id != 0)) {
				// vehicle_imu_status gyro valid

				if ((device_id != 0) && (imu_status.get().gyro_device_id == device_id)) {
					selected_device_id_valid = true;
				}

				// record first valid IMU as a backup option
				if (device_id_first_valid_imu == 0) {
					device_id_first_valid_imu = imu_status.get().gyro_device_id;
				}
			}
		}

		// if no gyro selected or healthy then use fallback
		if ((device_id == 0) || !selected_device_id_valid) {
			device_id = device_id_first_valid_imu;
		}

		if ((_selected_gyro_device_id != device_id) || force) {

			const bool device_id_valid = (device_id != 0);

			// see if the selected sensor publishes sensor_imu_fifo
			for (uint8_t i = 0; i < MAX_SENSOR_COUNT; i++) {
				uORB::SubscriptionData<sensor_imu_fifo_s> sensor_imu_fifo_sub{ORB_ID(sensor_imu_fifo), i};

				if (sensor_imu_fifo_sub.advertised()
				    && (sensor_imu_fifo_sub.get().timestamp != 0)
				    && (sensor_imu_fifo_sub.get().device_id != 0)
				    && (time_now_us < sensor_imu_fifo_sub.get().timestamp + 1_s)) {

					// if no gyro was selected use the first valid sensor_gyro_fifo
					if (!device_id_valid) {
						device_id = sensor_imu_fifo_sub.get().device_id;
						PX4_WARN("no gyro selected, using sensor_imu_fifo:%" PRIu8 " %" PRIu32, i, sensor_imu_fifo_sub.get().device_id);
					}

					if (sensor_imu_fifo_sub.get().device_id == device_id) {
						if (_sensor_imu_fifo_sub.ChangeInstance(i) && _sensor_imu_fifo_sub.registerCallback()) {
							// make sure non-FIFO sub is unregistered
							_sensor_gyro_sub.unregisterCallback();

							_gyro_calibration.set_device_id(sensor_imu_fifo_sub.get().device_id);

							_selected_gyro_device_id = sensor_imu_fifo_sub.get().device_id;

							_timestamp_sample_last = 0;
							_filter_sample_rate_hz = 1.f / (sensor_imu_fifo_sub.get().dt * 1e-6f);
							_update_sample_rate = true;
							_reset_filters = true;
							_gyro_bias.zero();
							_fifo_available = true;

							perf_count(_selection_changed_perf);
							PX4_DEBUG("selecting sensor_imu_fifo:%" PRIu8 " %" PRIu32, i, _selected_gyro_device_id);
							return true;

						} else {
							PX4_ERR("unable to register callback for sensor_imu_fifo:%" PRIu8 " %" PRIu32,
								i, sensor_imu_fifo_sub.get().device_id);
						}
					}
				}
			}

			for (uint8_t i = 0; i < MAX_SENSOR_COUNT; i++) {
				uORB::SubscriptionData<sensor_gyro_s> sensor_gyro_sub{ORB_ID(sensor_gyro), i};

				if (sensor_gyro_sub.advertised()
				    && (sensor_gyro_sub.get().timestamp != 0)
				    && (sensor_gyro_sub.get().device_id != 0)
				    && (time_now_us < sensor_gyro_sub.get().timestamp + 1_s)) {

					// if no gyro was selected use the first valid sensor_gyro
					if (!device_id_valid) {
						device_id = sensor_gyro_sub.get().device_id;
						PX4_WARN("no gyro selected, using sensor_gyro:%" PRIu8 " %" PRIu32, i, sensor_gyro_sub.get().device_id);
					}

					if (sensor_gyro_sub.get().device_id == device_id) {
						if (_sensor_gyro_sub.ChangeInstance(i) && _sensor_gyro_sub.registerCallback()) {
							// make sure FIFO sub is unregistered
							_sensor_imu_fifo_sub.unregisterCallback();

							_gyro_calibration.set_device_id(sensor_gyro_sub.get().device_id);

							_selected_gyro_device_id = sensor_gyro_sub.get().device_id;

							_timestamp_sample_last = 0;
							_filter_sample_rate_hz = NAN;
							_update_sample_rate = true;
							_reset_filters = true;
							_gyro_bias.zero();
							_fifo_available = false;

							perf_count(_selection_changed_perf);
							PX4_DEBUG("selecting sensor_gyro:%" PRIu8 " %" PRIu32, i, _selected_gyro_device_id);
							return true;

						} else {
							PX4_ERR("unable to register callback for sensor_gyro:%" PRIu8 " %" PRIu32,
								i, sensor_gyro_sub.get().device_id);
						}
					}
				}
			}

			if (device_id != 0) {
				PX4_ERR("unable to find or subscribe to selected sensor (%" PRIu32 ")", device_id);
			}

			_selected_gyro_device_id = 0;
		}
	}

	return false;
}

void VehicleAngularVelocity::ParametersUpdate(bool force)
{
	// Check if parameters have changed
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		const bool nf0_enabled_prev = (_param_imu_gyro_nf0_frq.get() > 0.f) && (_param_imu_gyro_nf0_bw.get() > 0.f);
		const bool nf1_enabled_prev = (_param_imu_gyro_nf1_frq.get() > 0.f) && (_param_imu_gyro_nf1_bw.get() > 0.f);

		updateParams();


		// accel
		for (auto &imu : _imus) {
			imu.accel.calibration.ParametersUpdate();
		}

		// accel low pass cutoff frequency changed
		for (auto &lp : _accel_lp_filter) {
			if (fabsf(lp.get_cutoff_freq() - _param_imu_accel_cutoff.get()) > 0.01f) {
				break;
			}
		}

		// gyro
		const bool nf0_enabled = (_param_imu_gyro_nf0_frq.get() > 0.f) && (_param_imu_gyro_nf0_bw.get() > 0.f);
		const bool nf1_enabled = (_param_imu_gyro_nf1_frq.get() > 0.f) && (_param_imu_gyro_nf1_bw.get() > 0.f);

		for (auto &imu : _imus) {
			imu.gyro.calibration.ParametersUpdate();
		}

		// IMU_GYRO_RATEMAX
		if (_param_imu_gyro_ratemax.get() <= 0) {
			const int32_t imu_gyro_ratemax = _param_imu_gyro_ratemax.get();
			_param_imu_gyro_ratemax.reset();
			PX4_WARN("IMU_GYRO_RATEMAX invalid (%" PRId32 "), resetting to default %" PRId32 ")", imu_gyro_ratemax,
				 _param_imu_gyro_ratemax.get());
		}

		// constrain IMU_GYRO_RATEMAX 50-10,000 Hz
		const int32_t imu_gyro_ratemax = constrain(_param_imu_gyro_ratemax.get(), (int32_t)50, (int32_t)10'000);

		if (imu_gyro_ratemax != _param_imu_gyro_ratemax.get()) {
			PX4_WARN("IMU_GYRO_RATEMAX updated %" PRId32 " -> %" PRIu32, _param_imu_gyro_ratemax.get(), imu_gyro_ratemax);
			_param_imu_gyro_ratemax.set(imu_gyro_ratemax);
			_param_imu_gyro_ratemax.commit_no_notification();
		}

		// gyro low pass cutoff frequency changed
		for (auto &lp : _gyro_lp_filter_velocity) {
			if (fabsf(lp.get_cutoff_freq() - _param_imu_gyro_cutoff.get()) > 0.01f) {
				_reset_filters = true;
				break;
			}
		}

		// gyro notch filter 0 frequency or bandwidth changed
		for (auto &nf : _gyro_notch_filter0_velocity) {
			const bool nf_freq_changed = (fabsf(nf.getNotchFreq() - _param_imu_gyro_nf0_frq.get()) > 0.01f);
			const bool nf_bw_changed   = (fabsf(nf.getBandwidth() - _param_imu_gyro_nf0_bw.get()) > 0.01f);

			if ((nf0_enabled_prev != nf0_enabled) || (nf0_enabled && (nf_freq_changed || nf_bw_changed))) {
				_reset_filters = true;
				break;
			}
		}

		// gyro notch filter 1 frequency or bandwidth changed
		for (auto &nf : _gyro_notch_filter1_velocity) {
			const bool nf_freq_changed = (fabsf(nf.getNotchFreq() - _param_imu_gyro_nf1_frq.get()) > 0.01f);
			const bool nf_bw_changed   = (fabsf(nf.getBandwidth() - _param_imu_gyro_nf1_bw.get()) > 0.01f);

			if ((nf1_enabled_prev != nf1_enabled) || (nf1_enabled && (nf_freq_changed || nf_bw_changed))) {
				_reset_filters = true;
				break;
			}
		}

		// gyro derivative low pass cutoff changed
		for (auto &lp : _gyro_derivative_lp_filter) {
			if (fabsf(lp.getCutoffFreq() - _param_imu_dgyro_cutoff.get()) > 0.01f) {
				_reset_filters = true;
				break;
			}
		}

#if !defined(CONSTRAINED_FLASH)

		if (_param_imu_gyro_dnf_en.get() & DynamicNotch::EscRpm) {

			const int32_t esc_rpm_harmonics = math::constrain(_param_imu_gyro_dnf_hmc.get(), (int32_t)1, (int32_t)10);

			if (_dynamic_notch_filter_esc_rpm && (esc_rpm_harmonics != _esc_rpm_harmonics)) {
				delete[] _dynamic_notch_filter_esc_rpm;
				_dynamic_notch_filter_esc_rpm = nullptr;
				_esc_rpm_harmonics = 0;
			}

			if (_dynamic_notch_filter_esc_rpm == nullptr) {

				_dynamic_notch_filter_esc_rpm = new NotchFilterHarmonic[esc_rpm_harmonics];

				if (_dynamic_notch_filter_esc_rpm) {
					_esc_rpm_harmonics = esc_rpm_harmonics;

					if (_dynamic_notch_filter_esc_rpm_disable_perf == nullptr) {
						_dynamic_notch_filter_esc_rpm_disable_perf = perf_alloc(PC_COUNT,
								MODULE_NAME": gyro dynamic notch filter ESC RPM disable");
					}

					if (_dynamic_notch_filter_esc_rpm_init_perf == nullptr) {
						_dynamic_notch_filter_esc_rpm_init_perf = perf_alloc(PC_COUNT,
								MODULE_NAME": gyro dynamic notch filter ESC RPM init");
					}

					if (_dynamic_notch_filter_esc_rpm_update_perf == nullptr) {
						_dynamic_notch_filter_esc_rpm_update_perf = perf_alloc(PC_COUNT,
								MODULE_NAME": gyro dynamic notch filter ESC RPM update");
					}

				} else {
					_esc_rpm_harmonics = 0;

					perf_free(_dynamic_notch_filter_esc_rpm_disable_perf);
					perf_free(_dynamic_notch_filter_esc_rpm_init_perf);
					perf_free(_dynamic_notch_filter_esc_rpm_update_perf);

					_dynamic_notch_filter_esc_rpm_disable_perf = nullptr;
					_dynamic_notch_filter_esc_rpm_init_perf = nullptr;
					_dynamic_notch_filter_esc_rpm_update_perf = nullptr;
				}
			}

		} else {
			DisableDynamicNotchEscRpm();
		}

		if (_param_imu_gyro_dnf_en.get() & DynamicNotch::FFT) {
			if (_dynamic_notch_filter_fft_disable_perf == nullptr) {
				_dynamic_notch_filter_fft_disable_perf = perf_alloc(PC_COUNT, MODULE_NAME": gyro dynamic notch filter FFT disable");
				_dynamic_notch_filter_fft_update_perf = perf_alloc(PC_COUNT, MODULE_NAME": gyro dynamic notch filter FFT update");
			}

		} else {
			DisableDynamicNotchFFT();
		}

#endif // !CONSTRAINED_FLASH
	}
}

Vector3f VehicleAngularVelocity::GetResetAcceleration() const
{
	if (_acceleration_last_publish != 0) {
		// acceleration filtering is performed on raw unscaled data
		//  start with last valid vehicle body frame acceleration and compute equivalent raw data (for current sensor selection)
		Vector3f acceleration_uncalibrated{_accel_calibration.Uncorrect(_acceleration + _accel_bias)};

		if (PX4_ISFINITE(acceleration_uncalibrated(0))
		    && PX4_ISFINITE(acceleration_uncalibrated(1))
		    && PX4_ISFINITE(acceleration_uncalibrated(2))) {

			return acceleration_uncalibrated;
		}
	}

	return Vector3f{0.f, 0.f, 0.f};
}

Vector3f VehicleAngularVelocity::GetResetAngularVelocity() const
{
	if (_angular_velocity_last_publish != 0) {
		// angular velocity filtering is performed on raw unscaled data
		//  start with last valid vehicle body frame angular velocity and compute equivalent raw data (for current sensor selection)
		Vector3f angular_velocity_uncalibrated{_gyro_calibration.Uncorrect(_angular_velocity + _gyro_bias)};

		if (angular_velocity_uncalibrated.isAllFinite()) {
			return angular_velocity_uncalibrated;
		}
	}

	return Vector3f{0.f, 0.f, 0.f};
}

Vector3f VehicleAngularVelocity::GetResetAngularAcceleration() const
{
	if (_angular_velocity_last_publish != 0) {
		// angular acceleration filtering is performed on unscaled angular velocity data
		//  start with last valid vehicle body frame angular acceleration and compute equivalent raw data (for current sensor selection)
		Vector3f angular_acceleration{_gyro_calibration.rotation().I() *_angular_acceleration};

		if (angular_acceleration.isAllFinite()) {
			return angular_acceleration;
		}
	}

	return Vector3f{0.f, 0.f, 0.f};
}

void VehicleAngularVelocity::DisableDynamicNotchEscRpm()
{
#if !defined(CONSTRAINED_FLASH)

	if (_dynamic_notch_filter_esc_rpm) {
		for (int harmonic = 0; harmonic < _esc_rpm_harmonics; harmonic++) {
			for (int axis = 0; axis < 3; axis++) {
				for (int esc = 0; esc < MAX_NUM_ESCS; esc++) {
					_dynamic_notch_filter_esc_rpm[harmonic][axis][esc].disable();
					_esc_available.set(esc, false);
					perf_count(_dynamic_notch_filter_esc_rpm_disable_perf);
				}
			}
		}
	}

#endif // !CONSTRAINED_FLASH
}

void VehicleAngularVelocity::DisableDynamicNotchFFT()
{
#if !defined(CONSTRAINED_FLASH)

	if (_dynamic_notch_fft_available) {
		for (int axis = 0; axis < 3; axis++) {
			for (int peak = 0; peak < MAX_NUM_FFT_PEAKS; peak++) {
				_dynamic_notch_filter_fft[axis][peak].disable();
				perf_count(_dynamic_notch_filter_fft_disable_perf);
			}
		}

		_dynamic_notch_fft_available = false;
	}

#endif // !CONSTRAINED_FLASH
}

void VehicleAngularVelocity::UpdateDynamicNotchEscRpm(const hrt_abstime &time_now_us, bool force)
{
#if !defined(CONSTRAINED_FLASH)
	const bool enabled = _dynamic_notch_filter_esc_rpm && (_param_imu_gyro_dnf_en.get() & DynamicNotch::EscRpm);

	if (enabled && (_esc_status_sub.updated() || force)) {

		bool axis_init[3] {false, false, false};

		esc_status_s esc_status;

		if (_esc_status_sub.copy(&esc_status) && (time_now_us < esc_status.timestamp + DYNAMIC_NOTCH_FITLER_TIMEOUT)) {

			const float bandwidth_hz = _param_imu_gyro_dnf_bw.get();
			const float freq_min = math::max(_param_imu_gyro_dnf_min.get(), bandwidth_hz);

			for (size_t esc = 0; esc < math::min(esc_status.esc_count, (uint8_t)MAX_NUM_ESCS); esc++) {
				const esc_report_s &esc_report = esc_status.esc[esc];

				const bool esc_connected = (esc_status.esc_online_flags & (1 << esc)) || (esc_report.esc_rpm != 0);

				// only update if ESC RPM range seems valid
				if (esc_connected && (time_now_us < esc_report.timestamp + DYNAMIC_NOTCH_FITLER_TIMEOUT)) {

					const float esc_hz = abs(esc_report.esc_rpm) / 60.f;

					const bool force_update = force || !_esc_available[esc]; // force parameter update or notch was previously disabled

					for (int harmonic = 0; harmonic < _esc_rpm_harmonics; harmonic++) {
						// as RPM drops leave the notch filter "parked" at the minimum rather than disabling
						//  keep harmonics separated by half the notch filter bandwidth
						const float frequency_hz = math::max(esc_hz * (harmonic + 1), freq_min + (harmonic * 0.5f * bandwidth_hz));

						// update filter parameters if frequency changed or forced
						for (int axis = 0; axis < 3; axis++) {
							auto &nf = _dynamic_notch_filter_esc_rpm[harmonic][axis][esc];

							const float notch_freq_delta = fabsf(nf.getNotchFreq() - frequency_hz);

							const bool notch_freq_changed = (notch_freq_delta > 0.1f);

							// only allow initializing one new filter per axis each iteration
							const bool allow_update = !axis_init[axis] || (nf.initialized() && notch_freq_delta < nf.getBandwidth());

							if ((force_update || notch_freq_changed) && allow_update) {
								if (nf.setParameters(_filter_sample_rate_hz, frequency_hz, bandwidth_hz)) {
									perf_count(_dynamic_notch_filter_esc_rpm_update_perf);

									if (!nf.initialized()) {
										perf_count(_dynamic_notch_filter_esc_rpm_init_perf);
										axis_init[axis] = true;
									}
								}
							}
						}
					}

					_esc_available.set(esc, true);
					_last_esc_rpm_notch_update[esc] = esc_report.timestamp;
				}
			}
		}

		// check notch filter timeout
		for (size_t esc = 0; esc < MAX_NUM_ESCS; esc++) {
			if (_esc_available[esc] && (time_now_us > _last_esc_rpm_notch_update[esc] + DYNAMIC_NOTCH_FITLER_TIMEOUT)) {
				bool all_disabled = true;

				// disable notch filters from highest frequency to lowest
				for (int harmonic = _esc_rpm_harmonics - 1; harmonic >= 0; harmonic--) {
					for (int axis = 0; axis < 3; axis++) {
						auto &nf = _dynamic_notch_filter_esc_rpm[harmonic][axis][esc];

						if (nf.getNotchFreq() > 0.f) {
							if (nf.initialized() && !axis_init[axis]) {
								nf.disable();
								perf_count(_dynamic_notch_filter_esc_rpm_disable_perf);
								axis_init[axis] = true;
							}
						}

						if (nf.getNotchFreq() > 0.f) {
							all_disabled = false;
						}
					}
				}

				if (all_disabled) {
					_esc_available.set(esc, false);
				}
			}
		}
	}

#endif // !CONSTRAINED_FLASH
}

void VehicleAngularVelocity::UpdateDynamicNotchFFT(const hrt_abstime &time_now_us, bool force)
{
#if !defined(CONSTRAINED_FLASH)
	const bool enabled = _param_imu_gyro_dnf_en.get() & DynamicNotch::FFT;

	if (enabled && (_sensor_gyro_fft_sub.updated() || force)) {

		if (!_dynamic_notch_fft_available) {
			// force update filters if previously disabled
			force = true;
		}

		sensor_gyro_fft_s sensor_gyro_fft;

		if (_sensor_gyro_fft_sub.copy(&sensor_gyro_fft)
		    && (sensor_gyro_fft.device_id == _selected_gyro_device_id)
		    && (time_now_us < sensor_gyro_fft.timestamp + DYNAMIC_NOTCH_FITLER_TIMEOUT)
		    && ((fabsf(sensor_gyro_fft.sensor_sample_rate_hz - _filter_sample_rate_hz) / _filter_sample_rate_hz) < 0.02f)) {

			static constexpr float peak_freq_min = 10.f; // lower bound TODO: configurable?

			const float bandwidth = math::constrain(sensor_gyro_fft.resolution_hz, 8.f, 30.f); // TODO: base on numerical limits?

			float *peak_frequencies[] {sensor_gyro_fft.peak_frequencies_x, sensor_gyro_fft.peak_frequencies_y, sensor_gyro_fft.peak_frequencies_z};

			for (int axis = 0; axis < 3; axis++) {
				for (int peak = 0; peak < MAX_NUM_FFT_PEAKS; peak++) {

					const float peak_freq = peak_frequencies[axis][peak];

					auto &nf = _dynamic_notch_filter_fft[axis][peak];

					if (peak_freq > peak_freq_min) {
						// update filter parameters if frequency changed or forced
						if (force || !nf.initialized() || (fabsf(nf.getNotchFreq() - peak_freq) > 0.1f)) {
							nf.setParameters(_filter_sample_rate_hz, peak_freq, bandwidth);
							perf_count(_dynamic_notch_filter_fft_update_perf);
						}

						_dynamic_notch_fft_available = true;

					} else {
						// disable this notch filter (if it isn't already)
						if (nf.getNotchFreq() > 0.f) {
							nf.disable();
							perf_count(_dynamic_notch_filter_fft_disable_perf);
						}
					}
				}
			}

		} else {
			DisableDynamicNotchFFT();
		}
	}

#endif // !CONSTRAINED_FLASH
}

float VehicleAngularVelocity::FilterAcceleration(int axis, float data[], int N)
{
	// Apply general low-pass filter (IMU_ACCEL_CUTOFF)
	_accel_lp_filter[axis].applyArray(data, N);

	// return last filtered sample
	return data[N - 1];
}

float VehicleAngularVelocity::FilterAngularVelocity(int axis, float data[], int N)
{
#if !defined(CONSTRAINED_FLASH)

	// Apply dynamic notch filter from ESC RPM
	if (_dynamic_notch_filter_esc_rpm) {
		for (int esc = 0; esc < MAX_NUM_ESCS; esc++) {
			if (_esc_available[esc]) {
				for (int harmonic = 0; harmonic < _esc_rpm_harmonics; harmonic++) {
					if (_dynamic_notch_filter_esc_rpm[harmonic][axis][esc].getNotchFreq() > 0.f) {
						_dynamic_notch_filter_esc_rpm[harmonic][axis][esc].applyArray(data, N);
					}
				}
			}
		}
	}

	// Apply dynamic notch filter from FFT
	if (_dynamic_notch_fft_available) {
		for (int peak = MAX_NUM_FFT_PEAKS - 1; peak >= 0; peak--) {
			if (_dynamic_notch_filter_fft[axis][peak].getNotchFreq() > 0.f) {
				_dynamic_notch_filter_fft[axis][peak].applyArray(data, N);
			}
		}
	}

#endif // !CONSTRAINED_FLASH

	// Apply general notch filter 0 (IMU_GYRO_NF0_FRQ)
	if (_gyro_notch_filter0_velocity[axis].getNotchFreq() > 0.f) {
		_gyro_notch_filter0_velocity[axis].applyArray(data, N);
	}

	// Apply general notch filter 1 (IMU_GYRO_NF1_FRQ)
	if (_gyro_notch_filter1_velocity[axis].getNotchFreq() > 0.f) {
		_gyro_notch_filter1_velocity[axis].applyArray(data, N);
	}

	// Apply general low-pass filter (IMU_GYRO_CUTOFF)
	_gyro_lp_filter_velocity[axis].applyArray(data, N);

	// return last filtered sample
	return data[N - 1];
}

float VehicleAngularVelocity::FilterAngularAcceleration(int axis, float inverse_dt_s, float data[], int N)
{
	// angular acceleration: Differentiate & apply specific angular acceleration (D-term) low-pass (IMU_DGYRO_CUTOFF)
	float angular_acceleration_filtered = 0.f;

	for (int n = 0; n < N; n++) {
		const float angular_acceleration = (data[n] - _angular_velocity_raw_prev(axis)) * inverse_dt_s;
		angular_acceleration_filtered = _gyro_derivative_lp_filter[axis].update(angular_acceleration);
		_angular_velocity_raw_prev(axis) = data[n];
	}

	return angular_acceleration_filtered;
}

void VehicleAngularVelocity::Run()
{
	perf_begin(_cycle_perf);

	// backup schedule
	ScheduleDelayed(10_ms);

	const hrt_abstime time_now_us = hrt_absolute_time();

	// update corrections first to set _selected_sensor
	const bool selection_updated = SensorSelectionUpdate(time_now_us);

	if (selection_updated || _update_sample_rate) {
		if (!UpdateSampleRate()) {
			// sensor sample rate required to run
			perf_end(_cycle_perf);
			return;
		}
	}

	ParametersUpdate();

	for (auto &imu : _imus) {
		imu.accel.calibration.SensorCorrectionsUpdate();
		imu.gyro.calibration.SensorCorrectionsUpdate();
	}

	SensorBiasUpdate(selection_updated);

	if (_reset_filters) {
		ResetFilters(time_now_us);

		if (_reset_filters) {
			// not safe to run until filters configured
			perf_end(_cycle_perf);
			return;
		}
	}

	UpdateDynamicNotchEscRpm(time_now_us);
	UpdateDynamicNotchFFT(time_now_us);

	for (int sensor_instance = 0; sensor_instance < MAX_SENSOR_COUNT; sensor_instance++) {
		UpdateSensorImuFifo(sensor_instance);
		// UpdateSensorAccel(sensor_instance);
		// UpdateSensorGyro(sensor_instance);
	}




	// process all outstanding messages


	// force reselection on timeout
	if (time_now_us > _angular_velocity_last_publish + 500_ms) {
		SensorSelectionUpdate(true);
	}

	perf_end(_cycle_perf);
}

void VehicleAngularVelocity::UpdateSensorImuFifo(uint8_t sensor_instance)
{

	// sensor_imu_fifo
	sensor_imu_fifo_s sensor_imu_fifo;
	auto &s = _sensor_imu_fifo_subs[sensor_instance];

	if (s.sub.update(&sensor_imu_fifo)) {

		if (s.sub.get_last_generation() == s.last_generation + 1) {

			if ((s.timestamp_sample_last != 0)
			    && (sensor_imu_fifo.timestamp_sample > s.timestamp_sample_last)) {

				const float interval_us = sensor_imu_fifo.timestamp_sample - s.timestamp_sample_last;
				s.mean_interval_us.update(interval_us);
			}

		}


		s.last_generation = s.sub.get_last_generation();


		// TODO: find corresponding IMU slot
		IMU &imu = _imus[sensor_instance];

		imu.accel.calibration.set_device_id(sensor_imu_fifo.device_id);
		imu.gyro.calibration.set_device_id(sensor_imu_fifo.device_id);

		imu.accel.error_count = sensor_imu_fifo.error_count;
		imu.gyro.error_count = sensor_imu_fifo.error_count;

		// _status.accel_error_count = imu.accel.error_count;
		// _status.gyro_error_count = imu.gyro.error_count;

		// temperature average
		if (PX4_ISFINITE(sensor_imu_fifo.temperature)) {
			imu.accel.temperature.update(sensor_imu_fifo.temperature);
			imu.gyro.temperature.update(sensor_imu_fifo.temperature);
		}

		const int N = sensor_imu_fifo.samples;

		const float dt_s = sensor_imu_fifo.dt * 1e-6f;
		const float inverse_dt_s = 1e6f / sensor_imu_fifo.dt;


		// integrate accel
		for (int n = 0; n < N; n++) {
			const Vector3f accel_raw{
				static_cast<float>(sensor_imu_fifo.accel_x[n]) *sensor_imu_fifo.accel_scale,
				static_cast<float>(sensor_imu_fifo.accel_y[n]) *sensor_imu_fifo.accel_scale,
				static_cast<float>(sensor_imu_fifo.accel_z[n]) *sensor_imu_fifo.accel_scale
			};
			imu.accel.integrator.put(accel_raw, dt_s);
			imu.accel.raw_mean.update(accel_raw);
		}

		{
			int clip_counter[3] {
				clipping(sensor_imu_fifo.accel_x, sensor_imu_fifo.samples),
				clipping(sensor_imu_fifo.accel_y, sensor_imu_fifo.samples),
				clipping(sensor_imu_fifo.accel_z, sensor_imu_fifo.samples),
			};

			if (clip_counter[0] > 0 || clip_counter[1] > 0 || clip_counter[2] > 0) {
				// rotate sensor clip counts into vehicle body frame
				const Vector3f clipping{imu.accel.calibration.rotation() *Vector3f{(float)clip_counter[0], (float)clip_counter[1], (float)clip_counter[2]}};

				// round to get reasonble clip counts per axis (after board rotation)
				const uint8_t clip_x = roundf(fabsf(clipping(0)));

				if (clip_x > 0) {
					imu.accel.clipping_total[0] += clip_x;
					//_delta_velocity_clipping |= vehicle_imu_s::CLIPPING_X;
				}

				const uint8_t clip_y = roundf(fabsf(clipping(1)));

				if (clip_y > 0) {
					imu.accel.clipping_total[1] += clip_y;
					//_delta_velocity_clipping |= vehicle_imu_s::CLIPPING_Y;
				}

				const uint8_t clip_z = roundf(fabsf(clipping(2)));

				if (clip_z > 0) {
					imu.accel.clipping_total[2] += clip_z;
					//_delta_velocity_clipping |= vehicle_imu_s::CLIPPING_Z;
				}
			}
		}



		// integrate gyro
		for (int n = 0; n < N; n++) {
			const Vector3f gyro_raw{
				static_cast<float>(sensor_imu_fifo.gyro_x[n]) *sensor_imu_fifo.gyro_scale,
				static_cast<float>(sensor_imu_fifo.gyro_y[n]) *sensor_imu_fifo.gyro_scale,
				static_cast<float>(sensor_imu_fifo.gyro_z[n]) *sensor_imu_fifo.gyro_scale
			};
			imu.gyro.integrator.put(gyro_raw, dt_s);
			imu.gyro.raw_mean.update(gyro_raw);
		}



		if (imu.accel.integrator.integral_ready() && imu.gyro.integrator.integral_ready()) {
			PublishImu(imu);
		}

		// TODO: call to filter data
		if (imu.primary) {

			static constexpr int FIFO_SIZE_MAX = sizeof(sensor_imu_fifo.gyro_x) / sizeof(sensor_imu_fifo.gyro_x[0]);

			if ((sensor_imu_fifo.dt > 0) && (N > 0) && (N <= FIFO_SIZE_MAX)) {
				Vector3f acceleration_uncalibrated;
				Vector3f angular_velocity_uncalibrated;
				Vector3f angular_acceleration_uncalibrated;

				// accel
				{
					int16_t *raw_data_array[] {sensor_imu_fifo.accel_x, sensor_imu_fifo.accel_y, sensor_imu_fifo.accel_z};

					for (int axis = 0; axis < 3; axis++) {
						// copy raw int16 sensor samples to float array for filtering
						float data[FIFO_SIZE_MAX];

						for (int n = 0; n < N; n++) {
							data[n] = sensor_imu_fifo.accel_scale * raw_data_array[axis][n];
						}

						// save last filtered sample
						acceleration_uncalibrated(axis) = FilterAcceleration(axis, data, N);
					}
				}

				// gyro
				{
					int16_t *raw_data_array[] {sensor_imu_fifo.gyro_x, sensor_imu_fifo.gyro_y, sensor_imu_fifo.gyro_z};

					for (int axis = 0; axis < 3; axis++) {
						// copy raw int16 sensor samples to float array for filtering
						float data[FIFO_SIZE_MAX];

						for (int n = 0; n < N; n++) {
							data[n] = sensor_imu_fifo.gyro_scale * raw_data_array[axis][n];
						}

						// save last filtered sample
						angular_velocity_uncalibrated(axis) = FilterAngularVelocity(axis, data, N);
						angular_acceleration_uncalibrated(axis) = FilterAngularAcceleration(axis, inverse_dt_s, data, N);
					}
				}

				// Publish
				if (!s.sub.updated()) {

					if (sensor_imu_fifo.timestamp_sample >= _angular_velocity_last_publish + _publish_interval_min_us) {

						if (CalibrateAndPublishAcceleration(sensor_imu_fifo.timestamp_sample, acceleration_uncalibrated)) {

							//perf_end(_cycle_perf);
							//return;
						}

						if (CalibrateAndPublishAngularVelocity(sensor_imu_fifo.timestamp_sample,
										       angular_velocity_uncalibrated, angular_acceleration_uncalibrated)) {

							//perf_end(_cycle_perf);
							//return;
						}

						// shift last publish time forward, but don't let it get further behind than the interval
						// _angular_velocity_last_publish = math::constrain(_angular_velocity_last_publish + _publish_interval_min_us,
						// 				sensor_imu_fifo.timestamp_sample - _publish_interval_min_us, sensor_imu_fifo.timestamp_sample);

					}
				}
			}
		}
	}
}

void VehicleAngularVelocity::UpdateSensorAccel(uint8_t sensor_instance)
{
	// process all outstanding messages
	sensor_accel_s sensor_accel;

	while (_sensor_accel_sub.update(&sensor_accel)) {
		if (PX4_ISFINITE(sensor_accel.x) && PX4_ISFINITE(sensor_accel.y) && PX4_ISFINITE(sensor_accel.z)) {

			Vector3f acceleration_uncalibrated;

			float raw_data_array[] {sensor_accel.x, sensor_accel.y, sensor_accel.z};

			for (int axis = 0; axis < 3; axis++) {
				// copy sensor sample to float array for filtering
				float data[1] {raw_data_array[axis]};

				// save last filtered sample
				acceleration_uncalibrated(axis) = FilterAcceleration(axis, data);
			}

			// Publish
			if (!_sensor_accel_sub.updated()) {
				if (CalibrateAndPublishAcceleration(sensor_accel.timestamp_sample, acceleration_uncalibrated)) {

					//perf_end(_cycle_perf);
					//return;
				}
			}
		}
	}
}

void VehicleAngularVelocity::UpdateSensorGyro(uint8_t sensor_instance)
{
	sensor_gyro_s sensor_gyro;

	while (_sensor_gyro_sub.update(&sensor_gyro)) {
		if (Vector3f(sensor_gyro.x, sensor_gyro.y, sensor_gyro.z).isAllFinite()) {

			if (_timestamp_sample_last == 0 || (sensor_gyro.timestamp_sample <= _timestamp_sample_last)) {
				_timestamp_sample_last = sensor_gyro.timestamp_sample - 1e6f / _filter_sample_rate_hz;
			}

			const float inverse_dt_s = 1.f / math::constrain(((sensor_gyro.timestamp_sample - _timestamp_sample_last) * 1e-6f),
						   0.00002f, 0.02f);
			_timestamp_sample_last = sensor_gyro.timestamp_sample;

			Vector3f angular_velocity_uncalibrated;
			Vector3f angular_acceleration_uncalibrated;

			float raw_data_array[] {sensor_gyro.x, sensor_gyro.y, sensor_gyro.z};

			for (int axis = 0; axis < 3; axis++) {
				// copy sensor sample to float array for filtering
				float data[1] {raw_data_array[axis]};

				// save last filtered sample
				angular_velocity_uncalibrated(axis) = FilterAngularVelocity(axis, data);
				angular_acceleration_uncalibrated(axis) = FilterAngularAcceleration(axis, inverse_dt_s, data);
			}

			// Publish
			if (!_sensor_gyro_sub.updated()) {
				if (CalibrateAndPublishAngularVelocity(sensor_gyro.timestamp_sample,
								       angular_velocity_uncalibrated, angular_acceleration_uncalibrated)) {


					// shift last publish time forward, but don't let it get further behind than the interval
					_angular_velocity_last_publish = math::constrain(_angular_velocity_last_publish + _publish_interval_min_us,
									 sensor_gyro.timestamp_sample - _publish_interval_min_us, sensor_gyro.timestamp_sample);

					perf_end(_cycle_perf);
					return;
				}
			}
		}
	}

}

bool VehicleAngularVelocity::PublishImu(VehicleAngularVelocity::IMU &imu)
{
	bool updated = false;

	vehicle_imu_s vehicle_imu;
	Vector3f delta_angle;
	Vector3f delta_velocity;

	//const Vector3f accumulated_coning_corrections = imu.gyro.integrator.accumulated_coning_corrections();

	if (imu.accel.integrator.reset(delta_velocity, vehicle_imu.delta_velocity_dt)
	    && imu.gyro.integrator.reset(delta_angle, vehicle_imu.delta_angle_dt)) {

		if (imu.accel.calibration.enabled() && imu.gyro.calibration.enabled()) {

			// delta angle: apply offsets, scale, and board rotation
			imu.gyro.calibration.SensorCorrectionsUpdate();
			const float gyro_dt_s = 1.e-6f * vehicle_imu.delta_angle_dt;
			const Vector3f angular_velocity{imu.gyro.calibration.Correct(delta_angle / gyro_dt_s)};
			//UpdateGyroVibrationMetrics(angular_velocity);
			const Vector3f delta_angle_corrected{angular_velocity * gyro_dt_s};

			// accumulate delta angle coning corrections
			//_coning_norm_accum += accumulated_coning_corrections.norm() * gyro_dt_s;
			//_coning_norm_accum_total_time_s += gyro_dt_s;


			// delta velocity: apply offsets, scale, and board rotation
			imu.accel.calibration.SensorCorrectionsUpdate();
			const float accel_dt_s = 1.e-6f * vehicle_imu.delta_velocity_dt;
			const Vector3f acceleration{imu.accel.calibration.Correct(delta_velocity / accel_dt_s)};
			//UpdateAccelVibrationMetrics(acceleration);
			const Vector3f delta_velocity_corrected{acceleration * accel_dt_s};

#if 0
			// vehicle_imu_status
			//  publish before vehicle_imu so that error counts are available synchronously if needed
			const bool status_publish_interval_exceeded = (hrt_elapsed_time(&_status.timestamp) >= kIMUStatusPublishingInterval);

			if (_raw_accel_mean.valid() && _raw_gyro_mean.valid()
			    && _accel_mean_interval_us.valid() && _gyro_mean_interval_us.valid()
			    && (_publish_status || status_publish_interval_exceeded)
			   ) {

				// Accel
				{
					_status.accel_device_id = _accel_calibration.device_id();

					_status.accel_rate_hz = 1e6f / _accel_mean_interval_us.mean();
					_status.accel_raw_rate_hz = _status.accel_rate_hz;

					// accel mean and variance
					const Dcmf &R = _accel_calibration.rotation();
					Vector3f(R * _raw_accel_mean.mean()).copyTo(_status.mean_accel);

					// variance from R * COV * R^T
					const Matrix3f cov = R * _raw_accel_mean.covariance() * R.transpose();
					cov.diag().copyTo(_status.var_accel);

					// temperature
					if ((_accel_temperature_sum_count > 0) && PX4_ISFINITE(_accel_temperature_sum)) {
						_status.temperature_accel = _accel_temperature_sum / _accel_temperature_sum_count;

					} else {
						_status.temperature_accel = NAN;
					}
				}

				// Gyro
				{
					_status.gyro_device_id = imu.gyro.calibration.device_id();

					_status.gyro_rate_hz = 1e6f / _gyro_mean_interval_us.mean();
					_status.gyro_raw_rate_hz = _status.gyro_rate_hz;

					// gyro mean and variance
					const Dcmf &R = imu.gyro.calibration.rotation();
					Vector3f(R * _raw_gyro_mean.mean()).copyTo(_status.mean_gyro);

					// variance from R * COV * R^T
					const Matrix3f cov = R * _raw_gyro_mean.covariance() * R.transpose();
					cov.diag().copyTo(_status.var_gyro);


					// Gyro delta angle coning metric = length of coning corrections averaged since last status publication
					_status.delta_angle_coning_metric = _coning_norm_accum / _coning_norm_accum_total_time_s;
					_coning_norm_accum = 0;
					_coning_norm_accum_total_time_s = 0;

					// temperature
					if ((_gyro_temperature_sum_count > 0) && PX4_ISFINITE(_gyro_temperature_sum)) {
						_status.temperature_gyro = _gyro_temperature_sum / _gyro_temperature_sum_count;

					} else {
						_status.temperature_gyro = NAN;
					}
				}

				// publish
				_status.timestamp = hrt_absolute_time();
				_vehicle_imu_status_pub.publish(_status);

				_publish_status = false;

				if (status_publish_interval_exceeded) {
					_raw_accel_mean.reset();
					_accel_temperature_sum = NAN;
					_accel_temperature_sum_count = 0;

					_raw_gyro_mean.reset();
					_gyro_temperature_sum = NAN;
					_gyro_temperature_sum_count = 0;
				}
			}

#endif

			// publish vehicle_imu
			//vehicle_imu.timestamp_sample = _gyro_timestamp_sample_last;
			vehicle_imu.accel_device_id = imu.accel.calibration.device_id();
			vehicle_imu.gyro_device_id = imu.gyro.calibration.device_id();
			delta_angle_corrected.copyTo(vehicle_imu.delta_angle);
			delta_velocity_corrected.copyTo(vehicle_imu.delta_velocity);
			//vehicle_imu.delta_velocity_clipping = _delta_velocity_clipping;
			vehicle_imu.accel_calibration_count = imu.accel.calibration.calibration_count();
			vehicle_imu.gyro_calibration_count = imu.gyro.calibration.calibration_count();
			vehicle_imu.timestamp = hrt_absolute_time();
			imu.vehicle_imu_pub.publish(vehicle_imu);

			// reset clip counts
			//_delta_velocity_clipping = 0;

			// record gyro publication latency and integrated samples
			//_gyro_publish_latency_mean_us.update(imu.timestamp - _gyro_timestamp_last);
			//_gyro_update_latency_mean_us.update(imu.timestamp - _gyro_timestamp_sample_last);

			updated = true;
		}
	}

	return updated;
}

bool VehicleAngularVelocity::CalibrateAndPublishAcceleration(const hrt_abstime &timestamp_sample,
		const Vector3f &acceleration_uncalibrated)
{
	if (timestamp_sample >= _acceleration_last_publish + _publish_interval_min_us) {
		// Publish vehicle_acceleration
		vehicle_acceleration_s v_acceleration;
		v_acceleration.timestamp_sample = timestamp_sample;

		// Acceleration: rotate sensor frame to board, scale raw data to SI, apply calibration, and remove in-run estimated bias
		_acceleration = _accel_calibration.Correct(acceleration_uncalibrated) - _accel_bias;
		_acceleration.copyTo(v_acceleration.xyz);

		v_acceleration.timestamp = hrt_absolute_time();
		_vehicle_acceleration_pub.publish(v_acceleration);

		return true;
	}

	return false;
}

bool VehicleAngularVelocity::CalibrateAndPublishAngularVelocity(const hrt_abstime &timestamp_sample,
		const Vector3f &angular_velocity_uncalibrated, const Vector3f &angular_acceleration_uncalibrated)
{
	if (timestamp_sample >= _angular_velocity_last_publish + _publish_interval_min_us) {
		// Publish vehicle_angular_velocity
		vehicle_angular_velocity_s angular_velocity;
		angular_velocity.timestamp_sample = timestamp_sample;

		// Angular velocity: rotate sensor frame to board, scale raw data to SI, apply calibration, and remove in-run estimated bias
		_angular_velocity = _gyro_calibration.Correct(angular_velocity_uncalibrated) - _gyro_bias;
		_angular_velocity.copyTo(angular_velocity.xyz);

		// Angular acceleration: rotate sensor frame to board, scale raw data to SI, apply any additional configured rotation
		_angular_acceleration = _gyro_calibration.rotation() * angular_acceleration_uncalibrated;
		_angular_acceleration.copyTo(angular_velocity.xyz_derivative);

		angular_velocity.timestamp = hrt_absolute_time();
		_vehicle_angular_velocity_pub.publish(angular_velocity);

		return true;
	}

	return false;
}

void VehicleAngularVelocity::PrintStatus()
{
	PX4_INFO_RAW("[vehicle_angular_velocity] selected sensor: %" PRIu32
		     ", rate: %.1f Hz %s, estimated bias: [%.5f %.5f %.5f]\n",
		     _gyro_calibration.device_id(), (double)_filter_sample_rate_hz, _fifo_available ? "FIFO" : "",
		     (double)_gyro_bias(0), (double)_gyro_bias(1), (double)_gyro_bias(2));

	_gyro_calibration.PrintStatus();


	// PX4_INFO_RAW("[vehicle_imu_fifo] %" PRIu8 " - IMU: %" PRIu32 ", interval: %.1f us (SD %.1f us)\n",
	// 	     _instance, _accel_calibration.device_id(), (double)_interval_us, (double)sqrtf(_interval_best_variance));

	// PX4_INFO_RAW("gyro update mean sample latency: %.6f s, publish latency %.6f s, gyro interval %.6f s",
	// 	     (double)_update_latency_mean.mean()(0),
	// 	     (double)_update_latency_mean.mean()(1),
	// 	     (double)(_interval_us * 1e-6f));

	// perf_print_counter(_imu_generation_gap_perf);

	// _accel_calibration.PrintStatus();
	// _gyro_calibration.PrintStatus();



	perf_print_counter(_cycle_perf);
	perf_print_counter(_filter_reset_perf);
	perf_print_counter(_selection_changed_perf);
#if !defined(CONSTRAINED_FLASH)
	perf_print_counter(_dynamic_notch_filter_esc_rpm_disable_perf);
	perf_print_counter(_dynamic_notch_filter_esc_rpm_init_perf);
	perf_print_counter(_dynamic_notch_filter_esc_rpm_update_perf);

	perf_print_counter(_dynamic_notch_filter_fft_disable_perf);
	perf_print_counter(_dynamic_notch_filter_fft_update_perf);
#endif // CONSTRAINED_FLASH
}

} // namespace sensors
