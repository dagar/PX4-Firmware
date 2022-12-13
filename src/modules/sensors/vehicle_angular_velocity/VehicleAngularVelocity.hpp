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

#pragma once

#include <containers/Bitset.hpp>
#include <lib/sensor_calibration/Accelerometer.hpp>
#include <lib/sensor_calibration/Gyroscope.hpp>
#include <lib/mathlib/math/Limits.hpp>
#include <lib/mathlib/math/WelfordMean.hpp>
#include <lib/mathlib/math/WelfordMeanVector.hpp>
#include <lib/matrix/matrix/math.hpp>
#include <lib/mathlib/math/filter/AlphaFilter.hpp>
#include <lib/mathlib/math/filter/LowPassFilter2p.hpp>
#include <lib/mathlib/math/filter/NotchFilter.hpp>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/esc_status.h>
#include <uORB/topics/estimator_selector_status.h>
#include <uORB/topics/estimator_sensor_bias.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/sensor_gyro_fft.h>
#include <uORB/topics/sensor_imu_fifo.h>
#include <uORB/topics/sensor_selection.h>

// publications
#include <uORB/topics/sensor_combined.h> // legacy
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_acceleration.h>
#include <uORB/topics/vehicle_imu.h>
#include <uORB/topics/vehicle_imu_status.h>

#include <Integrator.hpp>

using namespace time_literals;

namespace sensors
{

class VehicleAngularVelocity : public ModuleParams, public px4::ScheduledWorkItem
{
public:
	VehicleAngularVelocity();
	~VehicleAngularVelocity() override;

	void PrintStatus();
	bool Start();
	void Stop();

private:
	void Run() override;

	bool CalibrateAndPublishAcceleration(const hrt_abstime &timestamp_sample,
					     const matrix::Vector3f &acceleration_uncalibrated);
	bool CalibrateAndPublishAngularVelocity(const hrt_abstime &timestamp_sample,
						const matrix::Vector3f &angular_velocity_uncalibrated,
						const matrix::Vector3f &angular_acceleration_uncalibrated);

	inline float FilterAcceleration(int axis, float data[], int N = 1);
	inline float FilterAngularVelocity(int axis, float data[], int N = 1);
	inline float FilterAngularAcceleration(int axis, float inverse_dt_s, float data[], int N = 1);

	void DisableDynamicNotchEscRpm();
	void DisableDynamicNotchFFT();
	void ParametersUpdate(bool force = false);

	void ResetFilters(const hrt_abstime &time_now_us);

	void SensorBiasUpdate(bool force = false);

	bool SensorSelectionUpdate(const hrt_abstime &time_now_us, bool force = false);

	void UpdateDynamicNotchEscRpm(const hrt_abstime &time_now_us, bool force = false);
	void UpdateDynamicNotchFFT(const hrt_abstime &time_now_us, bool force = false);

	bool UpdateSampleRate();

	void UpdateSensorImuFifo(uint8_t sensor_instance);
	void UpdateSensorAccel(uint8_t sensor_instance);
	void UpdateSensorGyro(uint8_t sensor_instance);

	// scaled appropriately for current sensor
	matrix::Vector3f GetResetAcceleration() const;
	matrix::Vector3f GetResetAngularVelocity() const;
	matrix::Vector3f GetResetAngularAcceleration() const;

	// return the square of two floating point numbers
	static constexpr float sq(float var) { return var * var; }

	static constexpr int MAX_SENSOR_COUNT = 4;

	uORB::Publication<vehicle_acceleration_s>     _vehicle_acceleration_pub{ORB_ID(vehicle_acceleration)};
	uORB::Publication<vehicle_angular_velocity_s> _vehicle_angular_velocity_pub{ORB_ID(vehicle_angular_velocity)};

	uORB::Subscription _estimator_selector_status_sub{ORB_ID(estimator_selector_status)};
	uORB::Subscription _estimator_sensor_bias_sub{ORB_ID(estimator_sensor_bias)};
#if !defined(CONSTRAINED_FLASH)
	uORB::Subscription _esc_status_sub {ORB_ID(esc_status)};
	uORB::Subscription _sensor_gyro_fft_sub {ORB_ID(sensor_gyro_fft)};
#endif // !CONSTRAINED_FLASH

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::SubscriptionCallbackWorkItem _sensor_selection_sub{this, ORB_ID(sensor_selection)};

	uORB::SubscriptionCallbackWorkItem _sensor_accel_sub{this, ORB_ID(sensor_accel)};
	uORB::SubscriptionCallbackWorkItem _sensor_gyro_sub{this, ORB_ID(sensor_gyro)};
	uORB::SubscriptionCallbackWorkItem _sensor_imu_fifo_sub{this, ORB_ID(sensor_imu_fifo)};

	uORB::Subscription _sensor_accel_subs[MAX_SENSOR_COUNT] {ORB_ID(sensor_accel), ORB_ID(sensor_accel), ORB_ID(sensor_accel), ORB_ID(sensor_accel)};
	uORB::Subscription _sensor_gyro_subs[MAX_SENSOR_COUNT] {ORB_ID(sensor_gyro), ORB_ID(sensor_gyro), ORB_ID(sensor_gyro), ORB_ID(sensor_gyro)};


	math::WelfordMeanVector<float, 3> _raw_accel_mean[MAX_SENSOR_COUNT] {};
	math::WelfordMeanVector<float, 3> _raw_gyro_mean[MAX_SENSOR_COUNT] {};

	struct SensorSubscription {
		SensorSubscription(px4::WorkItem *work_item, ORB_ID orb_id, uint8_t instance) :
			sub(work_item, orb_id, instance)
		{}

		~SensorSubscription() = default;

		uORB::SubscriptionCallbackWorkItem sub;
		unsigned last_generation{0};

		hrt_abstime timestamp_last{0};
		hrt_abstime timestamp_sample_last{0};

		math::WelfordMean<float> mean_interval_us{};
	};

	SensorSubscription _sensor_imu_fifo_subs[MAX_SENSOR_COUNT] {
		{this, ORB_ID::sensor_imu_fifo, 0},
		{this, ORB_ID::sensor_imu_fifo, 1},
		{this, ORB_ID::sensor_imu_fifo, 2},
		{this, ORB_ID::sensor_imu_fifo, 3}
	};

	struct IMU {

		// struct InFlightCalibration {
		// 	matrix::Vector3f offset{};
		// 	matrix::Vector3f bias_variance{};
		// 	bool valid{false};
		// };

		struct {
			uint32_t device_id{0};
			calibration::Accelerometer calibration{};
			math::WelfordMeanVector<float, 3> raw_mean{};
			math::WelfordMean<float> temperature{};
			float scale{1.f};
			uint32_t error_count{0};
			sensors::Integrator integrator{};
			matrix::Vector3f estimated_bias{};

			uint32_t clipping_total[3] {}; // clipping
		} accel{};

		struct {
			uint32_t device_id{0};
			calibration::Gyroscope calibration{};
			math::WelfordMeanVector<float, 3> raw_mean{};
			math::WelfordMean<float> temperature{};
			float scale{1.f};
			uint32_t error_count{0};
			sensors::IntegratorConing integrator{};
			matrix::Vector3f estimated_bias{};

			uint32_t clipping_total[3] {}; // clipping
		} gyro{};

		bool primary{false};

		uORB::PublicationMulti<vehicle_imu_s> vehicle_imu_pub{ORB_ID(vehicle_imu)};
		uORB::PublicationMulti<vehicle_imu_status_s> vehicle_imu_status_pub{ORB_ID(vehicle_imu_status)};
	} _imus[MAX_SENSOR_COUNT] {};

	bool PublishImu(IMU &imu);

	calibration::Accelerometer _accel_calibration{};
	calibration::Gyroscope _gyro_calibration{};

	matrix::Vector3f _accel_bias{};
	matrix::Vector3f _gyro_bias{};

	matrix::Vector3f _acceleration{};
	matrix::Vector3f _angular_velocity{};
	matrix::Vector3f _angular_acceleration{};

	matrix::Vector3f _angular_velocity_raw_prev{};
	hrt_abstime _timestamp_sample_last{0};

	hrt_abstime _publish_interval_min_us{0};
	hrt_abstime _last_publish{0};

	float _filter_sample_rate_hz{NAN};

	// acceleration filters
	math::LowPassFilter2p<float> _accel_lp_filter[3] {};

	// angular velocity filters
	math::LowPassFilter2p<float> _gyro_lp_filter_velocity[3] {};
	math::NotchFilter<float> _gyro_notch_filter0_velocity[3] {};
	math::NotchFilter<float> _gyro_notch_filter1_velocity[3] {};

#if !defined(CONSTRAINED_FLASH)

	enum DynamicNotch {
		EscRpm = 1,
		FFT    = 2,
	};

	static constexpr hrt_abstime DYNAMIC_NOTCH_FITLER_TIMEOUT = 3_s;

	// ESC RPM
	static constexpr int MAX_NUM_ESCS = sizeof(esc_status_s::esc) / sizeof(esc_status_s::esc[0]);

	using NotchFilterHarmonic = math::NotchFilter<float>[3][MAX_NUM_ESCS];
	NotchFilterHarmonic *_dynamic_notch_filter_esc_rpm{nullptr};

	int _esc_rpm_harmonics{0};
	px4::Bitset<MAX_NUM_ESCS> _esc_available{};
	hrt_abstime _last_esc_rpm_notch_update[MAX_NUM_ESCS] {};

	perf_counter_t _dynamic_notch_filter_esc_rpm_disable_perf{nullptr};
	perf_counter_t _dynamic_notch_filter_esc_rpm_init_perf{nullptr};
	perf_counter_t _dynamic_notch_filter_esc_rpm_update_perf{nullptr};

	// FFT
	static constexpr int MAX_NUM_FFT_PEAKS = sizeof(sensor_gyro_fft_s::peak_frequencies_x)
			/ sizeof(sensor_gyro_fft_s::peak_frequencies_x[0]);

	math::NotchFilter<float> _dynamic_notch_filter_fft[3][MAX_NUM_FFT_PEAKS] {};

	perf_counter_t _dynamic_notch_filter_fft_disable_perf{nullptr};
	perf_counter_t _dynamic_notch_filter_fft_update_perf{nullptr};

	bool _dynamic_notch_fft_available{false};
#endif // !CONSTRAINED_FLASH

	// angular acceleration filter
	AlphaFilter<float> _gyro_derivative_lp_filter[3] {};

	uint32_t _selected_accel_device_id{0};
	uint32_t _selected_gyro_device_id{0};

	bool _reset_filters{true};
	bool _fifo_available{false};
	bool _update_sample_rate{true};

	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": IMU filter")};
	perf_counter_t _filter_reset_perf{perf_alloc(PC_COUNT, MODULE_NAME": IMU filter reset")};
	perf_counter_t _selection_changed_perf{perf_alloc(PC_COUNT, MODULE_NAME": IMU selection changed")};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::IMU_ACCEL_CUTOFF>) _param_imu_accel_cutoff,
		(ParamInt<px4::params::IMU_INTEG_RATE>) _param_imu_integ_rate,
#if !defined(CONSTRAINED_FLASH)
		(ParamInt<px4::params::IMU_GYRO_DNF_EN>) _param_imu_gyro_dnf_en,
		(ParamInt<px4::params::IMU_GYRO_DNF_HMC>) _param_imu_gyro_dnf_hmc,
		(ParamFloat<px4::params::IMU_GYRO_DNF_BW>) _param_imu_gyro_dnf_bw,
		(ParamFloat<px4::params::IMU_GYRO_DNF_MIN>) _param_imu_gyro_dnf_min,
#endif // !CONSTRAINED_FLASH
		(ParamFloat<px4::params::IMU_GYRO_CUTOFF>) _param_imu_gyro_cutoff,
		(ParamFloat<px4::params::IMU_GYRO_NF0_FRQ>) _param_imu_gyro_nf0_frq,
		(ParamFloat<px4::params::IMU_GYRO_NF0_BW>) _param_imu_gyro_nf0_bw,
		(ParamFloat<px4::params::IMU_GYRO_NF1_FRQ>) _param_imu_gyro_nf1_frq,
		(ParamFloat<px4::params::IMU_GYRO_NF1_BW>) _param_imu_gyro_nf1_bw,
		(ParamInt<px4::params::IMU_GYRO_RATEMAX>) _param_imu_gyro_ratemax,
		(ParamFloat<px4::params::IMU_DGYRO_CUTOFF>) _param_imu_dgyro_cutoff
	)
};

} // namespace sensors
