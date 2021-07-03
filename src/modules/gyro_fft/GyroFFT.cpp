/****************************************************************************
 *
 *   Copyright (c) 2020-2021 PX4 Development Team. All rights reserved.
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

#include "GyroFFT.hpp"

#include <drivers/drv_hrt.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>

using namespace matrix;

GyroFFT::GyroFFT() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
	for (int i = 0; i < MAX_NUM_PEAKS; i++) {
		_sensor_gyro_fft.peak_frequencies_x[i] = NAN;
		_sensor_gyro_fft.peak_frequencies_y[i] = NAN;
		_sensor_gyro_fft.peak_frequencies_z[i] = NAN;

		_sensor_gyro_fft.peak_frequencies_x_raw[i] = NAN;
		_sensor_gyro_fft.peak_frequencies_y_raw[i] = NAN;
		_sensor_gyro_fft.peak_frequencies_z_raw[i] = NAN;

		_sensor_gyro_fft.peak_magnitude_x[i] = NAN;
		_sensor_gyro_fft.peak_magnitude_y[i] = NAN;
		_sensor_gyro_fft.peak_magnitude_z[i] = NAN;
	}
}

GyroFFT::~GyroFFT()
{
	perf_free(_cycle_perf);
	perf_free(_cycle_interval_perf);
	perf_free(_fft_perf);
	perf_free(_gyro_generation_gap_perf);
}

bool GyroFFT::init()
{
	_imu_gyro_fft_len = 128;

	if (!SensorSelectionUpdate(true)) {
		ScheduleDelayed(500_ms);
	}

	return true;
}

bool GyroFFT::SensorSelectionUpdate(bool force)
{
	if (_sensor_selection_sub.updated() || (_selected_sensor_device_id == 0) || force) {
		sensor_selection_s sensor_selection{};
		_sensor_selection_sub.copy(&sensor_selection);

		if ((sensor_selection.gyro_device_id != 0) && (_selected_sensor_device_id != sensor_selection.gyro_device_id)) {
			for (uint8_t i = 0; i < MAX_SENSOR_COUNT; i++) {
				uORB::SubscriptionData<sensor_gyro_s> sensor_gyro_sub{ORB_ID(sensor_gyro), i};

				if (sensor_gyro_sub.get().device_id == sensor_selection.gyro_device_id) {
					if (_sensor_gyro_sub.ChangeInstance(i) && _sensor_gyro_sub.registerCallback()) {
						_sensor_gyro_sub.set_required_updates(sensor_gyro_s::ORB_QUEUE_LENGTH - 1);
						_selected_sensor_device_id = sensor_selection.gyro_device_id;
						return true;
					}
				}
			}

			PX4_ERR("unable to find or subscribe to selected sensor (%d)", sensor_selection.gyro_device_id);
		}
	}

	return false;
}

void GyroFFT::VehicleIMUStatusUpdate(bool force)
{
	if (_vehicle_imu_status_sub.updated() || force) {
		vehicle_imu_status_s vehicle_imu_status;

		if (_vehicle_imu_status_sub.copy(&vehicle_imu_status)) {
			// find corresponding vehicle_imu_status instance if the device_id doesn't match
			if (vehicle_imu_status.gyro_device_id != _selected_sensor_device_id) {

				for (uint8_t imu_status = 0; imu_status < MAX_SENSOR_COUNT; imu_status++) {
					uORB::Subscription imu_status_sub{ORB_ID(vehicle_imu_status), imu_status};

					if (imu_status_sub.copy(&vehicle_imu_status)) {
						if (vehicle_imu_status.gyro_device_id == _selected_sensor_device_id) {
							_vehicle_imu_status_sub.ChangeInstance(imu_status);
							break;
						}
					}
				}
			}

			// update gyro sample rate
			if ((vehicle_imu_status.gyro_device_id == _selected_sensor_device_id) && (vehicle_imu_status.gyro_rate_hz > 0)) {
				_gyro_sample_rate_hz = vehicle_imu_status.gyro_rate_hz;
				return;
			}
		}
	}
}

// helper function used for frequency estimation
static float tau(float x)
{
	// tau(x) = 1/4 * log(3x^2 + 6x + 1) – sqrt(6)/24 * log((x + 1 – sqrt(2/3))  /  (x + 1 + sqrt(2/3)))
	float p1 = logf(3.f * powf(x, 2.f) + 6.f * x + 1.f);
	float part1 = x + 1.f - sqrtf(2.f / 3.f);
	float part2 = x + 1.f + sqrtf(2.f / 3.f);
	float p2 = logf(part1 / part2);
	return (0.25f * p1 - sqrtf(6.f) / 24.f * p2);
}

float GyroFFT::EstimatePeakFrequency(int axis, int32_t k)
{
	if (k > 2) {
		// find peak location using Quinn's Second Estimator (2020-06-14: http://dspguru.com/dsp/howtos/how-to-interpolate-fft-peak/)
		const auto &dft = _sliding_dft[axis];

		const float divider = (dft.dft[k].real() * dft.dft[k].real() + dft.dft[k].imag() * dft.dft[k].imag());

		// ap = (X[k + 1].r * X[k].r + X[k+1].i * X[k].i) / (X[k].r * X[k].r + X[k].i * X[k].i)
		float ap = (dft.dft[k + 1].real() * dft.dft[k].real() + dft.dft[k + 1].imag() * dft.dft[k].imag()) / divider;

		// dp = -ap / (1 – ap)
		float dp = -ap  / (1.f - ap);

		// am = (X[k - 1].r * X[k].r + X[k – 1].i * X[k].i) / (X[k].r * X[k].r + X[k].i * X[k].i)
		float am = (dft.dft[k - 1].real() * dft.dft[k].real() + dft.dft[k - 1].imag() * dft.dft[k].imag()) / divider;

		// dm = am / (1 – am)
		float dm = am / (1.f - am);

		// d = (dp + dm) / 2 + tau(dp * dp) – tau(dm * dm)
		float d = (dp + dm) / 2.f + tau(dp * dp) - tau(dm * dm);

		// k’ = k + d
		float adjusted_bin = k + d;
		float peak_freq_adjusted = (_gyro_sample_rate_hz * adjusted_bin / _imu_gyro_fft_len);

		return peak_freq_adjusted;
	}

	return NAN;
}

void GyroFFT::Run()
{
	if (should_exit()) {
		_sensor_gyro_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	// backup schedule
	ScheduleDelayed(500_ms);

	perf_begin(_cycle_perf);
	perf_count(_cycle_interval_perf);

	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		updateParams();
	}

	const bool selection_updated = SensorSelectionUpdate();
	VehicleIMUStatusUpdate(selection_updated);

	// run on sensor gyro updates
	sensor_gyro_s sensor_gyro;

	while (_sensor_gyro_sub.update(&sensor_gyro)) {
		if (_sensor_gyro_sub.get_last_generation() != _gyro_last_generation + 1) {
			// force reset if we've missed a sample
			perf_count(_gyro_generation_gap_perf);
		}

		_gyro_last_generation = _sensor_gyro_sub.get_last_generation();

		perf_begin(_fft_perf);
		_sliding_dft[0].update(sensor_gyro.x);
		_sliding_dft[1].update(sensor_gyro.y);
		_sliding_dft[2].update(sensor_gyro.z);
		perf_end(_fft_perf);
	}

	Update(sensor_gyro.timestamp_sample);

	perf_end(_cycle_perf);
}

void GyroFFT::Update(const hrt_abstime &timestamp_sample)
{
	sensor_gyro_fft_s sensor_gyro_fft{};

	//bool publish = false;
	const float resolution_hz = _gyro_sample_rate_hz / _imu_gyro_fft_len;

	const float max_freq_hz = math::min(_param_imu_gyro_fft_max.get(), math::min(_gyro_sample_rate_hz / 2.f,
					    _param_imu_gyro_ratemax.get() / 2.f));

	for (int axis = 0; axis < 3; axis++) {
		// if we have enough samples begin processing
		if (_sliding_dft[axis].is_data_valid()) {

			// sum all
			float bin_mag_sum = 0;

			for (int bucket_index = 1; bucket_index < _imu_gyro_fft_len; bucket_index++) {
				bin_mag_sum += std::norm(_sliding_dft[axis].dft[bucket_index]);
			}

			sensor_gyro_fft.total_energy[axis] = bin_mag_sum;


			bool peaks_detected = false;
			uint32_t peaks_magnitude[MAX_NUM_PEAKS] {};
			int peak_index[MAX_NUM_PEAKS] {};
			float peaks_snr[MAX_NUM_PEAKS] {};

			float *peak_frequencies_raw[] {sensor_gyro_fft.peak_frequencies_x_raw, sensor_gyro_fft.peak_frequencies_y_raw, sensor_gyro_fft.peak_frequencies_z_raw};

			// start at 2 to skip DC
			// output is ordered [real[0], imag[0], real[1], imag[1], real[2], imag[2] ... real[(N/2)-1], imag[(N/2)-1]
			for (int bucket_index = 1; bucket_index < _imu_gyro_fft_len; bucket_index++) {
				const float freq_hz = bucket_index * resolution_hz;

				if (freq_hz >= max_freq_hz) {
					break;
				}

				const float fft_magnitude_squared = std::norm(_sliding_dft[axis].dft[bucket_index]);

				float snr = 10.f * log10f((_imu_gyro_fft_len - 1) * fft_magnitude_squared / (bin_mag_sum - fft_magnitude_squared));

				static constexpr float MIN_SNR = 0.f; // TODO: configurable?

				if (snr > MIN_SNR) {
					for (int i = 0; i < MAX_NUM_PEAKS; i++) {
						if (fft_magnitude_squared > peaks_magnitude[i]) {
							peaks_magnitude[i] = fft_magnitude_squared;
							peaks_snr[i] = snr;

							peak_frequencies_raw[axis][i] = freq_hz;

							peak_index[i] = bucket_index;

							peaks_detected = true;
							break;
						}
					}
				}
			}

			if (peaks_detected) {
				float *peak_frequencies[] {sensor_gyro_fft.peak_frequencies_x, sensor_gyro_fft.peak_frequencies_y, sensor_gyro_fft.peak_frequencies_z};
				float *peak_magnitude[] {sensor_gyro_fft.peak_magnitude_x, sensor_gyro_fft.peak_magnitude_y, sensor_gyro_fft.peak_magnitude_z};
				float *peak_snr[] {sensor_gyro_fft.peak_snr_x, sensor_gyro_fft.peak_snr_y, sensor_gyro_fft.peak_snr_z};

				int num_peaks_found = 0;

				for (int i = 0; i < MAX_NUM_PEAKS; i++) {
					if ((peak_index[i] > 0) && (peak_index[i] < _imu_gyro_fft_len) && (peaks_magnitude[i] > 0)) {
						const float freq = EstimatePeakFrequency(axis, peak_index[i]);

						if (freq >= _param_imu_gyro_fft_min.get() && freq <= max_freq_hz) {

							if (!PX4_ISFINITE(peak_frequencies[axis][num_peaks_found])
							    || (fabsf(peak_frequencies[axis][num_peaks_found] - freq) > 0.01f)) {

								//publish = true;
								sensor_gyro_fft.timestamp_sample = timestamp_sample;
							}

							peak_frequencies[axis][num_peaks_found] = freq;
							peak_magnitude[axis][num_peaks_found] = peaks_magnitude[i];
							peak_snr[axis][num_peaks_found] = peaks_snr[i];

							num_peaks_found++;
						}
					}
				}

				// mark remaining slots empty
				for (int i = num_peaks_found; i < MAX_NUM_PEAKS; i++) {
					peak_frequencies[axis][i] = NAN;
					peak_magnitude[axis][i] = 0;
					peak_snr[axis][i] = NAN;
				}
			}
		}
	}

	sensor_gyro_fft.device_id = _selected_sensor_device_id;
	sensor_gyro_fft.sensor_sample_rate_hz = _gyro_sample_rate_hz;
	sensor_gyro_fft.resolution_hz = resolution_hz;
	sensor_gyro_fft.timestamp = hrt_absolute_time();
	_sensor_gyro_fft_pub.publish(sensor_gyro_fft);
}

int GyroFFT::task_spawn(int argc, char *argv[])
{
	GyroFFT *instance = new GyroFFT();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int GyroFFT::print_status()
{
	PX4_INFO("gyro sample rate: %.3f Hz", (double)_gyro_sample_rate_hz);
	perf_print_counter(_cycle_perf);
	perf_print_counter(_cycle_interval_perf);
	perf_print_counter(_fft_perf);
	perf_print_counter(_gyro_generation_gap_perf);
	return 0;
}

int GyroFFT::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int GyroFFT::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("gyro_fft", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int gyro_fft_main(int argc, char *argv[])
{
	return GyroFFT::main(argc, argv);
}
