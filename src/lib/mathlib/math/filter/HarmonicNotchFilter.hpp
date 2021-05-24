/****************************************************************************
 *
 *   Copyright (C) 2021 PX4 Development Team. All rights reserved.
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

/*
 * @file NotchFilter.hpp
 *
 * @brief Implementation of a Notch filter.
 *
 * @author Mathieu Bresciani <brescianimathieu@gmail.com>
 * @author Samuel Garcin <samuel.garcin@wecorpindustries.com>
 */

#pragma once

#include <px4_platform_common/defines.h>
#include <cmath>
#include <float.h>
#include <matrix/math.hpp>

namespace math
{

template<typename T>
class HarmonicNotchFilter
{
public:
	HarmonicNotchFilter() = default;
	~HarmonicNotchFilter() = default;

	void setParameters(float sample_freq, float notch_freq, float bandwidth)
	{
		_notch_freq = notch_freq; // math::max(notch_freq, 0.0003f * sample_freq);
		_bandwidth = bandwidth; // math::max(bandwidth, 0.000625f * sample_freq);
		_sample_freq = sample_freq;

		if (notch_freq <= 0.f || notch_freq >= (sample_freq / 2.f)) {
			disable();
			return;
		}

		// TODO: add protections
		//   - bandwidth not below certain percentage of sample_freq
		//   - notch_freq not below certain percentage of sample_freq
		//   - notch_freq not exceed nyquist

		const double alpha = tan(M_PI * (double)bandwidth / (double)sample_freq);
		const double beta0 = -cos(2. * M_PI * (double)notch_freq * 1 / (double)sample_freq);
		const double beta1 = -cos(2. * M_PI * (double)notch_freq * 2 / (double)sample_freq);
		const double beta2 = -cos(2. * M_PI * (double)notch_freq * 3 / (double)sample_freq);
		const double a0_inv = 1. / (alpha + 1.);

		_b0 = a0_inv;
		_b1[0] = 2.0 * beta0 * a0_inv;
		_b1[1] = 2.0 * beta1 * a0_inv;
		_b1[2] = 2.0 * beta2 * a0_inv;
		_b2 = a0_inv;

		_a1[0] = _b1[0];
		_a1[1] = _b1[1];
		_a1[2] = _b1[2];
		_a2 = (1.0 - alpha) * a0_inv;
	}

	/**
	 * Add a new raw value to the filter using the Direct form II
	 *
	 * @return retrieve the filtered result
	 */
	inline T apply(const T &sample)
	{
		// Direct Form I implementation
		const T output = _b0 * (double)sample
				 + _b1 * _delay_element_1
				 + _b2 * _delay_element_2
				 - _a1 * _delay_element_output_1
				 - _a2 * _delay_element_output_2;

		// shift inputs
		_delay_element_2 = _delay_element_1;
		_delay_element_1 = sample;

		// shift outputs
		_delay_element_output_2 = _delay_element_output_1;
		_delay_element_output_1 = output;

		return output;
	}

	/**
	 * Add new raw values to the filter using the Direct form I.
	 *
	 * @return retrieve the filtered result
	 */
	inline void applyArray(T samples[], uint8_t num_samples)
	{
		bool finite = true;

		for (int n = 0; n < num_samples; n++) {
			if (_count == 0) {
				_delay_element_1 = samples[n];
				_delay_element_output_1 = samples[n];

			} else if (_count == 1) {
				_delay_element_2 = _delay_element_1;
				_delay_element_output_2 = _delay_element_output_1;
				_delay_element_1 = samples[n];
				_delay_element_output_1 = samples[n];

			} else {
				// Direct Form I implementation
				double output = _b0 * (double)samples[n] + _b1 * _delay_element_1 + _b2 * _delay_element_2
						- _a1 * _delay_element_output_1 -
						_a2 * _delay_element_output_2;

				// don't allow bad values to propagate via the filter
				if (!isFinite(output)) {
					_count = 0;
					finite = false;
					output = samples[n];
				}





				// shift inputs
				_delay_element_2 = _delay_element_1;
				_delay_element_1 = samples[n];

				// shift outputs
				_delay_element_output_2 = _delay_element_output_1;
				_delay_element_output_1 = output;

				// writes value to array
				if (_initialized) {
					samples[n] = output;
				}
			}

			_count++;

		}

		if (!_initialized && finite) {
			if (_count > 100) {
				_initialized = true;
			}
		}
	}

	float getNotchFreq() const { return _notch_freq; }
	float getBandwidth() const { return _bandwidth; }

	/**
	 * Bypasses the filter update to directly set different filter coefficients.
	 * Note: the filtered frequency and quality factor saved on the filter lose their
	 * physical meaning if you use this method to change the filter's coefficients.
	 * Used for creating clones of a specific filter.
	 */
	void setCoefficients(float a[2], float b[3])
	{
		_a1 = a[0];
		_a2 = a[1];
		_b0 = b[0];
		_b1 = b[1];
		_b2 = b[2];
	}

	void reset()
	{
		_delay_element_1 = 0;
		_delay_element_2 = 0;
		_delay_element_output_1 = 0;
		_delay_element_output_2 = 0;

		_count = 0;
		_initialized = false;
	}

	void reset(const T &sample1, const T &sample2)
	{
		// _delay_element_1 = sample;
		// _delay_element_2 = sample;
		// _delay_element_output_1 = sample;
		// _delay_element_output_2 = sample;

		_delay_element_1 = 0;
		_delay_element_2 = 0;
		_delay_element_output_1 = 0;
		_delay_element_output_2 = 0;

		apply(sample2);
		apply(sample1);

		// for (int i = 0; i < 100; i++) {
		// 	const T output = apply(sample);

		// 	if (fabsf(output - sample) > 1.f) {
		// 		fprintf(stderr, "reset large error (%d/%d) sample: %.6f, output: %.6f, error=%.6f\n", i + 1, 100, (double)sample,
		// 			(double)output, (double)(sample - output));
		// 	}
		// }

		_count = 0;
		_initialized = false;
	}

	void disable()
	{
		// no filtering
		_b0 = 1.0f;
		_b1 = 0.0f;
		_b2 = 0.0f;

		_a1 = 0.0f;
		_a2 = 0.0f;

		_initialized = false;
	}

protected:
	float _notch_freq{};
	float _bandwidth{};
	float _sample_freq{};

	// All the coefficients are normalized by a0, so a0 becomes 1 here
	double _a0{1.f};
	double _a1[3] {};
	double _a2{};

	double _b1[3] {};
	double _b2{};

	double _delay_element_1;
	double _delay_element_2;
	double _delay_element_output_1[3];
	double _delay_element_output_2[3];

	bool _initialized{false};

	int _count{0};
};

} // namespace math
