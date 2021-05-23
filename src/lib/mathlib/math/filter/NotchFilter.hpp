/****************************************************************************
 *
 *   Copyright (C) 2019 PX4 Development Team. All rights reserved.
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

inline bool isFinite(const float &value)
{
	return PX4_ISFINITE(value);
}

inline bool isFinite(const matrix::Vector3f &value)
{
	return PX4_ISFINITE(value(0)) && PX4_ISFINITE(value(1)) && PX4_ISFINITE(value(2));
}

template<typename T>
class NotchFilter
{
public:
	NotchFilter() = default;
	~NotchFilter() = default;

	void setParameters(float sample_freq, float notch_freq, float bandwidth)
	{
		_notch_freq = notch_freq;
		_bandwidth = bandwidth;
		_sample_freq = sample_freq;

		if (notch_freq <= 0.f) {
			disable();
			return;
		}

		// TODO: add protections
		//   - bandwidth not below certain percentage of sample_freq
		//   - notch_freq not below certain percentage of sample_freq
		//   - notch_freq not exceed nyquist

		const double alpha = tan(M_PI * (double)bandwidth / (double)sample_freq);
		const double beta = -cos(2. * M_PI * (double)notch_freq / (double)sample_freq);
		const double a0_inv = 1. / (alpha + 1.);

		_b0 = a0_inv;
		_b1 = 2.0 * beta * a0_inv;
		_b2 = a0_inv;

		_a1 = _b1;
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
			// Direct Form II implementation
			double output = _b0 * (double)samples[n] + _b1 * _delay_element_1 + _b2 * _delay_element_2 - _a1 *
					_delay_element_output_1 -
					_a2 * _delay_element_output_2;

			// if (_count < 1000) {
			// 	fprintf(stderr, "%d: %.4f -> %.4f (delay in:[%.4f, %.4f] out:[%.4f, %.4f])\n",
			// 		_count++, (double)samples[n], (double)output,
			// 		(double)_delay_element_1, (double)_delay_element_2,
			// 		(double)_delay_element_output_1, (double)_delay_element_output_2
			// 	       );
			// }

			// don't allow bad values to propagate via the filter
			if (!isFinite(output)) {
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

		if (!_initialized && finite) {
			if (_count++ > 100) {
				_initialized = true;
			}
		}
	}

	float getNotchFreq() const { return _notch_freq; }
	float getBandwidth() const { return _bandwidth; }

	// Used in unit test only
	void getCoefficients(float a[3], float b[3]) const
	{
		a[0] = _a0;
		a[1] = _a1;
		a[2] = _a2;
		b[0] = _b0;
		b[1] = _b1;
		b[2] = _b2;
	}

	float getMagnitudeResponse(float frequency) const
	{
		// float w = 2.f * M_PI_F * frequency / _sample_freq;

		// float numerator = _b0 * _b0 + _b1 * _b1 + _b2 * _b2
		// 		  + 2.f * (_b0 * _b1 + _b1 * _b2) * cosf(w) + 2.f * _b0 * _b2 * cosf(2.f * w);

		// float denominator = 1.f + _a1 * _a1 + _a2 * _a2 + 2.f * (_a1 + _a1 * _a2) * cosf(w) + 2.f * _a2 * cosf(2.f * w);

		// return sqrtf(numerator / denominator);
		return 0;
	}

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

	void setParametersQ(float Q = 0.707f)
	{
		float K = tanf(M_PI_F * _sample_freq);

		float norm = 1.f / (1.f + K / Q + K * K);

		_a0 = (1.f + K * K) * norm;
		_a1 = 2.f * (K * K - 1) * norm;
		_a2 = _a0;

		_b1 = _a1;
		_b2 = (1.f - K / Q + K * K) * norm;
	}

	void PrintStatus()
	{
		printf("%.1f/%.1f Hz BW: %.1f a: [%.6f, %.6f, %.6f] b: [%.6f, %.6f, %.6f]\n",
		       (double)_notch_freq, (double)_sample_freq, (double)_bandwidth,
		       (double)1.f, (double)_a1, (double)_a2,
		       (double)_b0, (double)_b1, (double)_b2);
	}

protected:
	float _notch_freq{};
	float _bandwidth{};
	float _sample_freq{};

	// All the coefficients are normalized by a0, so a0 becomes 1 here
	double _a0{1.f};
	double _a1{};
	double _a2{};

	double _b0{1.f};
	double _b1{};
	double _b2{};

	double _delay_element_1;
	double _delay_element_2;
	double _delay_element_output_1;
	double _delay_element_output_2;

	bool _initialized{false};

	int _count{0};
};

} // namespace math
