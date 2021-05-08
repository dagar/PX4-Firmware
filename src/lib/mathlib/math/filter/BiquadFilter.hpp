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

#pragma once

#include <math.h>

#include <px4_platform_common/defines.h>

namespace math
{

inline bool isFinite(const float &value)
{
	return PX4_ISFINITE(value);
}

inline bool isFinite(const matrix::Vector2f &value)
{
	return PX4_ISFINITE(value(0)) && PX4_ISFINITE(value(1));
}

inline bool isFinite(const matrix::Vector3f &value)
{
	return PX4_ISFINITE(value(0)) && PX4_ISFINITE(value(1)) && PX4_ISFINITE(value(2));
}

template<typename T>
class BiquadFilter
{
public:
	BiquadFilter() = default;

	// Transposed Direct Form II
	inline T apply(const T &in)
	{
		const T out = in * _a[0] + _z1;
		_z1 = in * _a[1] + _z2 - b[1] * out;
		_z2 = in * a[2] - b[2] * out;
		return out;
	}

	// Filter array of samples in place using Transposed Direct Form II
	inline void applyArray(T samples[], int num_samples)
	{
		for (int n = 0; n < num_samples; n++) {
			const T in = samples[n];
			samples[n] = in * _a[0] + _z1;
			_z1 = in * _a[1] + _z2 - _b[1] * samples[n];
			_z2 = in * _a[2] - _b[2] * samples[n];
		}
	}

	// Return the cutoff frequency
	float cutoff_freq() const { return _cutoff_freq; }

	float getMagnitudeResponse(float frequency) const
	{
		float w = 2.f * M_PI * frequency / _sample_freq;

		float numerator = _b[0] * _b[0] + _b[1] * _b[1] + _b[2] * _b[2]
				  + 2.f * (_b[0] * _b[1] + _b[1] * _b[2]) * cosf(w) + 2.f * _b[0] * _b[2] * cosf(2.f * w);

		float denominator = 1.f + _a[1] * _a[1] + _a[2] * _a[2] + 2.f * (_a[1] + _a[1] * _a[2]) * cosf(w) + 2.f * _a[2] * cosf(
					    2.f * w);

		return sqrtf(numerator / denominator);
	}

	// Used in unit test only
	void getCoefficients(float a[3], float b[3]) const
	{
		a[0] = _a[0];
		a[1] = _a[1];
		a[2] = _a[2];
		b[0] = _b[0];
		b[1] = _b[1];
		b[2] = _b[2];
	}

	// Reset the filter state to this value
	T reset(const T &sample)
	{
		const float denominator = _b[0] + _b[1] + _b[2];

		if (fabsf(denominator) > 0.f) {
			const T dval = sample / denominator;
			_z1 = dval;
			_z2 = dval;

		} else {
			_z1 = sample;
			_z2 = sample;
		}

		return apply(sample);
	}

protected:

	float _cutoff_freq{0.f};
	float _sample_freq{0.f};

	float _a[3] {};
	float _b[3] {};

	T _z1{}; // buffered sample -1
	T _z2{}; // buffered sample -2
};

template<typename T>
class ButterworthLowPass final : public BiquadFilter<T>
{
public:
	ButterworthLowPass(float sample_freq, float cutoff_freq)
	{
		// set initial parameters
		set_lowpass_butterworth(sample_freq, cutoff_freq);
	}

	void set_lowpass_butterworth(float sample_freq, float cutoff_freq)
	{
		double Q = 0.7071;
		double peakGain = 6;

		double V = pow(10, fabs(peakGain) / 20.0);
		double K = tan(M_PI * cutoff_freq);

		double norm = 1 / (1 + K / Q + K * K);
		_a[0] = K * K * norm;
		_a[1] = 2 * _a[0];
		_a[2] = _a[0];
		_b[0] = 1.f; // ?
		_b[1] = 2 * (K * K - 1) * norm;
		_b[2] = (1 - K / Q + K * K) * norm;
	}
};

template<typename T>
class ButterworthLowPass final : public BiquadFilter<T>
{
public:
	ButterworthLowPass(float sample_freq, float cutoff_freq)
	{
		// set initial parameters
		set_lowpass_butterworth(sample_freq, cutoff_freq);
	}

	void set_lowpass_butterworth(float sample_freq, float cutoff_freq)
	{
		double Q = 0.7071;
		double peakGain = 6;

		double V = pow(10, fabs(peakGain) / 20.0);
		double K = tan(M_PI * cutoff_freq);

		double norm = 1 / (1 + K / Q + K * K);
		_a[0] = K * K * norm;
		_a[1] = 2 * _a[0];
		_a[2] = _a[0];
		_b[0] = 1.f; // ?
		_b[1] = 2 * (K * K - 1) * norm;
		_b[2] = (1 - K / Q + K * K) * norm;
	}
};


template<typename T>
class NotchFilter final : public BiquadFilter<T>
{
public:
	NotchFilter() = default;

	void setParameters(float sample_freq, float notch_freq, float bandwidth)
	{
		double Q = 0.7071;
		double peakGain = 6;

		double V = pow(10, fabs(peakGain) / 20.0);
		double K = tan(M_PI * cutoff_freq);

		double norm = 1 / (1 + K / Q + K * K);
		_a[0] = (1 + K * K) * norm;
		_a[1] = 2 * (K * K - 1) * norm;
		_a[2] = _a[0];
		_b[0] = 1.f;
		_b[1] = _a[1];
		_b[2] = (1 - K / Q + K * K) * norm;
	}

private:

	float _bandwidth{0.f};
};

} // namespace math
