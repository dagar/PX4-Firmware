/****************************************************************************
 *
 *   Copyright (C) 2020 PX4 Development Team. All rights reserved.
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
 * @file LowPassFilter.hpp
 *
 * @brief Implementation of a low-pass filter.
 *
 */

#pragma once

#include <px4_platform_common/defines.h>
#include <cmath>
#include <float.h>
#include <matrix/math.hpp>

namespace math
{

template<typename T>
class LowPassFilter
{
public:
	LowPassFilter() = default;
	~LowPassFilter() = default;

	void setParameters(float sample_freq, float cutoff_freq);

	/**
	 * Add a new raw value to the filter
	 *
	 * @return retrieve the filtered result
	 */
	inline T apply(const T &sample)
	{
		// Direct Form II implementation
		const T delay_element_0{sample - _delay_element_1 *_a1 - _delay_element_2 * _a2};
		const T output{delay_element_0 *_b0 + _delay_element_1 *_b1 + _delay_element_2 * _b2};

		_delay_element_2 = _delay_element_1;
		_delay_element_1 = delay_element_0;

		return output;
	}

	float getCutoffFreq() const { return _cutoff_freq; }

	T reset(const T &sample);

protected:
	float _cutoff_freq{0.f};

	float _a1{0.f};
	float _a2{0.f};

	float _b0{1.f};
	float _b1{0.f};
	float _b2{0.f};

	T _delay_element_1{};
	T _delay_element_2{};
};

template<typename T>
void LowPassFilter<T>::setParameters(float sample_freq, float cutoff_freq)
{
	_cutoff_freq = cutoff_freq;

	// reset delay elements on filter change
	_delay_element_1 = {};
	_delay_element_2 = {};

	if (cutoff_freq <= 0.f) {
		// no filtering
		_b0 = 1.0f;
		_b1 = 0.0f;
		_b2 = 0.0f;

		_a1 = 0.0f;
		_a2 = 0.0f;

		return;
	}

	const float fr = sample_freq / _cutoff_freq;
	const float ohm = tanf(M_PI_F / fr);
	const float c = 1.0f + 2.0f * cosf(M_PI_F / 4.0f) * ohm + ohm * ohm;

	_b0 = ohm * ohm / c;
	_b1 = 2.0f * _b0;
	_b2 = _b0;

	_a1 = 2.0f * (ohm * ohm - 1.0f) / c;
	_a2 = (1.0f - 2.0f * cosf(M_PI_F / 4.0f) * ohm + ohm * ohm) / c;
}

template<typename T>
T LowPassFilter<T>::reset(const T &sample)
{
	T dval = sample;

	if (fabsf(_b0 + _b1 + _b2) > FLT_EPSILON) {
		dval = dval / (_b0 + _b1 + _b2);
	}

	_delay_element_1 = dval;
	_delay_element_2 = dval;

	return apply(sample);
}

} // namespace math
