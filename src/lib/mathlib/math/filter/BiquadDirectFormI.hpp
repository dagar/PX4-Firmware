/****************************************************************************
 *
 *   Copyright (C) 2024 PX4 Development Team. All rights reserved.
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

#include <mathlib/math/Functions.hpp>
#include <cmath>
#include <float.h>
#include <matrix/math.hpp>

namespace math
{

template<typename T>
class BiquadDirectFormI
{
public:
	BiquadDirectFormI() = default;
	~BiquadDirectFormI() = default;

	/**
	 * Add a new raw value to the filter using the Direct Form I
	 *
	 * @return retrieve the filtered result
	 */
	inline T apply(const T &sample)
	{
		if (!_initialized) {
			reset(sample);
			_initialized = true;
		}

		// Direct Form I implementation
		T output = _b0 * sample + _b1 * _delay_element_1 + _b2 * _delay_element_2
			   - _a1 * _delay_element_output_1 - _a2 * _delay_element_output_2;

		// shift inputs
		_delay_element_2 = _delay_element_1;
		_delay_element_1 = sample;

		// shift outputs
		_delay_element_output_2 = _delay_element_output_1;
		_delay_element_output_1 = output;

		return output;
	}

	const T &getState() const { return _delay_element_output_1; }

	void getCoefficients(float a[3], float b[3]) const
	{
		a[0] = 1.f;
		a[1] = _a1;
		a[2] = _a2;

		b[0] = _b0;
		b[1] = _b1;
		b[2] = _b2;
	}

	void setCoefficients(float a[2], float b[3])
	{
		_a1 = a[0];
		_a2 = a[1];

		_b0 = b[0];
		_b1 = b[1];
		_b2 = b[2];

		reset();
	}



	bool initialized() const { return _initialized; }

	void reset() { _initialized = false; }

	void reset(const T &sample)
	{
		const T input = isFinite(sample) ? sample : T{};

		_delay_element_1 = _delay_element_2 = input;
		_delay_element_output_1 = _delay_element_output_2 = input * (_b0 + _b1 + _b2) / (1 + _a1 + _a2);

		if (!isFinite(_delay_element_1) || !isFinite(_delay_element_2)) {
			_delay_element_output_1 = _delay_element_output_2 = {};
		}

		_initialized = true;
	}

	void disable()
	{
		// no filtering
		_b0 = 1.f;
		_b1 = 0.f;
		_b2 = 0.f;

		_a1 = 0.f;
		_a2 = 0.f;

		_initialized = false;
	}

protected:

	T _delay_element_1{};
	T _delay_element_2{};
	T _delay_element_output_1{};
	T _delay_element_output_2{};

	// All the coefficients are normalized by a0, so a0 becomes 1 here
	float _a1{0.f};
	float _a2{0.f};

	float _b0{1.f};
	float _b1{0.f};
	float _b2{0.f};

	bool _initialized{false};
};

} // namespace math
