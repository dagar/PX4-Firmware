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

#pragma once

namespace math
{

template<typename StateType>
class Biquad
{
public:
	Biquad(float sample_freq, float cutoff_freq)
	{
		// set initial parameters
		set_cutoff_frequency(sample_freq, cutoff_freq);
	}

	// Change filter parameters
	void set_cutoff_frequency(float sample_freq, float cutoff_freq);

	/**
	 * Add a new raw value to the filter
	 *
	 * @return retrieve the filtered result
	 */
	float apply(float sample);

	// Return the cutoff frequency
	float get_cutoff_freq() const { return _cutoff_freq; }

	// Reset the filter state to this value
	float reset(float sample);

	/**
	 * Filter a sample with the coefficients provided here and the State provided as an argument.
	 * \param s The sample to be filtered.
	 * \param state The Delay lines (instance of a state from State.h)
	 **/
	inline float filter(float s) const
	{
		return _state.filter(s, *this);
	}

protected:
	StateType _state{};

	float _cutoff_freq{0.0f};

	float _a0{0.f}; // 1st IIR coefficient
	float _a1{0.f};
	float _a2{0.f};

	float _b0{1.f};
	float _b1{0.f};
	float _b2{0.f};
};

} // namespace math



class DirectFormI
{
public:
	DirectFormI()
	{
		reset();
	}

	void reset()
	{
		_x1 = 0;
		_x2 = 0;
		_y1 = 0;
		_y2 = 0;
	}

	/**
	 * State for applying a second order section to a sample using Direct Form I
	 *
	 * Difference equation:
	 *
	 *  y[n] = (b0/a0)*x[n] + (b1/a0)*x[n-1] + (b2/a0)*x[n-2]
	 *                      - (a1/a0)*y[n-1] - (a2/a0)*y[n-2]
	 **/
	inline float filter(const float in, const Biquad &s)
	{
		float out = s._b0 * in + s._b1 * _x1 + s._b2 * _x2 - s._a1 * _y1 - s._a2 * _y2;

		_x2 = _x1;
		_y2 = _y1;
		_x1 = in;
		_y1 = out;

		return out;
	}

protected:
	float _x2{0.f}; // x[n-2]
	float _y2{0.f}; // y[n-2]
	float _x1{0.f}; // x[n-1]
	float _y1{0.f}; // y[n-1]
};

class DirectFormII
{
public:
	DirectFormII()
	{
		reset();
	}

	void reset()
	{
		_v1 = 0.f;
		_v2 = 0.f;
	}

	/**
	 * State for applying a second order section to a sample using Direct Form II
	 *
	 * Difference equation:
	 *
	 *  v[n] =         x[n] - (a1/a0)*v[n-1] - (a2/a0)*v[n-2]
	 *  y(n) = (b0/a0)*v[n] + (b1/a0)*v[n-1] + (b2/a0)*v[n-2]
	 *
	 **/
	float filter(const float in, const Biquad &s)
	{
		float w   = in - s._a1 * _v1 - s._a2 * _v2;
		float out =      s._b0 * w   + s._b1 * _v1 + s._b2 * _v2;

		_v2 = _v1;
		_v1 = w;

		return out;
	}

private:
	float _v1{0.f}; // v[-1]
	float _v2{0.f}; // v[-2]
};


class TransposedDirectFormII
{
public:
	TransposedDirectFormII()
	{
		reset();
	}

	void reset()
	{
		_s1 = 0;
		_s1_1 = 0;
		_s2 = 0;
		_s2_1 = 0;
	}

	inline float filter(const float in, const Biquad &s)
	{
		float out = _s1_1 + s._b0 * in;
		_s1 = _s2_1 + s._b1 * in - s._a1 * out;
		_s2 = s._b2 * in - s._a2 * out;
		_s1_1 = _s1;
		_s2_1 = _s2;

		return out;
	}

private:
	float _s1{0.f};
	float _s1_1{0.f};
	float _s2{0.f};
	float _s2_1{0.f};
};
