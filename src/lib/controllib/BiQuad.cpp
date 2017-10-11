/****************************************************************************
 *
 *   Copyright (C) 2017 PX4 Development Team. All rights reserved.
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

/**
 * @file BiQuad.hpp
 *
 * https://en.wikipedia.org/wiki/Digital_biquad_filter
 *
 * Using Direct form 1 which is the most
 * numerically stable.
 */

#pragma once

#include <px4_defines.h>
#include <cstdint>
#include "matrix/math.hpp"

namespace control
{

template<class Type, size_t M>
class __EXPORT BiQuad
{
public:
// methods

	/**
	 * p = where b coeffs are num, a coeffs are denom
	 *
	 * a0 is always 1 (normalized biquad)
	 */
	BiQuad(Type b0, Type b1, Type b2, Type a1, Type a2) :
		_b0(b0),
		_b1(b1),
		_b2(b2),
		_a1(a1),
		_a2(a2),
		_xp(),
		_xpp(),
		_yp(),
		_ypp(),
		_initialized(false)
	{
	};

	matrix::Vector<Type, M> update(matrix::Matrix<Type, M, 1> x)
	{
		// make sure input is finite
		for (int i = 0; i < M; i++) {
			if (!PX4_ISFINITE(x(i, 0))) {
				x(i, 0) = 0;
			}
		}

		// initialization
		if (!_initialized) {
			// setting to input as we are using this for
			// low pass typically
			_xp = x;
			_yp = x;
			_xpp = x;
			_ypp = x;
			_initialized = true;
		}

		// difference equation
		matrix::Vector<Type, M> y = _b0 * x + _b1 * _xp + _b2 * _xpp - _a1 * _yp - _a2 * _ypp;

		// make sure output is finite
		for (int i = 0; i < M; i++) {
			if (!PX4_ISFINITE(y(i))) {
				y(i) = 0;
			}
		}

		// update history
		_xpp = _xp;
		_ypp = _yp;
		_xp = x;
		_yp = y;

		return y;
	}
private:
// attributes
	Type _b0;
	Type _b1;
	Type _b2;
	Type _a1;
	Type _a2;
	matrix::Vector<Type, M> _xp; // x[k-1]
	matrix::Vector<Type, M> _xpp; // x[k-2]
	matrix::Vector<Type, M> _yp; // y[k-1]
	matrix::Vector<Type, M> _ypp; // y[k-2]
	bool _initialized;
};

} // namespace control
