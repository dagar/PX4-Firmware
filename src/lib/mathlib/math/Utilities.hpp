/****************************************************************************
 *
 *   Copyright (c) 2020-2022 PX4 Development Team. All rights reserved.
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

#ifndef MATH_UTILITIES_H
#define MATH_UTILITIES_H

#include <matrix/math.hpp>

namespace math
{

namespace Utilities
{

// converts Tait-Bryan 312 sequence of rotations from frame 1 to frame 2
// to the corresponding rotation matrix that rotates from frame 2 to frame 1
// rot312(0) - First rotation is a RH rotation about the Z axis (rad)
// rot312(1) - Second rotation is a RH rotation about the X axis (rad)
// rot312(2) - Third rotation is a RH rotation about the Y axis (rad)
// See http://www.atacolorado.com/eulersequences.doc
template<typename FloatType>
inline matrix::Dcm<FloatType> taitBryan312ToRotMat(const matrix::Vector3<FloatType> &rot312)
{
	// Calculate the frame2 to frame 1 rotation matrix from a 312 Tait-Bryan rotation sequence
	const FloatType c2 = ::cos(rot312(2)); // third rotation is pitch
	const FloatType s2 = ::sin(rot312(2));
	const FloatType s1 = ::sin(rot312(1)); // second rotation is roll
	const FloatType c1 = ::cos(rot312(1));
	const FloatType s0 = ::sin(rot312(0)); // first rotation is yaw
	const FloatType c0 = ::cos(rot312(0));

	matrix::Dcm<FloatType> R;
	R(0, 0) = c0 * c2 - s0 * s1 * s2;
	R(1, 1) = c0 * c1;
	R(2, 2) = c2 * c1;
	R(0, 1) = -c1 * s0;
	R(0, 2) = s2 * c0 + c2 * s1 * s0;
	R(1, 0) = c2 * s0 + s2 * s1 * c0;
	R(1, 2) = s0 * s2 - s1 * c0 * c2;
	R(2, 0) = -s2 * c1;
	R(2, 1) = s1;

	return R;
}

template<typename FloatType>
inline matrix::Dcm<FloatType> quatToInverseRotMat(const matrix::Quaternion<FloatType> &quat)
{
	const FloatType q00 = quat(0) * quat(0);
	const FloatType q11 = quat(1) * quat(1);
	const FloatType q22 = quat(2) * quat(2);
	const FloatType q33 = quat(3) * quat(3);
	const FloatType q01 = quat(0) * quat(1);
	const FloatType q02 = quat(0) * quat(2);
	const FloatType q03 = quat(0) * quat(3);
	const FloatType q12 = quat(1) * quat(2);
	const FloatType q13 = quat(1) * quat(3);
	const FloatType q23 = quat(2) * quat(3);

	matrix::Dcm<FloatType> dcm;
	dcm(0, 0) = q00 + q11 - q22 - q33;
	dcm(1, 1) = q00 - q11 + q22 - q33;
	dcm(2, 2) = q00 - q11 - q22 + q33;
	dcm(1, 0) = 2.0f * (q12 - q03);
	dcm(2, 0) = 2.0f * (q13 + q02);
	dcm(0, 1) = 2.0f * (q12 + q03);
	dcm(2, 1) = 2.0f * (q23 - q01);
	dcm(0, 2) = 2.0f * (q13 - q02);
	dcm(1, 2) = 2.0f * (q23 + q01);

	return dcm;
}

// We should use a 3-2-1 Tait-Bryan (yaw-pitch-roll) rotation sequence
// when there is more roll than pitch tilt and a 3-1-2 rotation sequence
// when there is more pitch than roll tilt to avoid gimbal lock.
template<typename FloatType>
inline bool shouldUse321RotationSequence(const matrix::Dcm<FloatType> &R)
{
	return fabsf(R(2, 0)) < fabsf(R(2, 1));
}

template<typename FloatType>
inline float getEuler321Yaw(const matrix::Dcm<FloatType> &R)
{
	return atan2f(R(1, 0), R(0, 0));
}

template<typename FloatType>
inline float getEuler312Yaw(const matrix::Dcm<FloatType> &R)
{
	return ::atan2(-R(0, 1), R(1, 1));
}

template<typename FloatType>
inline FloatType getEuler321Yaw(const matrix::Quaternion<FloatType> &q)
{
	// Values from yaw_input_321.c file produced by
	// https://github.com/PX4/ecl/blob/master/matlab/scripts/Inertial%20Nav%20EKF/quat2yaw321.m
	const FloatType a = static_cast<FloatType>(2.) * (q(0) * q(3) + q(1) * q(2));
	const FloatType b = sq(q(0)) + sq(q(1)) - sq(q(2)) - sq(q(3));
	return ::atan2(a, b);
}

template<typename FloatType>
inline FloatType getEuler312Yaw(const matrix::Quaternion<FloatType> &q)
{
	// Values from yaw_input_312.c file produced by
	// https://github.com/PX4/ecl/blob/master/matlab/scripts/Inertial%20Nav%20EKF/quat2yaw312.m
	const FloatType a = 2. * (q(0) * q(3) - q(1) * q(2));
	const FloatType b = sq(q(0)) - sq(q(1)) + sq(q(2)) - sq(q(3));
	return ::atan2(a, b);
}

template<typename FloatType>
inline FloatType getEulerYaw(const matrix::Dcm<FloatType> &R)
{
	if (shouldUse321RotationSequence(R)) {
		return getEuler321Yaw(R);

	} else {
		return getEuler312Yaw(R);
	}
}

template<typename FloatType>
inline FloatType getEulerYaw(const matrix::Quaternion<FloatType> &q)
{
	return getEulerYaw(matrix::Dcm<FloatType>(q));
}

template<typename FloatType>
inline matrix::Dcm<FloatType> updateEuler321YawInRotMat(FloatType yaw, const matrix::Dcm<FloatType> &rot_in)
{
	matrix::Euler<FloatType> euler321(rot_in);
	euler321(2) = yaw;
	return matrix::Dcm<FloatType>(euler321);
}

template<typename FloatType>
inline matrix::Dcm<FloatType> updateEuler312YawInRotMat(FloatType yaw, const matrix::Dcm<FloatType> &rot_in)
{
	const matrix::Vector3<FloatType> rotVec312(yaw,  // yaw
			asinf(rot_in(2, 1)),  // roll
			atan2f(-rot_in(2, 0), rot_in(2, 2)));  // pitch
	return taitBryan312ToRotMat(rotVec312);
}

// Checks which euler rotation sequence to use and update yaw in rotation matrix
template<typename FloatType>
inline matrix::Dcm<FloatType> updateYawInRotMat(FloatType yaw, const matrix::Dcm<FloatType> &rot_in)
{
	if (shouldUse321RotationSequence(rot_in)) {
		return updateEuler321YawInRotMat(yaw, rot_in);

	} else {
		return updateEuler312YawInRotMat(yaw, rot_in);
	}
}

} // namespace Utilities
} // namespace math

#endif // MATH_UTILITIES_H
