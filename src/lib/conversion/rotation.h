/****************************************************************************
 *
 *   Copyright (C) 2013 PX4 Development Team. All rights reserved.
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
 * @file rotation.h
 *
 * Vector rotation library
 */

#ifndef ROTATION_H_
#define ROTATION_H_

#include <stdint.h>
#include <unistd.h>

#include <mathlib/mathlib.h>
#include <matrix/math.hpp>

/**
 * Enum for board and external compass rotations.
 * This enum maps from board attitude to airframe attitude.
 */
enum Rotation {
	ROTATION_NONE                = 0,
	ROTATION_YAW_45              = 1,
	ROTATION_YAW_90              = 2,
	ROTATION_YAW_135             = 3,
	ROTATION_YAW_180             = 4,
	ROTATION_YAW_225             = 5,
	ROTATION_YAW_270             = 6,
	ROTATION_YAW_315             = 7,
	ROTATION_ROLL_180            = 8,
	ROTATION_ROLL_180_YAW_45     = 9,
	ROTATION_ROLL_180_YAW_90     = 10,
	ROTATION_ROLL_180_YAW_135    = 11,
	ROTATION_PITCH_180           = 12,
	ROTATION_ROLL_180_YAW_225    = 13,
	ROTATION_ROLL_180_YAW_270    = 14,
	ROTATION_ROLL_180_YAW_315    = 15,
	ROTATION_ROLL_90             = 16,
	ROTATION_ROLL_90_YAW_45      = 17,
	ROTATION_ROLL_90_YAW_90      = 18,
	ROTATION_ROLL_90_YAW_135     = 19,
	ROTATION_ROLL_270            = 20,
	ROTATION_ROLL_270_YAW_45     = 21,
	ROTATION_ROLL_270_YAW_90     = 22,
	ROTATION_ROLL_270_YAW_135    = 23,
	ROTATION_PITCH_90            = 24,
	ROTATION_PITCH_270           = 25,
	ROTATION_PITCH_180_YAW_90    = 26,
	ROTATION_PITCH_180_YAW_270   = 27,
	ROTATION_ROLL_90_PITCH_90    = 28,
	ROTATION_ROLL_180_PITCH_90   = 29,
	ROTATION_ROLL_270_PITCH_90   = 30,
	ROTATION_ROLL_90_PITCH_180   = 31,
	ROTATION_ROLL_270_PITCH_180  = 32,
	ROTATION_ROLL_90_PITCH_270   = 33,
	ROTATION_ROLL_180_PITCH_270  = 34,
	ROTATION_ROLL_270_PITCH_270  = 35,
	ROTATION_ROLL_90_PITCH_180_YAW_90 = 36,
	ROTATION_ROLL_90_YAW_270          = 37,
	ROTATION_ROLL_90_PITCH_68_YAW_293 = 38,
	ROTATION_PITCH_315                = 39,
	ROTATION_ROLL_90_PITCH_315        = 40,
	ROTATION_ROLL_270_YAW_180         = 41,

	ROTATION_MAX
};

typedef struct {
	uint16_t roll;
	uint16_t pitch;
	uint16_t yaw;
} rot_lookup_t;

static constexpr rot_lookup_t rot_lookup[] = {
	{  0,   0,   0 },
	{  0,   0,  45 },
	{  0,   0,  90 },
	{  0,   0, 135 },
	{  0,   0, 180 },
	{  0,   0, 225 },
	{  0,   0, 270 },
	{  0,   0, 315 },
	{180,   0,   0 },
	{180,   0,  45 },
	{180,   0,  90 },
	{180,   0, 135 },
	{  0, 180,   0 },
	{180,   0, 225 },
	{180,   0, 270 },
	{180,   0, 315 },
	{ 90,   0,   0 },
	{ 90,   0,  45 },
	{ 90,   0,  90 },
	{ 90,   0, 135 },
	{270,   0,   0 },
	{270,   0,  45 },
	{270,   0,  90 },
	{270,   0, 135 },
	{  0,  90,   0 },
	{  0, 270,   0 },
	{  0, 180,  90 },
	{  0, 180, 270 },
	{ 90,  90,   0 },
	{180,  90,   0 },
	{270,  90,   0 },
	{ 90, 180,   0 },
	{270, 180,   0 },
	{ 90, 270,   0 },
	{180, 270,   0 },
	{270, 270,   0 },
	{ 90, 180,  90 },
	{ 90,   0, 270 },
	{ 90,  68, 293 },
	{  0, 315,   0 },
	{ 90, 315,   0 },
	{270,   0, 180 },
};

/**
 * Get the rotation matrix
 */
matrix::Dcmf get_rot_matrix(enum Rotation rot);

/**
 * Get the rotation quaternion
 */
matrix::Quatf get_rot_quaternion(enum Rotation rot);

template<typename T>
static constexpr bool rotate_swap(enum Rotation rot, T &x, T &y, T &z)
{
	switch (rot) {
	case ROTATION_YAW_90: {
			T tmp = x;
			x = -y;
			y = tmp;
			return true;
		}

	case ROTATION_YAW_180:
		x = -x;
		y = -y;
		return true;

	case ROTATION_YAW_270: {
			T tmp = x;
			x = y;
			y = -tmp;
			return true;
		}

	case ROTATION_ROLL_180:
		y = -y;
		z = -z;
		return true;

	case ROTATION_ROLL_180_YAW_90:

	// FALLTHROUGH
	case ROTATION_PITCH_180_YAW_270: {
			T tmp = x;
			x = y;
			y = tmp;
			z = -z;
			return true;
		}

	case ROTATION_PITCH_180:
		x = -x;
		z = -z;
		return true;

	case ROTATION_ROLL_180_YAW_270:

	// FALLTHROUGH
	case ROTATION_PITCH_180_YAW_90: {
			T tmp = x;
			x = -y;
			y = -tmp;
			z = -z;
			return true;
		}

	case ROTATION_ROLL_90: {
			T tmp = z;
			z = y;
			y = -tmp;
			return true;
		}

	case ROTATION_ROLL_90_YAW_90: {
			T tmp = x;
			x = z;
			z = y;
			y = tmp;
			return true;
		}

	case ROTATION_ROLL_270: {
			T tmp = z;
			z = -y;
			y = tmp;
			return true;
		}

	case ROTATION_ROLL_270_YAW_90: {
			T tmp = x;
			x = -z;
			z = -y;
			y = tmp;
			return true;
		}

	case ROTATION_PITCH_90: {
			T tmp = z;
			z = -x;
			x = tmp;
			return true;
		}

	case ROTATION_PITCH_270: {
			T tmp = z;
			z = x;
			x = -tmp;
			return true;
		}

	case ROTATION_ROLL_180_PITCH_270: {
			T tmp = z;
			z = x;
			x = tmp;
			y = -y;
			return true;
		}

	case ROTATION_ROLL_90_YAW_270: {
			T tmp = x;
			x = -z;
			z = y;
			y = -tmp;
			return true;
		}

	case ROTATION_ROLL_90_PITCH_90: {
			T tmp = x;
			x = y;
			y = -z;
			z = -tmp;
			return true;
		}

	case ROTATION_ROLL_180_PITCH_90: {
			T tmp = x;
			x = -z;
			y = -y;
			z = -tmp;
			return true;
		}

	case ROTATION_ROLL_270_PITCH_90: {
			T tmp = x;
			x = -y;
			y = z;
			z = -tmp;
			return true;
		}

	case ROTATION_ROLL_90_PITCH_180:

	// FALLTHROUGH
	case ROTATION_ROLL_270_YAW_180: {
			T tmp = y;
			x = -x;
			y = -z;
			z = -tmp;
			return true;
		}

	case ROTATION_ROLL_270_PITCH_180: {
			T tmp = y;
			x = -x;
			y = z;
			z = tmp;
			return true;
		}

	case ROTATION_ROLL_90_PITCH_270: {
			T tmp = x;
			x = -y;
			y = -z;
			z = tmp;
			return true;
		}

	case ROTATION_ROLL_270_PITCH_270: {
			T tmp = x;
			x = y;
			y = z;
			z = tmp;
			return true;
		}

	case ROTATION_ROLL_90_PITCH_180_YAW_90: {
			T tmp = x;
			x = z;
			z = -y;
			y = -tmp;
			return true;
		}

	default:
		break;
	}

	return false;
}

static constexpr void rotate_3i(enum Rotation rot, int16_t &x, int16_t &y, int16_t &z)
{
	if (!rotate_swap(rot, x, y, z)) {
		// otherwise use rotation matrix
		const matrix::Vector3f r{get_rot_matrix(rot) *matrix::Vector3f{(float)x, (float)y, (float)z}};
		x = roundf(r(0));
		y = roundf(r(1));
		z = roundf(r(2));
	}
}

/**
 * rotate a 3 element float vector in-place
 */
static constexpr void rotate_3f(enum Rotation rot, float &x, float &y, float &z)
{
	if (!rotate_swap(rot, x, y, z)) {
		// otherwise use rotation matrix
		const matrix::Vector3f r{get_rot_matrix(rot) *matrix::Vector3f{(float)x, (float)y, (float)z}};
		x = r(0);
		y = r(1);
		z = r(2);
	}
}


matrix::Vector3f rotated_3f(enum Rotation rot, const matrix::Vector3f &v);

#endif /* ROTATION_H_ */
