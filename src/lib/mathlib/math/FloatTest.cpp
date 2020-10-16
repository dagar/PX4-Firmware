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

#include <gtest/gtest.h>
#include "Float.hpp"

using namespace math;

static constexpr float inf = 1/0.0;
static constexpr float minus_inf = -1/0.0;
static constexpr float minus_zero = -1 / inf;
static constexpr float nan = 0.0/0.0;

TEST(FloatTest, big)
{
	ASSERT_TRUE(nearlyEqual(1000000.f, 1000001.f));
	ASSERT_TRUE(nearlyEqual(1000001.f, 1000000.f));
	ASSERT_FALSE(nearlyEqual(10000.f, 10001.f));
	ASSERT_FALSE(nearlyEqual(10001.f, 10000.f));
}

TEST(FloatTest, negativeLargeNumbers)
{
	ASSERT_TRUE(nearlyEqual(-1000000.f, -1000001.f));
	ASSERT_TRUE(nearlyEqual(-1000001.f, -1000000.f));
	ASSERT_FALSE(nearlyEqual(-10000.f, -10001.f));
	ASSERT_FALSE(nearlyEqual(-10001.f, -10000.f));
}

TEST(FloatTest, nearOne)
{
	ASSERT_TRUE(nearlyEqual(1.0000001f, 1.0000002f));
	ASSERT_TRUE(nearlyEqual(1.0000002f, 1.0000001f));
	ASSERT_FALSE(nearlyEqual(1.0002f, 1.0001f));
	ASSERT_FALSE(nearlyEqual(1.0001f, 1.0002f));
}

TEST(FloatTest, nearNegativeOne)
{
	ASSERT_TRUE(nearlyEqual(-1.000001f, -1.000002f));
	ASSERT_TRUE(nearlyEqual(-1.000002f, -1.000001f));
	ASSERT_FALSE(nearlyEqual(-1.0001f, -1.0002f));
	ASSERT_FALSE(nearlyEqual(-1.0002f, -1.0001f));
}

TEST(FloatTest, zeroAndOne)
{
	ASSERT_TRUE(nearlyEqual(0.000000001000001f, 0.000000001000002f));
	ASSERT_TRUE(nearlyEqual(0.000000001000002f, 0.000000001000001f));
	ASSERT_FALSE(nearlyEqual(0.000000000001002f, 0.000000000001001f));
	ASSERT_FALSE(nearlyEqual(0.000000000001001f, 0.000000000001002f));
}

TEST(FloatTest, negativeOneAndZero)
{
	ASSERT_TRUE(nearlyEqual(-0.000000001000001f, -0.000000001000002f));
	ASSERT_TRUE(nearlyEqual(-0.000000001000002f, -0.000000001000001f));
	ASSERT_FALSE(nearlyEqual(-0.000000000001002f, -0.000000000001001f));
	ASSERT_FALSE(nearlyEqual(-0.000000000001001f, -0.000000000001002f));
}

TEST(FloatTest, smallDiff)
{
	ASSERT_TRUE(nearlyEqual(0.3f, 0.30000003f));
	ASSERT_TRUE(nearlyEqual(-0.3f, -0.30000003f));
}

TEST(FloatTest, zeroComparisons)
{
	ASSERT_TRUE(nearlyEqual(0.0f, 0.0f));
	ASSERT_TRUE(nearlyEqual(0.0f, -0.0f));
	ASSERT_TRUE(nearlyEqual(-0.0f, -0.0f));
	ASSERT_FALSE(nearlyEqual(0.00000001f, 0.0f));
	ASSERT_FALSE(nearlyEqual(0.0f, 0.00000001f));
	ASSERT_FALSE(nearlyEqual(-0.00000001f, 0.0f));
	ASSERT_FALSE(nearlyEqual(0.0f, -0.00000001f));

	ASSERT_TRUE(nearlyEqual(0.0f, 1e-40f, 0.01f));
	ASSERT_TRUE(nearlyEqual(1e-40f, 0.0f, 0.01f));
	ASSERT_FALSE(nearlyEqual(1e-40f, 0.0f, 0.000001f));
	ASSERT_FALSE(nearlyEqual(0.0f, 1e-40f, 0.000001f));

	ASSERT_TRUE(nearlyEqual(0.0f, -1e-40f, 0.1f));
	ASSERT_TRUE(nearlyEqual(-1e-40f, 0.0f, 0.1f));
	ASSERT_FALSE(nearlyEqual(-1e-40f, 0.0f, 0.00000001f));
	ASSERT_FALSE(nearlyEqual(0.0f, -1e-40f, 0.00000001f));
}

// Comparisons involving extreme values (overflow potential)
TEST(FloatTest, extremeValues)
{
	ASSERT_TRUE(nearlyEqual(FLT_MAX, FLT_MAX));
	ASSERT_FALSE(nearlyEqual(FLT_MAX, -FLT_MAX));
	ASSERT_FALSE(nearlyEqual(-FLT_MAX, FLT_MAX));
	ASSERT_FALSE(nearlyEqual(FLT_MAX, FLT_MAX / 2));
	ASSERT_FALSE(nearlyEqual(FLT_MAX, -FLT_MAX / 2));
	ASSERT_FALSE(nearlyEqual(-FLT_MAX, FLT_MAX / 2));
}

TEST(FloatTest, infinities)
{
	ASSERT_TRUE(nearlyEqual(INFINITY, INFINITY));
	ASSERT_TRUE(nearlyEqual(-INFINITY, -INFINITY));
	ASSERT_FALSE(nearlyEqual(-INFINITY, INFINITY));
	ASSERT_FALSE(nearlyEqual(INFINITY, FLT_MAX));
	ASSERT_FALSE(nearlyEqual(-INFINITY, -FLT_MAX));
}

TEST(FloatTest, nan)
{
	ASSERT_FALSE(nearlyEqual(NAN, NAN));
	ASSERT_FALSE(nearlyEqual(NAN, 0.0f));
	ASSERT_FALSE(nearlyEqual(-0.0f, NAN));
	ASSERT_FALSE(nearlyEqual(NAN, -0.0f));
	ASSERT_FALSE(nearlyEqual(0.0f, NAN));
	ASSERT_FALSE(nearlyEqual(NAN, INFINITY));
	ASSERT_FALSE(nearlyEqual(INFINITY, NAN));
	ASSERT_FALSE(nearlyEqual(NAN, -INFINITY));
	ASSERT_FALSE(nearlyEqual(-INFINITY, NAN));
	ASSERT_FALSE(nearlyEqual(NAN, FLT_MAX));
	ASSERT_FALSE(nearlyEqual(FLT_MAX, NAN));
	ASSERT_FALSE(nearlyEqual(NAN, -FLT_MAX));
	ASSERT_FALSE(nearlyEqual(-FLT_MAX, NAN));
	ASSERT_FALSE(nearlyEqual(NAN, FLT_MIN));
	ASSERT_FALSE(nearlyEqual(FLT_MIN, NAN));
	ASSERT_FALSE(nearlyEqual(NAN, -FLT_MIN));
	ASSERT_FALSE(nearlyEqual(-FLT_MIN, NAN));
}

// Comparisons of numbers on opposite sides of 0
TEST(FloatTest, oppositeZeros)
{
	ASSERT_FALSE(nearlyEqual(1.000000001f, -1.0f));
	ASSERT_FALSE(nearlyEqual(-1.0f, 1.000000001f));
	ASSERT_FALSE(nearlyEqual(-1.000000001f, 1.0f));
	ASSERT_FALSE(nearlyEqual(1.0f, -1.000000001f));
	ASSERT_TRUE(nearlyEqual(10 * FLT_MIN, 10 * -FLT_MIN));
	ASSERT_FALSE(nearlyEqual(10000 * FLT_MIN, 10000 * -FLT_MIN));
}

TEST(FloatTest, closeToZero)
{
	ASSERT_TRUE(nearlyEqual(FLT_MIN, FLT_MIN));
	ASSERT_TRUE(nearlyEqual(FLT_MIN, -FLT_MIN));
	ASSERT_TRUE(nearlyEqual(-FLT_MIN, FLT_MIN));
	ASSERT_TRUE(nearlyEqual(FLT_MIN, 0));
	ASSERT_TRUE(nearlyEqual(0, FLT_MIN));
	ASSERT_TRUE(nearlyEqual(-FLT_MIN, 0));
	ASSERT_TRUE(nearlyEqual(0, -FLT_MIN));

	ASSERT_FALSE(nearlyEqual(0.000000001f, -FLT_MIN));
	ASSERT_FALSE(nearlyEqual(0.000000001f, FLT_MIN));
	ASSERT_FALSE(nearlyEqual(FLT_MIN, 0.000000001f));
	ASSERT_FALSE(nearlyEqual(-FLT_MIN, 0.000000001f));
}
