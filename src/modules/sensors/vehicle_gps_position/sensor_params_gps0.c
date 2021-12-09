/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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
 * ID of the GPS that the settings are for
 *
 * @category system
 * @group Sensor Calibration
 */
PARAM_DEFINE_INT32(SENS_GPS0_ID, 0);

/**
 * X position of GPS antenna in body frame (forward axis with origin relative to vehicle centre of gravity)
 *
 * @group Sensors
 * @unit m
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(SENS_GPS0_POS_X, 0.0f);

/**
 * Y position of GPS antenna in body frame (right axis with origin relative to vehicle centre of gravity)
 *
 * @group EKF2
 * @unit m
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(SENS_GPS0_POS_Y, 0.0f);

/**
 * Z position of GPS antenna in body frame (down axis with origin relative to vehicle centre of gravity)
 *
 * @group EKF2
 * @unit m
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(SENS_GPS0_POS_Z, 0.0f);

/**
 * Accelerometer 0 priority.
 *
 * @value -1  Uninitialized
 * @value 0   Disabled
 * @value 1   Min
 * @value 25  Low
 * @value 50  Medium (Default)
 * @value 75  High
 * @value 100 Max
 *
 * @category system
 * @group Sensor Calibration
 */
PARAM_DEFINE_INT32(SENS_GPS0_PRIO, -1);
