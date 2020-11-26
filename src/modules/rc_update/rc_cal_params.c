/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
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
 * @file rc_params.c
 *
 * Parameters defined for RC.
 *
 */

/**
 * RC channel 1 minimum
 *
 * Minimum value for RC channel 1
 *
 * @min 800.0
 * @max 1500.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC1_MIN, 1000.0f);

/**
 * RC channel 1 trim
 *
 * Mid point value (same as min for throttle)
 *
 * @min 800.0
 * @max 2200.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC1_TRIM, 1500.0f);

/**
 * RC channel 1 maximum
 *
 * Maximum value for RC channel 1
 *
 * @min 1500.0
 * @max 2200.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC1_MAX, 2000.0f);

/**
 * RC channel 1 reverse
 *
 * Set to -1 to reverse channel.
 *
 * @min -1.0
 * @max 1.0
 * @value -1.0 Reverse
 * @value 1.0 Normal
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC1_REV, 1.0f);

/**
 * RC channel 1 dead zone
 *
 * The +- range of this value around the trim value will be considered as zero.
 *
 * @min 0.0
 * @max 100.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC1_DZ, 10.0f);

/**
 * RC channel 2 minimum
 *
 * Minimum value for this channel.
 *
 * @min 800.0
 * @max 1500.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC2_MIN, 1000.0f);

/**
 * RC channel 2 trim
 *
 * Mid point value (has to be set to the same as min for throttle channel).
 *
 * @min 800.0
 * @max 2200.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC2_TRIM, 1500.0f);

/**
 * RC channel 2 maximum
 *
 * Maximum value for this channel.
 *
 * @min 1500.0
 * @max 2200.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC2_MAX, 2000.0f);

/**
 * RC channel 2 reverse
 *
 * Set to -1 to reverse channel.
 *
 * @min -1.0
 * @max 1.0
 * @value -1.0 Reverse
 * @value 1.0 Normal
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC2_REV, 1.0f);

/**
 * RC channel 2 dead zone
 *
 * The +- range of this value around the trim value will be considered as zero.
 *
 * @min 0.0
 * @max 100.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC2_DZ, 10.0f);

/**
 * RC channel 3 minimum
 *
 * Minimum value for this channel.
 *
 * @min 800.0
 * @max 1500.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC3_MIN, 1000);

/**
 * RC channel 3 trim
 *
 * Mid point value (has to be set to the same as min for throttle channel).
 *
 * @min 800.0
 * @max 2200.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC3_TRIM, 1500);

/**
 * RC channel 3 maximum
 *
 * Maximum value for this channel.
 *
 * @min 1500.0
 * @max 2200.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC3_MAX, 2000);

/**
 * RC channel 3 reverse
 *
 * Set to -1 to reverse channel.
 *
 * @min -1.0
 * @max 1.0
 * @value -1.0 Reverse
 * @value 1.0 Normal
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC3_REV, 1.0f);

/**
 * RC channel 3 dead zone
 *
 * The +- range of this value around the trim value will be considered as zero.
 *
 * @min 0.0
 * @max 100.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC3_DZ, 10.0f);

/**
 * RC channel 4 minimum
 *
 * Minimum value for this channel.
 *
 * @min 800.0
 * @max 1500.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC4_MIN, 1000);

/**
 * RC channel 4 trim
 *
 * Mid point value (has to be set to the same as min for throttle channel).
 *
 * @min 800.0
 * @max 2200.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC4_TRIM, 1500);

/**
 * RC channel 4 maximum
 *
 * Maximum value for this channel.
 *
 * @min 1500.0
 * @max 2200.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC4_MAX, 2000);

/**
 * RC channel 4 reverse
 *
 * Set to -1 to reverse channel.
 *
 * @min -1.0
 * @max 1.0
 * @value -1.0 Reverse
 * @value 1.0 Normal
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC4_REV, 1.0f);

/**
 * RC channel 4 dead zone
 *
 * The +- range of this value around the trim value will be considered as zero.
 *
 * @min 0.0
 * @max 100.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC4_DZ, 10.0f);

/**
 * RC channel 5 minimum
 *
 * Minimum value for this channel.
 *
 * @min 800.0
 * @max 1500.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC5_MIN, 1000);

/**
 * RC channel 5 trim
 *
 * Mid point value (has to be set to the same as min for throttle channel).
 *
 * @min 800.0
 * @max 2200.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC5_TRIM, 1500);

/**
 * RC channel 5 maximum
 *
 * Maximum value for this channel.
 *
 * @min 1500.0
 * @max 2200.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC5_MAX, 2000);

/**
 * RC channel 5 reverse
 *
 * Set to -1 to reverse channel.
 *
 * @min -1.0
 * @max 1.0
 * @value -1.0 Reverse
 * @value 1.0 Normal
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC5_REV, 1.0f);

/**
 * RC channel 5 dead zone
 *
 * The +- range of this value around the trim value will be considered as zero.
 *
 * @min 0.0
 * @max 100.0
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC5_DZ,  10.0f);

/**
 * RC channel 6 minimum
 *
 * Minimum value for this channel.
 *
 * @unit us
 * @min 800.0
 * @max 1500.0
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC6_MIN, 1000);

/**
 * RC channel 6 trim
 *
 * Mid point value (has to be set to the same as min for throttle channel).
 *
 * @min 800.0
 * @max 2200.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC6_TRIM, 1500);

/**
 * RC channel 6 maximum
 *
 * Maximum value for this channel.
 *
 * @min 1500.0
 * @max 2200.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC6_MAX, 2000);

/**
 * RC channel 6 reverse
 *
 * Set to -1 to reverse channel.
 *
 * @min -1.0
 * @max 1.0
 * @value -1.0 Reverse
 * @value 1.0 Normal
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC6_REV, 1.0f);

/**
 * RC channel 6 dead zone
 *
 * The +- range of this value around the trim value will be considered as zero.
 *
 * @min 0.0
 * @max 100.0
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC6_DZ, 10.0f);

/**
 * RC channel 7 minimum
 *
 * Minimum value for this channel.
 *
 * @unit us
 * @min 800.0
 * @max 1500.0
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC7_MIN, 1000);

/**
 * RC channel 7 trim
 *
 * Mid point value (has to be set to the same as min for throttle channel).
 *
 * @min 800.0
 * @max 2200.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC7_TRIM, 1500);

/**
 * RC channel 7 maximum
 *
 * Maximum value for this channel.
 *
 * @min 1500.0
 * @max 2200.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC7_MAX, 2000);

/**
 * RC channel 7 reverse
 *
 * Set to -1 to reverse channel.
 *
 * @min -1.0
 * @max 1.0
 * @value -1.0 Reverse
 * @value 1.0 Normal
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC7_REV, 1.0f);

/**
 * RC channel 7 dead zone
 *
 * The +- range of this value around the trim value will be considered as zero.
 *
 * @min 0.0
 * @max 100.0
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC7_DZ, 10.0f);

/**
 * RC channel 8 minimum
 *
 * Minimum value for this channel.
 *
 * @min 800.0
 * @max 1500.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC8_MIN, 1000);

/**
 * RC channel 8 trim
 *
 * Mid point value (has to be set to the same as min for throttle channel).
 *
 * @min 800.0
 * @max 2200.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC8_TRIM, 1500);

/**
 * RC channel 8 maximum
 *
 * Maximum value for this channel.
 *
 * @min 1500.0
 * @max 2200.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC8_MAX, 2000);

/**
 * RC channel 8 reverse
 *
 * Set to -1 to reverse channel.
 *
 * @min -1.0
 * @max 1.0
 * @value -1.0 Reverse
 * @value 1.0 Normal
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC8_REV, 1.0f);

/**
 * RC channel 8 dead zone
 *
 * The +- range of this value around the trim value will be considered as zero.
 *
 * @min 0.0
 * @max 100.0
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC8_DZ, 10.0f);

/**
 * RC channel 9 minimum
 *
 * Minimum value for this channel.
 *
 * @min 800.0
 * @max 1500.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC9_MIN, 1000);

/**
 * RC channel 9 trim
 *
 * Mid point value (has to be set to the same as min for throttle channel).
 *
 * @min 800.0
 * @max 2200.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC9_TRIM, 1500);

/**
 * RC channel 9 maximum
 *
 * Maximum value for this channel.
 *
 * @min 1500.0
 * @max 2200.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC9_MAX, 2000);

/**
 * RC channel 9 reverse
 *
 * Set to -1 to reverse channel.
 *
 * @min -1.0
 * @max 1.0
 * @value -1.0 Reverse
 * @value 1.0 Normal
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC9_REV, 1.0f);

/**
 * RC channel 9 dead zone
 *
 * The +- range of this value around the trim value will be considered as zero.
 *
 * @min 0.0
 * @max 100.0
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC9_DZ, 0.0f);

/**
 * RC channel 10 minimum
 *
 * Minimum value for this channel.
 *
 * @min 800.0
 * @max 1500.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC10_MIN, 1000);

/**
 * RC channel 10 trim
 *
 * Mid point value (has to be set to the same as min for throttle channel).
 *
 * @min 800.0
 * @max 2200.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC10_TRIM, 1500);

/**
 * RC channel 10 maximum
 *
 * Maximum value for this channel.
 *
 * @min 1500.0
 * @max 2200.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC10_MAX, 2000);

/**
 * RC channel 10 reverse
 *
 * Set to -1 to reverse channel.
 *
 * @min -1.0
 * @max 1.0
 * @value -1.0 Reverse
 * @value 1.0 Normal
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC10_REV, 1.0f);

/**
 * RC channel 10 dead zone
 *
 * The +- range of this value around the trim value will be considered as zero.
 *
 * @min 0.0
 * @max 100.0
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC10_DZ, 0.0f);

/**
 * RC channel 11 minimum
 *
 * Minimum value for this channel.
 *
 * @min 800.0
 * @max 1500.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC11_MIN, 1000);

/**
 * RC channel 11 trim
 *
 * Mid point value (has to be set to the same as min for throttle channel).
 *
 * @min 800.0
 * @max 2200.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC11_TRIM, 1500);

/**
 * RC channel 11 maximum
 *
 * Maximum value for this channel.
 *
 * @min 1500.0
 * @max 2200.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC11_MAX, 2000);

/**
 * RC channel 11 reverse
 *
 * Set to -1 to reverse channel.
 *
 * @min -1.0
 * @max 1.0
 * @value -1.0 Reverse
 * @value 1.0 Normal
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC11_REV, 1.0f);

/**
 * RC channel 11 dead zone
 *
 * The +- range of this value around the trim value will be considered as zero.
 *
 * @min 0.0
 * @max 100.0
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC11_DZ, 0.0f);

/**
 * RC channel 12 minimum
 *
 * Minimum value for this channel.
 *
 * @min 800.0
 * @max 1500.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC12_MIN, 1000);

/**
 * RC channel 12 trim
 *
 * Mid point value (has to be set to the same as min for throttle channel).
 *
 * @min 800.0
 * @max 2200.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC12_TRIM, 1500);

/**
 * RC channel 12 maximum
 *
 * Maximum value for this channel.
 *
 * @min 1500.0
 * @max 2200.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC12_MAX, 2000);

/**
 * RC channel 12 reverse
 *
 * Set to -1 to reverse channel.
 *
 * @min -1.0
 * @max 1.0
 * @value -1.0 Reverse
 * @value 1.0 Normal
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC12_REV, 1.0f);

/**
 * RC channel 12 dead zone
 *
 * The +- range of this value around the trim value will be considered as zero.
 *
 * @min 0.0
 * @max 100.0
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC12_DZ, 0.0f);

/**
 * RC channel 13 minimum
 *
 * Minimum value for this channel.
 *
 * @min 800.0
 * @max 1500.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC13_MIN, 1000);

/**
 * RC channel 13 trim
 *
 * Mid point value (has to be set to the same as min for throttle channel).
 *
 * @min 800.0
 * @max 2200.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC13_TRIM, 1500);

/**
 * RC channel 13 maximum
 *
 * Maximum value for this channel.
 *
 * @min 1500.0
 * @max 2200.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC13_MAX, 2000);

/**
 * RC channel 13 reverse
 *
 * Set to -1 to reverse channel.
 *
 * @min -1.0
 * @max 1.0
 * @value -1.0 Reverse
 * @value 1.0 Normal
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC13_REV, 1.0f);

/**
 * RC channel 13 dead zone
 *
 * The +- range of this value around the trim value will be considered as zero.
 *
 * @min 0.0
 * @max 100.0
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC13_DZ, 0.0f);

/**
 * RC channel 14 minimum
 *
 * Minimum value for this channel.
 *
 * @min 800.0
 * @max 1500.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC14_MIN, 1000);

/**
 * RC channel 14 trim
 *
 * Mid point value (has to be set to the same as min for throttle channel).
 *
 * @min 800.0
 * @max 2200.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC14_TRIM, 1500);

/**
 * RC channel 14 maximum
 *
 * Maximum value for this channel.
 *
 * @min 1500.0
 * @max 2200.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC14_MAX, 2000);

/**
 * RC channel 14 reverse
 *
 * Set to -1 to reverse channel.
 *
 * @min -1.0
 * @max 1.0
 * @value -1.0 Reverse
 * @value 1.0 Normal
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC14_REV, 1.0f);

/**
 * RC channel 14 dead zone
 *
 * The +- range of this value around the trim value will be considered as zero.
 *
 * @min 0.0
 * @max 100.0
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC14_DZ, 0.0f);

/**
 * RC channel 15 minimum
 *
 * Minimum value for this channel.
 *
 * @min 800.0
 * @max 1500.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC15_MIN, 1000);

/**
 * RC channel 15 trim
 *
 * Mid point value (has to be set to the same as min for throttle channel).
 *
 * @min 800.0
 * @max 2200.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC15_TRIM, 1500);

/**
 * RC channel 15 maximum
 *
 * Maximum value for this channel.
 *
 * @min 1500.0
 * @max 2200.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC15_MAX, 2000);

/**
 * RC channel 15 reverse
 *
 * Set to -1 to reverse channel.
 *
 * @min -1.0
 * @max 1.0
 * @value -1.0 Reverse
 * @value 1.0 Normal
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC15_REV, 1.0f);

/**
 * RC channel 15 dead zone
 *
 * The +- range of this value around the trim value will be considered as zero.
 *
 * @min 0.0
 * @max 100.0
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC15_DZ, 0.0f);

/**
 * RC channel 16 minimum
 *
 * Minimum value for this channel.
 *
 * @min 800.0
 * @max 1500.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC16_MIN, 1000);

/**
 * RC channel 16 trim
 *
 * Mid point value (has to be set to the same as min for throttle channel).
 *
 * @min 800.0
 * @max 2200.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC16_TRIM, 1500);

/**
 * RC channel 16 maximum
 *
 * Maximum value for this channel.
 *
 * @min 1500.0
 * @max 2200.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC16_MAX, 2000);

/**
 * RC channel 16 reverse
 *
 * Set to -1 to reverse channel.
 *
 * @min -1.0
 * @max 1.0
 * @value -1.0 Reverse
 * @value 1.0 Normal
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC16_REV, 1.0f);

/**
 * RC channel 16 dead zone
 *
 * The +- range of this value around the trim value will be considered as zero.
 *
 * @min 0.0
 * @max 100.0
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC16_DZ, 0.0f);

/**
 * RC channel 17 minimum
 *
 * Minimum value for this channel.
 *
 * @min 800.0
 * @max 1500.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC17_MIN, 1000);

/**
 * RC channel 17 trim
 *
 * Mid point value (has to be set to the same as min for throttle channel).
 *
 * @min 800.0
 * @max 2200.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC17_TRIM, 1500);

/**
 * RC channel 17 maximum
 *
 * Maximum value for this channel.
 *
 * @min 1500.0
 * @max 2200.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC17_MAX, 2000);

/**
 * RC channel 17 reverse
 *
 * Set to -1 to reverse channel.
 *
 * @min -1.0
 * @max 1.0
 * @value -1.0 Reverse
 * @value 1.0 Normal
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC17_REV, 1.0f);

/**
 * RC channel 17 dead zone
 *
 * The +- range of this value around the trim value will be considered as zero.
 *
 * @min 0.0
 * @max 100.0
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC17_DZ, 0.0f);

/**
 * RC channel 18 minimum
 *
 * Minimum value for this channel.
 *
 * @min 800.0
 * @max 1500.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC18_MIN, 1000);

/**
 * RC channel 18 trim
 *
 * Mid point value (has to be set to the same as min for throttle channel).
 *
 * @min 800.0
 * @max 2200.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC18_TRIM, 1500);

/**
 * RC channel 18 maximum
 *
 * Maximum value for this channel.
 *
 * @min 1500.0
 * @max 2200.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC18_MAX, 2000);

/**
 * RC channel 18 reverse
 *
 * Set to -1 to reverse channel.
 *
 * @min -1.0
 * @max 1.0
 * @value -1.0 Reverse
 * @value 1.0 Normal
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC18_REV, 1.0f);

/**
 * RC channel 18 dead zone
 *
 * The +- range of this value around the trim value will be considered as zero.
 *
 * @min 0.0
 * @max 100.0
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC18_DZ, 0.0f);
