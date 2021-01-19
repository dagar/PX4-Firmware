/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
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
 * @file control_allocator_params.c
 *
 * Parameters for the control allocator.
 *
 * @author Julien Lecoeur <julien.lecoeur@gmail.com>
 */


/**
 * Airframe ID
 *
 * This is used to retrieve pre-computed control effectiveness matrix
 *
 * @min 0
 * @max 2
 * @value 0 Multirotor
 * @value 1 Standard VTOL (WIP)
 * @value 2 Tiltrotor VTOL (WIP)
 * @value 3 Plane (WIP)
 * @group Control Allocation
 */
PARAM_DEFINE_INT32(CA_AIRFRAME, 0);

/**
 * Control allocation method
 *
 * @value 0 Pseudo-inverse with output clipping (default)
 * @value 1 Pseudo-inverse with sequential desaturation technique
 * @min 0
 * @max 1
 * @group Control Allocation
 */
PARAM_DEFINE_INT32(CA_METHOD, 0);

/**
 * Battery power level scaler
 *
 * This compensates for voltage drop of the battery over time by attempting to
 * normalize performance across the operating range of the battery. The copter
 * should constantly behave as if it was fully charged with reduced max acceleration
 * at lower battery percentages. i.e. if hover is at 0.5 throttle at 100% battery,
 * it will still be 0.5 at 60% battery.
 *
 * @boolean
 * @group Control Allocation
 */
PARAM_DEFINE_INT32(CA_BAT_SCALE_EN, 0);

/**
 * Airspeed scaler
 *
 * This compensates for the variation of flap effectiveness with airspeed.
 *
 * @boolean
 * @group Control Allocation
 */
PARAM_DEFINE_INT32(CA_AIR_SCALE_EN, 0);
