/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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
 * Log GPS communication data
 *
 * If this is set to 1, all GPS communication data will be published via uORB,
 * and written to the log file as gps_dump message.
 *
 * If this is set to 2, the main GPS is configured to output RTCM data,
 * which is then logged as gps_dump and can be used for PPK.
 *
 * @min 0
 * @max 2
 * @value 0 Disable
 * @value 1 Full communication
 * @value 2 RTCM output (PPK)
 * @group GNSS Ublox
 */
PARAM_DEFINE_INT32(UBX_DUMP_COMM, 0);

/**
 * u-blox GPS dynamic platform model
 *
 * u-blox receivers support different dynamic platform models to adjust the navigation engine to
 * the expected application environment.
 *
 * @min 0
 * @max 9
 * @value 2 stationary
 * @value 4 automotive
 * @value 6 airborne with <1g acceleration
 * @value 7 airborne with <2g acceleration
 * @value 8 airborne with <4g acceleration
 *
 * @reboot_required true
 *
 * @group GNSS Ublox
 */
PARAM_DEFINE_INT32(UBX_DYNMODEL, 7);

/**
 * Enable sat info (if available)
 *
 * Enable publication of satellite info (ORB_ID(satellite_info)) if possible.
 * Not available on MTK.
 *
 * @boolean
 * @reboot_required true
 * @group GNSS Ublox
 */
PARAM_DEFINE_INT32(UBX_SAT_INFO, 0);

/**
 * u-blox GPS Mode
 *
 * Select the u-blox configuration setup. Most setups will use the default, including RTK and
 * dual GPS without heading.
 *
 * The Heading mode requires 2 F9P devices to be attached. The main GPS will act as rover and output
 * heading information, whereas the secondary will act as moving base.
 * Modes 1 and 2 require each F9P UART1 to be connected to the Autopilot. In addition, UART2 on the
 * F9P units are connected to each other.
 * Modes 3 and 4 only require UART1 on each F9P connected to the Autopilot or Can Node. UART RX DMA is required.
 * RTK is still possible with this setup.
 *
 * @min 0
 * @max 1
 * @value 0 Default
 * @value 1 Heading (Rover With Moving Base UART1 Connected To Autopilot, UART2 Connected To Moving Base)
 * @value 2 Moving Base (UART1 Connected To Autopilot, UART2 Connected To Rover)
 * @value 3 Heading (Rover With Moving Base UART1 Connected to Autopilot Or Can Node At 921600)
 * @value 4 Moving Base (Moving Base UART1 Connected to Autopilot Or Can Node At 921600)
 *
 * @reboot_required true
 * @group GNSS Ublox
 */
PARAM_DEFINE_INT32(UBX_MODE, 0);

/**
 * Heading/Yaw offset for dual antenna GPS
 *
 * Heading offset angle for dual antenna GPS setups that support heading estimation.
 *
 * Set this to 0 if the antennas are parallel to the forward-facing direction of the vehicle and the rover antenna is in
 * front. The offset angle increases clockwise.
 *
 * Set this to 90 if the rover antenna is placed on the right side of the vehicle and the moving base antenna is on the left side.
 *
 * @min 0
 * @max 360
 * @unit deg
 * @reboot_required true
 * @decimal 0
 *
 * @group GNSS Ublox
 */
PARAM_DEFINE_FLOAT(UBX_YAW_OFFSET, 0.f);

/**
 * GNSS Systems for Primary GPS (integer bitmask)
 *
 * This integer bitmask controls the set of GNSS systems used by the receiver. Check your
 * receiver's documentation on how many systems are supported to be used in parallel.
 *
 * Currently this functionality is just implemented for u-blox receivers.
 *
 * When no bits are set, the receiver's default configuration should be used.
 *
 * Set bits true to enable:
 * 0 : Use GPS (with QZSS)
 * 1 : Use SBAS (multiple GPS augmentation systems)
 * 2 : Use Galileo
 * 3 : Use BeiDou
 * 4 : Use GLONASS
 *
 * @min 0
 * @max 31
 * @bit 0 GPS (with QZSS)
 * @bit 1 SBAS
 * @bit 2 Galileo
 * @bit 3 BeiDou
 * @bit 4 GLONASS
 *
 * @reboot_required true
 * @group GPS
 */
PARAM_DEFINE_INT32(UBX_GNSS, 0);
