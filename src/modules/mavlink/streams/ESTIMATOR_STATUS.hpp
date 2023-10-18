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

#ifndef ESTIMATOR_STATUS_HPP
#define ESTIMATOR_STATUS_HPP

#include <uORB/topics/estimator_selector_status.h>
#include <uORB/topics/estimator_status.h>
#include <uORB/topics/failsafe_flags.h>

class MavlinkStreamEstimatorStatus : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamEstimatorStatus(mavlink); }

	static constexpr const char *get_name_static() { return "ESTIMATOR_STATUS"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_ESTIMATOR_STATUS; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _estimator_status_sub.advertised() ? MAVLINK_MSG_ID_ESTIMATOR_STATUS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit MavlinkStreamEstimatorStatus(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _estimator_selector_status_sub{ORB_ID(estimator_selector_status)};
	uORB::Subscription _estimator_status_sub{ORB_ID(estimator_status)};
	uORB::Subscription _failsafe_flags_sub{ORB_ID(failsafe_flags)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};

	bool send() override
	{
		// use primary estimator_status
		if (_estimator_selector_status_sub.updated()) {
			estimator_selector_status_s estimator_selector_status;

			if (_estimator_selector_status_sub.copy(&estimator_selector_status)) {
				if (estimator_selector_status.primary_instance != _estimator_status_sub.get_instance()) {
					_estimator_status_sub.ChangeInstance(estimator_selector_status.primary_instance);
				}
			}
		}

		estimator_status_s est;
		failsafe_flags_s failsafe_flags;
		vehicle_local_position_s vehicle_local_position;

		if (_failsafe_flags_sub.update(&failsafe_flags) && _estimator_status_sub.update(&est) && _vehicle_local_position_sub.update(&vehicle_local_position)) {
			mavlink_estimator_status_t est_msg{};
			est_msg.time_usec = est.timestamp;

			// flags: Bitmap indicating which EKF outputs are valid.
			est_msg.flags = 0;

			// ESTIMATOR_ATTITUDE=1, /* True if the attitude estimate is good | */
			if (!failsafe_flags.attitude_invalid) {
				est_msg.flags |= ESTIMATOR_ATTITUDE;
			}

			// ESTIMATOR_VELOCITY_HORIZ=2, /* True if the horizontal velocity estimate is good | */
			if (!failsafe_flags.local_velocity_invalid) {
				est_msg.flags |= ESTIMATOR_VELOCITY_HORIZ;
			}

			// ESTIMATOR_VELOCITY_VERT=4, /* True if the  vertical velocity estimate is good | */
			if (!failsafe_flags.local_velocity_invalid || !failsafe_flags.local_altitude_invalid) {
				est_msg.flags |= ESTIMATOR_VELOCITY_VERT;
			}

			// ESTIMATOR_POS_HORIZ_REL=8, /* True if the horizontal position (relative) estimate is good | */
			if (!failsafe_flags.local_position_invalid) {
				est_msg.flags |= ESTIMATOR_POS_HORIZ_REL;
			}

			// ESTIMATOR_POS_HORIZ_ABS=16, /* True if the horizontal position (absolute) estimate is good | */
			if (!failsafe_flags.global_position_invalid) {
				est_msg.flags |= ESTIMATOR_POS_HORIZ_ABS;
			}

			// ESTIMATOR_POS_VERT_ABS=32, /* True if the vertical position (absolute) estimate is good | */
			if (!failsafe_flags.local_altitude_invalid) {
				est_msg.flags |= ESTIMATOR_POS_VERT_ABS;
			}

			// ESTIMATOR_POS_VERT_AGL=64, /* True if the vertical position (above ground) estimate is good | */
			if (vehicle_local_position.dist_bottom_valid) {
				est_msg.flags |= ESTIMATOR_POS_VERT_AGL;
			}

			// ESTIMATOR_CONST_POS_MODE=128, /* True if the EKF is in a constant position mode and is not using external measurements (eg GPS or optical flow) | */
			if (!failsafe_flags.local_velocity_invalid) {
				// TODO: dead reckoning?
				est_msg.flags |= ESTIMATOR_CONST_POS_MODE;
			}

			// ESTIMATOR_PRED_POS_HORIZ_REL=256, /* True if the EKF has sufficient data to enter a mode that will provide a (relative) position estimate | */
			if (!failsafe_flags.local_velocity_invalid && !failsafe_flags.local_position_invalid) {
				// TODO: review
				est_msg.flags |= ESTIMATOR_PRED_POS_HORIZ_REL;
			}

			// ESTIMATOR_PRED_POS_HORIZ_ABS=512, /* True if the EKF has sufficient data to enter a mode that will provide a (absolute) position estimate | */
			if (!failsafe_flags.global_position_invalid) {
				est_msg.flags |= ESTIMATOR_PRED_POS_HORIZ_ABS;
			}

			// ESTIMATOR_GPS_GLITCH=1024, /* True if the EKF has detected a GPS glitch | */
			if (!failsafe_flags.global_position_invalid || est.gps_check_fail_flags) {
				// TODO: review
				est_msg.flags |= ESTIMATOR_PRED_POS_HORIZ_ABS;
			}

			// ESTIMATOR_ACCEL_ERROR=2048, /* True if the EKF has detected bad accelerometer data | */
			if () {
				// TODO:
				// bool fs_bad_acc_bias          # 15 - true if bad delta velocity bias estimates have been detected
				// bool fs_bad_acc_vertical      # 16 - true if bad vertical accelerometer data has been detected
				// bool fs_bad_acc_clipping      # 17 - true if delta velocity data contains clipping (asymmetric railing)
				est_msg.flags |= ESTIMATOR_ACCEL_ERROR;
			}


			// vel_ratio: Velocity innovation test ratio
			est_msg.vel_ratio = est.vel_test_ratio;

			// pos_horiz_ratio: Horizontal position innovation test ratio
			est_msg.pos_horiz_ratio = est.pos_test_ratio;

			// pos_vert_ratio: Vertical position innovation test ratio
			est_msg.pos_vert_ratio = est.hgt_test_ratio;

			// mag_ratio: Magnetometer innovation test ratio
			est_msg.mag_ratio = est.mag_test_ratio;

			// hagl_ratio: Height above terrain innovation test ratio
			est_msg.hagl_ratio = est.hagl_test_ratio;

			// tas_ratio: True airspeed innovation test ratio
			est_msg.tas_ratio = est.tas_test_ratio;

			// pos_horiz_accuracy: Horizontal position 1-STD accuracy relative to the EKF local origin
			est_msg.pos_horiz_accuracy = est.pos_horiz_accuracy;

			// pos_vert_accuracy: Vertical position 1-STD accuracy relative to the EKF local origin
			est_msg.pos_vert_accuracy = est.pos_vert_accuracy;

			mavlink_msg_estimator_status_send_struct(_mavlink->get_channel(), &est_msg);

			return true;
		}

		return false;
	}
};

#endif // ESTIMATOR_STATUS_HPP
