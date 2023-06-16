/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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

#include "ekf.h"

#include "aux_global_position.hpp"

#if defined(CONFIG_EKF2_AUX_GLOBAL_POSITION) && defined(MODULE_NAME)

void AuxGlobalPosition::update(Ekf &ekf, const estimator::imuSample &imu_delayed)
{

#if defined(MODULE_NAME)

	if (_global_odometry_sub.updated()) {

		global_odometry_s global_odometry{};
		_global_odometry_sub.copy(&global_odometry);

		const int64_t time_us = global_odometry.timestamp_sample - static_cast<int64_t>(_param_ekf2_agp_delay.get() * 1000);

		AuxGlobalPositionSample sample{};
		sample.time_us = time_us;
		sample.latitude = global_odometry.latitude;
		sample.longitude = global_odometry.longitude;
		sample.positional_uncertainty = global_odometry.positional_uncertainty;
		sample.position_valid = global_odometry.valid;
		sample.altitude = global_odometry.altitude_agl;
		sample.heading = global_odometry.heading;
		sample.heading_valid = global_odometry.heading_valid;

		_aux_global_position_buffer.push(sample);
	}

#endif // MODULE_NAME


	AuxGlobalPositionSample aux_global_position_sample;

	if (_aux_global_position_buffer.pop_first_older_than(imu_delayed.time_us, &aux_global_position_sample)) {

		if (!ekf.global_origin_valid()) {
			return;
		}

		const Vector2f position = ekf.global_origin().project(aux_global_position_sample.latitude, aux_global_position_sample.longitude);
		//const float hgt = ekf.getEkfGlobalOriginAltitude() - (float)aux_global_position_sample.altitude;

		estimator_aid_source2d_s aid_src{};

		// relax the upper observation noise limit which prevents bad GPS perturbing the position estimate
		float pos_noise = math::max(aux_global_position_sample.positional_uncertainty, _param_ekf2_agp_noise.get());
		const float pos_var = sq(pos_noise);
		const Vector2f pos_obs_var(pos_var, pos_var);
		ekf.updateHorizontalPositionAidSrcStatus(aux_global_position_sample.time_us,
				position,                                   // observation
				pos_obs_var,                                // observation variance
				math::max(_param_ekf2_agp_gate.get(), 1.f), // innovation gate
				aid_src);
		aid_src.fusion_enabled = _param_ekf2_agp_ctrl.get() & 0b001; // bit 0 Horizontal position

		if (aux_global_position_sample.position_valid && ekf.control_status_flags().yaw_align) {
			ekf.fuseHorizontalPosition(aid_src);
		}

#if defined(MODULE_NAME)
		aid_src.timestamp = hrt_absolute_time();
		_estimator_aid_src_aux_global_position_pub.publish(aid_src);
#endif // MODULE_NAME
	}
}

#endif // CONFIG_EKF2_AUX_GLOBAL_POSITION
