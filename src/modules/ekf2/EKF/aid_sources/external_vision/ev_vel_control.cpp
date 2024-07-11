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
 * @file ev_vel_control.cpp
 * Control functions for ekf external vision velocity fusion
 */

#include "ekf.h"

void Ekf::controlEvVelFusion(const extVisionSample &ev_sample, const bool common_starting_conditions_passing,
			     const bool ev_reset, const bool quality_sufficient, estimator_aid_source3d_s &aid_src)
{
	// determine if we should use EV velocity aiding
	bool continuing_conditions_passing = (_params.ev_ctrl & static_cast<int32_t>(EvCtrl::VEL))
					     && _control_status.flags.tilt_align
					     && ev_sample.vel.isAllFinite()
					     && ev_sample.velocity_var.isAllFinite();

	if (!continuing_conditions_passing) {
		stopEvVelFusion();
		return;
	}

	const float minimum_variance = math::max(sq(0.01f), sq(_params.ev_vel_noise));

	Vector3f measurement_var{
		math::max(ev_sample.velocity_var(0), minimum_variance),
		math::max(ev_sample.velocity_var(1), minimum_variance),
		math::max(ev_sample.velocity_var(2), minimum_variance)
	};

	const float eps = 1e-5f;
	bool ev_q_valid = ev_sample.quat.isAllFinite()
			  && (ev_sample.quat.length() > (1.f - eps))
			  && (ev_sample.quat.length() < (1.f + eps))
			  && (ev_sample.quat.max() <= 1.f);


	switch (ev_sample.vel_frame) {
	case VelocityFrame::LOCAL_FRAME_NED:
		if (_control_status.flags.yaw_align) {
			controlEvVelWorldFusion(ev_sample.time_us, ev_sample.vel, measurement_var,
						common_starting_conditions_passing, ev_reset, quality_sufficient,
						aid_src);

		} else if (ev_q_valid) {
			// rotate measurement into body frame
			Vector3f measurement = ev_sample.quat.rotateVectorInverse(ev_sample.vel);

			controlEvVelBodyFusion(ev_sample.time_us, measurement, measurement_var,
					       common_starting_conditions_passing, ev_reset, quality_sufficient,
					       aid_src);

		} else {
			// TODO: consider still using if !yaw_align, !ev_yaw, and !ev_q_valid?
			stopEvVelFusion();
		}

		break;

	case VelocityFrame::LOCAL_FRAME_FRD:
		if (_control_status.flags.ev_yaw) {
			controlEvVelWorldFusion(ev_sample.time_us, ev_sample.vel, measurement_var,
						common_starting_conditions_passing, ev_reset, quality_sufficient,
						aid_src);

		} else if (ev_q_valid) {

			// rotate measurement into body frame
			Vector3f measurement = ev_sample.quat.rotateVectorInverse(ev_sample.vel);
			measurement_var = ev_sample.quat.rotateVectorInverse(measurement_var);

			if (!_control_status.flags.ev_vel) {
				Eulerf euler(ev_sample.quat);
				ECL_INFO("EV vel LOCAL_FRAME_FRD, rotating to body frame ([%.3f, %.3f, %.3f]) [%.3f, %.3f, %.3f]->[%.3f, %.3f, %.3f]",
					 (double)euler.phi(), (double)euler.theta(), (double)euler.psi(),
					 (double)ev_sample.vel(0), (double)ev_sample.vel(1), (double)ev_sample.vel(2),
					 (double)measurement(0), (double)measurement(1), (double)measurement(2)
					);

				ECL_INFO("EV vel LOCAL_FRAME_FRD, rotating to body frame var: [%.3f, %.3f, %.3f]->[%.3f, %.3f, %.3f]",
					 (double)ev_sample.velocity_var(0), (double)ev_sample.velocity_var(1), (double)ev_sample.velocity_var(2),
					 (double)measurement_var(0), (double)measurement_var(1), (double)measurement_var(2)
					);
			}

			controlEvVelBodyFusion(ev_sample.time_us, measurement, measurement_var,
					       common_starting_conditions_passing, ev_reset, quality_sufficient,
					       aid_src);

		} else {
			// TODO: consider still using if !yaw_align, !ev_yaw, and !ev_q_valid?
			stopEvVelFusion();
		}

		break;

	case VelocityFrame::BODY_FRAME_FRD:
		controlEvVelBodyFusion(ev_sample.time_us, ev_sample.vel, measurement_var,
				       common_starting_conditions_passing, ev_reset, quality_sufficient,
				       aid_src);
		break;

	default:
		stopEvVelFusion();
		break;
	}
}

void Ekf::stopEvVelFusion()
{
	if (_control_status.flags.ev_vel) {

		_control_status.flags.ev_vel = false;
	}
}
