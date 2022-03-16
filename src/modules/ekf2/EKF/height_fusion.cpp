/****************************************************************************
 *
 *   Copyright (c) 2021 PX4. All rights reserved.
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
 * @file height_fusion.cpp
 * Function for fusing height (range, baro, GNSS alt, ...) measurements
 */

#include "ekf.h"

void Ekf::updateBaroHgt(const baroSample &baro_sample, estimator_aid_source_1d_s &baro_hgt)
{
	// innovation gate size
	float innov_gate = fmaxf(_params.baro_innov_gate, 1.f);

	// observation variance - user parameter defined
	float obs_var = sq(fmaxf(_params.baro_noise, 0.01f));

	// vertical position innovation - baro measurement has opposite sign to earth z axis
	baro_hgt.observation = -(_baro_sample_delayed.hgt - _baro_b_est.getBias() - _baro_hgt_offset);
	baro_hgt.observation_variance = obs_var;

	baro_hgt.innovation = _state.pos(2) - baro_hgt.observation;
	baro_hgt.innovation_variance = P(9, 9) + obs_var;

	// Compensate for positive static pressure transients (negative vertical position innovations)
	// caused by rotor wash ground interaction by applying a temporary deadzone to baro innovations.
	if (_control_status.flags.gnd_effect && (_params.gnd_effect_deadzone > 0.f)) {

		const float deadzone_start = 0.0f;
		const float deadzone_end = deadzone_start + _params.gnd_effect_deadzone;

		if (baro_hgt.innovation < -deadzone_start) {
			if (baro_hgt.innovation <= -deadzone_end) {
				baro_hgt.innovation += deadzone_end;

			} else {
				baro_hgt.innovation = -deadzone_start;
			}
		}
	}

	baro_hgt.test_ratio = sq(baro_hgt.innovation) / (sq(innov_gate) * baro_hgt.innovation_variance);

	baro_hgt.innovation_rejected = (baro_hgt.test_ratio > 1.f);

	// special case if there is bad vertical acceleration data, then don't reject measurement,
	// but limit innovation to prevent spikes that could destabilise the filter
	if (_fault_status.flags.bad_acc_vertical && baro_hgt.innovation_rejected) {
		const float innov_limit = innov_gate * sqrtf(baro_hgt.innovation_variance);
		baro_hgt.innovation = math::constrain(baro_hgt.innovation, -innov_limit, innov_limit);
		baro_hgt.innovation_rejected = false;
	}

	baro_hgt.fusion_enabled = _control_status.flags.baro_hgt;

	baro_hgt.timestamp_sample = baro_sample.time_us;

	baro_hgt.fused = false; // reset
}

void Ekf::fuseBaroHgt(estimator_aid_source_1d_s &baro_hgt)
{
	if (baro_hgt.fusion_enabled
	    && !baro_hgt.innovation_rejected
	    && fuseVelPosHeight(baro_hgt.innovation, baro_hgt.innovation_variance, 5)) {

		baro_hgt.fused = true;
		baro_hgt.time_last_fuse = _time_last_imu;
	}
}

void Ekf::updateRngHgt(estimator_aid_source_1d_s &range_hgt)
{
	// observation variance - user parameter defined
	float obs_var = fmaxf(sq(_params.range_noise) + sq(_params.range_noise_scaler * _range_sensor.getDistBottom()), 0.01f);

	// innovation gate size
	float innov_gate = fmaxf(_params.range_innov_gate, 1.f);

	// vertical position innovation, use range finder with tilt correction
	range_hgt.observation = (-math::max(_range_sensor.getDistBottom(), _params.rng_gnd_clearance)) + _rng_hgt_offset;
	range_hgt.observation_variance = obs_var;

	range_hgt.innovation = _state.pos(2) - range_hgt.observation;
	range_hgt.innovation_variance = P(9, 9) + obs_var;

	range_hgt.test_ratio = sq(range_hgt.innovation) / (sq(innov_gate) * range_hgt.innovation_variance);

	range_hgt.innovation_rejected = (range_hgt.test_ratio > 1.f);

	// special case if there is bad vertical acceleration data, then don't reject measurement,
	// but limit innovation to prevent spikes that could destabilise the filter
	if (_fault_status.flags.bad_acc_vertical && range_hgt.innovation_rejected) {
		const float innov_limit = innov_gate * sqrtf(range_hgt.innovation_variance);
		range_hgt.innovation = math::constrain(range_hgt.innovation, -innov_limit, innov_limit);
		range_hgt.innovation_rejected = false;
	}

	range_hgt.fusion_enabled = _control_status.flags.rng_hgt;

	range_hgt.timestamp_sample = _range_sensor.getSampleAddress()->time_us;

	range_hgt.fused = false; // reset
}

void Ekf::fuseRngHgt(estimator_aid_source_1d_s &range_hgt)
{
	if (range_hgt.fusion_enabled
	    && !range_hgt.innovation_rejected
	    && fuseVelPosHeight(range_hgt.innovation, range_hgt.innovation_variance, 5)) {

		range_hgt.fused = true;
		range_hgt.time_last_fuse = _time_last_imu;
	}
}

void Ekf::fuseEvHgt()
{
	// calculate the innovation assuming the external vision observation is in local NED frame
	_ev_pos_innov(2) = _state.pos(2) - _ev_sample_delayed.pos(2);

	// innovation gate size
	float innov_gate = fmaxf(_params.ev_pos_innov_gate, 1.f);

	// observation variance - defined externally
	float obs_var = fmaxf(_ev_sample_delayed.posVar(2), sq(0.01f));

	// _ev_pos_test_ratio(1) is the vertical test ratio
	fuseVerticalPosition(_ev_pos_innov(2), innov_gate, obs_var,
			     _ev_pos_innov_var(2), _ev_pos_test_ratio(1));
}
