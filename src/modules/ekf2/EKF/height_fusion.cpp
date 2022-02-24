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

void Ekf::updateBaroHgt()
{
	// vertical position innovation - baro measurement has opposite sign to earth z axis
	const float unbiased_baro = _baro_sample_delayed.hgt - _baro_b_est.getBias();

	if (!PX4_ISFINITE(_baro_hgt_offset)) {
		_baro_hgt_offset = _state.pos(2) + unbiased_baro;
	}

	if (!_control_status.flags.baro_hgt) {
		// apply a 10 second first order low pass filter to baro offset
		const float local_time_step = math::constrain(1e-6f * _delta_time_baro_us, 0.f, 1.f);
		const float offset_rate_correction = 0.1f * (_state.pos(2) + unbiased_baro - _baro_hgt_offset);
		_baro_hgt_offset += local_time_step * math::constrain(offset_rate_correction, -0.1f, 0.1f);
	}

	auto &baro_hgt = _aid_src_baro_hgt;

	baro_hgt.innovation = _state.pos(2) + unbiased_baro - _baro_hgt_offset;

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

	// innovation gate size
	float innov_gate = fmaxf(_params.baro_innov_gate, 1.f);

	// observation variance - user parameter defined
	float obs_var = sq(fmaxf(_params.baro_noise, 0.01f));

	baro_hgt.innovation_variance = P(9, 9) + obs_var;
	baro_hgt.test_ratio = sq(baro_hgt.innovation) / (sq(innov_gate) * baro_hgt.innovation_variance);
	baro_hgt.innovation_rejected = (baro_hgt.test_ratio <= 0.f || baro_hgt.test_ratio > 1.f);

	baro_hgt.fused = false; // reset

	if (_control_status.flags.baro_hgt) {

		baro_hgt.fusion_enabled = true;

		bool innov_check_pass = !baro_hgt.innovation_rejected;

		// if there is bad vertical acceleration data, then don't reject measurement,
		// but limit innovation to prevent spikes that could destabilise the filter
		float innovation;

		if (_fault_status.flags.bad_acc_vertical && !innov_check_pass) {
			const float innov_limit = innov_gate * sqrtf(baro_hgt.innovation_variance);
			innovation = math::constrain(baro_hgt.innovation, -innov_limit, innov_limit);
			innov_check_pass = true;
			baro_hgt.innovation_rejected = false;

		} else {
			innovation = baro_hgt.innovation;
		}

		if (innov_check_pass) {
			if (fuseVelPosHeight(innovation, baro_hgt.innovation_variance, 5)) {
				baro_hgt.fused = true;
				baro_hgt.time_last_fuse = _time_last_imu;
			}
		}

	} else {
		baro_hgt.fusion_enabled = false;
	}

	baro_hgt.timestamp_sample = _baro_sample_delayed.time_us;
}

void Ekf::updateRngHgt()
{
	if (!PX4_ISFINITE(_rng_hgt_offset)) {
		_rng_hgt_offset = _state.pos(2) - (-math::max(_range_sensor.getDistBottom(), _params.rng_gnd_clearance));
	}

	if (!_control_status.flags.rng_hgt) {

		float innov = _state.pos(2) - (-math::max(_range_sensor.getDistBottom(), _params.rng_gnd_clearance)) - _rng_hgt_offset;

		// apply a 10 second first order low pass filter to offset
		const float local_time_step = math::constrain(1e-6f * _delta_time_rng_us, 0.f, 1.f);
		const float offset_rate_correction = 0.1f * innov;
		_rng_hgt_offset += local_time_step * math::constrain(offset_rate_correction, -0.1f, 0.1f);
	}

	auto &rng_hgt = _aid_src_rng_hgt;

	// use range finder with tilt correction
	rng_hgt.innovation = _state.pos(2) - (-math::max(_range_sensor.getDistBottom(),
					      _params.rng_gnd_clearance)) - _rng_hgt_offset;

	// innovation gate size
	float innov_gate = fmaxf(_params.range_innov_gate, 1.f);

	// observation variance - user parameter defined
	float obs_var = fmaxf(sq(_params.range_noise) + sq(_params.range_noise_scaler * _range_sensor.getDistBottom()), 0.01f);

	rng_hgt.innovation_variance = P(9, 9) + obs_var;
	rng_hgt.test_ratio = sq(rng_hgt.innovation) / (sq(innov_gate) * rng_hgt.innovation_variance);
	rng_hgt.innovation_rejected = (rng_hgt.test_ratio <= 0.f || rng_hgt.test_ratio > 1.f);

	rng_hgt.fused = false; // reset

	if (_control_status.flags.baro_hgt) {

		rng_hgt.fusion_enabled = true;

		bool innov_check_pass = !rng_hgt.innovation_rejected;

		// if there is bad vertical acceleration data, then don't reject measurement,
		// but limit innovation to prevent spikes that could destabilise the filter
		float innovation;

		if (_fault_status.flags.bad_acc_vertical && !innov_check_pass) {
			const float innov_limit = innov_gate * sqrtf(rng_hgt.innovation_variance);
			innovation = math::constrain(rng_hgt.innovation, -innov_limit, innov_limit);
			innov_check_pass = true;
			rng_hgt.innovation_rejected = false;

		} else {
			innovation = rng_hgt.innovation;
		}

		if (innov_check_pass) {
			if (fuseVelPosHeight(innovation, rng_hgt.innovation_variance, 5)) {
				rng_hgt.fused = true;
				rng_hgt.time_last_fuse = _time_last_imu;
			}
		}

	} else {
		rng_hgt.fusion_enabled = false;
	}

	rng_hgt.timestamp_sample = _baro_sample_delayed.time_us;
}
