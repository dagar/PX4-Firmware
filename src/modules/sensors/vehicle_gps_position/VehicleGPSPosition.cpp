/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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

#include "VehicleGPSPosition.hpp"

#include <px4_platform_common/log.h>

using namespace matrix;
using namespace time_literals;

VehicleGPSPosition::VehicleGPSPosition() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::att_pos_ctrl)
{
}

VehicleGPSPosition::~VehicleGPSPosition()
{
	Stop();
}

bool
VehicleGPSPosition::Start()
{
	// force initial updates
	ParametersUpdate(true);

	// needed to change the active sensor if the primary stops updating
	bool registered_1 = _sensor_gps_sub[0].registerCallback();
	bool registered_2 = _sensor_gps_sub[1].registerCallback();

	return registered_1 || registered_2;
}

void
VehicleGPSPosition::Stop()
{
	Deinit();

	// clear all registered callbacks
	for (auto &sub : _sensor_sub) {
		sub.unregisterCallback();
	}
}

void
VehicleGPSPosition::ParametersUpdate(bool force)
{
	// Check if parameters have changed
	if (_params_sub.updated() || force) {
		// clear update
		parameter_update_s param_update;
		_params_sub.copy(&param_update);

		updateParams();
	}
}

void
VehicleGPSPosition::Run()
{
	ParametersUpdate();

	// read gps1 data if available
	bool gps1_updated = _sensor_gps_sub[0].updated();

	if (gps1_updated) {
		sensor_gps_s gps;

		if (_sensor_gps_sub[0].copy(&gps)) {
			_gps_state[0].time_usec = gps.timestamp;
			_gps_state[0].lat = gps.lat;
			_gps_state[0].lon = gps.lon;
			_gps_state[0].alt = gps.alt;
			_gps_state[0].yaw = gps.heading;
			_gps_state[0].yaw_offset = gps.heading_offset;
			_gps_state[0].fix_type = gps.fix_type;
			_gps_state[0].eph = gps.eph;
			_gps_state[0].epv = gps.epv;
			_gps_state[0].sacc = gps.s_variance_m_s;
			_gps_state[0].vel_m_s = gps.vel_m_s;
			_gps_state[0].vel_ned[0] = gps.vel_n_m_s;
			_gps_state[0].vel_ned[1] = gps.vel_e_m_s;
			_gps_state[0].vel_ned[2] = gps.vel_d_m_s;
			_gps_state[0].vel_ned_valid = gps.vel_ned_valid;
			_gps_state[0].nsats = gps.satellites_used;
			//TODO: add gdop to gps topic
			_gps_state[0].gdop = 0.0f;
			_gps_alttitude_ellipsoid[0] = gps.alt_ellipsoid;
		}
	}

	// check for second GPS receiver data
	bool gps2_updated = _sensor_gps_sub[1].updated();

	if (gps2_updated) {
		sensor_gps_s gps;

		if (_sensor_gps_sub[1].copy(&gps)) {
			_gps_state[1].time_usec = gps.timestamp;
			_gps_state[1].lat = gps.lat;
			_gps_state[1].lon = gps.lon;
			_gps_state[1].alt = gps.alt;
			_gps_state[1].yaw = gps.heading;
			_gps_state[1].yaw_offset = gps.heading_offset;
			_gps_state[1].fix_type = gps.fix_type;
			_gps_state[1].eph = gps.eph;
			_gps_state[1].epv = gps.epv;
			_gps_state[1].sacc = gps.s_variance_m_s;
			_gps_state[1].vel_m_s = gps.vel_m_s;
			_gps_state[1].vel_ned[0] = gps.vel_n_m_s;
			_gps_state[1].vel_ned[1] = gps.vel_e_m_s;
			_gps_state[1].vel_ned[2] = gps.vel_d_m_s;
			_gps_state[1].vel_ned_valid = gps.vel_ned_valid;
			_gps_state[1].nsats = gps.satellites_used;
			//TODO: add gdop to gps topic
			_gps_state[1].gdop = 0.0f;
			_gps_alttitude_ellipsoid[1] = gps.alt_ellipsoid;
		}
	}

	if ((_param_sens_gps_mask.get() == 0) && gps1_updated) {
		// When GPS blending is disabled we always use the first receiver instance
		_ekf.setGpsData(_gps_state[0].time_usec, _gps_state[0]);

	} else if ((_param_sens_gps_mask.get() > 0) && (gps1_updated || gps2_updated)) {
		// blend dual receivers if available

		// calculate blending weights
		if (!blend_gps_data()) {
			// handle case where the blended states cannot be updated
			if (_gps_state[0].fix_type > _gps_state[1].fix_type) {
				// GPS 1 has the best fix status so use that
				_gps_select_index = 0;

			} else if (_gps_state[1].fix_type > _gps_state[0].fix_type) {
				// GPS 2 has the best fix status so use that
				_gps_select_index = 1;

			} else if (_gps_select_index == 2) {
				// use last receiver we received data from
				if (gps1_updated) {
					_gps_select_index = 0;

				} else if (gps2_updated) {
					_gps_select_index = 1;
				}
			}

			// Only use selected receiver data if it has been updated
			if ((gps1_updated && _gps_select_index == 0) || (gps2_updated && _gps_select_index == 1)) {
				_gps_new_output_data = true;

			} else {
				_gps_new_output_data = false;
			}
		}

		if (_gps_new_output_data) {
			// correct the _gps_state data for steady state offsets and write to _gps_output
			apply_gps_offsets();

			// calculate a blended output from the offset corrected receiver data
			if (_gps_select_index == 2) {
				calc_gps_blend_output();
			}

			// write selected GPS to EKF
			_ekf.setGpsData(_gps_output[_gps_select_index].time_usec, _gps_output[_gps_select_index]);

			// log blended solution as a third GPS instance
			vehicle_gps_position_s gps;
			gps.timestamp = _gps_output[_gps_select_index].time_usec;
			gps.lat = _gps_output[_gps_select_index].lat;
			gps.lon = _gps_output[_gps_select_index].lon;
			gps.alt = _gps_output[_gps_select_index].alt;
			gps.fix_type = _gps_output[_gps_select_index].fix_type;
			gps.eph = _gps_output[_gps_select_index].eph;
			gps.epv = _gps_output[_gps_select_index].epv;
			gps.s_variance_m_s = _gps_output[_gps_select_index].sacc;
			gps.vel_m_s = _gps_output[_gps_select_index].vel_m_s;
			gps.vel_n_m_s = _gps_output[_gps_select_index].vel_ned[0];
			gps.vel_e_m_s = _gps_output[_gps_select_index].vel_ned[1];
			gps.vel_d_m_s = _gps_output[_gps_select_index].vel_ned[2];
			gps.vel_ned_valid = _gps_output[_gps_select_index].vel_ned_valid;
			gps.satellites_used = _gps_output[_gps_select_index].nsats;
			gps.heading = _gps_output[_gps_select_index].yaw;
			gps.heading_offset = _gps_output[_gps_select_index].yaw_offset;
			gps.selected = _gps_select_index;

			// Publish to the EKF blended GPS topic
			_blended_gps_pub.publish(gps);

			// clear flag to avoid re-use of the same data
			_gps_new_output_data = false;
		}
	}
}

bool
VehicleGPSPosition::blend_gps_data()
{
	// zero the blend weights
	memset(&_blend_weights, 0, sizeof(_blend_weights));

	/*
	 * If both receivers have the same update rate, use the oldest non-zero time.
	 * If two receivers with different update rates are used, use the slowest.
	 * If time difference is excessive, use newest to prevent a disconnected receiver
	 * from blocking updates.
	 */

	// Calculate the time step for each receiver with some filtering to reduce the effects of jitter
	// Find the largest and smallest time step.
	float dt_max = 0.0f;
	float dt_min = 0.3f;

	for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
		float raw_dt = 0.001f * (float)(_gps_state[i].time_usec - _time_prev_us[i]);

		if (raw_dt > 0.0f && raw_dt < 0.3f) {
			_gps_dt[i] = 0.1f * raw_dt + 0.9f * _gps_dt[i];
		}

		if (_gps_dt[i] > dt_max) {
			dt_max = _gps_dt[i];
			_gps_slowest_index = i;
		}

		if (_gps_dt[i] < dt_min) {
			dt_min = _gps_dt[i];
		}
	}

	// Find the receiver that is last be updated
	uint64_t max_us = 0; // newest non-zero system time of arrival of a GPS message
	uint64_t min_us = -1; // oldest non-zero system time of arrival of a GPS message

	for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
		// Find largest and smallest times
		if (_gps_state[i].time_usec > max_us) {
			max_us = _gps_state[i].time_usec;
			_gps_newest_index = i;
		}

		if ((_gps_state[i].time_usec < min_us) && (_gps_state[i].time_usec > 0)) {
			min_us = _gps_state[i].time_usec;
			_gps_oldest_index = i;
		}
	}

	if ((max_us - min_us) > 300000) {
		// A receiver has timed out so fall out of blending
		return false;
	}

	/*
	 * If the largest dt is less than 20% greater than the smallest, then we have  receivers
	 * running at the same rate then we wait until we have two messages with an arrival time
	 * difference that is less than 50% of the smallest time step and use the time stamp from
	 * the newest data.
	 * Else we have two receivers at different update rates and use the slowest receiver
	 * as the timing reference.
	 */

	if ((dt_max - dt_min) < 0.2f * dt_min) {
		// both receivers assumed to be running at the same rate
		if ((max_us - min_us) < (uint64_t)(5e5f * dt_min)) {
			// data arrival within a short time window enables the two measurements to be blended
			_gps_time_ref_index = _gps_newest_index;
			_gps_new_output_data = true;
		}

	} else {
		// both receivers running at different rates
		_gps_time_ref_index = _gps_slowest_index;

		if (_gps_state[_gps_time_ref_index].time_usec > _time_prev_us[_gps_time_ref_index]) {
			// blend data at the rate of the slower receiver
			_gps_new_output_data = true;
		}
	}

	if (_gps_new_output_data) {
		_gps_blended_state.time_usec = _gps_state[_gps_time_ref_index].time_usec;

		// calculate the sum squared speed accuracy across all GPS sensors
		float speed_accuracy_sum_sq = 0.0f;

		if (_param_sens_gps_mask.get() & BLEND_MASK_USE_SPD_ACC) {
			for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
				if (_gps_state[i].fix_type >= 3 && _gps_state[i].sacc > 0.0f) {
					speed_accuracy_sum_sq += _gps_state[i].sacc * _gps_state[i].sacc;

				} else {
					// not all receivers support this metric so set it to zero and don't use it
					speed_accuracy_sum_sq = 0.0f;
					break;
				}
			}
		}

		// calculate the sum squared horizontal position accuracy across all GPS sensors
		float horizontal_accuracy_sum_sq = 0.0f;

		if (_param_sens_gps_mask.get() & BLEND_MASK_USE_HPOS_ACC) {
			for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
				if (_gps_state[i].fix_type >= 2 && _gps_state[i].eph > 0.0f) {
					horizontal_accuracy_sum_sq += _gps_state[i].eph * _gps_state[i].eph;

				} else {
					// not all receivers support this metric so set it to zero and don't use it
					horizontal_accuracy_sum_sq = 0.0f;
					break;
				}
			}
		}

		// calculate the sum squared vertical position accuracy across all GPS sensors
		float vertical_accuracy_sum_sq = 0.0f;

		if (_param_sens_gps_mask.get() & BLEND_MASK_USE_VPOS_ACC) {
			for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
				if (_gps_state[i].fix_type >= 3 && _gps_state[i].epv > 0.0f) {
					vertical_accuracy_sum_sq += _gps_state[i].epv * _gps_state[i].epv;

				} else {
					// not all receivers support this metric so set it to zero and don't use it
					vertical_accuracy_sum_sq = 0.0f;
					break;
				}
			}
		}

		// Check if we can do blending using reported accuracy
		bool can_do_blending = (horizontal_accuracy_sum_sq > 0.0f || vertical_accuracy_sum_sq > 0.0f
					|| speed_accuracy_sum_sq > 0.0f);

		// if we can't do blending using reported accuracy, return false and hard switch logic will be used instead
		if (!can_do_blending) {
			return false;
		}

		float sum_of_all_weights = 0.0f;

		// calculate a weighting using the reported speed accuracy
		float spd_blend_weights[GPS_MAX_RECEIVERS] = {};

		if (speed_accuracy_sum_sq > 0.0f && (_param_sens_gps_mask.get() & BLEND_MASK_USE_SPD_ACC)) {
			// calculate the weights using the inverse of the variances
			float sum_of_spd_weights = 0.0f;

			for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
				if (_gps_state[i].fix_type >= 3 && _gps_state[i].sacc >= 0.001f) {
					spd_blend_weights[i] = 1.0f / (_gps_state[i].sacc * _gps_state[i].sacc);
					sum_of_spd_weights += spd_blend_weights[i];
				}
			}

			// normalise the weights
			if (sum_of_spd_weights > 0.0f) {
				for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
					spd_blend_weights[i] = spd_blend_weights[i] / sum_of_spd_weights;
				}

				sum_of_all_weights += 1.0f;
			}
		}

		// calculate a weighting using the reported horizontal position
		float hpos_blend_weights[GPS_MAX_RECEIVERS] = {};

		if (horizontal_accuracy_sum_sq > 0.0f && (_param_sens_gps_mask.get() & BLEND_MASK_USE_HPOS_ACC)) {
			// calculate the weights using the inverse of the variances
			float sum_of_hpos_weights = 0.0f;

			for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
				if (_gps_state[i].fix_type >= 2 && _gps_state[i].eph >= 0.001f) {
					hpos_blend_weights[i] = horizontal_accuracy_sum_sq / (_gps_state[i].eph * _gps_state[i].eph);
					sum_of_hpos_weights += hpos_blend_weights[i];
				}
			}

			// normalise the weights
			if (sum_of_hpos_weights > 0.0f) {
				for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
					hpos_blend_weights[i] = hpos_blend_weights[i] / sum_of_hpos_weights;
				}

				sum_of_all_weights += 1.0f;
			}
		}

		// calculate a weighting using the reported vertical position accuracy
		float vpos_blend_weights[GPS_MAX_RECEIVERS] = {};

		if (vertical_accuracy_sum_sq > 0.0f && (_param_sens_gps_mask.get() & BLEND_MASK_USE_VPOS_ACC)) {
			// calculate the weights using the inverse of the variances
			float sum_of_vpos_weights = 0.0f;

			for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
				if (_gps_state[i].fix_type >= 3 && _gps_state[i].epv >= 0.001f) {
					vpos_blend_weights[i] = vertical_accuracy_sum_sq / (_gps_state[i].epv * _gps_state[i].epv);
					sum_of_vpos_weights += vpos_blend_weights[i];
				}
			}

			// normalise the weights
			if (sum_of_vpos_weights > 0.0f) {
				for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
					vpos_blend_weights[i] = vpos_blend_weights[i] / sum_of_vpos_weights;
				}

				sum_of_all_weights += 1.0f;
			};
		}

		// calculate an overall weight
		for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
			_blend_weights[i] = (hpos_blend_weights[i] + vpos_blend_weights[i] + spd_blend_weights[i]) / sum_of_all_weights;
		}

		// With updated weights we can calculate a blended GPS solution and
		// offsets for each physical receiver
		update_gps_blend_states();
		update_gps_offsets();
		_gps_select_index = 2;
	}

	return true;
}

void
VehicleGPSPosition::update_gps_blend_states()
{
	// initialise the blended states so we can accumulate the results using the weightings for each GPS receiver.
	_gps_blended_state.time_usec = 0;
	_gps_blended_state.lat = 0;
	_gps_blended_state.lon = 0;
	_gps_blended_state.alt = 0;
	_gps_blended_state.fix_type = 0;
	_gps_blended_state.eph = FLT_MAX;
	_gps_blended_state.epv = FLT_MAX;
	_gps_blended_state.sacc = FLT_MAX;
	_gps_blended_state.vel_m_s = 0.0f;
	_gps_blended_state.vel_ned[0] = 0.0f;
	_gps_blended_state.vel_ned[1] = 0.0f;
	_gps_blended_state.vel_ned[2] = 0.0f;
	_gps_blended_state.vel_ned_valid = true;
	_gps_blended_state.nsats = 0;
	_gps_blended_state.gdop = FLT_MAX;

	_blended_antenna_offset.zero();

	// combine the the GPS states into a blended solution using the weights calculated in calc_blend_weights()
	for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
		// blend the timing data
		_gps_blended_state.time_usec += (uint64_t)((double)_gps_state[i].time_usec * (double)_blend_weights[i]);

		// use the highest status
		if (_gps_state[i].fix_type > _gps_blended_state.fix_type) {
			_gps_blended_state.fix_type = _gps_state[i].fix_type;
		}

		// calculate a blended average speed and velocity vector
		_gps_blended_state.vel_m_s += _gps_state[i].vel_m_s * _blend_weights[i];
		_gps_blended_state.vel_ned[0] += _gps_state[i].vel_ned[0] * _blend_weights[i];
		_gps_blended_state.vel_ned[1] += _gps_state[i].vel_ned[1] * _blend_weights[i];
		_gps_blended_state.vel_ned[2] += _gps_state[i].vel_ned[2] * _blend_weights[i];

		// Assume blended error magnitude, DOP and sat count is equal to the best value from contributing receivers
		// If any receiver contributing has an invalid velocity, then report blended velocity as invalid
		if (_blend_weights[i] > 0.0f) {

			if (_gps_state[i].eph > 0.0f
			    && _gps_state[i].eph < _gps_blended_state.eph) {
				_gps_blended_state.eph = _gps_state[i].eph;
			}

			if (_gps_state[i].epv > 0.0f
			    && _gps_state[i].epv < _gps_blended_state.epv) {
				_gps_blended_state.epv = _gps_state[i].epv;
			}

			if (_gps_state[i].sacc > 0.0f
			    && _gps_state[i].sacc < _gps_blended_state.sacc) {
				_gps_blended_state.sacc = _gps_state[i].sacc;
			}

			if (_gps_state[i].gdop > 0
			    && _gps_state[i].gdop < _gps_blended_state.gdop) {
				_gps_blended_state.gdop = _gps_state[i].gdop;
			}

			if (_gps_state[i].nsats > 0
			    && _gps_state[i].nsats > _gps_blended_state.nsats) {
				_gps_blended_state.nsats = _gps_state[i].nsats;
			}

			if (!_gps_state[i].vel_ned_valid) {
				_gps_blended_state.vel_ned_valid = false;
			}
		}

		// TODO read parameters for individual GPS antenna positions and blend
		// Vector3f temp_antenna_offset = _antenna_offset[i];
		// temp_antenna_offset *= _blend_weights[i];
		// _blended_antenna_offset += temp_antenna_offset;
	}

	/*
	 * Calculate the instantaneous weighted average location using  available GPS instances and store in  _gps_state.
	 * This is statistically the most likely location, but may not be stable enough for direct use by the EKF.
	*/

	// Use the GPS with the highest weighting as the reference position
	float best_weight = 0.0f;

	for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
		if (_blend_weights[i] > best_weight) {
			best_weight = _blend_weights[i];
			_gps_best_index = i;
			_gps_blended_state.lat = _gps_state[i].lat;
			_gps_blended_state.lon = _gps_state[i].lon;
			_gps_blended_state.alt = _gps_state[i].alt;
		}
	}

	// Convert each GPS position to a local NEU offset relative to the reference position
	Vector2f blended_NE_offset_m;
	blended_NE_offset_m.zero();
	float blended_alt_offset_mm = 0.0f;

	for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
		if ((_blend_weights[i] > 0.0f) && (i != _gps_best_index)) {
			// calculate the horizontal offset
			Vector2f horiz_offset{};
			get_vector_to_next_waypoint((_gps_blended_state.lat / 1.0e7),
						    (_gps_blended_state.lon / 1.0e7), (_gps_state[i].lat / 1.0e7), (_gps_state[i].lon / 1.0e7),
						    &horiz_offset(0), &horiz_offset(1));

			// sum weighted offsets
			blended_NE_offset_m += horiz_offset * _blend_weights[i];

			// calculate vertical offset
			float vert_offset = (float)(_gps_state[i].alt - _gps_blended_state.alt);

			// sum weighted offsets
			blended_alt_offset_mm += vert_offset * _blend_weights[i];
		}
	}

	// Add the sum of weighted offsets to the reference position to obtain the blended position
	double lat_deg_now = (double)_gps_blended_state.lat * 1.0e-7;
	double lon_deg_now = (double)_gps_blended_state.lon * 1.0e-7;
	double lat_deg_res, lon_deg_res;
	add_vector_to_global_position(lat_deg_now, lon_deg_now, blended_NE_offset_m(0), blended_NE_offset_m(1), &lat_deg_res,
				      &lon_deg_res);
	_gps_blended_state.lat = (int32_t)(1.0E7 * lat_deg_res);
	_gps_blended_state.lon = (int32_t)(1.0E7 * lon_deg_res);
	_gps_blended_state.alt += (int32_t)blended_alt_offset_mm;

	// Take GPS heading from the highest weighted receiver that is publishing a valid .heading value
	uint8_t gps_best_yaw_index = 0;
	best_weight = 0.0f;

	for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
		if (PX4_ISFINITE(_gps_state[i].yaw) && (_blend_weights[i] > best_weight)) {
			best_weight = _blend_weights[i];
			gps_best_yaw_index = i;
		}
	}

	_gps_blended_state.yaw = _gps_state[gps_best_yaw_index].yaw;
	_gps_blended_state.yaw_offset = _gps_state[gps_best_yaw_index].yaw_offset;
}

void
VehicleGPSPosition::update_gps_offsets()
{
	// Calculate filter coefficients to be applied to the offsets for each GPS position and height offset
	// Increase the filter time constant proportional to the inverse of the weighting
	// A weighting of 1 will make the offset adjust the slowest, a weighting of 0 will make it adjust with zero filtering
	float alpha[GPS_MAX_RECEIVERS] = {};
	float omega_lpf = 1.0f / fmaxf(_param_sens_gps_tau.get(), 1.0f);

	for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
		if (_gps_state[i].time_usec - _time_prev_us[i] > 0) {
			// calculate the filter coefficient that achieves the time constant specified by the user adjustable parameter
			float min_alpha = constrain(omega_lpf * 1e-6f * (float)(_gps_state[i].time_usec - _time_prev_us[i]),
						    0.0f, 1.0f);

			// scale the filter coefficient so that time constant is inversely proprtional to weighting
			if (_blend_weights[i] > min_alpha) {
				alpha[i] = min_alpha / _blend_weights[i];

			} else {
				alpha[i] = 1.0f;
			}

			_time_prev_us[i] = _gps_state[i].time_usec;
		}
	}

	// Calculate a filtered position delta for each GPS relative to the blended solution state
	for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
		Vector2f offset;
		get_vector_to_next_waypoint((_gps_state[i].lat / 1.0e7), (_gps_state[i].lon / 1.0e7),
					    (_gps_blended_state.lat / 1.0e7), (_gps_blended_state.lon / 1.0e7), &offset(0), &offset(1));
		_NE_pos_offset_m[i] = offset * alpha[i] + _NE_pos_offset_m[i] * (1.0f - alpha[i]);
		_hgt_offset_mm[i] = (float)(_gps_blended_state.alt - _gps_state[i].alt) *  alpha[i] +
				    _hgt_offset_mm[i] * (1.0f - alpha[i]);
	}

	// calculate offset limits from the largest difference between receivers
	Vector2f max_ne_offset{};
	float max_alt_offset = 0;

	for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
		for (uint8_t j = i; j < GPS_MAX_RECEIVERS; j++) {
			if (i != j) {
				Vector2f offset;
				get_vector_to_next_waypoint((_gps_state[i].lat / 1.0e7), (_gps_state[i].lon / 1.0e7),
							    (_gps_state[j].lat / 1.0e7), (_gps_state[j].lon / 1.0e7), &offset(0), &offset(1));
				max_ne_offset(0) = fmaxf(max_ne_offset(0), fabsf(offset(0)));
				max_ne_offset(1) = fmaxf(max_ne_offset(1), fabsf(offset(1)));
				max_alt_offset = fmaxf(max_alt_offset, fabsf((float)(_gps_state[i].alt - _gps_state[j].alt)));
			}
		}
	}

	// apply offset limits
	for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
		_NE_pos_offset_m[i](0) = constrain(_NE_pos_offset_m[i](0), -max_ne_offset(0), max_ne_offset(0));
		_NE_pos_offset_m[i](1) = constrain(_NE_pos_offset_m[i](1), -max_ne_offset(1), max_ne_offset(1));
		_hgt_offset_mm[i] = constrain(_hgt_offset_mm[i], -max_alt_offset, max_alt_offset);
	}
}

void
VehicleGPSPosition::apply_gps_offsets()
{
	// calculate offset corrected output for each physical GPS.
	for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
		// Add the sum of weighted offsets to the reference position to obtain the blended position
		double lat_deg_now = (double)_gps_state[i].lat * 1.0e-7;
		double lon_deg_now = (double)_gps_state[i].lon * 1.0e-7;
		double lat_deg_res, lon_deg_res;
		add_vector_to_global_position(lat_deg_now, lon_deg_now, _NE_pos_offset_m[i](0), _NE_pos_offset_m[i](1), &lat_deg_res,
					      &lon_deg_res);
		_gps_output[i].lat = (int32_t)(1.0E7 * lat_deg_res);
		_gps_output[i].lon = (int32_t)(1.0E7 * lon_deg_res);
		_gps_output[i].alt = _gps_state[i].alt + (int32_t)_hgt_offset_mm[i];

		// other receiver data is used uncorrected
		_gps_output[i].time_usec	= _gps_state[i].time_usec;
		_gps_output[i].fix_type		= _gps_state[i].fix_type;
		_gps_output[i].vel_m_s		= _gps_state[i].vel_m_s;
		_gps_output[i].vel_ned[0]	= _gps_state[i].vel_ned[0];
		_gps_output[i].vel_ned[1]	= _gps_state[i].vel_ned[1];
		_gps_output[i].vel_ned[2]	= _gps_state[i].vel_ned[2];
		_gps_output[i].eph		= _gps_state[i].eph;
		_gps_output[i].epv		= _gps_state[i].epv;
		_gps_output[i].sacc		= _gps_state[i].sacc;
		_gps_output[i].gdop		= _gps_state[i].gdop;
		_gps_output[i].nsats		= _gps_state[i].nsats;
		_gps_output[i].vel_ned_valid	= _gps_state[i].vel_ned_valid;
		_gps_output[i].yaw		= _gps_state[i].yaw;
		_gps_output[i].yaw_offset	= _gps_state[i].yaw_offset;
	}
}

void
VehicleGPSPosition::calc_gps_blend_output()
{
	// Convert each GPS position to a local NEU offset relative to the reference position
	// which is defined as the positon of the blended solution calculated from non offset corrected data
	Vector2f blended_NE_offset_m;
	blended_NE_offset_m.zero();
	float blended_alt_offset_mm = 0.0f;

	for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
		if (_blend_weights[i] > 0.0f) {
			// calculate the horizontal offset
			Vector2f horiz_offset{};
			get_vector_to_next_waypoint((_gps_blended_state.lat / 1.0e7),
						    (_gps_blended_state.lon / 1.0e7),
						    (_gps_output[i].lat / 1.0e7),
						    (_gps_output[i].lon / 1.0e7),
						    &horiz_offset(0),
						    &horiz_offset(1));

			// sum weighted offsets
			blended_NE_offset_m += horiz_offset * _blend_weights[i];

			// calculate vertical offset
			float vert_offset = (float)(_gps_output[i].alt - _gps_blended_state.alt);

			// sum weighted offsets
			blended_alt_offset_mm += vert_offset * _blend_weights[i];
		}
	}

	// Add the sum of weighted offsets to the reference position to obtain the blended position
	double lat_deg_now = (double)_gps_blended_state.lat * 1.0e-7;
	double lon_deg_now = (double)_gps_blended_state.lon * 1.0e-7;
	double lat_deg_res, lon_deg_res;
	add_vector_to_global_position(lat_deg_now, lon_deg_now, blended_NE_offset_m(0), blended_NE_offset_m(1), &lat_deg_res,
				      &lon_deg_res);
	_gps_output[GPS_BLENDED_INSTANCE].lat = (int32_t)(1.0E7 * lat_deg_res);
	_gps_output[GPS_BLENDED_INSTANCE].lon = (int32_t)(1.0E7 * lon_deg_res);
	_gps_output[GPS_BLENDED_INSTANCE].alt = _gps_blended_state.alt + (int32_t)blended_alt_offset_mm;

	// Copy remaining data from internal states to output
	_gps_output[GPS_BLENDED_INSTANCE].time_usec	= _gps_blended_state.time_usec;
	_gps_output[GPS_BLENDED_INSTANCE].fix_type	= _gps_blended_state.fix_type;
	_gps_output[GPS_BLENDED_INSTANCE].vel_m_s	= _gps_blended_state.vel_m_s;
	_gps_output[GPS_BLENDED_INSTANCE].vel_ned[0]	= _gps_blended_state.vel_ned[0];
	_gps_output[GPS_BLENDED_INSTANCE].vel_ned[1]	= _gps_blended_state.vel_ned[1];
	_gps_output[GPS_BLENDED_INSTANCE].vel_ned[2]	= _gps_blended_state.vel_ned[2];
	_gps_output[GPS_BLENDED_INSTANCE].eph		= _gps_blended_state.eph;
	_gps_output[GPS_BLENDED_INSTANCE].epv		= _gps_blended_state.epv;
	_gps_output[GPS_BLENDED_INSTANCE].sacc		= _gps_blended_state.sacc;
	_gps_output[GPS_BLENDED_INSTANCE].gdop		= _gps_blended_state.gdop;
	_gps_output[GPS_BLENDED_INSTANCE].nsats		= _gps_blended_state.nsats;
	_gps_output[GPS_BLENDED_INSTANCE].vel_ned_valid	= _gps_blended_state.vel_ned_valid;
	_gps_output[GPS_BLENDED_INSTANCE].yaw		= _gps_blended_state.yaw;
	_gps_output[GPS_BLENDED_INSTANCE].yaw_offset	= _gps_blended_state.yaw_offset;
}

void
VehicleGPSPosition::PrintStatus()
{
	PX4_INFO("selected sensor: %d", _selected_sensor);
}
