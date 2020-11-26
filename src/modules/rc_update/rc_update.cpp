/****************************************************************************
 *
 *   Copyright (c) 2016-2020 PX4 Development Team. All rights reserved.
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

#include "rc_update.h"

using namespace time_literals;

namespace RCUpdate
{

// TODO: find a better home for this
static bool operator ==(const manual_control_switches_s &a, const manual_control_switches_s &b)
{
	return (a.mode_slot == b.mode_slot &&
		a.mode_switch == b.mode_switch &&
		a.return_switch == b.return_switch &&
		a.rattitude_switch == b.rattitude_switch &&
		a.posctl_switch == b.posctl_switch &&
		a.loiter_switch == b.loiter_switch &&
		a.acro_switch == b.acro_switch &&
		a.offboard_switch == b.offboard_switch &&
		a.kill_switch == b.kill_switch &&
		a.arm_switch == b.arm_switch &&
		a.transition_switch == b.transition_switch &&
		a.gear_switch == b.gear_switch &&
		a.stab_switch == b.stab_switch &&
		a.man_switch == b.man_switch);
}

static bool operator !=(const manual_control_switches_s &a, const manual_control_switches_s &b) { return !(a == b); }


RCUpdate::RCUpdate() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
	for (unsigned i = 0; i < RC_MAX_CHAN_COUNT; i++) {
		_parameter_handles.min[i] = PARAM_INVALID;
		_parameter_handles.trim[i] = PARAM_INVALID;
		_parameter_handles.max[i] = PARAM_INVALID;
		_parameter_handles.rev[i] = PARAM_INVALID;
		_parameter_handles.dz[i] = PARAM_INVALID;
	}

	parameters_updated();

	// RC to parameter mapping for changing parameters with RC
	_parameter_handles.rc_map_param[0] = _param_rc_map_param1.handle();
	_parameter_handles.rc_map_param[1] = _param_rc_map_param2.handle();
	_parameter_handles.rc_map_param[2] = _param_rc_map_param3.handle();

	for (int i = 0; i < rc_parameter_map_s::RC_PARAM_MAP_NCHAN; i++) {
		_parameter_handles.rc_param[i] = PARAM_INVALID;
		_rc_function[RC_FUNCTION::PARAM_1 + i] = RC_FUNCTION::INVALID;
		_rc_parameter_map.valid[i] = false;
	}

	_rc_parameter_map.timestamp = 0;

	rc_parameter_map_poll(true /* forced */);
}

RCUpdate::~RCUpdate()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
	perf_free(_valid_data_interval_perf);
}

bool RCUpdate::init()
{
	if (!_input_rc_sub.registerCallback()) {
		PX4_ERR("input_rc callback registration failed!");
		return false;
	}

	return true;
}

static int32_t getRCCalibration(int index, param_t &param_handle, const char *param_suffix, int16_t min, int16_t max)
{
	char nbuf[16];
	sprintf(nbuf, "RC%d_%s", index + 1, param_suffix);

	// RCx_y
	if (param_handle == PARAM_INVALID) {
		param_handle = param_find(nbuf);

		if (param_handle == PARAM_INVALID) {
			PX4_ERR("unable to find parameter %s", nbuf);
			return 0;
		}
	}

	float value = 0.f;

	if (param_get(param_handle, &value) == PX4_OK) {
		int32_t result = math::constrain((int16_t)roundf(value), min, max);

		float result_float = result;

		if (fabsf(result_float - value) > FLT_EPSILON) {
			PX4_ERR("%s value invalid, correcting %.3f -> %.3f", nbuf, (double)value, (double)result_float);
			param_reset(param_handle);
		}

		return result;
	}

	return 0;
}

void RCUpdate::parameters_updated()
{
	for (int32_t i = 0; i < math::max((int32_t)_channel_count_max, _param_rc_chan_cnt.get()); i++) {

		_parameters.min[i]  = getRCCalibration(i, _parameter_handles.min[i],   "MIN", RC_PWM_MIN_US, RC_PWM_MAX_US); // RCx_MIN
		_parameters.trim[i] = getRCCalibration(i, _parameter_handles.trim[i], "TRIM", RC_PWM_MIN_US, RC_PWM_MAX_US); // RCx_TRIM
		_parameters.max[i]  = getRCCalibration(i, _parameter_handles.max[i],   "MAX", RC_PWM_MIN_US, RC_PWM_MAX_US); // RCx_MAX
		_parameters.dz[i]   = getRCCalibration(i, _parameter_handles.dz[i],     "DZ",             0,           100); // RCx_DZ

		// RCx_REV: -1 to reverse channel
		_parameters.rev[i] = (getRCCalibration(i, _parameter_handles.rev[i], "REV", -1, 1) < 0);
	}
}

void RCUpdate::rc_parameter_map_poll(bool forced)
{
	if (_rc_parameter_map_sub.updated() || forced) {
		const hrt_abstime timestamp_previous = _rc_parameter_map.timestamp;
		_rc_parameter_map_sub.copy(&_rc_parameter_map);

		if (_rc_parameter_map.timestamp > timestamp_previous) {
			/* update parameter handles to which the RC channels are mapped */
			for (int i = 0; i < rc_parameter_map_s::RC_PARAM_MAP_NCHAN; i++) {
				// reset existing
				_parameter_handles.rc_param[i] = PARAM_INVALID;
				_rc_function[RC_FUNCTION::PARAM_1 + i] = RC_FUNCTION::INVALID;

				if (_rc_parameter_map.valid[i]) {
					/* Set the handle by index if the index is set, otherwise use the id */
					if (_rc_parameter_map.param_index[i] >= 0) {
						_parameter_handles.rc_param[i] = param_for_used_index((unsigned)_rc_parameter_map.param_index[i]);

					} else {
						_parameter_handles.rc_param[i] = param_find(&_rc_parameter_map.param_id[i * (rc_parameter_map_s::PARAM_ID_LEN + 1)]);
					}

					if (_parameter_handles.rc_param[i] != PARAM_INVALID) {
						int32_t rc_map_param = 0;

						if (param_get(_parameter_handles.rc_param[i], &rc_map_param) == PX4_OK) {
							if (rc_map_param > 0) {
								_rc_function[RC_FUNCTION::PARAM_1 + i] = rc_map_param - 1;
							}
						}
					}
				}
			}

			PX4_DEBUG("rc to parameter map updated");

			for (int i = 0; i < rc_parameter_map_s::RC_PARAM_MAP_NCHAN; i++) {
				PX4_DEBUG("\ti %d param_id %s scale %.3f value0 %.3f, min %.3f, max %.3f",
					  i,
					  &_rc_parameter_map.param_id[i * (rc_parameter_map_s::PARAM_ID_LEN + 1)],
					  (double)_rc_parameter_map.scale[i],
					  (double)_rc_parameter_map.value0[i],
					  (double)_rc_parameter_map.value_min[i],
					  (double)_rc_parameter_map.value_max[i]
					 );
			}
		}
	}
}

float RCUpdate::get_rc_value(int8_t channel, float min_value, float max_value) const
{
	if (channel >= 0) {
		const uint16_t value = _param_rc_median_filter.get() ? _rc_filters[channel].median() : _rc_filters[channel].latest();

		if (value != 0) {
			float calibrated_value = calibratedChannelValue(channel, value);
			return math::constrain(calibrated_value, min_value, max_value);
		}
	}

	return 0.f;
}

void RCUpdate::set_params_from_rc()
{
	bool param_set = false;

	for (int i = 0; i < rc_parameter_map_s::RC_PARAM_MAP_NCHAN; i++) {
		if (_rc_parameter_map.valid[i] && (_rc_function[RC_FUNCTION::PARAM_1 + i] != RC_FUNCTION::INVALID)) {

			int channel = _rc_function[RC_FUNCTION::PARAM_1 + i];
			float rc_val = get_rc_value(channel, -1.f, 1.f);

			// Check if the value has changed,
			if (fabsf(rc_val - _param_rc_values[i]) > FLT_EPSILON) {
				_param_rc_values[i] = rc_val;
				float param_val = math::constrain(_rc_parameter_map.value0[i] + _rc_parameter_map.scale[i] * rc_val,
								  _rc_parameter_map.value_min[i], _rc_parameter_map.value_max[i]);

				param_set_no_notification(_parameter_handles.rc_param[i], &param_val);
				param_set = true;
			}
		}
	}

	if (param_set) {
		param_notify_changes();
	}
}

void RCUpdate::Run()
{
	if (should_exit()) {
		_input_rc_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

	// check for parameter updates
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		updateParams();
		parameters_updated();
	}

	rc_parameter_map_poll();

	/* read low-level values from FMU or IO RC inputs (PPM, Spektrum, S.Bus) */
	input_rc_s rc_input;

	if (_input_rc_sub.update(&rc_input)) {

		const uint8_t channel_limit = math::min(rc_input.channel_count, RC_MAX_CHAN_COUNT);

		// warn if the channel count is changing (possibly indication of error)
		if (!rc_input.rc_lost && (_channel_count_previous != rc_input.channel_count) && (_channel_count_previous > 0)) {
			PX4_ERR("RC channel count changed %d -> %d", _channel_count_previous, rc_input.channel_count);
		}

		_channel_count_previous = rc_input.channel_count;

		if (channel_limit > _channel_count_max) {
			_channel_count_max = channel_limit;
			parameters_updated();
		}

		/* detect RC signal loss */
		bool signal_lost = true;

		/* check flags and require at least four channels to consider the signal valid */
		if (rc_input.rc_lost || rc_input.rc_failsafe || rc_input.channel_count < 4) {
			/* signal is lost or no enough channels */
			signal_lost = true;

		} else if ((rc_input.input_source == input_rc_s::RC_INPUT_SOURCE_PX4FMU_PPM ||
			    rc_input.input_source == input_rc_s::RC_INPUT_SOURCE_PX4IO_PPM)
			   && (rc_input.channel_count == 16)) {

			// This is a specific RC lost check for RFD 868+/900 Modems on PPM.
			// The observation was that when RC is lost, 16 channels are active and the first 12 are 1000
			// and the remaining ones are 0.
			for (unsigned i = 0; i < 16; i++) {
				if (i < 12 && rc_input.values[i] > 999 && rc_input.values[i] < 1005) {
					signal_lost = true;

				} else if (rc_input.values[i] == 0) {
					signal_lost = true;

				} else {
					signal_lost = false;
					break;
				}
			}

		} else {
			/* signal looks good */
			signal_lost = false;

			/* check failsafe */
			int8_t fs_ch = _param_rc_map_throttle.get() - 1; // get channel mapped to throttle

			if (_param_rc_map_failsafe.get() > 0) { // if not 0, use channel number instead of rc.function mapping
				fs_ch = _param_rc_map_failsafe.get() - 1;
			}

			if (_param_rc_fails_thr.get() > 0 && fs_ch >= 0) {
				/* failsafe configured */
				if ((_param_rc_fails_thr.get() < _parameters.min[fs_ch] && rc_input.values[fs_ch] < _param_rc_fails_thr.get()) ||
				    (_param_rc_fails_thr.get() > _parameters.max[fs_ch] && rc_input.values[fs_ch] > _param_rc_fails_thr.get())) {
					/* failsafe triggered, signal is lost by receiver */
					signal_lost = true;
				}
			}
		}

		/* only publish manual control if the signal is still present and was present once */
		if (!signal_lost && (rc_input.timestamp_last_signal > _last_timestamp_signal)) {

			_last_timestamp_signal = rc_input.timestamp_last_signal;
			perf_count(_valid_data_interval_perf);

			// check if channels actually updated
			bool rc_updated = false;

			for (unsigned i = 0; i < channel_limit; i++) {
				// insert mapped channel into median filter (ringbuffer)
				_rc_filters[i].insert(rc_input.values[i]);

				if (_rc_values_previous[i] != rc_input.values[i]) {
					rc_updated = true;
				}
			}

			UpdateSwitches();

			// limit processing if there's no update
			if (rc_updated || (hrt_elapsed_time(&_last_manual_control_setpoint_publish) > 300_ms)) {

				/* initialize manual setpoint */
				manual_control_setpoint_s manual_control_setpoint{};

				/* set the timestamp to the last signal time */
				manual_control_setpoint.timestamp_sample = rc_input.timestamp_last_signal;
				manual_control_setpoint.data_source = manual_control_setpoint_s::SOURCE_RC;

				/* limit controls */
				manual_control_setpoint.y     = get_rc_value(_param_rc_map_roll.get()     - 1, -1.f, 1.f);
				manual_control_setpoint.x     = get_rc_value(_param_rc_map_pitch.get()    - 1, -1.f, 1.f);
				manual_control_setpoint.r     = get_rc_value(_param_rc_map_yaw.get()      - 1, -1.f, 1.f);
				manual_control_setpoint.z     = get_rc_value(_param_rc_map_throttle.get() - 1,  0.f, 1.f);

				manual_control_setpoint.flaps = get_rc_value(_param_rc_map_flaps.get()    - 1, -1.f, 1.f);

				manual_control_setpoint.aux1  = get_rc_value(_param_rc_map_aux1.get()     - 1, -1.f, 1.f);
				manual_control_setpoint.aux2  = get_rc_value(_param_rc_map_aux2.get()     - 1, -1.f, 1.f);
				manual_control_setpoint.aux3  = get_rc_value(_param_rc_map_aux3.get()     - 1, -1.f, 1.f);
				manual_control_setpoint.aux4  = get_rc_value(_param_rc_map_aux4.get()     - 1, -1.f, 1.f);
				manual_control_setpoint.aux5  = get_rc_value(_param_rc_map_aux5.get()     - 1, -1.f, 1.f);
				manual_control_setpoint.aux6  = get_rc_value(_param_rc_map_aux6.get()     - 1, -1.f, 1.f);

				/* publish manual_control_setpoint topic */
				manual_control_setpoint.timestamp = hrt_absolute_time();
				_manual_control_setpoint_pub.publish(manual_control_setpoint);
				_last_manual_control_setpoint_publish = manual_control_setpoint.timestamp;


				/* copy from mapped manual_control_setpoint control to control group 3 */
				actuator_controls_s actuator_group_3{};
				actuator_group_3.timestamp_sample = rc_input.timestamp_last_signal;
				actuator_group_3.control[0] = manual_control_setpoint.y;
				actuator_group_3.control[1] = manual_control_setpoint.x;
				actuator_group_3.control[2] = manual_control_setpoint.r;
				actuator_group_3.control[3] = manual_control_setpoint.z;
				actuator_group_3.control[4] = manual_control_setpoint.flaps;
				actuator_group_3.control[5] = manual_control_setpoint.aux1;
				actuator_group_3.control[6] = manual_control_setpoint.aux2;
				actuator_group_3.control[7] = manual_control_setpoint.aux3;

				/* publish actuator_controls_3 topic */
				actuator_group_3.timestamp = hrt_absolute_time();
				_actuator_group_3_pub.publish(actuator_group_3);

				/* Update parameters from RC Channels (tuning with RC) if activated */
				if (hrt_elapsed_time(&_last_rc_to_param_map_time) > 1_s) {
					set_params_from_rc();
					_last_rc_to_param_map_time = hrt_absolute_time();
				}
			}
		}

		memcpy(_rc_values_previous, rc_input.values, sizeof(rc_input.values[0]) * rc_input.channel_count);
		static_assert(sizeof(_rc_values_previous) == sizeof(rc_input.values));
	}

	perf_end(_loop_perf);
}

float RCUpdate::calibratedChannelValue(uint8_t channel, uint16_t value) const
{
	const uint8_t i = channel;

	float output = 0.f;

	/*
	* 1) Constrain to min/max values, as later processing depends on bounds.
	*/
	const uint16_t v = math::constrain(value, _parameters.min[i], _parameters.max[i]);

	/*
	* 2) Scale around the mid point differently for lower and upper range.
	*
	* This is necessary as they don't share the same endpoints and slope.
	*
	* First normalize to 0..1 range with correct sign (below or above center),
	* the total range is 2 (-1..1).
	* If center (trim) == min, scale to 0..1, if center (trim) == max,
	* scale to -1..0.
	*
	* As the min and max bounds were enforced in step 1), division by zero
	* cannot occur, as for the case of center == min or center == max the if
	* statement is mutually exclusive with the arithmetic NaN case.
	*
	* DO NOT REMOVE OR ALTER STEP 1!
	*/
	if (v > (_parameters.trim[i] + _parameters.dz[i])) {
		output = (v - _parameters.trim[i] - _parameters.dz[i]) / (float)(
				 _parameters.max[i] - _parameters.trim[i] - _parameters.dz[i]);

	} else if (v < (_parameters.trim[i] - _parameters.dz[i])) {
		output = (v - _parameters.trim[i] + _parameters.dz[i]) / (float)(
				 _parameters.trim[i] - _parameters.min[i] - _parameters.dz[i]);
	}

	if (_parameters.rev[i]) {
		output = -output;
	}

	return output;
}

switch_pos_t RCUpdate::get_rc_sw3pos_position(int8_t channel, float on_th, float mid_th) const
{
	if (channel >= 0) {
		const uint16_t value = _param_rc_median_filter.get() ? _rc_filters[channel].median() : _rc_filters[channel].latest();

		if (value != 0) {
			const bool on_inv = (on_th < 0.f);
			const bool mid_inv = (mid_th < 0.f);

			float rc_value = calibratedChannelValue(channel, value);
			float scaled_value = 0.5f * rc_value + 0.5f;

			if (on_inv ? scaled_value < on_th : scaled_value > on_th) {
				return manual_control_switches_s::SWITCH_POS_ON;

			} else if (mid_inv ? scaled_value < mid_th : scaled_value > mid_th) {
				return manual_control_switches_s::SWITCH_POS_MIDDLE;

			} else {
				return manual_control_switches_s::SWITCH_POS_OFF;
			}
		}
	}

	return manual_control_switches_s::SWITCH_POS_NONE;
}

switch_pos_t RCUpdate::get_rc_sw2pos_position(int8_t channel, float on_th) const
{
	if (channel >= 0) {
		const uint16_t value = _param_rc_median_filter.get() ? _rc_filters[channel].median() : _rc_filters[channel].latest();

		if (value != 0) {
			const bool on_inv = (on_th < 0.f);

			float rc_value = calibratedChannelValue(channel, value);
			float scaled_value = 0.5f * rc_value + 0.5f;

			if (on_inv ? scaled_value < on_th : scaled_value > on_th) {
				return manual_control_switches_s::SWITCH_POS_ON;

			} else {
				return manual_control_switches_s::SWITCH_POS_OFF;
			}
		}
	}

	return manual_control_switches_s::SWITCH_POS_NONE;
}

void RCUpdate::UpdateSwitches()
{
	manual_control_switches_s switches{};

	/* set mode slot to unassigned */
	switches.mode_slot = manual_control_switches_s::MODE_SLOT_NONE;

	// check mode slot (RC_MAP_FLTMODE) or legacy mode switch (RC_MAP_MODE_SW), but not both
	if (_param_rc_map_fltmode.get() > 0) {
		// number of valid slots
		static constexpr int num_slots = manual_control_switches_s::MODE_SLOT_NUM;

		// the half width of the range of a slot is the total range
		// divided by the number of slots, again divided by two
		static constexpr float slot_width_half = 2.f / num_slots / 2.f;

		// min is -1, max is +1, range is 2. We offset below min and max
		static constexpr float slot_min = -1.f - 0.05f;
		static constexpr float slot_max =  1.f + 0.05f;

		// the slot gets mapped by first normalizing into a 0..1 interval using min
		// and max. Then the right slot is obtained by multiplying with the number of
		// slots. And finally we add half a slot width to ensure that integer rounding
		// will take us to the correct final index.

		const int channel = _param_rc_map_fltmode.get() - 1;
		const uint16_t value = _param_rc_median_filter.get() ? _rc_filters[channel].median() : _rc_filters[channel].latest();

		if ((channel >= 0) && (value != 0)) {
			float rc_value = calibratedChannelValue(channel, value);

			switches.mode_slot = (((((rc_value - slot_min) * num_slots) + slot_width_half) / (slot_max - slot_min)) +
					      (1.f / num_slots)) + 1;

			if (switches.mode_slot > num_slots) {
				switches.mode_slot = num_slots;
			}
		}

	} else if (_param_rc_map_mode_sw.get() > 0) {
		switches.mode_switch = get_rc_sw3pos_position(_param_rc_map_mode_sw.get() - 1, _param_rc_auto_th.get(),
				       _param_rc_assist_th.get());

		// only used with legacy mode switch
		switches.man_switch       = get_rc_sw2pos_position(_param_rc_map_man_sw.get()    - 1, _param_rc_man_th.get());
		switches.acro_switch      = get_rc_sw2pos_position(_param_rc_map_acro_sw.get()   - 1, _param_rc_acro_th.get());
		switches.rattitude_switch = get_rc_sw2pos_position(_param_rc_map_ratt_sw.get()   - 1, _param_rc_ratt_th.get());
		switches.stab_switch      = get_rc_sw2pos_position(_param_rc_map_stab_sw.get()   - 1, _param_rc_stab_th.get());
		switches.posctl_switch    = get_rc_sw2pos_position(_param_rc_map_posctl_sw.get() - 1, _param_rc_posctl_th.get());
	}

	switches.arm_switch        = get_rc_sw2pos_position(_param_rc_map_arm_sw.get()    - 1, _param_rc_armswitch_th.get());
	switches.gear_switch       = get_rc_sw2pos_position(_param_rc_map_gear_sw.get()   - 1, _param_rc_gear_th.get());
	switches.kill_switch       = get_rc_sw2pos_position(_param_rc_map_kill_sw.get()   - 1, _param_rc_killswitch_th.get());
	switches.loiter_switch     = get_rc_sw2pos_position(_param_rc_map_loiter_sw.get() - 1, _param_rc_loiter_th.get());
	switches.offboard_switch   = get_rc_sw2pos_position(_param_rc_map_offb_sw.get()   - 1, _param_rc_offb_th.get());
	switches.return_switch     = get_rc_sw2pos_position(_param_rc_map_return_sw.get() - 1, _param_rc_return_th.get());
	switches.transition_switch = get_rc_sw2pos_position(_param_rc_map_trans_sw.get()  - 1, _param_rc_trans_th.get());

	// last 2 switch updates identical (simple protection from bad RC data)
	if (switches == _manual_switches_previous) {
		const bool switches_changed = (switches != _manual_switches_last_publish);

		// publish immediately on change or at ~1 Hz
		if (switches_changed || (hrt_elapsed_time(&_manual_switches_last_publish.timestamp) >= 1_s)) {
			uint32_t switch_changes = _manual_switches_last_publish.switch_changes;

			if (switches_changed) {
				switch_changes++;
			}

			_manual_switches_last_publish = switches;
			_manual_switches_last_publish.switch_changes = switch_changes;
			_manual_switches_last_publish.timestamp = hrt_absolute_time();
			_manual_control_switches_pub.publish(_manual_switches_last_publish);
		}
	}

	_manual_switches_previous = switches;
}

int RCUpdate::task_spawn(int argc, char *argv[])
{
	RCUpdate *instance = new RCUpdate();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int RCUpdate::print_status()
{
	PX4_INFO_RAW("Running\n");

	if (_channel_count_max > 0) {
		PX4_INFO_RAW(" #  MIN  MAX TRIM  DZ REV\n");

		for (int i = 0; i < _channel_count_max; i++) {
			PX4_INFO_RAW("%2d %4d %4d %4d %3d %3d\n", i, _parameters.min[i], _parameters.max[i], _parameters.trim[i],
				     _parameters.dz[i], _parameters.rev[i]);
		}
	}

	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	perf_print_counter(_valid_data_interval_perf);

	return 0;
}

int RCUpdate::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int RCUpdate::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
The rc_update module handles RC channel mapping: read the raw input channels (`input_rc`),
then apply the calibration, map the RC channels to the configured channels & mode switches,
low-pass filter, and then publish as `rc_channels` and `manual_control_setpoint`.

### Implementation
To reduce control latency, the module is scheduled on input_rc publications.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("rc_update", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

} // namespace RCUpdate

extern "C" __EXPORT int rc_update_main(int argc, char *argv[])
{
	return RCUpdate::RCUpdate::main(argc, argv);
}
