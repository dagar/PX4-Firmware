/****************************************************************************
 *
 *   Copyright (c) 2012-2015, 2017 PX4 Development Team. All rights reserved.
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

#include "RCInput.hpp"

work_s	RCInput::_work = {};

RCInput::~RCInput()
{
	orb_unsubscribe(_armed_sub);
	orb_unsubscribe(_vehicle_cmd_sub);

#ifdef ADC_RC_RSSI_CHANNEL
	orb_unsubscribe(_adc_sub);
#endif /* ADC_RC_RSSI_CHANNEL */

	orb_unadvertise(_to_input_rc);

#ifdef RC_SERIAL_PORT
	dsm_deinit();
#endif
}

int
RCInput::task_spawn(int argc, char *argv[])
{
	RCInput *obj = new RCInput();

	int ret = obj->start();

	if (ret < 0) {
		delete obj;
		return ret;
	}

	wait_until_running(); // this will wait until _object is set from the cycle method
	_task_id = task_id_is_work_queue;

	return 0;
}

int RCInput::start()
{
	/* schedule a cycle to start things */
	return work_queue(HPWORK, &_work, (worker_t)&RCInput::cycle_trampoline, this, 0);
}

void
RCInput::cycle_trampoline(void *arg)
{
	RCInput *dev = reinterpret_cast<RCInput *>(arg);
	dev->cycle();
}

void
RCInput::fill_rc_in(uint16_t raw_rc_count_local,
		    uint16_t raw_rc_values_local[input_rc_s::RC_INPUT_MAX_CHANNELS],
		    hrt_abstime now, bool frame_drop, bool failsafe,
		    unsigned frame_drops, uint8_t input_source, int rssi = -1)
{
	_rc_in.channel_count = raw_rc_count_local;



	if (_rc_in.channel_count > input_rc_s::RC_INPUT_MAX_CHANNELS) {
		_rc_in.channel_count = input_rc_s::RC_INPUT_MAX_CHANNELS;
	}

	unsigned valid_chans = 0;

	for (unsigned i = 0; i < _rc_in.channel_count; i++) {
		_rc_in.values[i] = raw_rc_values_local[i];

		if (raw_rc_values_local[i] != UINT16_MAX) {
			valid_chans++;
		}
	}

	_rc_in.timestamp = now;
	_rc_in.timestamp_last_signal = _rc_in.timestamp;
	_rc_in.input_source = input_source;

	/* fake rssi if no value was provided */
	if (rssi == -1) {

		/* set RSSI if analog RSSI input is present */
		if (_analog_rc_rssi_stable) {
			float rssi_analog = ((_analog_rc_rssi_volt - 0.2f) / 3.0f) * 100.0f;

			if (rssi_analog > 100.0f) {
				rssi_analog = 100.0f;
			}

			if (rssi_analog < 0.0f) {
				rssi_analog = 0.0f;
			}

			_rc_in.rssi = rssi_analog;

		} else {
			_rc_in.rssi = 255;
		}

	} else {
		_rc_in.rssi = rssi;
	}

	if (valid_chans == 0) {
		_rc_in.rssi = 0;
	}

	_rc_in.rc_failsafe = failsafe;
	_rc_in.rc_lost = (valid_chans == 0);
	_rc_in.rc_lost_frame_count = frame_drops;
	_rc_in.rc_total_frame_count = 0;

	/* lazily advertise on first publication */
	if (_to_input_rc == nullptr) {
		int instance;
		_to_input_rc = orb_advertise_multi(ORB_ID(input_rc), &_rc_in, &instance, ORB_PRIO_DEFAULT);

	} else {
		orb_publish(ORB_ID(input_rc), _to_input_rc, &_rc_in);
	}

	_rc_scan_locked = true;
}

#ifdef RC_SERIAL_PORT

void RCInput::set_rc_scan_state(RC_SCAN newState)
{
	_rc_scan_begin = 0;
	_rc_scan_state = newState;
}

void RCInput::rc_io_invert(bool invert)
{
	INVERT_RC_INPUT(invert);
}

#endif /* RC_SERIAL_PORT */

void
RCInput::cycle()
{
	if (_object == nullptr) { // not initialized yet

		_armed_sub = orb_subscribe(ORB_ID(actuator_armed));
		_vehicle_cmd_sub = orb_subscribe(ORB_ID(vehicle_command));

#ifdef ADC_RC_RSSI_CHANNEL
		_adc_sub = orb_subscribe(ORB_ID(adc_report));
#endif /* ADC_RC_RSSI_CHANNEL */

#ifdef RC_SERIAL_PORT

#ifdef RF_RADIO_POWER_CONTROL
		// power radio on
		RF_RADIO_POWER_CONTROL(true);
#endif /* RF_RADIO_POWER_CONTROL */

		// dsm_init sets some file static variables and returns a file descriptor
		_rcs_fd = dsm_init(RC_SERIAL_PORT);

		// assume SBUS input
		sbus_config(_rcs_fd, false);

#ifdef GPIO_PPM_IN
		// disable CPPM input by mapping it away from the timer capture input
		px4_arch_unconfiggpio(GPIO_PPM_IN);
#endif /* GPIO_PPM_IN */

#endif /* RC_SERIAL_PORT */

		_object = this;
	}

	/* check arming state */
	bool updated = false;
	orb_check(_armed_sub, &updated);

	if (updated) {
		actuator_armed_s actuator_armed;
		orb_copy(ORB_ID(actuator_armed), _armed_sub, &actuator_armed);
		_armed = actuator_armed.armed;
	}

#ifdef RC_SERIAL_PORT
	/* vehicle command */
	orb_check(_vehicle_cmd_sub, &updated);

	if (updated) {
		struct vehicle_command_s cmd;
		orb_copy(ORB_ID(vehicle_command), _vehicle_cmd_sub, &cmd);

		// Check for a DSM pairing command
		if (((unsigned int)cmd.command == vehicle_command_s::VEHICLE_CMD_START_RX_PAIR) && ((int)cmd.param1 == 0)) {
			dsm_bind_start((int)cmd.param2);
		}
	}

#endif /* RC_SERIAL_PORT */

	/* update ADC sampling */
#ifdef ADC_RC_RSSI_CHANNEL
	orb_check(_adc_sub, &updated);

	if (updated) {

		struct adc_report_s adc;
		orb_copy(ORB_ID(adc_report), _adc_sub, &adc);
		const unsigned adc_chans = sizeof(adc.channel_id) / sizeof(adc.channel_id[0]);

		for (unsigned i = 0; i < adc_chans; i++) {
			if (adc.channel_id[i] == ADC_RC_RSSI_CHANNEL) {

				if (_analog_rc_rssi_volt < 0.0f) {
					_analog_rc_rssi_volt = adc.channel_value[i];
				}

				_analog_rc_rssi_volt = _analog_rc_rssi_volt * 0.995f + adc.channel_value[i] * 0.005f;

				/* only allow this to be used if we see a high RSSI once */
				if (_analog_rc_rssi_volt > 2.5f) {
					_analog_rc_rssi_stable = true;
				}
			}
		}
	}

#endif /* ADC_RC_RSSI_CHANNEL */

	bool rc_updated = false;

#ifdef RC_SERIAL_PORT

	hrt_abstime cycle_timestamp = hrt_absolute_time();

	// This block scans for a supported serial RC input and locks onto the first one found
	// Scan for 300 msec, then switch protocol
	constexpr hrt_abstime rc_scan_max = 300 * 1000;

	// read all available data from the serial RC input UART
	uint8_t rcs_buf[SBUS_BUFFER_SIZE];
	ssize_t newBytes = ::read(_rcs_fd, &rcs_buf[0], SBUS_BUFFER_SIZE);

	switch (_rc_scan_state) {
	case RC_SCAN_SBUS:
		if (_rc_scan_begin == 0) {
			_rc_scan_begin = cycle_timestamp;

			// Configure serial port for SBUS
			sbus_config(_rcs_fd, false);
			rc_io_invert(true);

		} else if (_rc_scan_locked || (cycle_timestamp - _rc_scan_begin < rc_scan_max)) {

			// parse new data
			if (newBytes > 0) {
				uint16_t raw_rc_values[input_rc_s::RC_INPUT_MAX_CHANNELS] = {};
				uint16_t raw_rc_count = 0;
				bool sbus_failsafe = false;
				bool sbus_frame_drop = false;
				unsigned frame_drops = 0;

				rc_updated = sbus_parse(cycle_timestamp, &rcs_buf[0], newBytes, &raw_rc_values[0], &raw_rc_count, &sbus_failsafe,
							&sbus_frame_drop, &frame_drops, input_rc_s::RC_INPUT_MAX_CHANNELS);

				if (rc_updated) {
					// we have a new SBUS frame. Publish it.
					fill_rc_in(raw_rc_count, raw_rc_values, cycle_timestamp, sbus_frame_drop, sbus_failsafe, frame_drops,
						   input_rc_s::RC_INPUT_SOURCE_PX4FMU_SBUS);
				}
			}

		} else {
			// Scan the next protocol
			set_rc_scan_state(RC_SCAN_DSM);
		}

		break;

	case RC_SCAN_DSM:
		if (_rc_scan_begin == 0) {
			_rc_scan_begin = cycle_timestamp;

			// Configure serial port for DSM
			dsm_config(_rcs_fd);
			rc_io_invert(false);

		} else if (_rc_scan_locked || (cycle_timestamp - _rc_scan_begin < rc_scan_max)) {

			if (newBytes > 0) {
				uint16_t raw_rc_values[input_rc_s::RC_INPUT_MAX_CHANNELS] = {};
				uint16_t raw_rc_count = 0;
				bool dsm_11_bit = false;
				unsigned frame_drops = 0;

				// parse new data
				rc_updated = dsm_parse(cycle_timestamp, &rcs_buf[0], newBytes, &raw_rc_values[0], &raw_rc_count, &dsm_11_bit,
						       &frame_drops, input_rc_s::RC_INPUT_MAX_CHANNELS);

				if (rc_updated) {
					// we have a new DSM frame. Publish it.
					fill_rc_in(raw_rc_count, raw_rc_values, cycle_timestamp, false, false, frame_drops,
						   input_rc_s::RC_INPUT_SOURCE_PX4FMU_DSM);
				}
			}

		} else {
			// Scan the next protocol
			set_rc_scan_state(RC_SCAN_ST24);
		}

		break;

	case RC_SCAN_ST24:
		if (_rc_scan_begin == 0) {
			_rc_scan_begin = cycle_timestamp;

			// Configure serial port for DSM
			dsm_config(_rcs_fd);
			rc_io_invert(false);

		} else if (_rc_scan_locked || (cycle_timestamp - _rc_scan_begin < rc_scan_max)) {

			if (newBytes > 0) {
				// parse new data
				uint8_t st24_rssi = 0;
				uint8_t lost_count = 0;
				uint16_t raw_rc_count = 0;
				uint16_t raw_rc_values[input_rc_s::RC_INPUT_MAX_CHANNELS] = {};

				for (ssize_t i = 0; i < newBytes; i++) {
					/* set updated flag if one complete packet was parsed */
					st24_rssi = RC_INPUT_RSSI_MAX;

					int ret = st24_decode(rcs_buf[i], &st24_rssi, &lost_count, &raw_rc_count, raw_rc_values,
							      input_rc_s::RC_INPUT_MAX_CHANNELS);

					if (ret == PX4_OK) {
						rc_updated = true;
					}
				}

				// The st24 will keep outputting RC channels and RSSI even if RC has been lost.
				// The only way to detect RC loss is therefore to look at the lost_count.
				if (rc_updated && lost_count == 0) {
					// we have a new ST24 frame. Publish it.
					fill_rc_in(raw_rc_count, raw_rc_values, cycle_timestamp, false, false, 0, input_rc_s::RC_INPUT_SOURCE_PX4FMU_ST24,
						   st24_rssi);
				}
			}

		} else {
			// Scan the next protocol
			set_rc_scan_state(RC_SCAN_SUMD);
		}

		break;

	case RC_SCAN_SUMD:
		if (_rc_scan_begin == 0) {
			_rc_scan_begin = cycle_timestamp;

			// Configure serial port for DSM
			dsm_config(_rcs_fd);
			rc_io_invert(false);

		} else if (_rc_scan_locked || (cycle_timestamp - _rc_scan_begin < rc_scan_max)) {

			if (newBytes > 0) {
				// parse new data
				uint8_t sumd_rssi = 0;
				uint8_t rx_count = 0;
				uint16_t raw_rc_count = 0;
				uint16_t raw_rc_values[input_rc_s::RC_INPUT_MAX_CHANNELS] = {};
				bool sumd_failsafe = false;

				for (ssize_t i = 0; i < newBytes; i++) {
					/* set updated flag if one complete packet was parsed */
					sumd_rssi = RC_INPUT_RSSI_MAX;
					int ret = sumd_decode(rcs_buf[i], &sumd_rssi, &rx_count, &raw_rc_count, raw_rc_values,
							      input_rc_s::RC_INPUT_MAX_CHANNELS, &sumd_failsafe);

					if (ret == PX4_OK) {
						rc_updated = true;
					}
				}

				if (rc_updated) {
					// we have a new SUMD frame. Publish it.
					fill_rc_in(raw_rc_count, raw_rc_values, cycle_timestamp, false, sumd_failsafe, 0,
						   input_rc_s::RC_INPUT_SOURCE_PX4FMU_SUMD, sumd_rssi);
				}
			}

		} else {
			// Scan the next protocol
			set_rc_scan_state(RC_SCAN_PPM);
		}

		break;

	case RC_SCAN_PPM:

		// skip PPM if it's not supported
#ifdef HRT_PPM_CHANNEL
		if (_rc_scan_begin == 0) {

			_rc_scan_begin = cycle_timestamp;

			// Configure timer input pin for CPPM
			px4_arch_configgpio(GPIO_PPM_IN);
			rc_io_invert(false);

		} else if (_rc_scan_locked || (cycle_timestamp - _rc_scan_begin < rc_scan_max)) {

			// see if we have new PPM input data
			if ((ppm_last_valid_decode != _rc_in.timestamp_last_signal) && (ppm_decoded_channels > 3)) {

				rc_updated = true;
				_rc_in.rc_ppm_frame_length = ppm_frame_length;
				_rc_in.timestamp_last_signal = ppm_last_valid_decode;

				fill_rc_in(ppm_decoded_channels, ppm_buffer, cycle_timestamp, false, false, 0, input_rc_s::RC_INPUT_SOURCE_PX4FMU_PPM);
			}

		} else {
			// disable CPPM input by mapping it away from the timer capture input
			px4_arch_unconfiggpio(GPIO_PPM_IN);

			// Scan the next protocol
			set_rc_scan_state(RC_SCAN_SBUS);
		}

#else // skip PPM if it's not supported
		set_rc_scan_state(RC_SCAN_SBUS);
#endif // HRT_PPM_CHANNEL

		break;
	}

#else  // RC_SERIAL_PORT not defined

#ifdef HRT_PPM_CHANNEL

	// see if we have new PPM input data
	if ((ppm_last_valid_decode != _rc_in.timestamp_last_signal) && (ppm_decoded_channels > 3)) {
		// we have a new PPM frame. Publish it.
		rc_updated = true;
		fill_rc_in(ppm_decoded_channels, ppm_buffer, hrt_absolute_time(), false, false, 0,
			   input_rc_s::RC_INPUT_SOURCE_PX4FMU_PPM);
	}

#endif  // HRT_PPM_CHANNEL

#endif  // RC_SERIAL_PORT

	if (!rc_updated && ((hrt_elapsed_time(&_rc_in.timestamp_last_signal)) > 1000000)) {
		_rc_scan_locked = false;
	}

	if (!should_exit()) {

		// Schedule next cycle.
		work_queue(HPWORK, &_work, (worker_t)&RCInput::cycle_trampoline, this,
			   USEC2TICK(1000000 / RC_INPUT_UPDATE_RATE_HZ));

	} else {
		exit_and_cleanup();
	}
}

void
RCInput::dsm_bind_start(int dsmMode)
{
	if (!_armed) {
		PX4_INFO("DSM_BIND_START DSM%s RX", (dsmMode == 0) ? "2" : ((dsmMode == 1) ? "-X" : "-X8"));

		/* specify 11ms DSMX. RX will automatically fall back to 22ms or DSM2 if necessary */
		/* only allow DSM2, DSM-X and DSM-X with more than 7 channels */
		if (dsmMode == DSM2_BIND_PULSES ||
		    dsmMode == DSMX_BIND_PULSES ||
		    dsmMode == DSMX8_BIND_PULSES) {

			dsm_bind(DSM_CMD_BIND_POWER_DOWN, 0);

			dsm_bind(DSM_CMD_BIND_SET_RX_OUT, 0);
			usleep(500000);

			dsm_bind(DSM_CMD_BIND_POWER_UP, 0);
			usleep(72000);

			irqstate_t flags = px4_enter_critical_section();
			dsm_bind(DSM_CMD_BIND_SEND_PULSES, dsmMode);
			px4_leave_critical_section(flags);
			usleep(50000);

			dsm_bind(DSM_CMD_BIND_REINIT_UART, 0);

		} else {
			PX4_ERR("DSM bind invalid mode requested");
		}

	} else {
		PX4_WARN("system armed, bind request rejected");
	}
}

int RCInput::custom_command(int argc, char *argv[])
{
	/* start the FMU if not running */
	if (!is_running()) {
		int ret = RCInput::task_spawn(argc, argv);

		if (ret) {
			return ret;
		}
	}

	return print_usage("unknown command");
}

int RCInput::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This module is responsible for rc input on boards a separate IO chip (eg. Pixracer).

It does the RC input parsing and auto-selecting the method. Supported methods are:
- PPM
- SBUS
- DSM
- SUMD
- ST24

)DESCR_STR");

	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int RCInput::print_status()
{
	PX4_INFO("Max update rate: %i Hz", RC_INPUT_UPDATE_RATE_HZ);

	PX4_INFO("RC scan state: %s", RC_SCAN_STRING[_rc_scan_state]);

	PX4_INFO("RC last signal received: %.4f s", (double)hrt_elapsed_time(&_rc_in.timestamp_last_signal) / 1e6);
	PX4_INFO("RC channel count: %d", _rc_in.channel_count);
	PX4_INFO("RC lost: %d", _rc_in.rc_lost);
	PX4_INFO("RC failsafe: %d", _rc_in.rc_failsafe);

	return 0;
}

extern "C" __EXPORT int rc_input_main(int argc, char *argv[]);

int
rc_input_main(int argc, char *argv[])
{
	return RCInput::main(argc, argv);
}
