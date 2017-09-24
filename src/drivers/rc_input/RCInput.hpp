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

#ifndef RCINPUT_HPP_
#define RCINPUT_HPP_

#include <drivers/drv_gpio.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_rc_input.h>
#include <lib/rc/dsm.h>
#include <lib/rc/sbus.h>
#include <lib/rc/st24.h>
#include <lib/rc/sumd.h>
#include <px4_config.h>
#include <px4_getopt.h>
#include <px4_log.h>
#include <px4_module.h>
#include <px4_workqueue.h>
#include <systemlib/systemlib.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/adc_report.h>
#include <uORB/topics/input_rc.h>
#include <uORB/topics/vehicle_command.h>

#ifdef HRT_PPM_CHANNEL
# include <systemlib/ppm_decode.h>
#endif

class RCInput : public ModuleBase<RCInput>
{
public:

	RCInput() = default;
	virtual ~RCInput();

	// no copy, assignment, move, move assignment
	RCInput(const RCInput &) = delete;
	RCInput &operator=(const RCInput &) = delete;
	RCInput(RCInput &&) = delete;
	RCInput &operator=(RCInput &&) = delete;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static RCInput *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/**
	 * run the main loop: if running as task, continuously iterate, otherwise execute only one single cycle
	 */
	void cycle();

	/** @see ModuleBase::print_status() */
	int print_status() override;

private:

	/** Run main loop at this rate in Hz. */
	static constexpr uint32_t RC_INPUT_UPDATE_RATE_HZ = 200;

	enum RC_SCAN {
		RC_SCAN_PPM = 0,
		RC_SCAN_SBUS,
		RC_SCAN_DSM,
		RC_SCAN_SUMD,
		RC_SCAN_ST24
	} _rc_scan_state{RC_SCAN_SBUS};

	char const *RC_SCAN_STRING[5] = {
		"PPM",
		"SBUS",
		"DSM",
		"SUMD",
		"ST24"
	};

	void dsm_bind_start(int dsmMode);

	hrt_abstime _rc_scan_begin{0};
	bool _rc_scan_locked{false};

	static struct work_s	_work;

	// subscriptions
	int		_armed_sub{-1};
	int		_vehicle_cmd_sub{-1};

#ifdef ADC_RC_RSSI_CHANNEL
	int		_adc_sub {-1};
#endif /* ADC_RC_RSSI_CHANNEL */

	bool	_armed{false};

	// publications
	input_rc_s		_rc_in{};
	orb_advert_t	_to_input_rc{nullptr};

	float		_analog_rc_rssi_volt{-1.0f};
	bool		_analog_rc_rssi_stable{false};



	int		_rcs_fd{-1};

	static void	cycle_trampoline(void *arg);
	int 		start();

	void fill_rc_in(uint16_t raw_rc_count_local,
			uint16_t raw_rc_values_local[input_rc_s::RC_INPUT_MAX_CHANNELS],
			hrt_abstime now, bool frame_drop, bool failsafe,
			unsigned frame_drops, uint8_t input_source, int rssi);

	void set_rc_scan_state(RC_SCAN _rc_scan_state);
	void rc_io_invert();
	void rc_io_invert(bool invert);
};


#endif /* RCINPUT_HPP_ */
