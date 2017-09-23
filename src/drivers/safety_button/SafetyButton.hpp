/****************************************************************************
 *
 *   Copyright (c) 2012-2017, 2017 PX4 Development Team. All rights reserved.
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

#ifndef SAFETYBUTTON_HPP_
#define SAFETYBUTTON_HPP_

#include <cfloat>

#include <board_config.h>
#include <drivers/drv_hrt.h>
#include <px4_module.h>
#include <px4_workqueue.h>
#include <systemlib/circuit_breaker.h>
#include <systemlib/systemlib.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/safety.h>

#define CYCLE_COUNT 10			/* safety switch must be held for 1 second to activate */

/*
 * Define the various LED flash sequences for each system state.
 */
#define LED_PATTERN_FMU_OK_TO_ARM 		0x0003		/**< slow blinking			*/
#define LED_PATTERN_FMU_REFUSE_TO_ARM 	0x5555		/**< fast blinking			*/
#define LED_PATTERN_IO_ARMED 			0x5050		/**< long off, then double blink 	*/
#define LED_PATTERN_FMU_ARMED 			0x5500		/**< long off, then quad blink 		*/
#define LED_PATTERN_IO_FMU_ARMED 		0xffff		/**< constantly on			*/

class SafetyButton : public ModuleBase<SafetyButton>
{
public:

	SafetyButton() = default;
	virtual ~SafetyButton();

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static SafetyButton *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/**
	 * run the main loop: if running as task, continuously iterate, otherwise execute only one single cycle
	 */
	void cycle();

private:

	/** Run main loop at this rate in Hz. */
	static constexpr uint32_t SAFETY_BUTTON_UPDATE_RATE_HZ = 10;

	static struct work_s	_work;

	int		_armed_sub{-1};
	bool	_armed{false};

	bool		_safety_off{true};
	bool		_safety_disabled{false};

	orb_advert_t		_to_safety{nullptr};

	static void	cycle_trampoline(void *arg);
	int 		start();

	void safety_check_button();
	void flash_safety_button();
};


#endif /* SAFETYBUTTON_HPP_ */
