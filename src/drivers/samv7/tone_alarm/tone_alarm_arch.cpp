/****************************************************************************
 *
 *   Copyright (C) 2013, 2018 PX4 Development Team. All rights reserved.
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

/*
 * Low Level Driver for the PX4 audio alarm port. Subscribes to
 * tune_control and plays notes on this architecture specific
 * timer HW
 */

#include <lib/drivers/tone_alarm/ToneAlarmArchInterface.hpp>

#include <px4_config.h>
#include <px4_log.h>
#include <systemlib/px4_macros.h>
#include <debug.h>

#include <drivers/device/device.h>
#include <drivers/drv_tone_alarm.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <ctype.h>

#include <board_config.h>
#include <drivers/drv_hrt.h>

#include <chip.h>
#include <up_internal.h>
#include <up_arch.h>

#include <chip/sam_pinmap.h>
#include <sam_gpio.h>
#include <sam_tc.h>

#include <systemlib/err.h>
#include <circuit_breaker/circuit_breaker.h>

#include <px4_workqueue.h>

#include <lib/tunes/tunes.h>
#include <uORB/uORB.h>
#include <uORB/topics/tune_control.h>


/* Check that tone alarm and HRT timers are different */
#if defined(TONE_ALARM_CHANNEL)  && defined(HRT_TIMER_CHANNEL)
# if TONE_ALARM_CHANNEL == HRT_TIMER_CHANNEL
#   error TONE_ALARM_CHANNEL and HRT_TIMER_CHANNEL must use different timers.
# endif
#endif

/* Tone alarm configuration */
/**********************************************************/
/**********************************************************/
/**********************************************************/
/**********************************************************/
/**********************************************************/
/**********************************************************/
/********************* TONE_ALARM_NOT_DONE ****************/
/******           This code is not finished 	***********/
/**********************************************************/
/**********************************************************/
/**********************************************************/
/**********************************************************/
/**********************************************************/
/**********************************************************/
#ifdef TONE_ALARM_CHANNEL

#if defined(TONE_ALARM_TIMER)
# error "TONE_ALARM_TIMER should not be defined, instead define TONE_ALARM_CHANNEL from 0-11"
#endif

#if TONE_ALARM_CHANNEL == 0 || TONE_ALARM_CHANNEL == 1 || TONE_ALARM_CHANNEL == 2
# define TONE_ALARM_TIMER 	0
#endif
#if TONE_ALARM_CHANNEL == 3 || TONE_ALARM_CHANNEL == 4 || TONE_ALARM_CHANNEL == 5
# define TONE_ALARM_TIMER 	1
#endif
#if TONE_ALARM_CHANNEL == 6 || TONE_ALARM_CHANNEL == 7 || TONE_ALARM_CHANNEL == 8
# define TONE_ALARM_TIMER 	2
#endif
#if TONE_ALARM_CHANNEL == 9 || TONE_ALARM_CHANNEL == 10 || TONE_ALARM_CHANNEL == 11
# define TONE_ALARM_TIMER 	3
#endif

/* HRT configuration */
#if   TONE_ALARM_TIMER == 0
# define HRT_TIMER_BASE			SAM_TC012_BASE
# if !defined(CONFIG_SAMV7_TC0)
#  error "HRT_TIMER_CHANNEL 0-2 Require CONFIG_SAMV7_TC0=y"
# endif
#elif TONE_ALARM_TIMER == 1
# define HRT_TIMER_BASE			SAM_TC345_BASE
# if !defined(CONFIG_SAMV7_TC1)
#  error "HRT_TIMER_CHANNEL 3-5 Require CONFIG_SAMV7_TC1=y"
# endif
#elif TONE_ALARM_TIMER == 2
# define HRT_TIMER_BASE			SAM_TC678_BASE
# if !defined(CONFIG_SAMV7_TC2)
#  error "HRT_TIMER_CHANNEL 6-8 Require CONFIG_SAMV7_TC2=y"
# endif
#elif TONE_ALARM_TIMER == 3
# define HRT_TIMER_BASE			SAM_TC901_BASE
# if !defined(CONFIG_SAMV7_TC3)
#  error "HRT_TIMER_CHANNEL 9-11 Require CONFIG_SAMV7_TC3=y"
# endif
#else
# error "HRT_TIMER_CHANNEL should be defined valid value are from 0-11"
# endif

#define TONE_ALARM_CLOCK (BOARD_MCK_FREQUENCY/128)


/*
 * Timer register accessors
 */
#define REG(_reg)	(*(volatile uint32_t *)(SAM_TC0_BASE + SAM_TC_CHAN_OFFSET(HRT_TIMER_CHANNEL) + _reg))

#define rCCR	REG(SAM_TC_CCR_OFFSET)
#define rCMR 	REG(SAM_TC_CMR_OFFSET)
#define rSMMR 	REG(SAM_TC_SMMR_OFFSET)
#define rRAB 	REG(SAM_TC_RAB_OFFSET)
#define rCV 	REG(SAM_TC_CV_OFFSET)
#define rRA 	REG(SAM_TC_RA_OFFSET)
#define rRB 	REG(SAM_TC_RB_OFFSET)
#define rRC 	REG(SAM_TC_RC_OFFSET)
#define rSR 	REG(SAM_TC_SR_OFFSET)
#define rIER 	REG(SAM_TC_IER_OFFSET)
#define rIDR 	REG(SAM_TC_IDR_OFFSET)
#define rIMR 	REG(SAM_TC_IMR_OFFSET)
#define rEMR 	REG(SAM_TC_EMR_OFFSET)

int ToneAlarmArchInterface::init()
{
	/* configure the GPIO to the idle state */
	px4_arch_configgpio(GPIO_TONE_ALARM_IDLE);

#if TONE_ALARM_NOT_DONE	/* initialise the timer */
	rCR1 = 0;
	rCR2 = 0;
	rSMCR = 0;
	rDIER = 0;
	rCCER &= TONE_CCER;		/* unlock CCMR* registers */
	rCCMR1 = TONE_CCMR1;
	rCCMR2 = TONE_CCMR2;
	rCCER = TONE_CCER;
	rDCR = 0;

#ifdef rBDTR // If using an advanced timer, you need to activate the output
	rBDTR = ATIM_BDTR_MOE; // enable the main output of the advanced timer
#endif

	/* toggle the CC output each time the count passes 1 */
	TONE_rCCR = 1;

	/* default the timer to a prescale value of 1; playing notes will change this */
	rPSC = 0;

	/* make sure the timer is running */
	rCR1 = GTIM_CR1_CEN;
#endif
}

static unsigned frequency_to_divisor(unsigned frequency)
{
	float period = 0.5f / frequency;

	// and the divisor, rounded to the nearest integer
	unsigned divisor = (period * TONE_ALARM_CLOCK) + 0.5f;

	return divisor;
}

void ToneAlarmArchInterface::start_note(unsigned frequency)
{
	// compute the divisor
	unsigned divisor = frequency_to_divisor(frequency);

	// pick the lowest prescaler value that we can use
	// (note that the effective prescale value is 1 greater)
	unsigned prescale = divisor / 65536;

	// calculate the timer period for the selected prescaler value
	unsigned period = (divisor / (prescale + 1)) - 1;
#if TONE_ALARM_NOT_DONE
	rPSC = prescale;	// load new prescaler
	rARR = period;		// load new toggle period
	rEGR = GTIM_EGR_UG;	// force a reload of the period
	rCCER |= TONE_CCER;	// enable the output
#else
	prescale++;
	period++;
#endif
	// configure the GPIO to enable timer output
	px4_arch_configgpio(GPIO_TONE_ALARM);
}

void ToneAlarmArchInterface::stop_note()
{
	/* stop the current note */
#if TONE_ALARM_NOT_DONE
	rCCER &= ~TONE_CCER;
#endif
	/*
	 * Make sure the GPIO is not driving the speaker.
	 */
	px4_arch_configgpio(GPIO_TONE_ALARM_IDLE);
}
