/****************************************************************************
 *
 *   Copyright (C) 2013, 2016, 2018 PX4 Development Team. All rights reserved.
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
#include <drivers/device/device.h>
#include <drivers/drv_tone_alarm.h>
#include <drivers/drv_hrt.h>

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

/* Check that tone alarm and HRT timers are different */
#if defined(TONE_ALARM_TIMER) && defined(HRT_TIMER)
# if TONE_ALARM_TIMER == HRT_TIMER
#   error TONE_ALARM_TIMER and HRT_TIMER must use different timers.
# endif
#endif

/* Tone alarm configuration */
#if   TONE_ALARM_TIMER == 1
# define TONE_ALARM_BASE              STM32_TIM1_BASE
# define TONE_ALARM_CLOCK             STM32_APB2_TIM1_CLKIN
# define TONE_ALARM_CLOCK_POWER_REG   STM32_RCC_APB2ENR
# define TONE_ALARM_CLOCK_ENABLE      RCC_APB2ENR_TIM1EN
# ifdef CONFIG_STM32_TIM1
#  error Must not set CONFIG_STM32_TIM1 when TONE_ALARM_TIMER is 1
# endif
#elif TONE_ALARM_TIMER == 2
# define TONE_ALARM_BASE		STM32_TIM2_BASE
# define TONE_ALARM_CLOCK		STM32_APB1_TIM2_CLKIN
# define TONE_ALARM_CLOCK_POWER_REG   STM32_RCC_APB1ENR
# define TONE_ALARM_CLOCK_ENABLE	RCC_APB1ENR_TIM2EN
# ifdef CONFIG_STM32_TIM2
#  error Must not set CONFIG_STM32_TIM2 when TONE_ALARM_TIMER is 2
# endif
#elif TONE_ALARM_TIMER == 3
# define TONE_ALARM_BASE		STM32_TIM3_BASE
# define TONE_ALARM_CLOCK		STM32_APB1_TIM3_CLKIN
# define TONE_ALARM_CLOCK_POWER_REG   STM32_RCC_APB1ENR
# define TONE_ALARM_CLOCK_ENABLE	RCC_APB1ENR_TIM3EN
# ifdef CONFIG_STM32_TIM3
#  error Must not set CONFIG_STM32_TIM3 when TONE_ALARM_TIMER is 3
# endif
#elif TONE_ALARM_TIMER == 4
# define TONE_ALARM_BASE		STM32_TIM4_BASE
# define TONE_ALARM_CLOCK		STM32_APB1_TIM4_CLKIN
# define TONE_ALARM_CLOCK_POWER_REG   STM32_RCC_APB1ENR
# define TONE_ALARM_CLOCK_ENABLE	RCC_APB1ENR_TIM4EN
# ifdef CONFIG_STM32_TIM4
#  error Must not set CONFIG_STM32_TIM4 when TONE_ALARM_TIMER is 4
# endif
#elif TONE_ALARM_TIMER == 5
# define TONE_ALARM_BASE		STM32_TIM5_BASE
# define TONE_ALARM_CLOCK		STM32_APB1_TIM5_CLKIN
# define TONE_ALARM_CLOCK_POWER_REG   STM32_RCC_APB1ENR
# define TONE_ALARM_CLOCK_ENABLE	RCC_APB1ENR_TIM5EN
# ifdef CONFIG_STM32_TIM5
#  error Must not set CONFIG_STM32_TIM5 when TONE_ALARM_TIMER is 5
# endif
#elif TONE_ALARM_TIMER == 8
# define TONE_ALARM_BASE		STM32_TIM8_BASE
# define TONE_ALARM_CLOCK		STM32_APB2_TIM8_CLKIN
# define TONE_ALARM_CLOCK_POWER_REG   STM32_RCC_APB2ENR
# define TONE_ALARM_CLOCK_ENABLE	RCC_APB2ENR_TIM8EN
# ifdef CONFIG_STM32_TIM8
#  error Must not set CONFIG_STM32_TIM8 when TONE_ALARM_TIMER is 8
# endif
#elif TONE_ALARM_TIMER == 9
# define TONE_ALARM_BASE		STM32_TIM9_BASE
# define TONE_ALARM_CLOCK		STM32_APB2_TIM9_CLKIN
# define TONE_ALARM_CLOCK_POWER_REG   STM32_RCC_APB2ENR
# define TONE_ALARM_CLOCK_ENABLE	RCC_APB2ENR_TIM9EN
# ifdef CONFIG_STM32_TIM9
#  error Must not set CONFIG_STM32_TIM9 when TONE_ALARM_TIMER is 9
# endif
#elif TONE_ALARM_TIMER == 10
# define TONE_ALARM_BASE		STM32_TIM10_BASE
# define TONE_ALARM_CLOCK		STM32_APB2_TIM10_CLKIN
# define TONE_ALARM_CLOCK_POWER_REG   STM32_RCC_APB2ENR
# define TONE_ALARM_CLOCK_ENABLE	RCC_APB2ENR_TIM10EN
# ifdef CONFIG_STM32_TIM10
#  error Must not set CONFIG_STM32_TIM10 when TONE_ALARM_TIMER is 10
# endif
#elif TONE_ALARM_TIMER == 11
# define TONE_ALARM_BASE		STM32_TIM11_BASE
# define TONE_ALARM_CLOCK		STM32_APB2_TIM11_CLKIN
# define TONE_ALARM_CLOCK_POWER_REG   STM32_RCC_APB2ENR
# define TONE_ALARM_CLOCK_ENABLE	RCC_APB2ENR_TIM11EN
# ifdef CONFIG_STM32_TIM11
#  error Must not set CONFIG_STM32_TIM11 when TONE_ALARM_TIMER is 11
# endif
#elif TONE_ALARM_TIMER == 12
# define TONE_ALARM_BASE		STM32_TIM12_BASE
# define TONE_ALARM_CLOCK		STM32_APB1_TIM12_CLKIN
# define TONE_ALARM_CLOCK_POWER_REG   STM32_RCC_APB1ENR
# define TONE_ALARM_CLOCK_ENABLE	RCC_APB1ENR_TIM12EN
# ifdef CONFIG_STM32_TIM12
#  error Must not set CONFIG_STM32_TIM12 when TONE_ALARM_TIMER is 12
# endif
#elif TONE_ALARM_TIMER == 13
# define TONE_ALARM_BASE		STM32_TIM13_BASE
# define TONE_ALARM_CLOCK		STM32_APB1_TIM12_CLKIN
# define TONE_ALARM_CLOCK_POWER_REG   STM32_RCC_APB1ENR
# define TONE_ALARM_CLOCK_ENABLE	RCC_APB1ENR_TIM13EN
# ifdef CONFIG_STM32_TIM13
#  error Must not set CONFIG_STM32_TIM13 when TONE_ALARM_TIMER is 13
# endif
#elif TONE_ALARM_TIMER == 14
# define TONE_ALARM_BASE		STM32_TIM14_BASE
# define TONE_ALARM_CLOCK		STM32_APB1_TIM12_CLKIN
# define TONE_ALARM_CLOCK_POWER_REG   STM32_RCC_APB1ENR
# define TONE_ALARM_CLOCK_ENABLE	RCC_APB1ENR_TIM14EN
# ifdef CONFIG_STM32_TIM14
#  error Must not set CONFIG_STM32_TIM14 when TONE_ALARM_TIMER is 14
# endif
#else
# error Must set TONE_ALARM_TIMER to one of the timers between 1 and 14 (inclusive) to use this driver.
#endif

#if TONE_ALARM_CHANNEL == 1
# define TONE_CCMR1	(3 << 4)
# define TONE_CCMR2	0
# define TONE_CCER	(1 << 0)
# define TONE_rCCR	rCCR1
#elif TONE_ALARM_CHANNEL == 2
# define TONE_CCMR1	(3 << 12)
# define TONE_CCMR2	0
# define TONE_CCER	(1 << 4)
# define TONE_rCCR	rCCR2
#elif TONE_ALARM_CHANNEL == 3
# define TONE_CCMR1	0
# define TONE_CCMR2	(3 << 4)
# define TONE_CCER	(1 << 8)
# define TONE_rCCR	rCCR3
#elif TONE_ALARM_CHANNEL == 4
# define TONE_CCMR1	0
# define TONE_CCMR2	(3 << 12)
# define TONE_CCER	(1 << 12)
# define TONE_rCCR	rCCR4
#else
# error Must set TONE_ALARM_CHANNEL to a value between 1 and 4 to use this driver.
#endif


/*
 * Timer register accessors
 */
#define REG(_reg)	(*(volatile uint32_t *)(TONE_ALARM_BASE + _reg))

#if TONE_ALARM_TIMER == 1 || TONE_ALARM_TIMER == 8 // Note: If using TIM1 or TIM8, then you are using the ADVANCED timers and NOT the GENERAL TIMERS, therefore different registers
# define rCR1         REG(STM32_ATIM_CR1_OFFSET)
# define rCR2         REG(STM32_ATIM_CR2_OFFSET)
# define rSMCR        REG(STM32_ATIM_SMCR_OFFSET)
# define rDIER        REG(STM32_ATIM_DIER_OFFSET)
# define rSR          REG(STM32_ATIM_SR_OFFSET)
# define rEGR         REG(STM32_ATIM_EGR_OFFSET)
# define rCCMR1       REG(STM32_ATIM_CCMR1_OFFSET)
# define rCCMR2       REG(STM32_ATIM_CCMR2_OFFSET)
# define rCCER        REG(STM32_ATIM_CCER_OFFSET)
# define rCNT         REG(STM32_ATIM_CNT_OFFSET)
# define rPSC         REG(STM32_ATIM_PSC_OFFSET)
# define rARR         REG(STM32_ATIM_ARR_OFFSET)
# define rRCR         REG(STM32_ATIM_RCR_OFFSET)
# define rCCR1        REG(STM32_ATIM_CCR1_OFFSET)
# define rCCR2        REG(STM32_ATIM_CCR2_OFFSET)
# define rCCR3        REG(STM32_ATIM_CCR3_OFFSET)
# define rCCR4        REG(STM32_ATIM_CCR4_OFFSET)
# define rBDTR        REG(STM32_ATIM_BDTR_OFFSET)
# define rDCR         REG(STM32_ATIM_DCR_OFFSET)
# define rDMAR        REG(STM32_ATIM_DMAR_OFFSET)
#else
# define rCR1     	REG(STM32_GTIM_CR1_OFFSET)
# define rCR2     	REG(STM32_GTIM_CR2_OFFSET)
# define rSMCR    	REG(STM32_GTIM_SMCR_OFFSET)
# define rDIER    	REG(STM32_GTIM_DIER_OFFSET)
# define rSR      	REG(STM32_GTIM_SR_OFFSET)
# define rEGR     	REG(STM32_GTIM_EGR_OFFSET)
# define rCCMR1   	REG(STM32_GTIM_CCMR1_OFFSET)
# define rCCMR2   	REG(STM32_GTIM_CCMR2_OFFSET)
# define rCCER    	REG(STM32_GTIM_CCER_OFFSET)
# define rCNT     	REG(STM32_GTIM_CNT_OFFSET)
# define rPSC     	REG(STM32_GTIM_PSC_OFFSET)
# define rARR     	REG(STM32_GTIM_ARR_OFFSET)
# define rCCR1    	REG(STM32_GTIM_CCR1_OFFSET)
# define rCCR2    	REG(STM32_GTIM_CCR2_OFFSET)
# define rCCR3    	REG(STM32_GTIM_CCR3_OFFSET)
# define rCCR4    	REG(STM32_GTIM_CCR4_OFFSET)
# define rDCR     	REG(STM32_GTIM_DCR_OFFSET)
# define rDMAR    	REG(STM32_GTIM_DMAR_OFFSET)
#endif

int ToneAlarmArchInterface::init()
{
	/* configure the GPIO to the idle state */
	px4_arch_configgpio(GPIO_TONE_ALARM_IDLE);

#ifdef GPIO_TONE_ALARM_NEG
	px4_arch_configgpio(GPIO_TONE_ALARM_NEG);
#endif

	/* clock/power on our timer */
	modifyreg32(TONE_ALARM_CLOCK_POWER_REG, 0, TONE_ALARM_CLOCK_ENABLE);

	/* initialise the timer */
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

	return PX4_OK;
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

	rPSC = prescale;	// load new prescaler
	rARR = period;		// load new toggle period
	rEGR = GTIM_EGR_UG;	// force a reload of the period
	rCCER |= TONE_CCER;	// enable the output

	// configure the GPIO to enable timer output
	px4_arch_configgpio(GPIO_TONE_ALARM);
}

void ToneAlarmArchInterface::stop_note()
{
	/* stop the current note */
	rCCER &= ~TONE_CCER;

	/*
	 * Make sure the GPIO is not driving the speaker.
	 */
	px4_arch_configgpio(GPIO_TONE_ALARM_IDLE);
}
