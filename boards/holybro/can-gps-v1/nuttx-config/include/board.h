/************************************************************************************
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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
 * 3. Neither the name NuttX nor the names of its contributors may be
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
 ************************************************************************************/

#ifndef __ARCH_BOARD_BOARD_H
#define __ARCH_BOARD_BOARD_H

#include "board_dma_map.h"

#include <nuttx/config.h>

/* HSI - 16 MHz RC factory-trimmed
 * LSI - 32 KHz RC
 * HSE - 8  MHz Crystal
 * LSE - not installed
 */

#define STM32_BOARD_USEHSE      1
#define STM32_BOARD_XTAL        16000000
#define STM32_HSE_FREQUENCY     STM32_BOARD_XTAL

#define STM32_HSI_FREQUENCY     16000000ul
#define STM32_LSI_FREQUENCY     32000

/* Main PLL Configuration.
 *
 * PLL source is HSI = 16MHz
 * PLLN = 85, PLLM = 4, PLLP = 10, PLLQ = 2, PLLR = 2
 *
 * f(VCO Clock) = f(PLL Clock Input) x (PLLN / PLLM)
 * f(PLL_P) = f(VCO Clock) / PLLP
 * f(PLL_Q) = f(VCO Clock) / PLLQ
 * f(PLL_R) = f(VCO Clock) / PLLR
 *
 * Where:
 * 8 <= PLLN <= 127
 * 1 <= PLLM <= 16
 * PLLP = 2 through 31
 * PLLQ = 2, 4, 6, or 8
 * PLLR = 2, 4, 6, or 8
 *
 * Do not exceed 170MHz on f(PLL_P), f(PLL_Q), or f(PLL_R).
 * 64MHz <= f(VCO Clock) <= 344MHz.
 *
 * Given the above:
 *
 * f(VCO Clock) = HSI   x PLLN / PLLM
 *              = 16MHz x 85   / 4
 *              = 340MHz
 *
 * PLLPCLK      = f(VCO Clock) / PLLP
 *              = 340MHz       / 10
 *              = 34MHz
 *                (May be used for ADC)
 *
 * PLLQCLK      = f(VCO Clock) / PLLQ
 *              = 340MHz       / 2
 *              = 170MHz
 *                (May be used for QUADSPI, FDCAN, SAI1, I2S3. If set to
 *                48MHz, may be used for USB, RNG.)
 *
 * PLLRCLK      = f(VCO Clock) / PLLR
 *              = 340MHz       / 2
 *              = 170MHz
 *                (May be used for SYSCLK and most peripherals.)
 */

#define STM32_PLLCFGR_PLLSRC     RCC_PLLCFGR_PLLSRC_HSI
#define STM32_PLLCFGR_PLLCFG       (RCC_PLLCFGR_PLLPEN|RCC_PLLCFGR_PLLQEN|RCC_PLLCFGR_PLLREN)

#define STM32_PLLCFGR_PLLN             RCC_PLLCFGR_PLLN(85)
#define STM32_PLLCFGR_PLLM             RCC_PLLCFGR_PLLM(4)
#define STM32_PLLCFGR_PLLP             RCC_PLLCFGR_PLLPDIV(10)
#define STM32_PLLCFGR_PLLQ             RCC_PLLCFGR_PLLQ_2
#define STM32_PLLCFGR_PLLR             RCC_PLLCFGR_PLLR_2

#define STM32_VCO_FREQUENCY            ((STM32_HSI_FREQUENCY / 4) * 85)
#define STM32_PLLP_FREQUENCY           (STM32_VCO_FREQUENCY / 10)
#define STM32_PLLQ_FREQUENCY           (STM32_VCO_FREQUENCY / 2)
#define STM32_PLLR_FREQUENCY           (STM32_VCO_FREQUENCY / 2)

/* Use the PLL and set the SYSCLK source to be PLLR (170MHz) */
#define STM32_SYSCLK_SW                RCC_CFGR_SW_PLL
#define STM32_SYSCLK_SWS               RCC_CFGR_SWS_PLL
#define STM32_SYSCLK_FREQUENCY         STM32_PLLR_FREQUENCY

/* AHB clock (HCLK) is SYSCLK (170MHz) */
#define STM32_RCC_CFGR_HPRE            RCC_CFGR_HPRE_SYSCLK
#define STM32_HCLK_FREQUENCY           STM32_SYSCLK_FREQUENCY

/* APB1 clock (PCLK1) is HCLK (170MHz) */
#define STM32_RCC_CFGR_PPRE1           RCC_CFGR_PPRE1_HCLK
#define STM32_PCLK1_FREQUENCY          STM32_HCLK_FREQUENCY

/* APB2 clock (PCLK2) is HCLK (170MHz) */
#define STM32_RCC_CFGR_PPRE2           RCC_CFGR_PPRE2_HCLK
#define STM32_PCLK2_FREQUENCY          STM32_HCLK_FREQUENCY

/* APB2 timers 1, 8, 20 and 15-17 will receive PCLK2. */
/* Timers driven from APB2 will be PCLK2 */
#define STM32_APB2_TIM1_CLKIN   (STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM8_CLKIN   (STM32_PCLK2_FREQUENCY)
#define STM32_APB1_TIM15_CLKIN  (STM32_PCLK2_FREQUENCY)
#define STM32_APB1_TIM16_CLKIN  (STM32_PCLK2_FREQUENCY)
#define STM32_APB1_TIM17_CLKIN  (STM32_PCLK2_FREQUENCY)

/* APB1 timers 2-7 will be twice PCLK1 */

#define STM32_APB1_TIM2_CLKIN   (STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM3_CLKIN   (STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM4_CLKIN   (STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM6_CLKIN   (STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM7_CLKIN   (STM32_PCLK1_FREQUENCY)

/* USB divider -- Divide PLL clock by 1.5 */
#define STM32_CFGR_USBPRE       0

/* Timer Frequencies, if APBx is set to 1, frequency is same to APBx
 * otherwise frequency is 2xAPBx.
 */
#define BOARD_TIM1_FREQUENCY   (STM32_PCLK2_FREQUENCY)
#define BOARD_TIM2_FREQUENCY   (STM32_PCLK1_FREQUENCY)
#define BOARD_TIM3_FREQUENCY   (STM32_PCLK1_FREQUENCY)
#define BOARD_TIM4_FREQUENCY   (STM32_PCLK1_FREQUENCY)
#define BOARD_TIM5_FREQUENCY   (STM32_PCLK1_FREQUENCY)
#define BOARD_TIM6_FREQUENCY   (STM32_PCLK1_FREQUENCY)
#define BOARD_TIM7_FREQUENCY   (STM32_PCLK1_FREQUENCY)
#define BOARD_TIM8_FREQUENCY   (STM32_PCLK2_FREQUENCY)
#define BOARD_TIM15_FREQUENCY  (STM32_PCLK2_FREQUENCY)
#define BOARD_TIM16_FREQUENCY  (STM32_PCLK2_FREQUENCY)
#define BOARD_TIM17_FREQUENCY  (STM32_PCLK2_FREQUENCY)
#define BOARD_TIM20_FREQUENCY  (STM32_PCLK2_FREQUENCY)


/* Alternate function pin selections ************************************************/

/* UARTs */

#define GPIO_USART1_TX  /* PA9  */  GPIO_USART1_TX_1
#define GPIO_USART1_RX  /* PA10 */  GPIO_USART1_RX_1

#define GPIO_USART2_TX  /* PA2  */ GPIO_USART2_TX_1
#define GPIO_USART2_RX  /* PA3  */ GPIO_USART2_RX_1

/* CAN */

#define GPIO_CAN1_TX    /* PA12 */ GPIO_CAN1_TX_1
#define GPIO_CAN1_RX    /* PA11 */ GPIO_CAN1_RX_1

#define GPIO_CAN2_TX    /* PB13 */ GPIO_CAN2_TX_1
#define GPIO_CAN2_RX    /* PB12 */ GPIO_CAN2_RX_2

/* I2C */

#define GPIO_I2C1_SCL   /* PB6  */ GPIO_I2C1_SCL_1
#define GPIO_I2C1_SDA   /* PB7  */ GPIO_I2C1_SDA_1

#define GPIO_I2C2_SCL   /* PB10 */ GPIO_I2C2_SCL_1
#define GPIO_I2C2_SDA   /* PB3  */ GPIO_I2C2_SDA_3

/* SPI */

#define ADJ_SLEW_RATE(p) (((p) & ~GPIO_SPEED_MASK) | (GPIO_SPEED_2MHz))

#define GPIO_SPI1_MISO /* PA6 */ GPIO_SPI1_MISO_1
#define GPIO_SPI1_MOSI /* PB5 */ GPIO_SPI1_MOSI_2
#define GPIO_SPI1_SCK  /* PA5 */ ADJ_SLEW_RATE(GPIO_SPI1_SCK_1)

#endif /* __ARCH_BOARD_BOARD_H */
