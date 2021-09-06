/************************************************************************************
 * nuttx-config/include/board.h
 *
 *   Copyright (C) 2020 Gregory Nutt. All rights reserved.
 *   Authors: David Sidrane <david.sidrane@nscdg.com>
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
#pragma once

#include "board_dma_map.h"

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
# include <stdint.h>
#endif

#include "stm32_rcc.h"
#include "stm32_sdmmc.h"

/* Clocking *************************************************************************/
/* The board provides the following clock sources:
 *
 *   X1: 24 MHz crystal for HSE
 *
 * So we have these clock source available within the STM32
 *
 *   HSI: 16 MHz RC factory-trimmed internal oscillator
 *   HSE: 24 MHz crystal for HSE
 */
#define STM32_BOARD_XTAL        24000000ul

#define STM32_HSI_FREQUENCY     16000000ul
#define STM32_LSI_FREQUENCY     32000
#define STM32_HSE_FREQUENCY     STM32_BOARD_XTAL
#define STM32_LSE_FREQUENCY     0

#include <board_defaults.h>

/* UART/USART */
#define GPIO_USART2_TX   GPIO_USART2_TX_2      /* PD5 */
#define GPIO_USART2_RX   GPIO_USART2_RX_2      /* PD6 */
#define GPIO_USART2_CTS  GPIO_USART2_CTS_NSS_2 /* PD3 */
#define GPIO_USART2_RTS  GPIO_USART2_RTS_2     /* PD4 */

#define GPIO_USART3_TX   GPIO_USART3_TX_3      /* PD8  */
#define GPIO_USART3_RX   GPIO_USART3_RX_3      /* PD9  */
#define GPIO_USART3_CTS  GPIO_USART3_CTS_NSS_2 /* PD11 */
#define GPIO_USART3_RTS  GPIO_USART3_RTS_2     /* PD12 */

#define GPIO_UART4_TX    GPIO_UART4_TX_2       /* PA0 */
#define GPIO_UART4_RX    GPIO_UART4_RX_2       /* PA1 */

#define GPIO_USART6_TX   GPIO_USART6_TX_1      /* PC6 */
#define GPIO_USART6_RX   GPIO_USART6_RX_1      /* PC7 */

#define GPIO_UART7_TX    GPIO_UART7_TX_3       /* PE8 */
#define GPIO_UART7_RX    GPIO_UART7_RX_3       /* PE7 */

#define GPIO_UART8_TX    GPIO_UART8_TX_1       /* PE1 */
#define GPIO_UART8_RX    GPIO_UART8_RX_1       /* PE0 */


/* CAN */
#define GPIO_CAN1_RX     GPIO_CAN1_RX_3        /* PD0  */
#define GPIO_CAN1_TX     GPIO_CAN1_TX_3        /* PD1  */

#define GPIO_CAN2_RX     GPIO_CAN2_RX_1        /* PB12 */
#define GPIO_CAN2_TX     GPIO_CAN2_TX_2        /* PB6  */


/* SPI */
#define ADJ_SLEW_RATE(p) (((p) & ~GPIO_SPEED_MASK) | (GPIO_SPEED_2MHz))

#define GPIO_SPI1_SCK    ADJ_SLEW_RATE(GPIO_SPI1_SCK_1) /* PA5  */
#define GPIO_SPI1_MISO   GPIO_SPI1_MISO_1               /* PA6  */
#define GPIO_SPI1_MOSI   GPIO_SPI1_MOSI_1               /* PA7  */

#define GPIO_SPI2_SCK    ADJ_SLEW_RATE(GPIO_SPI2_SCK_4) /* PB13 */
#define GPIO_SPI2_MISO   GPIO_SPI2_MISO_1               /* PB14 */
#define GPIO_SPI2_MOSI   GPIO_SPI2_MOSI_1               /* PB15 */

#define GPIO_SPI4_SCK    ADJ_SLEW_RATE(GPIO_SPI4_SCK_2) /* PE2  */
#define GPIO_SPI4_MISO   GPIO_SPI4_MISO_2               /* PE5  */
#define GPIO_SPI4_MOSI   GPIO_SPI4_MOSI_2               /* PE6  */


/* I2C */
#define GPIO_I2C1_SCL GPIO_I2C1_SCL_2       /* PB8  */
#define GPIO_I2C1_SDA GPIO_I2C1_SDA_2       /* PB9  */

#define GPIO_I2C2_SCL GPIO_I2C2_SCL_1       /* PB10 */
#define GPIO_I2C2_SDA GPIO_I2C2_SDA_1       /* PB11 */
