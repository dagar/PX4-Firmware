/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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
 * @file SPI.hpp
 *
 * Base class for devices connected via SPI.
 */

#pragma once

#include "stm32_dma.h"
#include <nuttx/semaphore.h>

#include <arch/board/board.h>

#include <px4_config.h>

namespace device
{

using gpio_t = uint32_t;

struct SPI_CONFIG {
	uint8_t bus;

	gpio_t SCK;	// eg GPIO_SPI1_SCK
	gpio_t MISO;	// eg GPIO_SPI1_MISO
	gpio_t MOSI;	// eg GPIO_SPI1_MOSI

	uint32_t base_address;	// eg STM32_SPI1_BASE
	uint32_t clock;		// eg STM32_PCLK2_FREQUENCY

	uint8_t	DMA_RX_channel;	// The RX DMA channel number (eg DMAMAP_SPI1_RX_1)
	uint8_t	DMA_TX_channel;	// The TX DMA channel number (eg DMAMAP_SPI1_TX_2)


	uint32_t devices[];
};

/**
 * Abstract class for character device on SPI
 */
class SPIMaster
{
protected:

	SPIMaster() = default;
	~SPIMaster() = default;

	void	Init();
	void	Deinit();

	/**
	 * Perform a SPI transfer.
	 *
	 * @param buffer	Bytes to send to the device,
	 * @param len		Number of bytes to transfer.
	 */
	void	Transfer(uint8_t buffer[], uint16_t len);
	void	TransferDMA(uint8_t buffer[], uint16_t len);

	enum class SPI_MODE {
		MODE_0,	// CPOL=0 CHPHA=0
		MODE_1,	// CPOL=0 CHPHA=1
		MODE_2,	// CPOL=1 CHPHA=0
		MODE_3,	// CPOL=1 CHPHA=1

		INVALID,
	};

	enum class BIT_MODE {
		Bit8,
		Bit16,

		INVALID,
	};

private:
	enum class State {
		Stopped,
		Initializing,
		Transferring,
		WaitingForDMA,
		Ready,
	};

	struct SPIDevice {
		const gpio_t chip_select;
		SPIDevice(gpio_t cs) : chip_select(cs) {}
	};

	void		TransferStart(const SPIDevice &device);
	void		TransferFinish(const SPIDevice &device);

	void		ChangeFrequency(uint32_t frequency);
	void		ChangeMode(SPI_MODE mode);
	void		ChangeBitMode(BIT_MODE nbits);


	enum class Register : uint32_t {
		CR1	= STM32_SPI1_BASE + STM32_SPI_CR1_OFFSET,
		CR2	= STM32_SPI1_BASE + STM32_SPI_CR2_OFFSET,
		SR	= STM32_SPI1_BASE + STM32_SPI_SR_OFFSET,
		DR	= STM32_SPI1_BASE + STM32_SPI_DR_OFFSET,
		CRCPR	= STM32_SPI1_BASE + STM32_SPI_CRCPR_OFFSET,
		RXCRCR	= STM32_SPI1_BASE + STM32_SPI_RXCRCR_OFFSET,
		TXCRCR	= STM32_SPI1_BASE + STM32_SPI_TXCRCR_OFFSET,
	};

	uint8_t		RegisterRead8(Register reg);
	uint16_t	RegisterRead16(Register reg);

	void		RegisterWrite8(Register reg, uint8_t value);
	void		RegisterWrite16(Register reg, uint16_t value);

	void		RegisterModify(Register reg, uint16_t setbits, uint16_t clrbits);


	void		DMARXSetup(void *rxbuffer, size_t nwords);
	void		DMATXSetup(const void *txbuffer, size_t nwords);

	uint32_t		_frequency{0};			// Requested clock frequency
	BIT_MODE		_nbits{BIT_MODE::INVALID};	// Width of word in bits
	SPI_MODE		_mode{SPI_MODE::INVALID};	// Mode 0,1,2,3

	// RX DMA
	static void		DMARXCallback(DMA_HANDLE handle, uint8_t isr, void *arg);
	DMA_HANDLE		_rxdma;			// DMA channel handle for RX transfers
	sem_t			_rxsem;			// Wait for RX DMA to complete
	volatile uint8_t	_rxresult;		// Result of the RX DMA

	// TX DMA
	static void		DMATXCallback(DMA_HANDLE handle, uint8_t isr, void *arg);
	DMA_HANDLE		_txdma;			// DMA channel handle for TX transfers
	sem_t			_txsem;			// Wait for TX DMA to complete
	volatile uint8_t	_txresult;		// Result of the RX DMA

	SPIDevice	_devices[5] {
		SPIDevice(GPIO_SPI1_CS1_ICM20689),
		SPIDevice(GPIO_SPI1_CS2_ICM20602),
		SPIDevice(GPIO_SPI1_CS3_BMI055_GYRO),
		SPIDevice(GPIO_SPI1_CS4_BMI055_ACC),
		SPIDevice(GPIO_SPI1_CS5_AUX_MEM),
	};

	SPI_CONFIG	_config;


};

} // namespace device
