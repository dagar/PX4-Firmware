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

namespace device
{

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
		MODE_0 = 0, // CPOL=0 CHPHA=0
		MODE_1 = 1, // CPOL=0 CHPHA=1
		MODE_2 = 2, // CPOL=1 CHPHA=0
		MODE_3 = 3  // CPOL=1 CHPHA=1
	};

private:

	uint32_t	ChangeFrequency(uint32_t frequency);
	void		ChangeMode(SPI_MODE mode);
	void		ChangeBits(int nbits);


	uint8_t		RegisterRead8(int reg);
	uint16_t	RegisterRead16(int reg);

	void		RegisterWrite8(int reg, uint8_t value);
	void		RegisterWrite16(int reg, uint16_t value);

	void		RegisterModify(int reg, uint16_t setbits, uint16_t clrbits);


	void		DMARXSetup(void *rxbuffer, size_t nwords);
	void		DMATXSetup(const void *txbuffer, size_t nwords);


	uint32_t		_spibase{STM32_SPI1_BASE};		// SPIn base address
	uint32_t		_spiclock{STM32_PCLK2_FREQUENCY};	// Clocking for the SPI module

	uint32_t		_frequency_requested{0};	// Requested clock frequency
	uint32_t		_frequency_actual{0};		// Actual clock frequency
	int8_t			_nbits{0};			// Width of word in bits
	SPI_MODE		_mode{SPI_MODE::MODE_0};	// Mode 0,1,2,3

	// RX DMA
	static void		DMARXCallback(DMA_HANDLE handle, uint8_t isr, void *arg);
	DMA_HANDLE		_rxdma;			// DMA channel handle for RX transfers
	sem_t			_rxsem;			// Wait for RX DMA to complete
	uint8_t			_rxch{DMAMAP_SPI1_RX_1};	// The RX DMA channel number
	volatile uint8_t	_rxresult;		// Result of the RX DMA

	// TX DMA
	static void		DMATXCallback(DMA_HANDLE handle, uint8_t isr, void *arg);
	DMA_HANDLE		_txdma;			// DMA channel handle for TX transfers
	sem_t			_txsem;			// Wait for TX DMA to complete
	uint8_t			_txch{DMAMAP_SPI1_TX_2};	// The TX DMA channel number
	volatile uint8_t	_txresult;		// Result of the RX DMA

};

} // namespace device
