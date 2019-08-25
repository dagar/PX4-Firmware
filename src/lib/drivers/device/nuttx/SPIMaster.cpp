/****************************************************************************
 *
 *   Copyright (C) 2019 PX4 Development Team. All rights reserved.
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

#include "SPIMaster.hpp"

#include <px4_config.h>
#include <nuttx/arch.h>

#include "stm32_gpio.h"
#include "stm32_dma.h"

namespace device
{

static constexpr uint32_t SPI_CLOCK{STM32_PCLK2_FREQUENCY};	// Clocking for the SPI module


uint8_t
SPIMaster::RegisterRead8(Register reg)
{
	return (*(volatile uint8_t *)(reg));
}

uint16_t
SPIMaster::RegisterRead16(Register reg)
{
	return (*(volatile uint16_t *)(reg));
}

void
SPIMaster::RegisterWrite16(Register reg, uint16_t value)
{
	(*(volatile uint16_t *)(reg) = value);
}

void
SPIMaster::RegisterModify(Register reg, uint16_t setbits, uint16_t clrbits)
{
	uint16_t cr = RegisterRead16(reg);
	cr &= ~clrbits;
	cr |= setbits;
	RegisterWrite16(reg, cr);
}

void
SPIMaster::Init()
{
	// Configure SPI1 pins: SCK, MISO, and MOSI
	stm32_configgpio(GPIO_SPI1_SCK);
	stm32_configgpio(GPIO_SPI1_MISO);
	stm32_configgpio(GPIO_SPI1_MOSI);

	for (const auto &dev : _devices) {
		stm32_configgpio(dev.chip_select);
		stm32_gpiowrite(dev.chip_select, true);
	}

	/* Configure CR1 and CR2. Default configuration:
	 *   Mode 0:                        CR1.CPHA=0 and CR1.CPOL=0
	 *   Master:                        CR1.MSTR=1
	 *   MSB tranmitted first:          CR1.LSBFIRST=0
	 *   Replace NSS with SSI & SSI=1:  CR1.SSI=1 CR1.SSM=1 (prevents MODF error)
	 *   Two lines full duplex:         CR1.BIDIMODE=0 CR1.BIDIOIE=(Don't care) and CR1.RXONLY=0
	 */
	uint16_t clrbits = SPI_CR1_BR_MASK | SPI_CR1_LSBFIRST | SPI_CR1_RXONLY | SPI_CR1_BIDIOE | SPI_CR1_BIDIMODE;
	uint16_t setbits = SPI_CR1_MSTR | SPI_CR1_SSI | SPI_CR1_SSM;
	RegisterModify(Register::CR1, setbits, clrbits);

	ChangeMode(SPI_MODE::MODE_0);
	ChangeBitMode(BIT_MODE::Bit8);
	ChangeFrequency(400000);

	// CRCPOLY configuration
	RegisterWrite16(Register::CRCPR, 7);

#ifdef CONFIG_STM32F7_SPI_DMA
	//  Initialize the SPI semaphores that is used to wait for DMA completion.
	nxsem_init(&_rxsem, 0, 0);
	nxsem_setprotocol(&_rxsem, SEM_PRIO_NONE);

	nxsem_init(&_txsem, 0, 0);
	nxsem_setprotocol(&_txsem, SEM_PRIO_NONE);

	// Get DMA channels
	_rxdma = stm32_dmachannel(_rxch);
	_txdma = stm32_dmachannel(_txch);

	RegisterModify(Register::CR2, SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN, 0);
#endif // CONFIG_STM32F7_SPI_DMA

	// Enable SPI
	RegisterModify(Register::CR1, SPI_CR1_SPE, 0);
}

void
SPIMaster::Deinit()
{
	stm32_unconfiggpio(GPIO_SPI1_SCK);
	stm32_unconfiggpio(GPIO_SPI1_MISO);
	stm32_unconfiggpio(GPIO_SPI1_MOSI);

	for (const auto &dev : _devices) {
		stm32_unconfiggpio(dev.chip_select);
	}
}

void
SPIMaster::ChangeFrequency(uint32_t frequency)
{
	// Has the frequency changed?
	if (frequency != _frequency) {
		// Disable SPI then change frequency
		RegisterModify(Register::CR1, 0, SPI_CR1_SPE);

		// Choices are limited by PCLK frequency with a set of divisors
		if (frequency >= SPI_CLOCK >> 1) {
			// More than fPCLK/2.  This is as fast as we can go
			RegisterModify(Register::CR1, SPI_CR1_FPCLCKd2, SPI_CR1_BR_MASK); /* 000: fPCLK/2 */

		} else if (frequency >= SPI_CLOCK >> 2) {
			// Between fPCLCK/2 and fPCLCK/4, pick the slower
			RegisterModify(Register::CR1, SPI_CR1_FPCLCKd4, SPI_CR1_BR_MASK); /* 001: fPCLK/4 */

		} else if (frequency >= SPI_CLOCK >> 3) {
			// Between fPCLCK/4 and fPCLCK/8, pick the slower
			RegisterModify(Register::CR1, SPI_CR1_FPCLCKd8, SPI_CR1_BR_MASK); /* 010: fPCLK/8 */

		} else if (frequency >= SPI_CLOCK >> 4) {
			// Between fPCLCK/8 and fPCLCK/16, pick the slower
			RegisterModify(Register::CR1, SPI_CR1_FPCLCKd16, SPI_CR1_BR_MASK); /* 011: fPCLK/16 */

		} else if (frequency >= SPI_CLOCK >> 5) {
			// Between fPCLCK/16 and fPCLCK/32, pick the slower
			RegisterModify(Register::CR1, SPI_CR1_FPCLCKd32, SPI_CR1_BR_MASK); /* 100: fPCLK/32 */

		} else if (frequency >= SPI_CLOCK >> 6) {
			// Between fPCLCK/32 and fPCLCK/64, pick the slower
			RegisterModify(Register::CR1, SPI_CR1_FPCLCKd64, SPI_CR1_BR_MASK); /*  101: fPCLK/64 */

		} else if (frequency >= SPI_CLOCK >> 7) {
			// Between fPCLCK/64 and fPCLCK/128, pick the slower
			RegisterModify(Register::CR1, SPI_CR1_FPCLCKd128, SPI_CR1_BR_MASK); /* 110: fPCLK/128 */

		} else {
			// Less than fPCLK/128. This is as slow as we can go
			RegisterModify(Register::CR1, SPI_CR1_FPCLCKd256, SPI_CR1_BR_MASK); /* 111: fPCLK/256 */
		}

		// Enable SPI
		RegisterModify(Register::CR1, SPI_CR1_SPE, 0);

		// Save the new frequency selection
		_frequency = frequency;
	}
}

void
SPIMaster::ChangeMode(SPI_MODE mode)
{
	// Has the mode changed?
	if (mode != _mode) {
		// Disable SPI then change mode
		RegisterModify(Register::CR1, 0, SPI_CR1_SPE);

		switch (mode) {
		default:
		case SPI_MODE::MODE_0: /* CPOL=0; CPHA=0 */
			RegisterModify(Register::CR1, 0, SPI_CR1_CPOL | SPI_CR1_CPHA);
			break;

		case SPI_MODE::MODE_1: /* CPOL=0; CPHA=1 */
			RegisterModify(Register::CR1, SPI_CR1_CPHA, SPI_CR1_CPOL);
			break;

		case SPI_MODE::MODE_2: /* CPOL=1; CPHA=0 */
			RegisterModify(Register::CR1, SPI_CR1_CPOL, SPI_CR1_CPHA);
			break;

		case SPI_MODE::MODE_3: /* CPOL=1; CPHA=1 */
			RegisterModify(Register::CR1, SPI_CR1_CPOL | SPI_CR1_CPHA, 0);
			break;
		}

#ifdef CONFIG_STM32F7_SPI_DMA
		/* Enabling SPI causes a spurious received character indication
		 * which confuse the DMA controller so we disable DMA during that
		 * enabling; and flush the SPI RX FIFO before re-enabling DMA.
		 */
		uint16_t cr2bits = RegisterRead16(Register::CR2);
		RegisterModify(Register::CR2, 0, SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);
#endif // CONFIG_STM32F7_SPI_DMA

		// Re-enable SPI
		RegisterModify(Register::CR1, SPI_CR1_SPE, 0);

		while ((RegisterRead16(Register::SR) & SPI_SR_FRLVL_MASK) != 0) {
			// Flush SPI read FIFO
			RegisterRead16(Register::DR);
		}

#ifdef CONFIG_STM32F7_SPI_DMA
		// Re-enable DMA (with SPI disabled)
		RegisterModify(Register::CR1, 0, SPI_CR1_SPE);
		RegisterModify(Register::CR2, cr2bits & (SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN), 0);
		RegisterModify(Register::CR1, SPI_CR1_SPE, 0);
#endif // CONFIG_STM32F7_SPI_DMA

		// Save the new mode
		_mode = mode;
	}
}

void
SPIMaster::ChangeBitMode(BIT_MODE nbits)
{
	// Has the number of bits changed?
	if (nbits != _nbits) {

		// disable SPI
		RegisterModify(Register::CR1, 0, SPI_CR1_SPE);

		switch (nbits) {
		default:
		case BIT_MODE::Bit8:

			// RX FIFO Threshold = 1 byte
			RegisterModify(Register::CR2, SPI_CR2_DS_VAL(8) | SPI_CR2_FRXTH, SPI_CR2_DS_MASK);
			break;

		case BIT_MODE::Bit16:
			// RX FIFO Threshold = 2 bytes
			RegisterModify(Register::CR2, SPI_CR2_DS_VAL(16), SPI_CR2_DS_MASK | SPI_CR2_FRXTH);
			break;
		}

		// enable SPI
		RegisterModify(Register::CR1, SPI_CR1_SPE, 0);

		// Save the selection
		_nbits = nbits;
	}
}

void
SPIMaster::TransferStart(const SPIDevice &device)
{
	ChangeFrequency(1000000);
	ChangeMode(_mode);
	ChangeBitMode(BIT_MODE::Bit8);

	// Making sure the other peripherals are not selected
	for (const auto &dev : _devices) {
		stm32_gpiowrite(dev.chip_select, true);
	}

	// SPI select is active low
	stm32_gpiowrite(device.chip_select, false);
}

void
SPIMaster::TransferFinish(const SPIDevice &device)
{
	// and clean up
	stm32_gpiowrite(device.chip_select, true);
}

void
SPIMaster::Transfer(uint8_t buffer[], uint16_t len)
{
	TransferStart(_devices[0]);

	// Exchange one byte at a time
	for (int i = 0; i < len; i++) {
		// Wait until the transmit buffer is empty, then send the byte
		while ((RegisterRead16(Register::SR) & SPI_SR_TXE) == 0);

		RegisterWrite8(Register::DR, buffer[i]);

		// Wait until the receive buffer is not empty, then receive the byte
		while ((RegisterRead16(Register::SR) & SPI_SR_RXNE) == 0);

		buffer[i] = RegisterRead8(Register::DR);

		// Check and clear any error flags (Reading from the SR clears the error flags).
		RegisterRead16(Register::SR);
	}

	TransferFinish(_devices[0]);
}

void
SPIMaster::DMARXCallback(DMA_HANDLE handle, uint8_t isr, void *arg)
{
	SPIMaster *dev = (SPIMaster *)arg;

	// Wake-up the SPI driver
	dev->_rxresult = isr | 0x080;  // OR'ed with 0x80 to assure non-zero
	nxsem_post(&dev->_rxsem);
}

void
SPIMaster::DMATXCallback(DMA_HANDLE handle, uint8_t isr, void *arg)
{
	SPIMaster *dev = (SPIMaster *)arg;

	// Wake-up the SPI driver
	dev->_txresult = isr | 0x080;  // OR'ed with 0x80 to assure non-zero
	nxsem_post(&dev->_txsem);
}

void
SPIMaster::TransferDMA(uint8_t buffer[], uint16_t len)
{
	TransferStart(_devices[0]);

	// setup the DMA
	_rxresult = 0;
	_txresult = 0;

	uint32_t RXDMA8 = (DMA_SCR_PRILO | DMA_SCR_MSIZE_8BITS | DMA_SCR_PSIZE_8BITS | DMA_SCR_MINC | DMA_SCR_DIR_P2M);
	stm32_dmasetup(_rxdma, (uint32_t)Register::DR, (uint32_t)buffer, len, RXDMA8);

	uint32_t TXDMA8 = (DMA_SCR_PRILO | DMA_SCR_MSIZE_8BITS | DMA_SCR_PSIZE_8BITS | DMA_SCR_MINC | DMA_SCR_DIR_M2P);
	stm32_dmasetup(_txdma, (uint32_t)Register::DR, (uint32_t)buffer, len, TXDMA8);

	// Flush cache to physical memory
	up_flush_dcache((uintptr_t)buffer, (uintptr_t)buffer + len);

	// Start the DMAs
	stm32_dmastart(_rxdma, SPIMaster::DMARXCallback, this, false);
	stm32_dmastart(_txdma, SPIMaster::DMATXCallback, this, false);

	// Then wait for each to complete
	int ret = 0;

	// RX wait - Take the semaphore (perhaps waiting). If the result is zero, then the DMA must not really have completed???
	ret = 0;

	do {
		ret = nxsem_wait(&_rxsem);
	} while (ret == -EINTR || _rxresult == 0);

	// TX wait
	ret = 0;

	do {
		ret = nxsem_wait(&_txsem);
	} while (ret == -EINTR || _txresult == 0);

	// Force RAM re-read
	up_invalidate_dcache((uintptr_t)buffer, (uintptr_t)buffer + len);

	TransferFinish(_devices[0]);
}

} // namespace device
