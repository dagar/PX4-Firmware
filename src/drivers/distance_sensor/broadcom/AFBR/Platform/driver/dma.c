/*************************************************************************//**
 * @file
 * @brief    	This file is part of the AFBR-S50 API.
 * @details		This file provides DMA hardware support.
 *
 * @copyright	Copyright c 2016-2019, Avago Technologies GmbH.
 * 				All rights reserved.
 *
 *****************************************************************************/


/*******************************************************************************
 * Include Files
 ******************************************************************************/
#include "dma.h"

#include <stdbool.h>


/* CMSIS-style register definitions */
//#include "devices/MKL46Z4.h"
/* CPU specific feature definitions */
//#include "devices/MKL46Z4_features.h"

#include "board/board_config.h"

#include "driver/fsl_clock.h"
#include "board/board_config.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
static const IRQn_Type irqNumbers[4] = {DMA0_IRQn, DMA1_IRQn, DMA2_IRQn, DMA3_IRQn};

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void DMA0_IRQHandler(void);					/*!< ISR for DMA0 IRQ. */
void DMA1_IRQHandler(void);					/*!< ISR for DMA1_1 IRQ. */
void DMA2_IRQHandler(void);					/*!< ISR for DMA2_1 IRQ. */
void DMA3_IRQHandler(void);					/*!< ISR for DMA3 IRQ. */
static inline void DMA_IRQhandler(uint32_t channel);

/******************************************************************************
 * Variables
 ******************************************************************************/
static dma_callback_t myCallbacks[4] = {0};

/*******************************************************************************
 * Code
 ******************************************************************************/

void DMA_Init(void)
{
	static bool isInitialized = false;
	if(!isInitialized)
	{
	    /* Enable DMA clock. */
		CLOCK_EnableClock(kCLOCK_Dma0);

	    /* Enable DMAMUX clock and init. */
		CLOCK_EnableClock(kCLOCK_Dmamux0);

		/* Set IRQ priorities. */
		NVIC_SetPriority(DMA0_IRQn, IRQPRIO_DMA0);
		NVIC_SetPriority(DMA1_IRQn, IRQPRIO_DMA1);
		NVIC_SetPriority(DMA2_IRQn, IRQPRIO_DMA2);
		NVIC_SetPriority(DMA3_IRQn, IRQPRIO_DMA3);

	    /* Initialize the dmamux module to the reset state. */
	    for (int i = 0; i < FSL_FEATURE_DMAMUX_MODULE_CHANNEL; i++)
	    {
	    	DMAMUX0->CHCFG[i] &= (uint8_t)(~DMAMUX_CHCFG_ENBL_MASK);
	    	DMAMUX0->CHCFG[i] = (uint8_t)((DMAMUX0->CHCFG[i] & ~DMAMUX_CHCFG_SOURCE_MASK) | DMAMUX_CHCFG_SOURCE(0));
	    }

		isInitialized = true;
	}
}

void DMA_SetTransferDoneCallback(uint32_t channel, dma_callback_t f)
{
	assert(channel < 4);
	myCallbacks[channel] = f;
}

void DMA_RemoveTransferDoneCallback(uint32_t channel)
{
	assert(channel < 4);
	myCallbacks[channel] = 0;
}

void DMA_ClaimChannel(uint32_t channel, uint8_t source)
{
	assert(channel < 4);

    /* Enable NVIC interrupt. */
    EnableIRQ(irqNumbers[channel]);

    /* Configure DMAMUX channel */
  	DMAMUX0->CHCFG[channel] &= (uint8_t)(~DMAMUX_CHCFG_ENBL_MASK); // Disables the DMAMUX channel.
  	DMAMUX0->CHCFG[channel] = (uint8_t)((DMAMUX0->CHCFG[channel] & ~DMAMUX_CHCFG_SOURCE_MASK) |
  			DMAMUX_CHCFG_SOURCE(source)); // Configure the DMA request for the DMAMUX channel.
  	DMAMUX0->CHCFG[channel] |= DMAMUX_CHCFG_ENBL_MASK; // Enables the DMAMUX channel.
}
void DMA_StartChannel(uint32_t channel)
{
	assert(channel < 4);
	DMA0->DMA[channel].DCR |= DMA_DCR_ERQ_MASK;
}

void DMA_SetSource(uint32_t channel, uint32_t sourceAddr, uint32_t transferCount)
{
	assert(channel < 4);
    DMA0->DMA[channel].SAR = sourceAddr;							// set source address
    DMA0->DMA[channel].DSR_BCR = DMA_DSR_BCR_BCR(transferCount);	// set transfer count
}
void DMA_SetDestination(uint32_t channel, uint32_t destAddr, uint32_t transferCount)
{
	assert(channel < 4);
    DMA0->DMA[channel].DAR = destAddr;								// set destination address
    DMA0->DMA[channel].DSR_BCR = DMA_DSR_BCR_BCR(transferCount);	// set transfer count
}
void DMA_StopChannel(uint32_t channel)
{
	assert(channel < 4);
	DMA0->DMA[channel].DCR &= ~DMA_DCR_ERQ_MASK;
}
void DMA_ClearStatus(uint32_t channel)
{
	assert(channel < 4);
	DMA0->DMA[channel].DSR_BCR |= DMA_DSR_BCR_DONE(true);
}
uint32_t DMA_GetUnfinishedBytes(uint32_t channel)
{
	assert(channel < 4);
    return (DMA0->DMA[channel].DSR_BCR & DMA_DSR_BCR_BCR_MASK) >> DMA_DSR_BCR_BCR_SHIFT;
}

void DMA_ConfigTransfer(uint32_t channel, uint32_t size, dma_transfer_type_t type,
		uint32_t sourceAddr, uint32_t destAddr, uint32_t length)
{
	assert(channel < 4);

    uint8_t transfersize;
    uint8_t sinc, dinc;
    switch (size)
    {
        case 1:
            transfersize = 1; // 8-bit
            break;
        case 2:
            transfersize = 2; // 16-bit
            break;
        case 4:
            transfersize = 0; // 32-bit
            break;
        default:
            transfersize = 1; // 8-bit
    }

    switch (type)
    {
      case DMA_MEMORY_TO_PERIPHERAL:
          sinc = 1;
          dinc = 0;
          break;
      case DMA_PERIPHERAL_TO_MEMORY:
          sinc = 0;
          dinc = 1;
          break;
      case DMA_MEMORY_TO_MEMORY:
          sinc = 1;
          dinc = 1;
          break;
      case DMA_PERIPHERAL_TO_PERIPHERAL:
      default:
          sinc = 0;
          dinc = 0;
          break;
    }


    /* Clear the DMA status. */
	DMA0->DMA[channel].DSR_BCR |= DMA_DSR_BCR_DONE(true);

    /* Common configuration. */

    /* Set source address */
    DMA0->DMA[channel].SAR = sourceAddr;
    /* Set destination address */
    DMA0->DMA[channel].DAR = destAddr;
    /* Set transfer bytes */
    DMA0->DMA[channel].DSR_BCR = DMA_DSR_BCR_BCR(length);
    /* Set DMA Control Register */
    DMA0->DMA[channel].DCR = DMA_DCR_AA(0) |
    						 DMA_DCR_CS(1) |
    						 DMA_DCR_EADREQ(0) |
    						 DMA_DCR_D_REQ(1) |
    						 DMA_DCR_LINKCC(0) |
    						 DMA_DCR_EINT(1) |
    						 DMA_DCR_SMOD(0) |
    						 DMA_DCR_DMOD(0) |
    						 DMA_DCR_SSIZE(transfersize) |
    						 DMA_DCR_DSIZE(transfersize) |
    						 DMA_DCR_SINC(sinc) |
    						 DMA_DCR_DINC(dinc);

}

static inline void DMA_IRQhandler(uint32_t channel)
{
	uint32_t val = DMA0->DMA[channel].DSR_BCR; 				// get status
	DMA0->DMA[channel].DSR_BCR |= DMA_DSR_BCR_DONE(true); 	// clear status

    if (myCallbacks[channel])
    {
    	status_t status;

    	if (val & DMA_DSR_BCR_CE_MASK)
    		status = ERROR_DMA_CONFIG_ERR;
    	else if(val & DMA_DSR_BCR_BED_MASK)
    		status = ERROR_DMA_DEST_BUS_ERR;
        else if (val & DMA_DSR_BCR_BES_MASK)
        	status = ERROR_DMA_SRC_BUS_ERR;
        else
        	status = STATUS_OK;

        myCallbacks[channel](status);
    }
}

/* DMA IRQ handler with the same name in startup code*/
void DMA0_IRQHandler(void) { DMA_IRQhandler(0); }
void DMA1_IRQHandler(void) { DMA_IRQhandler(1); }
void DMA2_IRQHandler(void) { DMA_IRQhandler(2); }
void DMA3_IRQHandler(void) { DMA_IRQhandler(3); }
