/*************************************************************************//**
 * @file
 * @brief    	This file is part of the AFBR-S50 API.
 * @details		This file provides watchdog driver functionality.
 * 
 * @copyright	Copyright c 2016-2019, Avago Technologies GmbH.
 * 				All rights reserved.
 *
 *****************************************************************************/


/*******************************************************************************
 * Include Files
 ******************************************************************************/
#include "cop.h"

#include "board/board_config.h"
#include "driver/irq.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#if (DISABLE_WDOG) /* Watchdog disabled warning. The startup code will disable COP by default, if not DISABLE_WDOG=0. */
#warning "COP Watchdog is disabled! Compile with '-DDISABLE_WDOG=0' in order to enable it."
#endif

#define COP SIM /*!< Register alias. */

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
static volatile bool isInitialized = false;

/*******************************************************************************
 * Code
 ******************************************************************************/
void COP_Init(void)
{
	assert(!isInitialized); // COP can only be initialized/disabled once!

	COP->COPC = SIM_COPC_COPW(0)  		/*!< Set COP watchdog run mode to Normal mode (not Window mode) */
			  | SIM_COPC_COPCLKS(0) 	/*!< LPO clock, 1kHZ. */
		      | SIM_COPC_COPT(3);  		/*!< 2 to 10 clock cycles when clock source is LPO or in short timeout mode otherwise 2 to 18 clock cycles. */

	isInitialized = true;
}

void COP_Disable(void)
{
	assert(!isInitialized); // COP can only be initialized/disabled once!

	COP->COPC &= ~SIM_COPC_COPT_MASK;

	isInitialized = true;
}

void COP_Refresh(void)
{
	/* Disable the global interrupt to protect refresh sequence */
	IRQ_LOCK();
	COP->SRVCOP = 0x55U;  	/*!< First byte of refresh sequence */
	COP->SRVCOP = 0xAAU; 	/*!< Second byte of refresh sequence */
	IRQ_UNLOCK();
}

void COP_ResetSystem(void)
{
	NVIC_SystemReset();
}
