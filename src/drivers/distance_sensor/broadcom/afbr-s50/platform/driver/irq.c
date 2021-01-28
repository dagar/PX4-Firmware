/*************************************************************************//**
 * @file
 * @brief    	This file is part of the platform layer.
 * @details		This file provides an interface for enabling/disabling interrupts.
 *
 * @copyright	Copyright c 2016-2019, Avago Technologies GmbH.
 * 				All rights reserved.
 *****************************************************************************/

#include "irq.h"
#include "board/board_config.h"
#include <assert.h>

/*! Global lock level counter value. */
static volatile int g_irq_lock_ct;

void IRQ_UNLOCK(void)
{
	assert(g_irq_lock_ct > 0);

	if (--g_irq_lock_ct <= 0) {
		g_irq_lock_ct = 0;
		__enable_irq();
	}
}

void IRQ_LOCK(void)
{
	__disable_irq();
	g_irq_lock_ct++;
}


