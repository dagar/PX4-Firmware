/*************************************************************************//**
 * @file
 * @brief    	This file is part of the AFBR-S50 API.
 * @details		This file provides an interface for enabling/disabling interrupts.
 *
 * @copyright	Copyright c 2016-2019, Avago Technologies GmbH.
 * 				All rights reserved.
 *****************************************************************************/

#ifndef ARGUS_IRQ_H
#define ARGUS_IRQ_H

/*!***************************************************************************
 * @defgroup	argus_irq IRQ: Global Interrupt Control Layer
 * @ingroup		argus_platform
 *
 * @brief		Global Interrupt Control Layer
 *
 * @details		This module provides functionality to globally enable/disable
 *				interrupts by turning the I-bit in the CPSR on/off.
 *
 *				Here is a simple example implementation using the CMSIS functions
 *				"__enable_irq()" and "__disable_irq()". An integer counter is
 *				used to achieve nested interrupt disabling:
 *
 *				@code
 *
 *				// Global lock level counter value.
 *				static volatile int g_irq_lock_ct;
 *
 *				// Global unlock all interrupts using CMSIS function "__enable_irq()".
 *				void IRQ_UNLOCK(void)
 *				{
 *					assert(g_irq_lock_ct > 0);
 *					if (--g_irq_lock_ct <= 0)
 *					{
 *						g_irq_lock_ct = 0;
 *						__enable_irq();
 *					}
 *				}
 *
 *				// Global lock all interrupts using CMSIS function "__disable_irq()".
 *				void IRQ_LOCK(void)
 *				{
 *					__disable_irq();
 *					g_irq_lock_ct++;
 *				}
 *
 *				@endcode
 *
 * @note		The IRQ locking mechanism is used to create atomic sections
 * 				(within the scope of the AFBR-S50 API) that	are very few processor
 * 				instruction only. It does NOT lock interrupts for considerable
 * 				amounts of time.
 *
 * @note		The interrupts utilized by the AFBR-S50 API can be interrupted
 * 				by other, higher prioritized interrupts, e.g. some system
 * 				critical interrupts. In this case, the IRQ_LOCK/IRQ_UNLOCK
 * 				mechanism can be implemented such that only the interrupts
 * 				required for the AFBR-S50 API are locked. The above	example is
 * 				dedicated to a ARM Corex-M0 architecture, where interrupts
 *				can only disabled at a global scope. Other architectures like
 *				ARM Cortex-M4 allow selective disabling of interrupts.
 *
 * @addtogroup 	argus_irq
 * @{
 *****************************************************************************/

/*!***************************************************************************
 * @brief	Enable IRQ Interrupts
 *
 * @details	Enables IRQ interrupts by clearing the I-bit in the CPSR.
 * 			Can only be executed in Privileged modes.
 *****************************************************************************/
void IRQ_UNLOCK(void);

/*!***************************************************************************
 * @brief	Disable IRQ Interrupts
 *
 * @details	Disables IRQ interrupts by setting the I-bit in the CPSR.
 * 			Can only be executed in Privileged modes.
 *****************************************************************************/
void IRQ_LOCK(void);

/*! @} */
#endif // ARGUS_IRQ_H
