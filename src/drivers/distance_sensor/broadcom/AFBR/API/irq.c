#include <assert.h>
#include "main.h"
/*! Global lock level counter value. */
static volatile int g_irq_lock_ct;
/*!***************************************************************************
* @brief Enable IRQ Interrupts
*
* @details Enables IRQ interrupts by clearing the I-bit in the CPSR.
* Can only be executed in Privileged modes.
*
* @return -
*****************************************************************************/
void IRQ_UNLOCK(void)
{
assert(g_irq_lock_ct > 0);
if (--g_irq_lock_ct <= 0)
{
g_irq_lock_ct = 0;
__enable_irq();
}
}
/*!***************************************************************************
* @brief Disable IRQ Interrupts
*
* @details Disables IRQ interrupts by setting the I-bit in the CPSR.
* Can only be executed in Privileged modes.
*
* @return -
*****************************************************************************/
void IRQ_LOCK(void)
{
__disable_irq();
g_irq_lock_ct++;
}
