#include "argus.h"
#include "board/clock_config.h"
#include "driver/cop.h"
extern void SystemClock_Config(void);
/* Initialize the board with clocks. */
void BOARD_ClockInit(void)
{
SystemClock_Config();
}
/* No watchdog installed */
void COP_Disable(void) {}
