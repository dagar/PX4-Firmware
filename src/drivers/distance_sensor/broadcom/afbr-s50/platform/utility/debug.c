/*************************************************************************//**
 * @file
 * @brief    	This file is part of the AFBR-S50 API.
 * @details		This file provides debug functionality.
 *
 * @copyright	Copyright c 2016-2019, Avago Technologies GmbH.
 * 				All rights reserved.
 *
 *****************************************************************************/

/*******************************************************************************
 * Include Files
 ******************************************************************************/
#include "board/board_config.h"
#include "debug.h"

#if defined(CPU_MKL46Z256VLH4) || defined(CPU_MKL46Z256VLL4) || defined(CPU_MKL46Z256VMC4) || defined(CPU_MKL46Z256VMP4)
#include "driver/MKL46Z/slcd.h"
#endif

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

void Debug_CheckReset(void)
{
	// Check what reset has occurred
	uint32_t srs0 = (uint32_t)RCM->SRS0;
	uint32_t srs1 = (uint32_t)RCM->SRS1;

	if (srs0 & RCM_SRS0_WAKEUP_MASK) {
		error_log(" >>> Reset due to a \"Low Leakage Wakeup Reset\"! <<< ");
	}

	if (srs0 &  RCM_SRS0_LVD_MASK) {
		error_log(" >>> Reset due to a \"Low-Voltage Detect Reset\"! <<< ");
	}

#if defined(CPU_MKL46Z256VLH4) || defined(CPU_MKL46Z256VLL4) || defined(CPU_MKL46Z256VMC4) || defined(CPU_MKL46Z256VMP4)

	if (srs0 & RCM_SRS0_LOC_MASK) {
		error_log(" >>> Reset due to a \"Loss-of-Clock Reset\"! <<< ");
	}

	if (srs0 & RCM_SRS0_LOL_MASK) {
		error_log(" >>> Reset due to a \"Loss-of-Lock Reset\"! <<< ");
	}

#endif

	if (srs0 &  RCM_SRS0_WDOG_MASK) {
		error_log(" >>> Reset due to a \"Watchdog Reset\"! <<< ");
	}

	if (srs0 &  RCM_SRS0_PIN_MASK) {
		print(" >>> Reset due to a \"External Reset Pin\"! <<< ");
	}

	if (srs0 & RCM_SRS0_POR_MASK) {
		print(" >>> Reset due to a \"Power-On Reset\"! <<< ");
	}

	/*! @name SRS1 - System Reset Status Register 1 */
	if (srs1 & RCM_SRS1_LOCKUP_MASK) {
		error_log(" >>> Reset due to a \"Core Lockup\"! <<< ");
	}

	if (srs1 &  RCM_SRS1_SW_MASK) {
		print(" >>> Reset due to a \"Software Reset\"! <<< ");
	}

	if (srs1 &  RCM_SRS1_MDM_AP_MASK) {
		// Reset after flashing
		print(" >>> Reset due to a \"MDM-AP System Reset Request\"! <<< ");
	}

	if (srs1 &  RCM_SRS1_SACKERR_MASK) {
		error_log(" >>> Reset due to a \"Stop Mode Acknowledge Error Reset\"! <<< ");
	}
}

#ifndef NDEBUG
#if (defined(__CC_ARM)) || (defined(__ICCARM__))
void __aeabi_assert(const char *failedExpr, const char *file, int line)
{
#if defined(CPU_MKL46Z256VLH4) || defined(CPU_MKL46Z256VLL4) || defined(CPU_MKL46Z256VMC4) || defined(CPU_MKL46Z256VMP4)
	/* Display error on LCD. */
	SLCD_DisplayError(0xAA);
#endif

	/* Try to send printf message. */
	print("ASSERT: expression \"%s\" failed;\n"
	      "file \"%s\";\n"
	      "line \"%d\";\n",
	      failedExpr, file, line);

	/* Wait for sending print statement */
	for (volatile uint32_t i = 0; i < 1000000; i++) { __asm("nop"); }

	/* Stop. */
	for (;;) { BREAKPOINT(); }
}
#elif(defined(__REDLIB__))
void __assertion_failed(char *_Expr)
{
#if defined(CPU_MKL46Z256VLH4) || defined(CPU_MKL46Z256VLL4) || defined(CPU_MKL46Z256VMC4) || defined(CPU_MKL46Z256VMP4)
	/* Display error on LCD. */
	SLCD_DisplayError(0xAA);
#endif

	/* Try to send printf message. */
	print("ASSERT: \"%s\"", _Expr);

	/* Wait for sending print statement */
	for (volatile uint32_t i = 0; i < 1000000; i++) { __asm("nop"); }

	/* Stop. */
	for (;;) { BREAKPOINT(); }
}
#elif(defined(__GNUC__))
void __assert_func(const char *file, int line, const char *func, const char *failedExpr)
{
#if defined(CPU_MKL46Z256VLH4) || defined(CPU_MKL46Z256VLL4) || defined(CPU_MKL46Z256VMC4) || defined(CPU_MKL46Z256VMP4)
	/* Display error on LCD. */
	SLCD_DisplayError(0xAA);
#endif

	/* Try to send printf message. */
	print("ASSERT: expression \"%s\" failed;\n"
	      "file \"%s\";\n"
	      "line \"%d\";\n"
	      "function \"%s\";\n",
	      failedExpr, file, line, func);

	/* Wait for sending print statement */
	for (volatile uint32_t i = 0; i < 1000000; i++) { __asm("nop"); }

	/* Stop. */
	for (;;) {
		BREAKPOINT();
	}

}
#endif /* (defined(__CC_ARM)) ||  (defined (__ICCARM__)) */
#endif /* NDEBUG */
