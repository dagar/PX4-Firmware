/*************************************************************************//**
 * @file
 * @brief    	This file is part of the AFBR-S50 API.
 * @details		This file provides debug functionality.
 * 
 * @copyright	Copyright c 2016-2019, Avago Technologies GmbH.
 * 				All rights reserved.
 *
 *****************************************************************************/

#ifndef DEBUG_H
#define DEBUG_H

/*!***************************************************************************
 * @defgroup	debug Debug Utility
 * @ingroup		platform
 * @brief		Debug Utility Module
 * @details		This module provides debugging utility functions such as
 * 				logging, print, asserts and breakpoints.
 * @addtogroup 	debug
 * @{
 *****************************************************************************/

#include "platform/argus_print.h"

/*!***************************************************************************
 * @brief	Error logging function.
 *
 * @details	Redirected to print()
 *
 * @param	fmt format string (prinft style)
 * @param	... parameters
 *****************************************************************************/
#define error_log(fmt, ...) print("ERROR: " fmt "\n", ##__VA_ARGS__)

/*******************************************************************************
 * Debug Utility
 ******************************************************************************/

/*!***************************************************************************
 * @brief	Software breakpoint.
 * @details	Stops the debugger at the corresponding line of code.
 * 			Only active in debug configuration.
 *****************************************************************************/
#ifdef NDEBUG           /* required by ANSI standard */
#define BREAKPOINT() ((void)0)
#else
#define BREAKPOINT() __asm__ __volatile__ ("bkpt #0")
#endif

/*!***************************************************************************
 * @brief	Conditional software breakpoint.
 * @details	Stops the debugger at the corresponding line of code if the
 * 			condition is met.
 * 			Only active in debug configuration.
 * @param	x The condition to be fulfilled (boolean value).
 *****************************************************************************/
#ifdef NDEBUG           /* required by ANSI standard */
#define BREAKPOINT_IF(x) ((void)0)
#else
#define BREAKPOINT_IF(x) if(x) { BREAKPOINT(); }
#endif

/*!***************************************************************************
 * @brief	Assert function definition.
 *
 * @details	Called by the standard library.
 *
 * @param 	file
 * @param 	line
 * @param 	func
 * @param 	failedExpr
 *****************************************************************************/
#ifndef NDEBUG           /* required by ANSI standard */
void __assert_func(const char *file, int line, const char *func, const char *failedExpr);
#endif /* NDEBUG */

/*!***************************************************************************
 * @brief	Checks the reason of the latest system reset and does a printout.
 *****************************************************************************/
void Debug_CheckReset(void);

/*! @} */
#endif /* DEBUG_H */
