/*************************************************************************//**
 * @file
 * @brief    	This file is part of the AFBR-S50 API.
 * @details		This file provides an interface for the optional debug module.
 * 
 * @copyright	Copyright c 2016-2019, Avago Technologies GmbH.
 * 				All rights reserved.
 *****************************************************************************/

#ifndef ARGUS_PRINT_H
#define ARGUS_PRINT_H

/*!***************************************************************************
 * @defgroup	argus_log Debug: Logging Interface
 * @ingroup		argus_platform
 *
 * @brief		Logging interface for the AFBR-S50 API.
 *
 * @details		This interface provides logging utility functions.
 * 				Defines a printf-like function that is used to print error and
 * 				log messages.
 *
 * @addtogroup 	argus_log
 * @{
 *****************************************************************************/

#include "api/argus_def.h"

/*!***************************************************************************
 * @brief	A printf-like function to print formated data to an debugging interface.
 *
 * @details Writes the C string pointed by fmt_t to an output. If format
 * 			includes format specifiers (subsequences beginning with %), the
 * 			additional arguments following fmt_t are formatted and inserted in
 * 			the resulting string replacing their respective specifiers.
 *
 * 			To enable the print functionality, an implementation of the function
 * 			must be provided that maps the output to an interface like UART or
 * 			a debugging console, e.g. by forwarding to standard printf() method.
 *
 * @note	The implementation of this function is optional for the correct
 * 			execution of the API. If not implemented, a weak implementation
 * 			within the API will be used that does nothing. This will improve
 * 			the performance but no error messages are logged.
 *
 * @note	The naming is different from the standard printf() on purpose to
 * 			prevent builtin compiler optimizations.
 *
 * @param	fmt_s The usual print() format string.
 * @param	... The usual print() parameters. *
 * @return 	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t print(const char *fmt_s, ...);

/*! @} */
#endif /* ARGUS_PRINT_H */
