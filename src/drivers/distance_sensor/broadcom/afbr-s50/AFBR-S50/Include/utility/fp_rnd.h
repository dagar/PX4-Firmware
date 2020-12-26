/*************************************************************************//**
 * @file
 * @brief    	This file is part of the AFBR-S50 API.
 * @details		Provides definitions and basic macros for fixed point data types.
 * 
 * @copyright	Copyright c 2016-2019, Avago Technologies GmbH.
 * 				All rights reserved.
 *****************************************************************************/

#ifndef FP_RND_H
#define FP_RND_H

/*!***************************************************************************
 * @addtogroup 	fixedpoint
 * @{
 *****************************************************************************/

#include "fp_def.h"
#include <assert.h>

/*!***************************************************************************
 * @brief	Converting with rounding from UQx.n1 to UQx.n2.
 * @details	Equivalent to dividing by 2^n with correct rounding to unsigned
 * 			integer values.
 * @param	Q The number in (U)Qx.n1 fixed point format to be rounded.
 * @param	n The number of bits to be truncated/rounded,
 * 			    e.g. UQ8.8 -> UQ12.4 => n = 8 - 4 = 4.
 * @return	The rounded value in (U)Qx.n2 format.
 *****************************************************************************/
static inline uint32_t fp_rndu(uint32_t Q, uint_fast8_t n);
static inline uint32_t fp_rndu(uint32_t Q, uint_fast8_t n)
{
	if (n == 0)
		return Q;
	else if (n == 32U)
		return Q > 0x7FFFFFFFU ? 1U : 0U;
	else if (n > 32U)
		return 0;

	uint32_t tmp = (1U << (n - 1U));
	if (Q > UINT32_MAX - tmp)
		return (Q >> n) + 1U;
	else
		return ((Q + tmp) >> n);
}

/*!***************************************************************************
 * @brief	Converting with rounding from Qx.n1 to Qx.n2.
 * @details	Equivalent to dividing by 2^n with correct rounding to integer
 * 			values.
 * @param	Q The number in (U)Qx.n1 fixed point format to be rounded.
 * @param	n The number of bits to be truncated/rounded,
 * 			    e.g. Q7.8 -> Q11.4 => n = 8 - 4 = 4.
 * @return	The rounded value in (U)Qx.n2 format.
 *****************************************************************************/
static inline int32_t fp_rnds(int32_t Q, uint_fast8_t n);
static inline int32_t fp_rnds(int32_t Q, uint_fast8_t n)
{
	return (Q < 0) ? -fp_rndu(-Q, n) : fp_rndu(Q, n);
}

/*! @} */
#endif /* FP_RND_H */
