/*************************************************************************//**
 * @file
 * @brief    	This file is part of the AFBR-S50 API.
 * @details		Defines the Shot Noise Monitor (SNM) setup parameters.
 *
 * @copyright	Copyright (c) 2016-2020, Avago Technologies GmbH.
 * 				All rights reserved.
 *****************************************************************************/

#ifndef ARGUS_SNM_H
#define ARGUS_SNM_H

/*!***************************************************************************
 * @defgroup 	argussnm Shot Noise Monitor
 * @ingroup		argusdev
 *
 * @brief		Shot Noise Monitor (SNM) parameter definitions and API functions.
 *
 * @details		The SNM is an algorithm to monitor and react on shot noise
 * 				induced by harsh environment conditions like high ambient
 * 				light.
 *
 *				The AFBR-S50 API provides three modes:
 *				- Dynamic: Automatic mode, automatically adopts to current
 *						   ambient conditions.
 *				- Static (Outdoor): Static mode, optimized for outdoor applications.
 *				- Static (Indoor): Static mode, optimized for indoor applications.
 *				.
 *
 * @addtogroup 	argussnm
 * @{
 *****************************************************************************/

/*! The Shot Noise Monitor modes enumeration. */
typedef enum {
	/*! Static Shot Noise Monitoring Mode, optimized for indoor applications.
	 *  Assumes the best case scenario, i.e. no bad influence from ambient conditions.
	 *  Thus it uses a fixed setting that will result in the best performance.
	 *  Equivalent to Shot Noise Monitoring disabled. */
	SNM_MODE_STATIC_INDOOR = 0U,

	/*! Static Shot Noise Monitoring Mode, optimized for outdoor applications.
	 *  Assumes the worst case scenario, i.e. it uses a fixed setting that will
	 *  work under all ambient conditions. */
	SNM_MODE_STATIC_OUTDOOR = 1U,

	/*! Dynamic Shot Noise Monitoring Mode.
	 *  Adopts the system performance dynamically to the current ambient conditions. */
	SNM_MODE_DYNAMIC = 2U,

} argus_snm_mode_t;


/*! @} */
#endif /* ARGUS_SNM_H */
