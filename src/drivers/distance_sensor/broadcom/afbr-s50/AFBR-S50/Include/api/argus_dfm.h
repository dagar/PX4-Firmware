/*************************************************************************//**
 * @file
 * @brief    	This file is part of the AFBR-S50 API.
 * @details		Defines the dual frequency mode (DFM) setup parameters.
 * 
 * @copyright	Copyright (c) 2016-2020, Avago Technologies GmbH.
 * 				All rights reserved.
 *****************************************************************************/

#ifndef ARGUS_DFM_H
#define ARGUS_DFM_H

/*!***************************************************************************
 * @defgroup 	argusdfm Dual Frequency Mode
 * @ingroup		argusdev
 *
 * @brief		Dual Frequency Mode (DFM) parameter definitions and API functions.
 *
 * @details		The DFM is an algorithm to extend the unambiguous range of the
 * 				sensor by utilizing two detuned measurement frequencies.
 *
 *				The AFBR-S50 API provides three measurement modes:
 *				- 1X: Single Frequency Measurement
 *				- 4X: Dual Frequency Measurement w/ 4 times the unambiguous
 *				      range of the Single Frequency Measurement
 *				- 8X: Dual Frequency Measurement w/ 8 times the unambiguous
 *				      range of the Single Frequency Measurement
 *
 * @addtogroup 	argusdfm
 * @{
 *****************************************************************************/

/*! The Dual Frequency Mode frequency count. */
#define ARGUS_DFM_FRAME_COUNT (2U)

/*! The Dual Frequency Mode measurement modes count. Excluding the disabled mode. */
#define ARGUS_DFM_MODE_COUNT (2U) // expect off-mode!

/*! The Dual Frequency Mode measurement modes enumeration. */
typedef enum
{
	/*! Single Frequency Measurement Mode (w/ 1x Unambiguous Range). */
	DFM_MODE_OFF = 0U,

	/*! 4X Dual Frequency Measurement Mode (w/ 4x Unambiguous Range). */
	DFM_MODE_4X = 1U,

	/*! 8X Dual Frequency Measurement Mode (w/ 8x Unambiguous Range). */
	DFM_MODE_8X = 2U,

} argus_dfm_mode_t;


/*! @} */
#endif /* ARGUS_DFM_H */
