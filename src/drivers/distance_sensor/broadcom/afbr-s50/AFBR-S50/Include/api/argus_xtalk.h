/*************************************************************************//**
 * @file
 * @brief    	This file is part of the AFBR-S50 hardware API.
 * @details		Defines the generic device calibration API.
 *
 * @copyright	Copyright (c) 2016-2020, Avago Technologies GmbH.
 * 				All rights reserved.
 *****************************************************************************/

#ifndef ARGUS_XTALK_H
#define ARGUS_XTALK_H

/*!***************************************************************************
 * @addtogroup 	arguscal
 * @{
 *****************************************************************************/

#include "api/argus_def.h"

/*!***************************************************************************
 * @brief	Pixel Crosstalk Compensation Vector.
 * @details	Contains calibration data (per pixel) that belongs to the
 * 			RX-TX-Crosstalk compensation feature.
 *****************************************************************************/

/*! Pixel Crosstalk Vector */
typedef struct
{
	/*! Crosstalk Vector - Sine component.
	 *  Special Value: Q11_4_MIN == not available */
	q11_4_t dS;

	/*! Crosstalk Vector - Cosine component.
	 *  Special Value: Q11_4_MIN == not available */
	q11_4_t dC;

} xtalk_t;

/*!***************************************************************************
 * @brief	Pixel-To-Pixel Crosstalk Compensation Parameters.
 * @details	Contains calibration data that belongs to the pixel-to-pixel
 * 			crosstalk compensation feature.
 *****************************************************************************/
typedef struct
{
	/*! Pixel-To-Pixel Compensation on/off. */
	bool Enabled;

	/*! The relative threshold determines when the compensation is active for
	 *  each individual pixel. The value determines the ratio of the individual
	 *  pixel signal is with respect to the overall average signal. If the
	 *  ratio is smaller than the value, the compensation is active. Absolute
	 *  and relative conditions are connected with AND logic. */
	uq0_8_t RelativeThreshold;

	/*! The absolute threshold determines the minimum total crosstalk
	 *  amplitude (i.e. the average amplitude of all pixels weighted by
	 *  the Kc factor) that is required for the compensation to become
	 *  active. Set to 0 to always enable. Absolute and relative
	 *  conditions are connected with AND logic. */
	uq12_4_t AbsoluteTreshold;

	/*! The sine component of the Kc factor that determines the amount of the total
	 *  signal of all pixels that influences the individual signal of each pixel.
	 *  Higher values determine more influence on the individual pixel signal. */
	q3_12_t KcFactorS;

	/*! The cosine component of the Kc factor that determines the amount of the total
	 *  signal of all pixels that influences the individual signal of each pixel.
	 *  Higher values determine more influence on the individual pixel signal. */
	q3_12_t KcFactorC;

	/*! The sine component of the reference pixel Kc factor that determines the
	 *  amount of the total signal on all pixels that influences the individual
	 *  signal of the reference pixel.
	 *  Higher values determine more influence on the reference pixel signal. */
	q3_12_t KcFactorSRefPx;

	/*! The cosine component of the reference pixel Kc factor that determines the
	 *  amount of the total signal on all pixels that influences the individual
	 *  signal of the reference pixel.
	 *  Higher values determine more influence on the reference pixel signal. */
	q3_12_t KcFactorCRefPx;

} argus_cal_p2pxtalk_t;


/*! @} */
#endif /* ARGUS_XTALK_H */
