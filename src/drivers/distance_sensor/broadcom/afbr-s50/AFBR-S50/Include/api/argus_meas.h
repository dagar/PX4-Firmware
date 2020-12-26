/*************************************************************************//**
 * @file
 * @brief    	This file is part of the AFBR-S50 hardware API.
 * @details		Defines the generic measurement parameters and data structures.
 *
 * @copyright	Copyright (c) 2016-2019, Avago Technologies GmbH.
 * 				All rights reserved.
 *****************************************************************************/

#ifndef ARGUS_MEAS_H
#define ARGUS_MEAS_H

/*!***************************************************************************
 * @defgroup 	argusmeas Measurement/Device Control
 * @ingroup		argusapi
 *
 * @brief 		Measurement/Device control module
 *
 * @details		This module contains measurement and device control specific
 * 				definitions and methods.
 *
 * @addtogroup 	argusmeas
 * @{
 *****************************************************************************/

#include "argus_dca.h"
#include "argus_def.h"

/*! Number of raw data values. */
#define ARGUS_RAW_DATA_VALUES 132U // 33 channels * 4 phases

/*! Size of the raw data in bytes. */
#define ARGUS_RAW_DATA_SIZE (3U * ARGUS_RAW_DATA_VALUES) // 3 bytes * 33 channels * 4 phases

/*! The number channels for auxiliary measurements readout. */
#define ARGUS_AUX_CHANNEL_COUNT (5U)

/*! Size of the auxiliary data in bytes. */
#define ARGUS_AUX_DATA_SIZE (3U * ARGUS_AUX_CHANNEL_COUNT) // 3 bytes * x channels * 1 phase

/*!***************************************************************************
 * @brief	The device measurement configuration structure.
 * @details	The portion of the configuration data that belongs to the
 * 			measurement cycle. I.e. the data that defines a measurement frame.
 *****************************************************************************/
typedef struct
{
	/*! ADC channel enabled mask for the first
	 *  channels 0 .. 31 (active pixels channels).
	 *  See [pixel mapping](@ref argusmap) for more
	 *  details on the pixel mask. */
	uint32_t PxEnMask;

	/*! ADS channel enabled mask for the remaining
	 *  channels 31 .. 63 (miscellaneous values).
	 *  See [pixel mapping](@ref argusmap) for more
	 *  details on the channel mask. */
	uint32_t ChEnMask;

	/*! Pattern count per sample in uq10.6 format.
	 *  Determines the analog integration depth. */
	uq10_6_t AnalogIntegrationDepth;

	/*! Sample count per phase/frame.
	 *  Determines the digital integration depth. */
	uint16_t DigitalIntegrationDepth;

	/*! Laser current per sample in mA.
	 *  Determines the optical output power. */
	uq12_4_t OutputPower;

	/*! Charge pump voltage per sample in LSB.
	 *  Determines the pixel gain.  */
	uint8_t PixelGain;

	/*! PLL Frequency Offset, caused by temperature
	 *  compensation, in PLL_INT_PRD LSBs. */
	int8_t PllOffset;

	/*! The current state of the measurement frame:
	 *  - Measurement Mode,
	 *  - A/B Frame,
	 *  - PLL_Locked Bit,
	 *  - BGL Warning/Error,
	 *  - DCA State,
	 *  - ... */
	argus_state_t State;

} argus_meas_frame_t;

/*! @} */
#endif /* ARGUS_MEAS_H */
