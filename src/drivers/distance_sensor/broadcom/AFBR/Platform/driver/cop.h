/*************************************************************************//**
 * @file
 * @brief    	This file is part of the AFBR-S50 API.
 * @details		This file provides watchdog driver functionality.
 * 
 * @copyright	Copyright c 2016-2019, Avago Technologies GmbH.
 * 				All rights reserved.
 *****************************************************************************/

#ifndef COP_H
#define COP_H

/*!***************************************************************************
 * @defgroup	COP COP: Computer Operating Properly
 * @ingroup		driver
 * @brief		COP (Computer Operating Properly; Watchdog) Hardware Driver Module
 * @details		Provides driver functionality for watchdog hardware module.
 * @addtogroup 	COP
 * @{
 *****************************************************************************/


 /*!***************************************************************************
  * @brief	Initializes the COP module.
  *****************************************************************************/
void COP_Init(void);


/*!***************************************************************************
 * @brief	Feed the dog.
 *****************************************************************************/
void COP_Refresh(void);


/*!***************************************************************************
 * @brief	Enforce system reset!
 *****************************************************************************/
void COP_ResetSystem(void);


/*!***************************************************************************
 * @brief	Disable the COP timer.
 * @details	COP can only be disabled after startup. It can not be disabled
 * 			after it has been initialized.
 *****************************************************************************/
void COP_Disable(void);


/*! @} */
#endif /* COP_H */
