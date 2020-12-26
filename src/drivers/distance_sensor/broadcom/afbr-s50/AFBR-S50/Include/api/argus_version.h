/*************************************************************************//**
 * @file
 * @brief    	This file is part of the AFBR-S50 API.
 * @details		This file contains the current API version number.
 *
 * @copyright	Copyright c 2016-2019, Avago Technologies GmbH.
 * 				All rights reserved.
 *****************************************************************************/

#ifndef ARGUS_VERSION_H
#define ARGUS_VERSION_H

/*!***************************************************************************
 * @defgroup	version API Version
 * @ingroup		argusapi
 *
 * @brief		API and library core version number
 *
 * @details		Contains the AFBR-S50 API and Library Core Version Number.
 *
 * @addtogroup 	version
 * @{
 *****************************************************************************/

/*! Major version number of the AFBR-S50 API. */
#define ARGUS_API_VERSION_MAJOR    1

/*! Minor version number of the AFBR-S50 API. */
#define ARGUS_API_VERSION_MINOR    2

/*! Bugfix version number of the AFBR-S50 API. */
#define ARGUS_API_VERSION_BUGFIX   3

/*! Build version nunber of the AFBR-S50 API. */
#define ARGUS_API_VERSION_BUILD    "20201120091253"

/*****************************************************************************/

/*! Construct the version number for drivers. */
#define MAKE_VERSION(major, minor, bugfix) \
	(((major) << 24) | ((minor) << 16) | (bugfix))

/*! Version number of the AFBR-S50 API. */
#define ARGUS_API_VERSION MAKE_VERSION((ARGUS_API_VERSION_MAJOR), \
									   (ARGUS_API_VERSION_MINOR), \
									   (ARGUS_API_VERSION_BUGFIX))

/*! @} */
#endif /* ARGUS_VERSION_H */
