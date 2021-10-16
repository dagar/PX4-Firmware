
#pragma once


#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>


#include "uORB/topics/uORBTopics.hpp" // ORB_ID enum

//#include "px4/msg/vehicle_status.hpp"

//using vehicle_status_s = px4::msg::VehicleStatus;

//using orb_id_t = void *;




/**
 * Object metadata.
 */
struct orb_metadata {
	const char *o_name;		/**< unique object name */
	const uint16_t o_size;		/**< object size */
	const uint16_t o_size_no_padding;	/**< object size w/o padding at the end (for logger) */
	const char *o_fields;		/**< semicolon separated list of fields (with type) */
	uint8_t o_id;			/**< ORB_ID enum */
};

typedef const struct orb_metadata *orb_id_t;

/**
 * Maximum number of multi topic instances. This must be <= 10 (because it's the last char of the node path)
 */
#if defined(CONSTRAINED_MEMORY)
# define ORB_MULTI_MAX_INSTANCES 4
#else
# define ORB_MULTI_MAX_INSTANCES 10
#endif



#define ORB_ID(_name)		&__orb_##_name

#if defined(__cplusplus)
# define ORB_DECLARE(_name)		extern "C" const struct orb_metadata __orb_##_name __EXPORT
#else
# define ORB_DECLARE(_name)		extern const struct orb_metadata __orb_##_name __EXPORT
#endif


#define ORB_DEFINE(_name, _struct, _size_no_padding, _fields, _orb_id_enum)		\
	const struct orb_metadata __orb_##_name = {	\
		#_name,					\
		sizeof(_struct),		\
		_size_no_padding,			\
		_fields,				\
		_orb_id_enum				\
	}; struct hack


#ifndef PX4_INFO_RAW
# define PX4_INFO_RAW		printf
#endif

// Notes
//  - ORB_ID map to msg T (compile time)
//  - How to check if updated or track generation?


// const orb_metadata *meta
