

#pragma once

#include <ReaderWriterLock.hpp>
#include <lib/perf/perf_counter.h>

#include <systemlib/uthash/utarray.h>

class ParameterBackend
{

public:

	ParameterBackend();
	~ParameterBackend();

	unsigned	param_count();
	unsigned	param_count_used();
	param_t		param_find(const char *name, bool notification);


#if !defined(PARAM_NO_ORB)
	void notify_changes();
#endif // !PARAM_NO_ORB

private:

	// the following implements an RW-lock using 2 semaphores (used as mutexes). It gives
	// priority to readers, meaning a writer could suffer from starvation, but in our use-case
	// we only have short periods of reads and writes are rare.
	ReaderWriterLock		_rwlock;

	bitset<param_info_count>	_params_active;

	UT_array *			_param_values{nullptr};	/** flexible array holding modified parameter values */



	perf_counter_t _param_export_perf;
	perf_counter_t _param_find_perf;
	perf_counter_t _param_get_perf;
	perf_counter_t _param_set_perf;


#if !defined(PARAM_NO_ORB)
	/** parameter update topic handle */
	orb_advert_t	_param_topic{nullptr};
	unsigned	_param_instance{0};
#endif // !PARAM_NO_ORB

};
