

#include "ParameterBackend.hpp"


static constexpr uint16_t param_info_count = sizeof(px4::parameters) / sizeof(param_info_s);

// Storage for modified parameters.
struct param_wbuf_s {
	union param_value_u	val;
	param_t			param;
	bool			unsaved;
};

/** array info for the modified parameters array */
const UT_icd param_icd = {sizeof(param_wbuf_s), nullptr, nullptr, nullptr};


/**
 * Test whether a param_t is value.
 *
 * @param param			The parameter handle to test.
 * @return			True if the handle is valid.
 */
static constexpr bool
handle_in_range(param_t param)
{
	return (param < param_info_count);
}


/**
 * Compare two modified parameter structures to determine ordering.
 *
 * This function is suitable for passing to qsort or bsearch.
 */
static int
param_compare_values(const void *a, const void *b)
{
	struct param_wbuf_s *pa = (struct param_wbuf_s *)a;
	struct param_wbuf_s *pb = (struct param_wbuf_s *)b;

	if (pa->param < pb->param) {
		return -1;
	}

	if (pa->param > pb->param) {
		return 1;
	}

	return 0;
}


/** assert that the parameter store is locked */
static void
param_assert_locked()
{
	/* XXX */
}

/**
 * Locate the modified parameter structure for a parameter, if it exists.
 *
 * @param param			The parameter being searched.
 * @return			The structure holding the modified value, or
 *				nullptr if the parameter has not been modified.
 */
static param_wbuf_s *
param_find_changed(param_t param)
{
	param_wbuf_s *s = nullptr;

	param_assert_locked();

	if (params_active[param]) {

		if (param_values != nullptr) {
			param_wbuf_s key{};
			key.param = param;
			s = (param_wbuf_s *)utarray_find(param_values, &key, param_compare_values);
		}
	}

	return s;
}

void
ParameterBackend::notify_changes()
{
#if !defined(PARAM_NO_ORB)
	parameter_update_s pup = {};
	pup.timestamp = hrt_absolute_time();
	pup.instance = param_instance++;

	/*
	 * If we don't have a handle to our topic, create one now; otherwise
	 * just publish.
	 */
	if (param_topic == nullptr) {
		param_topic = orb_advertise(ORB_ID(parameter_update), &pup);

	} else {
		orb_publish(ORB_ID(parameter_update), param_topic, &pup);
	}

#endif
}

unsigned
ParameterBackend::param_count()
{
	return param_info_count;
}

unsigned
ParameterBackend::param_count_used()
{
	return _params_active.count();
}

param_t
ParameterBackend::param_find(const char *name, bool notification)
{
	perf_begin(param_find_perf);

	param_t middle;
	param_t front = 0;
	param_t last = param_info_count;

	/* perform a binary search of the known parameters */

	while (front <= last) {
		middle = front + (last - front) / 2;
		int ret = strcmp(name, param_name(middle));

		if (ret == 0) {
			if (notification) {
				param_set_used(middle);
			}

			perf_end(param_find_perf);
			return middle;

		} else if (middle == front) {
			/* An end point has been hit, but there has been no match */
			break;

		} else if (ret < 0) {
			last = middle;

		} else {
			front = middle;
		}
	}

	perf_end(param_find_perf);

	/* not found */
	return PARAM_INVALID;
}





ParameterBackend::ParameterBackend()
{
	_param_export_perf = perf_alloc(PC_ELAPSED, "param_export");
	_param_find_perf = perf_alloc(PC_ELAPSED, "param_find");
	_param_get_perf = perf_alloc(PC_ELAPSED, "param_get");
	_param_set_perf = perf_alloc(PC_ELAPSED, "param_set");
}

ParameterBackend::~ParameterBackend()
{
	perf_free(_param_export_perf);
	perf_free(_param_find_perf);
	perf_free(_param_get_perf);
	perf_free(_param_set_perf);
}
