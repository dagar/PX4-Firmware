/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#pragma once

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/atomic_bitset.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/sem.h>
#include <px4_platform_common/shutdown.h>
#include <containers/Bitset.hpp>
#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>


#include <lib/tinybson/tinybson.h>

#include "param.h"
#include <parameters/px4_parameters.hpp>

#include "ExhaustiveLayer.h"
#include "ConstLayer.h"
#include "DynamicSparseLayer.h"
#include "StaticSparseLayer.h"

#include "atomic_transaction.h"

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/actuator_armed.h>


#include <uORB/topics/parameter_update.h>
#include <uORB/topics/srv_parameter_get_request.h>
#include <uORB/topics/srv_parameter_get_response.h>
#include <uORB/topics/srv_parameter_set_request.h>
#include <uORB/topics/srv_parameter_set_response.h>



using namespace time_literals;

class ParameterServer : public px4::ScheduledWorkItem
{
public:
	ParameterServer();
	~ParameterServer() override;

	/**
	 * Look up a parameter by name.
	 *
	 * @param name		The canonical name of the parameter being looked up.
	 * @return		A handle to the parameter, or PARAM_INVALID if the parameter does not exist.
	 */
	param_t findParameter(const char *name, bool notification = true);

	/**
	 * Return the total number of parameters.
	 *
	 * @return		The number of parameters.
	 */
	unsigned count() const { return param_info_count; }

	/**
	 * Return the actually used number of parameters.
	 *
	 * @return		The number of parameters.
	 */
	unsigned countUsed() const { return _params_active.count(); }

	/**
	 * Wether a parameter is in use in the system.
	 *
	 * @return		True if it has been written or read
	 */
	bool isParameterUsed(param_t param) const;

	/**
	 * Mark a parameter as used. Only marked parameters will be sent to a GCS.
	 * A call to param_find() will mark a param as used as well.
	 *
	 * @param param		A handle returned by param_find or passed by param_foreach.
	 */
	bool setParameterUsed(param_t param);

	/**
	 * Look up an used parameter by index.
	 *
	 * @param index		The parameter to obtain the index for.
	 * @return		The index of the parameter in use, or -1 if the parameter does not exist.
	 */
	param_t forUsedIndex(unsigned index) const;

	/**
	 * Look up the index of a parameter.
	 *
	 * @param param		The parameter to obtain the index for.
	 * @return		The index, or -1 if the parameter does not exist.
	 */
	int getParameterIndex(param_t param) const;

	/**
	 * Look up the index of an used parameter.
	 *
	 * @param param		The parameter to obtain the index for.
	 * @return		The index of the parameter in use, or -1 if the parameter does not exist.
	 */
	int getParameterUsedIndex(param_t param) const;

	/**
	 * Obtain the name of a parameter.
	 *
	 * @param param		A handle returned by param_find or passed by param_foreach.
	 * @return		The name assigned to the parameter, or NULL if the handle is invalid.
	 */
	const char *getParameterName(param_t param) const;

	/**
	 * Obtain the volatile state of a parameter.
	 *
	 * @param param		A handle returned by param_find or passed by param_foreach.
	 * @return			true if the parameter is volatile
	 */
	bool isParameterVolatile(param_t param) const;

	/**
	 * Test whether a parameter's value has changed from the default.
	 *
	 * @return		If true, the parameter's value has not been changed from the default.
	 */
	bool isParameterValueDefault(param_t param);

	/**
	 * Test whether a parameter's value has been changed but not saved.
	 *
	 * @return		If true, the parameter's value has not been saved.
	 */
	bool isParameterValueUnsaved(param_t param);

	/**
	 * Obtain the type of a parameter.
	 *
	 * @param param		A handle returned by param_find or passed by param_foreach.
	 * @return		The type assigned to the parameter.
	 */
	param_type_t getParameterType(param_t param) const;

	/**
	 * Determine the size of a parameter.
	 *
	 * @param param		A handle returned by param_find or passed by param_foreach.
	 * @return		The size of the parameter's value.
	 */
	size_t getParameterSize(param_t param) const;

	/**
	 * Copy the value of a parameter.
	 *
	 * @param param		A handle returned by param_find or passed by param_foreach.
	 * @param val		Where to return the value, assumed to point to suitable storage for the parameter type.
	 * @return		Zero if the parameter's value could be returned, nonzero otherwise.
	 */
	int getParameterValue(param_t param, void *val);

	/**
	 * Copy the (airframe-specific) default value of a parameter.
	 *
	 * @param param		A handle returned by param_find or passed by param_foreach.
	 * @param default_val	Where to return the value, assumed to point to suitable storage for the parameter type.
	 * @return		Zero if the parameter's deafult value could be returned, nonzero otherwise.
	 */
	int getParameterDefaultValue(param_t param, void *default_val);

	/**
	 * Copy the system-wide default value of a parameter.
	 *
	 * @param param		A handle returned by param_find or passed by param_foreach.
	 * @param default_val	Where to return the value, assumed to point to suitable storage for the parameter type.
	 * @return		Zero if the parameter's deafult value could be returned, nonzero otherwise.
	 */
	int getParameterSystemDefaultValue(param_t param, void *default_val);

	/**
	 * Set the value of a parameter.
	 *
	 * @param param		A handle returned by param_find or passed by param_foreach.
	 * @param val		The value to set; assumed to point to a variable of the parameter type.
	 * @return		Zero if the parameter's value could be set from a scalar, nonzero otherwise.
	 */
	int setParameter(param_t param, const void *val, bool mark_saved = true, bool notify_changes = true);

	/**
	 * Set the default value of a parameter.
	 *
	 * @param param		A handle returned by param_find or passed by param_foreach.
	 * @param val		The default value to set; assumed to point to a variable of the parameter type.
	 * @return		Zero if the parameter's default value could be set from a scalar, nonzero otherwise.
	 */
	int setParameterDefaultValue(param_t param, const void *val);

	/**
	 * Notify the system about parameter changes. Can be used for example after several calls to
	 * param_set_no_notification() to avoid unnecessary system notifications.
	 */
	void notifyChanges();

	/**
	 * Reset a parameter to its default value.
	 *
	 * @param param		A handle returned by param_find or passed by param_foreach.
	 * @return		Zero on success, nonzero on failure
	 */
	int resetParameter(param_t param, bool notify = true, bool autosave = true);

	/**
	 * Reset all parameters to their default values.
	 */
	void resetAllParameters(bool auto_save = true);

	/**
	 * Reset all parameters to their default values except for excluded parameters.
	 *
	 * @param excludes			Array of param names to exclude from resetting. Use a wildcard
	 *							at the end to exclude parameters with a certain prefix.
	 * @param num_excludes		The number of excludes provided.
	 */
	void resetExcludes(const char *excludes[], int num_excludes);

	/**
	 * Reset only specific parameters to their default values.
	 *
	 * @param resets Array of param names to reset. Use a wildcard at the end to reset parameters with a certain prefix.
	 * @param num_resets The number of passed reset conditions in the resets array.
	 */
	void resetSpecificParameter(const char *resets[], int num_resets);

	/**
	 * Export changed parameters to a file.
	 * Note: this method requires a large amount of stack size!
	 *
	 * @param filename	Path to the default parameter file.
	 * @param filter	Filter parameters to be exported. The method should return true if
	 * 			the parameter should be exported. No filtering if nullptr is passed.
	 * @return		Zero on success, nonzero on failure.
	 */
	typedef bool(*param_filter_func)(param_t handle);
	int exportToFile(const char *filename, param_filter_func filter);

	/**
	 * Import parameters from a file, discarding any unrecognized parameters.
	 *
	 * This function merges the imported parameters with the current parameter set.
	 *
	 * @param fd		File descriptor to import from (-1 selects the FLASH storage).
	 * @return		Zero on success, nonzero if an error occurred during import.
	 *			Note that in the failure case, parameters may be inconsistent.
	 */
	int importFromFileDescriptor(int fd);

	/**
	 * Load parameters from a file.
	 *
	 * This function resets all parameters to their default values, then loads new
	 * values from a file.
	 *
	 * @param fd		File descriptor to import from (-1 selects the FLASH storage).
	 * @return		Zero on success, nonzero if an error occurred during import.
	 *			Note that in the failure case, parameters may be inconsistent.
	 */
	int loadFromFileDescriptor(int fd);

	/**
	 * Apply a function to each parameter.
	 *
	 * Note that the parameter set is not locked during the traversal. It also does
	 * not hold an internal state, so the callback function can block or sleep between
	 * parameter callbacks.
	 *
	 * @param func		The function to invoke for each parameter.
	 * @param arg		Argument passed to the function.
	 * @param only_changed	If true, the function is only called for parameters whose values have
	 *			been changed from the default.
	 * @param only_used	If true, the function is only called for parameters which have been
	 *			used in one of the running applications.
	 */
	void forEachParameter(void (*func)(void *arg, param_t param), void *arg, bool only_changed, bool only_used);

	/**
	 * Set the default parameter file name.
	 * This has no effect if the FLASH-based storage is enabled.
	 *
	 * @param filename	Path to the default parameter file.  The file is not required to
	 *			exist.
	 * @return		Zero on success.
	 */
	int setDefaultFile(const char *filename);

	/**
	 * Get the default parameter file name.
	 *
	 * @return		The path to the current default parameter file; either as
	 *			a result of a call to param_set_default_file, or the
	 *			built-in default.
	 */
	const char *getDefaultFile() const { return _param_default_file; }

	/**
	 * Set the backup parameter file name.
	 *
	 * @param filename	Path to the backup parameter file. The file is not required to
	 *			exist.
	 * @return		Zero on success.
	 */
	int setBackupFile(const char *filename);

	/**
	 * Get the backup parameter file name.
	 *
	 * @return		The path to the backup parameter file
	 */
	const char *getBackupFile() const { return _param_backup_file; }

	/**
	 * Save parameters to the default file.
	 * Note: this method requires a large amount of stack size!
	 *
	 * This function saves all parameters with non-default values.
	 *
	 * @return		Zero on success.
	 */
	int saveDefault(bool blocking);

	/**
	 * Load parameters from the default parameter file.
	 *
	 * @return		Zero on success.
	 */
	int loadDefault();

	/**
	 * Generate the hash of all parameters and their values
	 *
	 * @return		CRC32 hash of all param_ids and values
	 */
	uint32_t hashCheck();

	/**
	 * Print the status of the param system
	 *
	 */
	void printStatus();

	/**
	 * Enable/disable the param autosaving.
	 * Re-enabling with changed params will not cause an autosave.
	 * @param enable true: enable autosaving, false: disable autosaving
	 */
	void controlAutosave(bool enable);

private:

	static constexpr uint16_t param_info_count = sizeof(px4::parameters) / sizeof(param_info_s);

	/**
	 * Test whether a param_t is value.
	 *
	 * @param param			The parameter handle to test.
	 * @return			True if the handle is valid.
	 */
	static constexpr bool handle_in_range(param_t param) { return (param < param_info_count); }

	/**
	 * Automatically save the parameters after a timeout and limited rate.
	 *
	 * This needs to be called with the writer lock held (it's not necessary that it's the writer lock, but it
	 * needs to be the same lock as autosave_worker() and param_control_autosave() use).
	 */
	void autoSave(bool now = false);

	// internal parameter export, caller is responsible for locking
	int exportInternal(int fd, param_filter_func filter);

	int bsonImportCallback(bson_decoder_t decoder, bson_node_t node);
	static int importCallbackTrampoline(bson_decoder_t decoder, void *priv, bson_node_t node);
	int importFromFileDescriptorInternal(int fd);

	int verifyBsonExportCallback(bson_decoder_t decoder, bson_node_t node);
	static int verifyBsonExportTrampoline(bson_decoder_t decoder, void *priv, bson_node_t node);
	int verifyBsonExport(int fd);


	char *_param_default_file{nullptr};
	char *_param_backup_file{nullptr};

	px4::AtomicBitset<param_info_count> _params_active;  // params found
	px4::AtomicBitset<param_info_count> _params_unsaved;

	ConstLayer _firmware_defaults;
	DynamicSparseLayer _runtime_defaults{&_firmware_defaults};
	DynamicSparseLayer _user_config{&_runtime_defaults};

	perf_counter_t _export_perf{perf_alloc(PC_ELAPSED, "param: export")};
	perf_counter_t _find_count_perf{perf_alloc(PC_COUNT, "param: find")};
	perf_counter_t _get_count_perf{perf_alloc(PC_COUNT, "param: get")};
	perf_counter_t _set_perf{perf_alloc(PC_ELAPSED, "param: set")};


	pthread_mutex_t _file_mutex =
		PTHREAD_MUTEX_INITIALIZER; ///< this protects against concurrent param saves (file or flash access).


	unsigned int _param_instance{0};

	void Run() override;

	uORB::Publication<parameter_update_s> _parameter_update_pub{ORB_ID(parameter_update)};

	// srv: parameter_get
	uORB::SubscriptionCallbackWorkItem _srv_parameter_get_request_sub{this, ORB_ID(srv_parameter_get_request)};
	uORB::Publication<srv_parameter_get_response_s> _srv_parameter_get_response_pub{ORB_ID(srv_parameter_get_response)};

	// srv: parameter_set
	uORB::SubscriptionCallbackWorkItem _srv_parameter_set_request_sub{this, ORB_ID(srv_parameter_set_request)};
	uORB::Publication<srv_parameter_set_response_s> _srv_parameter_set_response_pub{ORB_ID(srv_parameter_set_response)};


	uORB::SubscriptionData<actuator_armed_s> _armed_sub{ORB_ID(actuator_armed)};


	// autosaving variables
	hrt_abstime _last_autosave_timestamp{0};
	px4::atomic_bool _autosave_scheduled{false};
	bool _autosave_disabled{false};
	int _autosave_retry_count{0};

	px4::atomic_bool _notify_scheduled{false};

};
