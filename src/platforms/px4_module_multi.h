/****************************************************************************
 *
 *   Copyright (c) 2017-2019 PX4 Development Team. All rights reserved.
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

/**
 * @file px4_module.h
 */

#pragma once

#include <px4_module.h>

template<class T, size_t MAX_INSTANCES>
class ModuleMultiBase : public ModuleBaseInterface
{
public:
	ModuleMultiBase() : ModuleBaseInterface(get_name_static(), _instance_count)
	{
		// track number of instances
		_instance_count++;
	}

	virtual ~ModuleMultiBase()
	{
		// track number of instances
		_instance_count--;
	}

#if defined(MODULE_NAME)
	static constexpr const char *get_name_static() { return MODULE_NAME; }
#endif // MODULE_NAME

	/**
	 * @brief main Main entry point to the module that should be
	 *        called directly from the module's main method.
	 * @param argc The task argument count.
	 * @param argc Pointer to the task argument variable array.
	 * @return Returns 0 iff successful, -1 otherwise.
	 */
	static int main(int argc, char *argv[])
	{
		if (argc <= 1 ||
		    strcmp(argv[1], "-h")    == 0 ||
		    strcmp(argv[1], "help")  == 0 ||
		    strcmp(argv[1], "info")  == 0 ||
		    strcmp(argv[1], "usage") == 0) {

			return T::print_usage();
		}

		// TODO:
		//  --instance or -i
		// eg
		//   start --instance 1

		if (strcmp(argv[1], "start") == 0) {
			// Pass the 'start' argument too, because later on px4_getopt() will ignore the first argument.
			return start_command_base(argc - 1, argv + 1);
		}

		if ((strcmp(argv[1], "status") == 0) || (strcmp(argv[1], "status-all") == 0)) {
			for (size_t i = 0; i < MAX_INSTANCES; i++) {
				PX4_INFO("status: %zu/%zu", i, MAX_INSTANCES);
				status_command(i);
			}

			return PX4_OK;
		}

		if ((strcmp(argv[1], "stop")  == 0) || (strcmp(argv[1], "stop-all") == 0)) {
			for (size_t i = 0; i < MAX_INSTANCES; i++) {
				PX4_INFO("stopping: %zu/%zu", i, MAX_INSTANCES);
				stop_command(i);
			}

			return PX4_OK;
		}

		lock_module(); // Lock here, as the method could access _object.
		int ret = T::custom_command(argc - 1, argv + 1);
		unlock_module();

		return ret;
	}

	/**
	 * @brief Entry point for px4_task_spawn_cmd() if the module runs in its own thread.
	 *        It does:
	 *        - instantiate the object
	 *        - call run() on it to execute the main loop
	 *        - cleanup: delete the object
	 * @param argc The task argument count.
	 * @param argc Pointer to the task argument variable array.
	 * @return Returns 0 iff successful, -1 otherwise.
	 */
	static int run_trampoline(int argc, char *argv[])
	{
		int ret = 0;

#ifdef __PX4_NUTTX
		// On NuttX task_create() adds the task name as first argument.
		argc -= 1;
		argv += 1;
#endif

		T *object = T::instantiate(argc, argv);

		if (object) {
			object->set_task_id(px4_getpid());

			// run
			object->run();

		} else {
			PX4_ERR("failed to instantiate object");
			ret = -1;
		}

		exit_and_cleanup(0); // TODO: dagar fix

		return ret;
	}

	/**
	 * @brief Stars the command, ('command start'), checks if if is already
	 *        running and calls T::task_spawn() if it's not.
	 * @param argc The task argument count.
	 * @param argc Pointer to the task argument variable array.
	 * @return Returns 0 iff successful, -1 otherwise.
	 */
	static int start_command_base(int argc, char *argv[])
	{
		int ret = 0;
		lock_module();

		// TODO: handle command line specified instances
		//  module decides instance slot?
		if (is_running(0) && false) {
			ret = -1;
			int instance = 0;
			PX4_ERR("Task %d already running", instance);

		} else {
			ret = T::task_spawn(argc, argv);

			if (ret < 0) {
				PX4_ERR("Task start failed (%i)", ret);
			}
		}

		unlock_module();
		return ret;
	}

	/**
	 * @brief Stops the command, ('command stop'), checks if it is running and if it is, request the module to stop
	 *        and waits for the task to complete.
	 * @return Returns 0 iff successful, -1 otherwise.
	 */
	static int stop_command(uint8_t instance) { return module_stop(get_name_static(), instance); }

	/**
	 * @brief Handle 'command status': check if running and call print_status() if it is
	 * @return Returns 0 iff successful, -1 otherwise.
	 */
	static int status_command(uint8_t instance) { return module_status(get_name_static(), instance); }

	/**
	 * @brief Main loop method for modules running in their own thread. Called from run_trampoline().
	 *        This method must return when should_exit() returns true.
	 */
	virtual void run() {}

	/**
	 * @brief Returns the status of the module.
	 * @return Returns true if the module is running, false otherwise.
	 */
	static bool is_running(uint8_t instance) { return module_running(get_name_static(), instance); }

protected:

	/**
	 * @brief Exits the module and delete the object. Called from within the module's thread.
	 *        For work queue modules, this needs to be called from the derived class in the
	 *        cycle method, when should_exit() returns true.
	 */
	static void exit_and_cleanup(uint8_t instance) { module_exit_and_cleanup(get_name_static(), instance); }

	/**
	 * @brief Waits until _object is initialized, (from the new thread). This can be called from task_spawn().
	 * @return Returns 0 iff successful, -1 on timeout or otherwise.
	 */
	static int wait_until_running(uint8_t instance) { return module_wait_until_running(get_name_static(), instance); }

	/**
	 * @brief Get the module's object instance, (this is null if it's not running).
	 */
	static T *get_instance(uint8_t instance) { return (T *)get_module_instance(get_name_static(), instance); }

	static uint8_t _instance_count;

	static uint8_t instance_count() { return _instance_count; }

};

template<class T, size_t MAX_INSTANCES>
uint8_t ModuleMultiBase<T, MAX_INSTANCES>::_instance_count = 0;
