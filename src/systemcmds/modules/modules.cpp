/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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
 * @file modules.cpp
 *
 * Modules tool.
 */

#include <px4_config.h>
#include <px4_log.h>
#include <px4_module.h>
#include <px4_posix.h>

__BEGIN_DECLS
__EXPORT int modules_main(int argc, char *argv[]);
__END_DECLS

static void print_usage()
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description


)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("modules", "command");

	PRINT_MODULE_USAGE_COMMAND_DESCR("show", "Show parameter values");
	PRINT_MODULE_USAGE_PARAM_FLAG('a', "Show all parameters (not just used)", true);
	PRINT_MODULE_USAGE_PARAM_FLAG('c', "Show only changed and used params", true);
	PRINT_MODULE_USAGE_PARAM_FLAG('q', "quiet mode, print only param value (name needs to be exact)", true);

	PRINT_MODULE_USAGE_COMMAND_DESCR("status", "Print status of parameter system");

	PRINT_MODULE_USAGE_COMMAND_DESCR("stop-all", "Stop all modules");
}

int
modules_main(int argc, char *argv[])
{
	if (argc >= 2) {
		if (!strcmp(argv[1], "status")) {
			modules_status_all();
			return PX4_OK;
		}

		if (!strcmp(argv[1], "stop-all")) {
			modules_stop_all();
			return PX4_OK;
		}
	}

	print_usage();
	return 1;
}
