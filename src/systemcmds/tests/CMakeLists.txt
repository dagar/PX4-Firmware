############################################################################
#
#   Copyright (c) 2015 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

set(EXTRA_SRCS)
if(${PX4_PLATFORM} STREQUAL "nuttx")
	list(APPEND EXTRA_SRCS
		test_time.cpp
		test_uart_break.cpp
		)
endif()

px4_add_module(
	MODULE systemcmds__tests
	MAIN tests
	STACK_MAIN 10000
	COMPILE_FLAGS
		-Wno-double-promotion
		-Wno-float-equal
		-Wno-missing-declarations
		-Wno-sign-compare # TODO: fix all sign-compare
		-Wno-unknown-warning-option
		-Wno-unused-but-set-variable
		-Wno-unused-result
		-Wno-unused-variable
		-fno-permissive
	SRCS
		test_adc.cpp
		test_autodeclination.cpp
		test_bezierQuad.cpp
		test_bson.cpp
		test_conv.cpp
		test_dataman.cpp
		test_file.cpp
		test_file2.cpp
		test_float.cpp
		test_hott_telemetry.cpp
		test_hrt.cpp
		test_int.cpp
		test_IntrusiveQueue.cpp
		test_jig_voltages.cpp
		test_led.cpp
		test_List.cpp
		test_mathlib.cpp
		test_matrix.cpp
		test_microbench_hrt.cpp
		test_microbench_math.cpp
		test_microbench_matrix.cpp
		test_microbench_uorb.cpp
		test_mixer.cpp
		test_mount.cpp
		test_param.cpp
		test_parameters.cpp
		test_perf.cpp
		test_ppm.cpp
		test_ppm_loopback.cpp
		test_rc.cpp
		test_search_min.cpp
		test_servo.cpp
		test_sleep.cpp
		test_smooth_z.cpp
		test_tone.cpp
		test_uart_baudchange.cpp
		test_uart_console.cpp
		test_uart_loopback.cpp
		test_uart_send.cpp
		test_versioning.cpp
		tests_main.cpp
		${EXTRA_SRCS}
	DEPENDS
		git_ecl
		ecl_geo_lookup # TODO: move this
		output_limit
		version
	)

add_subdirectory(hrt_test)
