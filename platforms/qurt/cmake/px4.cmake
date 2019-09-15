############################################################################
#
# Copyright (c) 2015-2019 PX4 Development Team. All rights reserved.
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

list(APPEND CMAKE_MODULE_PATH "${PX4_SOURCE_DIR}/boards/atlflight/cmake_hexagon")
include(toolchain/Toolchain-qurt)
include(fastrpc)
include(qurt_lib)
include(qurt_flags)

include_directories(${CMAKE_CURRENT_BINARY_DIR})

get_property(module_libraries GLOBAL PROPERTY PX4_MODULE_LIBRARIES)

px4_qurt_generate_builtin_commands(
	OUT ${PX4_BINARY_DIR}/apps
	MODULE_LIST ${module_libraries}
)

FASTRPC_STUB_GEN(px4muorb.idl)

add_definitions(-D__QAIC_SKEL_EXPORT=__EXPORT)

# Enable build without HexagonSDK to check link dependencies
if ("${QURT_ENABLE_STUBS}" STREQUAL "1")

	add_definitions(-D QURT_EXE_BUILD=1)

	include_directories(
                ${CMAKE_CURRENT_BINARY_DIR}
                ${FASTRPC_DSP_INCLUDES}
                )

	add_executable(px4
		${PX4_BINARY_DIR}/apps.cpp
		${PX4_BINARY_DIR}/platforms/qurt/px4muorb_skel.c
	)

	target_link_libraries(px4 PRIVATE ${module_libraries} ${df_driver_libs})

else()
	# Generate the DSP lib and stubs but not the apps side executable
	# The Apps side executable is generated via the posix_eagle_xxxx target
	QURT_LIB(LIB_NAME px4
		IDL_NAME px4muorb
		SOURCES
			${PX4_BINARY_DIR}/apps.cpp
		LINK_LIBS
			modules__muorb__adsp
			${module_libraries}
			${df_driver_libs}
			m
		)

endif()
