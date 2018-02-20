############################################################################
#
# Copyright (c) 2017 PX4 Development Team. All rights reserved.
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

#=============================================================================
#
#	Defined functions in this file
#
# 	utility functions
#
#		* px4_add_board
#

include(common/px4_base)

#=============================================================================
#
#	px4_add_board
#
#	This function creates a PX4 board.
#
#	Usage:
#		px4_add_module(
#			BOARD <string>
#			OS <string>
#			[ TOOLCHAIN ] <string>
#			)
#
#	Input:
#		BOARD			: name of board
#		OS			: posix, nuttx, qurt
#
#
#	Example:
#		px4_add_board(
#			BOARD nuttx_px4fmu-v2_default
#			OS nuttx
#			)
#
function(px4_add_board)

	px4_parse_function_args(
		NAME px4_add_board
		ONE_VALUE BOARD OS ARCH TOOLCHAIN ROMFS ROMFSROOT IO
		REQUIRED BOARD OS
		ARGN ${ARGN})

	set(OS ${OS} CACHE STRING "PX4 OS" FORCE)

	# HWCLASS -> CMAKE_SYSTEM_PROCESSOR
	if(ARCH STREQUAL "cortex-m7")
		set(CMAKE_SYSTEM_PROCESSOR "cortex-m7" PARENT_SCOPE)
	elseif(ARCH STREQUAL "cortex-m4")
		set(CMAKE_SYSTEM_PROCESSOR "cortex-m4" PARENT_SCOPE)
	elseif(ARCH STREQUAL "cortex-m3")
		set(CMAKE_SYSTEM_PROCESSOR "cortex-m3" PARENT_SCOPE)
	endif()
	set(CMAKE_SYSTEM_PROCESSOR ${CMAKE_SYSTEM_PROCESSOR} CACHE INTERNAL "system processor" FORCE)

	if(ARCH MATCHES "cortex-m")
		set(CMAKE_TOOLCHAIN_FILE ${PX4_SOURCE_DIR}/cmake/toolchains/Toolchain-arm-none-eabi.cmake CACHE INTERNAL "toolchain file" FORCE)
	endif()

	# ROMFS
	if("${ROMFS}" STREQUAL "y")
		if (NOT DEFINED ROMFSROOT)
			set(config_romfs_root px4fmu_common)
		else()
			set(config_romfs_root ${ROMFSROOT})
		endif()
		set(config_romfs_root ${config_romfs_root} PARENT_SCOPE)
	endif()

	# IO board placed in ROMFS
	if(config_romfs_root)
		set(config_io_board ${IO} PARENT_SCOPE)
	endif()

endfunction()
