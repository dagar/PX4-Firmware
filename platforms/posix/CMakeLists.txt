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

add_definitions(
	-D__PX4_POSIX
	-Dnoreturn_function=__attribute__\(\(noreturn\)\)
	)

include_directories(platforms/posix/include)

if ("${PX4_BOARD}" MATCHES "sitl")

	if(UNIX AND APPLE)
		add_definitions(
			-D__PX4_DARWIN
			-D__DF_DARWIN
			)

		if (CMAKE_CXX_COMPILER_VERSION VERSION_LESS 8.0)
			message(FATAL_ERROR "PX4 Firmware requires XCode 8 or newer on Mac OS. Version installed on this system: ${CMAKE_CXX_COMPILER_VERSION}")
		endif()

		execute_process(COMMAND uname -v OUTPUT_VARIABLE DARWIN_VERSION)
		string(REGEX MATCH "[0-9]+" DARWIN_VERSION ${DARWIN_VERSION})
		# message(STATUS "PX4 Darwin Version: ${DARWIN_VERSION}")
		if (DARWIN_VERSION LESS 16)
			add_definitions(
				-DCLOCK_MONOTONIC=1
				-DCLOCK_REALTIME=0
				-D__PX4_APPLE_LEGACY
				)
		endif()

	elseif(CYGWIN)
		add_definitions(
			-D__PX4_CYGWIN
			-D_GNU_SOURCE
			-D__USE_LINUX_IOCTL_DEFS
			-U__CUSTOM_FILE_IO__
			)
	else()
		add_definitions(
			-D__PX4_LINUX
			-D__DF_LINUX
			)
	endif()

elseif (("${PX4_BOARD}" MATCHES "navio2") OR ("${PX4_BOARD}" MATCHES "raspberrypi"))

	#TODO: move to board support

	add_definitions(
		-D__PX4_LINUX

		# For DriverFramework
		-D__DF_LINUX
		-D__DF_RPI
	)

elseif ("${PX4_BOARD}" MATCHES "bebop")

	#TODO: move to board support

	add_definitions(
		-D__PX4_LINUX
		-D__PX4_POSIX_BEBOP # TODO: remove

		# For DriverFramework
		-D__DF_LINUX
		-D__DF_BEBOP
	)

elseif ("${PX4_BOARD}" MATCHES "aerotenna_ocpoc")

	#TODO: move to board support

	add_definitions(
		-D__PX4_LINUX
		-D__PX4_POSIX_OCPOC # TODO: remove

		# For DriverFramework
		-D__DF_LINUX
		-D__DF_OCPOC
	)

elseif ("${PX4_BOARD}" MATCHES "beaglebone_blue")
	#TODO: move to board support
	add_definitions(
		-D__PX4_LINUX
		-D__PX4_POSIX_BBBLUE # TODO: remove

		# For DriverFramework
		-D__DF_LINUX
		-D__DF_BBBLUE
		-D__DF_BBBLUE_USE_RC_BMP280_IMP # optional

		-DRC_AUTOPILOT_EXT  # Enable extensions in Robotics Cape Library, TODO: remove
	)

	set(LIBROBOTCONTROL_INSTALL_DIR $ENV{LIBROBOTCONTROL_INSTALL_DIR})

	# On cross compile host system and native build system:
	#   a) select and define LIBROBOTCONTROL_INSTALL_DIR environment variable so that
	#      other unwanted headers will not be included
	#   b) install robotcontrol.h and rc/* into $LIBROBOTCONTROL_INSTALL_DIR/include
	#   c) install pre-built native (ARM) version of librobotcontrol.* into $LIBROBOTCONTROL_INSTALL_DIR/lib
	add_compile_options(-I${LIBROBOTCONTROL_INSTALL_DIR}/include)

	set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -L${LIBROBOTCONTROL_INSTALL_DIR}/lib")

endif()
