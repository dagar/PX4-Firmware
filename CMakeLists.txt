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
# CMAKE CODING STANDARD FOR PX4
#
# Structure
# ---------------------------------------------------------------------------
#
# * Common functions should be included in px_base.cmake.
#
# * OS/ board specific fucntions should be include in
#	px_impl_${OS}.cmake or px4_impl_${OS}_${BOARD}.cmake.
#
# Formatting
# ---------------------------------------------------------------------------
#
# * Use hard indents to match the px4 source code.
#
# * All function and script arguments are upper case.
#
# * All local variables are lower case.
#
# * All cmake functions are lowercase.
#
# * For else, endif, endfunction, etc, never put the name of the statement
#
# Functions/Macros
# ---------------------------------------------------------------------------
#
# * Use px4_parse_function_args to parse functions and check for required
#   arguments. Unless there is only one argument in the function and it is clear.
#
# * Never use macros. They allow overwriting global variables and this
#	makes variable declarations hard to locate.
#
# * If a target from add_custom_* is set in a function, explicitly pass it
#	as an output argument so that the target name is clear to the user.
#
# * Avoid use of global variables in functions. Functions in a nested
#	scope may use global variables, but this makes it difficult to
#	reuse functions.
#
# Included CMake Files
# ---------------------------------------------------------------------------
#
# * All variables in config files must have the prefix "config_".
#
# * Never set global variables in an included cmake file,
#	you may only define functions. This excludes config and Toolchain files.
#	This makes it clear to the user when variables are being set or targets
#	are being created.
#
# * Setting a global variable in a CMakeLists.txt file is ok, because
#	each CMakeLists.txt file has scope in the current directory and all
#	subdirectories, so it is not truly global.
#
# * All toolchain files should be included in the cmake
#	directory and named Toolchain-"name".cmake.
#
# Misc
# ---------------------------------------------------------------------------
#
# * If referencing a string variable, don't put it in quotes.
#	Don't do "${OS}" STREQUAL "posix",
#	instead type ${OS} STREQUAL "posix". This will throw an
#	error when ${OS} is not defined instead of silently
#	evaluating to false.
#
#=============================================================================

# check submodules before we do anything
execute_process(COMMAND Tools/check_submodules.sh WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

# Warning: Changing this modifies CMake's internal workings
# and leads to wrong toolchain detection
cmake_minimum_required(VERSION 3.1 FATAL_ERROR)

set(PX4_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
set(PX4_BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}")

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PX4_BINARY_DIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${PX4_BINARY_DIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${PX4_BINARY_DIR})

define_property(GLOBAL PROPERTY PX4_LIBRARIES
                 BRIEF_DOCS "PX4 libs"
                 FULL_DOCS "List of all PX4 module libraries"
                 )

#=============================================================================
# configuration
#

set(CONFIG "posix_sitl_default" CACHE STRING "desired configuration")

string(REPLACE "_" ";" config_args ${CONFIG})
list(GET config_args 0 OS)
list(GET config_args 1 VENDOR)
list(GET config_args 2 BOARD)
list(GET config_args 3 LABEL)
set(target_name "${OS}_${VENDOR}${BOARD}_${LABEL}")


file(GLOB configs
     LIST_DIRECTORIES false
     RELATIVE ${PX4_SOURCE_DIR}/platforms/${OS}/boards
     platforms/${OS}/boards/*/*/*.cmake
     )
     
foreach(config ${configs})
	
	string(REPLACE .cmake "" config_name ${config})

	string(REGEX REPLACE "[/]" _ config_name ${config_name})

	message(STATUS ${config}: ${target_name} : ${config_name})
	
	if (${config_name} MATCHES ${CONFIG})
		message(STATUS "MATCH MATCH ${CONFIG}")
	endif()
endforeach()

set(EXTERNAL_MODULES_LOCATION "" CACHE STRING "External modules source location")

if (NOT EXTERNAL_MODULES_LOCATION STREQUAL "")
	get_filename_component(EXTERNAL_MODULES_LOCATION "${EXTERNAL_MODULES_LOCATION}" ABSOLUTE)
endif()

list(APPEND CMAKE_MODULE_PATH
	${PX4_SOURCE_DIR}/cmake
	${PX4_SOURCE_DIR}/platforms/${OS}/cmake
	)

include(px4_base)
include(px4_impl_os)
include(px4_git)
include(px4_module_presets)

include(${PX4_SOURCE_DIR}/platforms/${OS}/boards/${VENDOR}/${BOARD}/${LABEL}.cmake)

# CMake build type
# Debug Release RelWithDebInfo MinSizeRel Coverage
if (NOT CMAKE_BUILD_TYPE)
	if (${OS} STREQUAL "nuttx")
		set(PX4_BUILD_TYPE "MinSizeRel")
	elseif (${OS} STREQUAL "bebop")
		set(PX4_BUILD_TYPE "MinSizeRel")
	else()
		set(PX4_BUILD_TYPE "RelWithDebInfo")
	endif()

	set(CMAKE_BUILD_TYPE ${PX4_BUILD_TYPE} CACHE STRING "Build type" FORCE)
endif()

set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug;Release;RelWithDebInfo;MinSizeRel;Coverage")

if (${CMAKE_BUILD_TYPE} STREQUAL "Coverage")
	include(coverage)
	include(sanitizers)
endif()

# version info from git
execute_process(
	COMMAND git describe --always --tags
	OUTPUT_VARIABLE git_tag
	OUTPUT_STRIP_TRAILING_WHITESPACE
	WORKING_DIRECTORY ${PX4_SOURCE_DIR}
	)

#=============================================================================

message(STATUS "PX4 VERSION: ${git_tag}")
message(STATUS "CONFIG: ${target_name}")
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")

#=============================================================================
# ccache
#
option(CCACHE "Use ccache if available" OFF)
find_program(CCACHE_PROGRAM ccache)
if (CCACHE AND CCACHE_PROGRAM)
	message(STATUS "Enabled ccache: ${CCACHE_PROGRAM}")
	set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "${CCACHE_PROGRAM}")
endif()

#=============================================================================
# project definition
#
project(px4 CXX C ASM)

set(package-contact "px4users@googlegroups.com")

#=============================================================================

# posix setup install paths, testing, cpack
if (${OS} STREQUAL "posix")

	# Define GNU standard installation directories
	include(GNUInstallDirs)

	if (NOT CMAKE_INSTALL_PREFIX)
		set(CMAKE_INSTALL_PREFIX "/usr" CACHE PATH "Install path prefix" FORCE)
	endif()
	
	# cmake testing only on posix
	enable_testing()
	include(CTest)
	
	include(px4_cpack)
endif()


#=============================================================================
# find programs and packages
#

# see if catkin was invoked to build this
if (CATKIN_DEVEL_PREFIX)
	message(STATUS "catkin ENABLED")
	find_package(catkin REQUIRED)
	if (catkin_FOUND)
		catkin_package()
	else()
		message(FATAL_ERROR "catkin not found")
	endif()
endif()

find_package(PythonInterp REQUIRED)

#=============================================================================
# require px4 module interface
#

# px4_os_prebuild_targets
if (COMMAND px4_os_prebuild_targets)
	px4_os_prebuild_targets(OUT prebuild_targets BOARD ${BOARD})
else()
	message(FATAL_ERROR "${config_module} must implement px4_os_prebuild_targets")
endif()

# common compiler flags
px4_add_common_flags()

# px4_os_add_flags
if (COMMAND px4_os_add_flags)
	# build flags
	px4_os_add_flags()
else()
	message(FATAL_ERROR "${config_module} must implement px4_os_add_flags")
endif()

#=============================================================================
# RTPS and micro-cdr
#

find_program(FASTRTPSGEN fastrtpsgen PATHS $ENV{FASTRTPSGEN_DIR})
if (FASTRTPSGEN AND (config_rtps_send_topics OR config_rtps_receive_topics))
	option(GENERATE_RTPS_BRIDGE "enable RTPS and microCDR" ON)

	# add micro-CDR
	px4_add_git_submodule(TARGET git_micro_cdr PATH "src/lib/micro-CDR")
	add_subdirectory(src/lib/micro-CDR)
	add_subdirectory(modules/micrortps_bridge/micrortps_client)

	# TODO: cleanup micro-CDR generation and dependencies
	add_dependencies(lib__micro-CDR prebuild_targets)
endif()

#=============================================================================
# DriverFramework
#

px4_add_git_submodule(TARGET git_driverframework PATH "src/lib/DriverFramework")

set(df_driver_libs)
foreach(driver ${config_df_driver_list})
	add_subdirectory(src/lib/DriverFramework/drivers/${driver})
	list(APPEND df_driver_libs df_${driver})
	message("Adding DF driver: ${driver}")
endforeach()

#=============================================================================
# external modules
#
if (NOT EXTERNAL_MODULES_LOCATION STREQUAL "")
	message(STATUS "External modules: ${EXTERNAL_MODULES_LOCATION}")
	add_subdirectory("${EXTERNAL_MODULES_LOCATION}/src" external_modules_src)

	set(config_module_list_external_expanded)
	foreach(external_module ${config_module_list_external})
		list(APPEND config_module_list_external_expanded
			${EXTERNAL_MODULES_LOCATION}/src/${external_module})
	endforeach()
	set(config_module_list
		${config_module_list}
		${config_module_list_external_expanded}
		)
endif()

#=============================================================================
# subdirectories
#

# required by all configs

px4_add_git_submodule(TARGET git_ecl PATH "src/lib/ecl")

add_subdirectory(msg)
add_subdirectory(src/lib/common)
add_subdirectory(src/lib/device)
add_subdirectory(src/lib/version)

foreach(module ${config_module_list})
	string(REGEX MATCH "^[./]" external_module ${module})
	if (external_module)
		STRING(REGEX REPLACE "//" "/" EXT_MODULE ${module})
		STRING(REGEX REPLACE "/" "__" EXT_MODULE_PREFIX ${EXT_MODULE})
		add_subdirectory(${module} ${PX4_BINARY_DIR}/${EXT_MODULE_PREFIX})
	else()
		add_subdirectory(src/${module})
	endif()
endforeach()

add_subdirectory(platforms)
