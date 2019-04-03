############################################################################
#
#   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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

if (CMAKE_HOST_APPLE OR CMAKE_HOST_WIN32)
	# copy with rsync and create file dependencies
	set(cp_cmd "rsync")
	set(cp_opts)
	list(APPEND cp_opts
		-rp
		--inplace
	)
else()
	# copy with hard links
	# archive, recursive, force, link (hardlinks)
	set(cp_cmd "cp")
	set(cp_opts "-aRfl")
endif()

set(NUTTX_CONFIG_DIR ${PX4_BOARD_DIR}/nuttx-config)

###############################################################################
# NuttX: copy to build directory
###############################################################################
file(RELATIVE_PATH CP_SRC ${CMAKE_SOURCE_DIR} ${NUTTX_SRC_DIR}/nuttx)
file(RELATIVE_PATH CP_DST ${CMAKE_SOURCE_DIR} ${PX4_BINARY_DIR}/NuttX)

# setup custom command to copy changes later
file(GLOB_RECURSE copy_nuttx_files LIST_DIRECTORIES false ${NUTTX_SRC_DIR}/nuttx/*)
add_custom_command(
	OUTPUT nuttx_copy.stamp
	COMMAND ${cp_cmd} ${cp_opts} ${CP_SRC} ${CP_DST}
	COMMAND ${CMAKE_COMMAND} -E touch ${CMAKE_CURRENT_BINARY_DIR}/nuttx_copy.stamp
	DEPENDS ${copy_nuttx_files} git_nuttx
	COMMENT "Copying NuttX/nuttx to ${CP_DST}"
	WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
)

###############################################################################
# NuttX apps: copy to build directory
###############################################################################
file(RELATIVE_PATH CP_SRC ${CMAKE_SOURCE_DIR} ${NUTTX_SRC_DIR}/apps)
file(RELATIVE_PATH CP_DST ${CMAKE_SOURCE_DIR} ${PX4_BINARY_DIR}/NuttX)

# setup custom command to copy changes later
file(GLOB_RECURSE copy_apps_files LIST_DIRECTORIES false ${NUTTX_SRC_DIR}/apps/*)
add_custom_command(
	OUTPUT nuttx_apps_copy.stamp
	COMMAND ${cp_cmd} ${cp_opts} ${CP_SRC} ${CP_DST}
	COMMAND ${CMAKE_COMMAND} -E touch ${CMAKE_CURRENT_BINARY_DIR}/nuttx_apps_copy.stamp
	DEPENDS ${copy_apps_files} git_nuttx_apps
	COMMENT "Copying NuttX/apps to ${CP_DST}"
	WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
)
set(APPS_DIR ${CMAKE_CURRENT_BINARY_DIR}/apps)


# If the board provides a Kconfig Use it or create an empty one
if(EXISTS ${NUTTX_CONFIG_DIR}/Kconfig)
	add_custom_command(
		OUTPUT nuttx_config_kconfig.stamp
		COMMAND ${CMAKE_COMMAND} -E copy ${NUTTX_CONFIG_DIR}/Kconfig ${NUTTX_DIR}/configs/dummy/Kconfig
		COMMAND ${CMAKE_COMMAND} -E touch ${CMAKE_CURRENT_BINARY_DIR}/nuttx_config_kconfig.stamp
		DEPENDS nuttx_copy.stamp
	)
else()
	add_custom_command(
		OUTPUT nuttx_config_kconfig.stamp
		COMMAND ${CMAKE_COMMAND} -E touch ${NUTTX_DIR}/configs/dummy/Kconfig
		COMMAND ${CMAKE_COMMAND} -E touch ${CMAKE_CURRENT_BINARY_DIR}/nuttx_config_kconfig.stamp
		DEPENDS nuttx_copy.stamp
	)
endif()

###############################################################################
# NuttX configure
###############################################################################

# copy NuttX config directory
add_custom_command(
	OUTPUT
		${PX4_BINARY_DIR}/NuttX/nuttx-config/include/board.h
		${PX4_BINARY_DIR}/NuttX/nuttx-config/include/nsh_romfsimg.h
		${PX4_BINARY_DIR}/NuttX/nuttx-config/scripts/script.ld
		nuttx_copy_config_dir.stamp
	COMMAND ${CMAKE_COMMAND} -E copy_directory ${NUTTX_CONFIG_DIR}/ ${PX4_BINARY_DIR}/NuttX/nuttx-config
	COMMAND ${CMAKE_COMMAND} -E make_directory ${PX4_BINARY_DIR}/NuttX/nuttx-config/src
	COMMAND ${CMAKE_COMMAND} -E copy_if_different ${NUTTX_SRC_DIR}/math.h ${NUTTX_DIR}/arch/arm/include/math.h # copy arm math.h into NuttX source
	COMMAND ${CMAKE_COMMAND} -E copy_if_different ${NUTTX_SRC_DIR}/nsh_romfsimg.h ${PX4_BINARY_DIR}/NuttX/nuttx-config/include/nsh_romfsimg.h
	COMMAND ${CMAKE_COMMAND} -E touch ${CMAKE_CURRENT_BINARY_DIR}/nuttx_copy_config_dir.stamp
	DEPENDS
		${NUTTX_SRC_DIR}/math.h
		${NUTTX_SRC_DIR}/nsh_romfsimg.h
		${NUTTX_CONFIG_DIR}/include/board.h
		${NUTTX_CONFIG_DIR}/scripts/script.ld
		nuttx_copy.stamp
	WORKING_DIRECTORY ${NUTTX_DIR}
	COMMENT "Copying NuttX config ${NUTTX_CONFIG_DIR}"
)

# NuttX defconfig
#  cmake should trigger reconfigure if defconfig changes
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${NUTTX_DEFCONFIG})
file(STRINGS ${NUTTX_DEFCONFIG} config_expanded REGEX "# Automatically generated file; DO NOT EDIT.")
if (NOT config_expanded)
	# copy compressed PX4 defconfig into nuttx and inflate
	add_custom_command(
		OUTPUT nuttx_config.stamp
		COMMAND ${CMAKE_COMMAND} -E copy_if_different ${NUTTX_DEFCONFIG} ${NUTTX_DIR}/.config
		COMMAND make -j1 --no-print-directory CONFIG_ARCH_BOARD_CUSTOM=y CONFIG_ARCH_BOARD_CUSTOM_NAME="px4" CONFIG_ARCH_BOARD_CUSTOM_DIR="../nuttx-config" CONFIG_APPS_DIR="../apps" APPSDIR="../apps" dirlinks
		COMMAND make -j1 --no-print-directory CONFIG_ARCH_BOARD_CUSTOM=y CONFIG_ARCH_BOARD_CUSTOM_NAME="px4" CONFIG_ARCH_BOARD_CUSTOM_DIR="../nuttx-config" CONFIG_APPS_DIR="../apps" APPSDIR="../apps" apps_preconfig
		COMMAND CONFIG_ARCH_BOARD_CUSTOM=y CONFIG_ARCH_BOARD_CUSTOM_NAME="px4" CONFIG_ARCH_BOARD_CUSTOM_DIR="../nuttx-config" CONFIG_APPS_DIR="../apps" APPSDIR="../apps" ${PYTHON} ${CMAKE_CURRENT_SOURCE_DIR}/tools/olddefconfig.py
		COMMAND ${CMAKE_COMMAND} -E touch ${CMAKE_CURRENT_BINARY_DIR}/nuttx_config.stamp
		DEPENDS
			${NUTTX_DEFCONFIG}
			nuttx_copy.stamp
			nuttx_apps_copy.stamp
			nuttx_copy_config_dir.stamp
			nuttx_config_kconfig.stamp
		WORKING_DIRECTORY ${NUTTX_DIR}
		COMMENT "Copying NuttX compressed config ${NUTTX_CONFIG_DIR} and inflating"
	)
else()
	# copy uncompressed PX4 defconfig into nuttx
	add_custom_command(
		OUTPUT nuttx_config.stamp
		COMMAND ${CMAKE_COMMAND} -E copy_if_different ${NUTTX_DEFCONFIG} ${NUTTX_DIR}/.config
		COMMAND ${CMAKE_COMMAND} -E touch ${CMAKE_CURRENT_BINARY_DIR}/nuttx_config.stamp
		DEPENDS
			${NUTTX_DEFCONFIG}
			nuttx_copy.stamp
			nuttx_apps_copy.stamp
			nuttx_copy_config_dir.stamp
			nuttx_config_kconfig.stamp
		WORKING_DIRECTORY ${NUTTX_DIR}
		COMMENT "Copying NuttX uncompressed config ${NUTTX_CONFIG_DIR}"
	)
endif()

###############################################################################
# NuttX build
###############################################################################

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Make.defs.in ${NUTTX_DIR}/Make.defs)

# verbose build settings (V=1 or VERBOSE=1)
option(PX4_NUTTX_VERBOSE "PX4 NuttX verbose build" off)

if (($ENV{V} MATCHES "1") OR ($ENV{VERBOSE} MATCHES "1"))
	message(STATUS "NuttX verbose build enabled")
	set(PX4_NUTTX_VERBOSE on)
endif()

if (PX4_NUTTX_VERBOSE)
	set(nuttx_build_options)
	set(nuttx_build_output)
	set(nuttx_build_uses_terminal "USES_TERMINAL")
else()
	set(nuttx_build_options "--quiet")
	set(nuttx_build_output ">nuttx_build.log")
	set(nuttx_build_uses_terminal)
endif()

# context
add_custom_command(
	OUTPUT nuttx_context.stamp
	COMMAND make -j1 --no-print-directory CONFIG_ARCH_BOARD_CUSTOM=y CONFIG_ARCH_BOARD_CUSTOM_NAME="px4" CONFIG_ARCH_BOARD_CUSTOM_DIR="../nuttx-config" CONFIG_APPS_DIR="../apps" APPSDIR="../apps" context
	COMMAND ${CMAKE_COMMAND} -E touch ${CMAKE_CURRENT_BINARY_DIR}/nuttx_context.stamp
	DEPENDS nuttx_config.stamp
	WORKING_DIRECTORY ${NUTTX_DIR}
	${nuttx_build_uses_terminal}
)
add_custom_target(nuttx_context DEPENDS nuttx_context.stamp)

# library of NuttX libraries
add_library(nuttx_build INTERFACE)
add_dependencies(nuttx_build nuttx_context)

# builtins
set(nuttx_builtin_list)
if (CONFIG_NSH_LIBRARY)
	# force builtins regeneration and apps rebuild if nuttx or px4 configuration have changed
	add_custom_command(
		OUTPUT builtins_clean.stamp
		COMMAND find ${APPS_DIR}/builtin/registry -name px4_\*.bdat -delete
		COMMAND find ${APPS_DIR}/builtin/registry -name px4_\*.pdat -delete
		COMMAND rm -f ${APPS_DIR}/builtin/builtin_list.h
		COMMAND ${CMAKE_COMMAND} -E touch builtins_clean.stamp
		DEPENDS
			nuttx_context.stamp
			${PX4_CONFIG_FILE}
	)

	foreach(module ${module_libraries})
		get_target_property(MAIN ${module} MAIN)
		get_target_property(STACK_MAIN ${module} STACK_MAIN)
		get_target_property(PRIORITY ${module} PRIORITY)

		if(MAIN)
			add_custom_command(OUTPUT ${APPS_DIR}/builtin/registry/px4_${MAIN}_main.bdat
				COMMAND echo "{ \"${MAIN}\", ${PRIORITY}, ${STACK_MAIN}, ${MAIN}_main }," > ${APPS_DIR}/builtin/registry/px4_${MAIN}_main.bdat
				COMMAND ${CMAKE_COMMAND} -E touch ${APPS_DIR}/builtin/registry/.updated
				DEPENDS
					builtins_clean.stamp
					nuttx_context.stamp
				VERBATIM
			)
			list(APPEND nuttx_builtin_list ${APPS_DIR}/builtin/registry/px4_${MAIN}_main.bdat)

			add_custom_command(OUTPUT ${APPS_DIR}/builtin/registry/px4_${MAIN}_main.pdat
				COMMAND echo "int ${MAIN}_main(int argc, char *argv[]);" > ${APPS_DIR}/builtin/registry/px4_${MAIN}_main.pdat
				COMMAND ${CMAKE_COMMAND} -E touch ${APPS_DIR}/builtin/registry/.updated
				DEPENDS
					builtins_clean.stamp
					nuttx_context.stamp
				VERBATIM
			)
			list(APPEND nuttx_builtin_list ${APPS_DIR}/builtin/registry/px4_${MAIN}_main.pdat)

		endif()
	endforeach()
endif()

# APPS

# libapps.a
add_custom_command(
	OUTPUT
		${APPS_DIR}/libapps.a
		${APPS_DIR}/platform/.built
	COMMAND find ${APPS_DIR} -type f -name \*.o -delete
	COMMAND make -j1 ${nuttx_build_options} --no-print-directory -C ../apps TOPDIR="${NUTTX_DIR}" libapps.a > nuttx_apps.log
	DEPENDS
		nuttx_context.stamp
		${nuttx_builtin_list}
	WORKING_DIRECTORY ${NUTTX_DIR}
	${nuttx_build_uses_terminal}
	)
add_custom_target(nuttx_apps_build DEPENDS ${APPS_DIR}/libapps.a)
add_library(nuttx_apps STATIC IMPORTED GLOBAL)
set_property(TARGET nuttx_apps PROPERTY IMPORTED_LOCATION ${APPS_DIR}/libapps.a)
add_dependencies(nuttx_build nuttx_apps_build)
target_link_libraries(nuttx_build INTERFACE nuttx_apps)

# helper for all targets
function(add_nuttx_dir nuttx_lib nuttx_lib_dir kernel extra)
	file(GLOB_RECURSE nuttx_lib_files
		LIST_DIRECTORIES false
		${CMAKE_CURRENT_SOURCE_DIR}/nuttx/${nuttx_lib_dir}/*
	)

	add_custom_command(OUTPUT ${NUTTX_DIR}/${nuttx_lib_dir}/lib${nuttx_lib}.a
		COMMAND find ${nuttx_lib_dir} -type f -name *.o -delete
		COMMAND make -j1 -C ${nuttx_lib_dir} ${nuttx_build_options} --no-print-directory all TOPDIR=${NUTTX_DIR} KERNEL=${kernel} EXTRADEFINES=${extra} ${nuttx_build_output}
		DEPENDS
			nuttx_context.stamp
			${nuttx_lib_files}
		WORKING_DIRECTORY ${NUTTX_DIR}
		${nuttx_build_uses_terminal}
	)
	add_custom_target(nuttx_${nuttx_lib}_build DEPENDS ${NUTTX_DIR}/${nuttx_lib_dir}/lib${nuttx_lib}.a)
	add_library(nuttx_${nuttx_lib} STATIC IMPORTED GLOBAL)
	set_property(TARGET nuttx_${nuttx_lib} PROPERTY IMPORTED_LOCATION ${NUTTX_DIR}/${nuttx_lib_dir}/lib${nuttx_lib}.a)

	add_dependencies(nuttx_build nuttx_${nuttx_lib}_build)
	target_link_libraries(nuttx_build INTERFACE nuttx_${nuttx_lib})
endfunction()

# add_nuttx_dir(NAME DIRECTORY KERNEL EXTRA)
add_nuttx_dir(arch arch/arm/src y -D__KERNEL__)
add_nuttx_dir(binfmt binfmt y -D__KERNEL__)
add_nuttx_dir(configs configs y -D__KERNEL__)
add_nuttx_dir(drivers drivers y -D__KERNEL__)
add_nuttx_dir(fs fs y -D__KERNEL__)
add_nuttx_dir(sched sched y -D__KERNEL__)
add_nuttx_dir(c libs/libc n "")
add_nuttx_dir(xx libs/libxx n "")
add_nuttx_dir(mm mm n "")

target_link_libraries(nuttx_arch INTERFACE nuttx_sched)
target_link_libraries(nuttx_drivers INTERFACE nuttx_arch)
target_link_libraries(nuttx_c INTERFACE nuttx_drivers)
target_link_libraries(nuttx_xx INTERFACE nuttx_c)

if (CONFIG_NET)
	add_nuttx_dir(net net y -D__KERNEL__)
endif()

###############################################################################
# NuttX oldconfig
add_custom_target(oldconfig_nuttx
	COMMAND make -j1 --no-print-directory --silent -C ${NUTTX_DIR} CONFIG_ARCH_BOARD_CUSTOM=y CONFIG_ARCH_BOARD_CUSTOM_NAME="px4" CONFIG_ARCH_BOARD_CUSTOM_DIR="../nuttx-config" CONFIG_APPS_DIR="../apps" APPSDIR="../apps" oldconfig
	DEPENDS nuttx_context.stamp
	WORKING_DIRECTORY ${NUTTX_DIR}
	COMMENT "Running NuttX make oldconfig for ${NUTTX_CONFIG}"
	USES_TERMINAL
)

# NuttX oldconfig + savedefconfig back to PX4
add_custom_target(oldconfig
	COMMAND make -j1 --no-print-directory --silent -C ${NUTTX_DIR} CONFIG_ARCH_BOARD_CUSTOM=y CONFIG_ARCH_BOARD_CUSTOM_NAME="px4" CONFIG_ARCH_BOARD_CUSTOM_DIR="../nuttx-config" CONFIG_APPS_DIR="../apps" APPSDIR="../apps" savedefconfig
	COMMAND ${CMAKE_COMMAND} -E copy ${NUTTX_DIR}/defconfig ${NUTTX_DEFCONFIG}
	DEPENDS oldconfig_nuttx
	WORKING_DIRECTORY ${NUTTX_DIR}
	COMMENT "Running make oldconfig then savedefconfig for ${NUTTX_CONFIG}"
	USES_TERMINAL
)

# NuttX oldconfig + copy uncompressed back to PX4
add_custom_target(oldconfig_uncompressed
	COMMAND ${CMAKE_COMMAND} -E copy ${NUTTX_DIR}/.config ${NUTTX_DEFCONFIG}
	DEPENDS oldconfig_nuttx
	WORKING_DIRECTORY ${NUTTX_DIR}
	COMMENT "Running make oldconfig then saving uncompressed defconfig for ${NUTTX_CONFIG}"
	USES_TERMINAL
)

###############################################################################
# NuttX olddefconfig
add_custom_target(olddefconfig_nuttx
	COMMAND make -j1 --no-print-directory --silent -C ${NUTTX_DIR} CONFIG_ARCH_BOARD_CUSTOM=y CONFIG_ARCH_BOARD_CUSTOM_NAME="px4" CONFIG_ARCH_BOARD_CUSTOM_DIR="../nuttx-config" CONFIG_APPS_DIR="../apps" APPSDIR="../apps" olddefconfig
	DEPENDS nuttx_context.stamp
	WORKING_DIRECTORY ${NUTTX_DIR}
	COMMENT "Running NuttX make olddefconfig for ${NUTTX_CONFIG}"
	USES_TERMINAL
	)

# NuttX olddefconfig + savedefconfig back to PX4
add_custom_target(olddefconfig
	COMMAND make -j1 --no-print-directory --silent -C ${NUTTX_DIR} CONFIG_ARCH_BOARD_CUSTOM=y CONFIG_ARCH_BOARD_CUSTOM_NAME="px4" CONFIG_ARCH_BOARD_CUSTOM_DIR="../nuttx-config" CONFIG_APPS_DIR="../apps" APPSDIR="../apps" savedefconfig
	COMMAND ${CMAKE_COMMAND} -E copy ${NUTTX_DIR}/defconfig ${NUTTX_DEFCONFIG}
	DEPENDS olddefconfig_nuttx
	WORKING_DIRECTORY ${NUTTX_DIR}
	COMMENT "Running make olddefconfig then savedefconfig for ${NUTTX_CONFIG}"
	USES_TERMINAL
)

# NuttX oldconfig + copy uncompressed back to PX4
add_custom_target(olddefconfig_uncompressed
	COMMAND ${CMAKE_COMMAND} -E copy ${NUTTX_DIR}/.config ${NUTTX_DEFCONFIG}
	DEPENDS olddefconfig_nuttx
	WORKING_DIRECTORY ${NUTTX_DIR}
	COMMENT "Running make olddefconfig then saving uncompressed defconfig for ${NUTTX_CONFIG}"
	USES_TERMINAL
)

###############################################################################
# NuttX menuconfig
add_custom_target(menuconfig_nuttx
	COMMAND make -j1 --no-print-directory --silent -C ${NUTTX_DIR} CONFIG_ARCH_BOARD_CUSTOM=y CONFIG_ARCH_BOARD_CUSTOM_NAME="px4" CONFIG_ARCH_BOARD_CUSTOM_DIR="../nuttx-config" CONFIG_APPS_DIR="../apps" APPSDIR="../apps" menuconfig
	DEPENDS nuttx_context.stamp
	WORKING_DIRECTORY ${NUTTX_DIR}
	COMMENT "Running NuttX make menuconfig for ${NUTTX_CONFIG}"
	USES_TERMINAL
)

# NuttX menuconfig + savedefconfig back to PX4
add_custom_target(menuconfig
	COMMAND make -j1 --no-print-directory --silent -C ${NUTTX_DIR} CONFIG_ARCH_BOARD_CUSTOM=y CONFIG_ARCH_BOARD_CUSTOM_NAME="px4" CONFIG_ARCH_BOARD_CUSTOM_DIR="../nuttx-config" CONFIG_APPS_DIR="../apps" APPSDIR="../apps" savedefconfig
	COMMAND ${CMAKE_COMMAND} -E copy ${NUTTX_DIR}/defconfig ${NUTTX_DEFCONFIG}
	DEPENDS menuconfig_nuttx
	WORKING_DIRECTORY ${NUTTX_DIR}
	COMMENT "Running make nuttx_menuconfig then savedefconfig for ${NUTTX_CONFIG}"
	USES_TERMINAL
)

# NuttX menuconfig + copy uncompressed back to PX4
add_custom_target(menuconfig_uncompressed
	COMMAND ${CMAKE_COMMAND} -E copy ${NUTTX_DIR}/.config ${NUTTX_DEFCONFIG}
	DEPENDS menuconfig_nuttx
	WORKING_DIRECTORY ${NUTTX_DIR}
	COMMENT "Running make nuttx_menuconfig then saving uncompressed defconfig for ${NUTTX_CONFIG}"
	USES_TERMINAL
)

###############################################################################
# NuttX qconfig
add_custom_target(qconfig_nuttx
	COMMAND make -j1 --no-print-directory --silent -C ${NUTTX_DIR} CONFIG_ARCH_BOARD_CUSTOM=y CONFIG_ARCH_BOARD_CUSTOM_NAME="px4" CONFIG_ARCH_BOARD_CUSTOM_DIR="../nuttx-config" CONFIG_APPS_DIR="../apps" APPSDIR="../apps" qconfig
	DEPENDS nuttx_context.stamp
	WORKING_DIRECTORY ${NUTTX_DIR}
	COMMENT "Running NuttX make qconfig for ${NUTTX_CONFIG}"
	USES_TERMINAL
)

# NuttX qconfig + savedefconfig back to PX4
add_custom_target(qconfig
	COMMAND make -j1 --no-print-directory --silent -C ${NUTTX_DIR} CONFIG_ARCH_BOARD_CUSTOM=y CONFIG_ARCH_BOARD_CUSTOM_NAME="px4" CONFIG_ARCH_BOARD_CUSTOM_DIR="../nuttx-config" CONFIG_APPS_DIR="../apps" APPSDIR="../apps" savedefconfig
	COMMAND ${CMAKE_COMMAND} -E copy ${NUTTX_DIR}/defconfig ${NUTTX_DEFCONFIG}
	DEPENDS qconfig_nuttx
	WORKING_DIRECTORY ${NUTTX_DIR}
	COMMENT "Running make qconfig then savedefconfig for ${NUTTX_CONFIG}"
	USES_TERMINAL
)

# NuttX qconfig + copy uncompressed back to PX4
add_custom_target(qconfig_uncompressed
	COMMAND ${CMAKE_COMMAND} -E copy ${NUTTX_DIR}/.config ${NUTTX_DEFCONFIG}
	DEPENDS qconfig_nuttx
	WORKING_DIRECTORY ${NUTTX_DIR}
	COMMENT "Running make qconfig then saving uncompressed defconfig for ${NUTTX_CONFIG}"
	USES_TERMINAL
)
