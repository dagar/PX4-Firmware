set(BOARD_DEFCONFIG ${PX4_CONFIG_FILE} CACHE FILEPATH "path to defconfig" FORCE)
set(BOARD_CONFIG ${PX4_BINARY_DIR}/boardconfig CACHE FILEPATH "path to config" FORCE)

execute_process(COMMAND ${PYTHON_EXECUTABLE} -c "import menuconfig" RESULT_VARIABLE ret)
if(ret EQUAL "1")
    message(FATAL_ERROR "kconfiglib is not installed or not in PATH\n"
                        "please install using \"pip3 install kconfiglib\"\n")
endif()

set(MENUCONFIG_PATH ${PYTHON_EXECUTABLE} -m menuconfig CACHE INTERNAL "menuconfig program" FORCE)
set(GUICONFIG_PATH ${PYTHON_EXECUTABLE} -m guiconfig CACHE INTERNAL "guiconfig program" FORCE)
set(DEFCONFIG_PATH ${PYTHON_EXECUTABLE} -m defconfig CACHE INTERNAL "defconfig program" FORCE)
set(SAVEDEFCONFIG_PATH ${PYTHON_EXECUTABLE} -m savedefconfig CACHE INTERNAL "savedefconfig program" FORCE)

set(COMMON_KCONFIG_ENV_SETTINGS
	PYTHON_EXECUTABLE=${PYTHON_EXECUTABLE}
	KCONFIG_CONFIG=${BOARD_CONFIG}
	# Set environment variables so that Kconfig can prune Kconfig source
	# files for other architectures
	PLATFORM=${PX4_PLATFORM}
	VENDOR=${PX4_BOARD_VENDOR}
	MODEL=${PX4_BOARD_MODEL}
	LABEL=${PX4_BOARD_LABEL}
	TOOLCHAIN=${CMAKE_TOOLCHAIN_FILE}
	ARCHITECTURE=${CMAKE_SYSTEM_PROCESSOR}
	ROMFSROOT=${config_romfs_root}
)

if(EXISTS ${BOARD_DEFCONFIG})

    # Depend on BOARD_DEFCONFIG so that we reconfigure on config change
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${BOARD_DEFCONFIG})

    if(${LABEL} MATCHES "default" OR ${LABEL} MATCHES "bootloader" OR ${LABEL} MATCHES "canbootloader")
        # Generate boardconfig from saved defconfig
        execute_process(COMMAND ${CMAKE_COMMAND} -E env ${COMMON_KCONFIG_ENV_SETTINGS}
                        ${DEFCONFIG_PATH} ${BOARD_DEFCONFIG}
                        WORKING_DIRECTORY ${PX4_SOURCE_DIR}
                        OUTPUT_VARIABLE DUMMY_RESULTS)
    else()
        # Generate boardconfig from default.px4board and {label}.px4board
        execute_process(COMMAND ${CMAKE_COMMAND} -E env ${COMMON_KCONFIG_ENV_SETTINGS}
                        ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/Tools/kconfig/merge_config.py Kconfig ${BOARD_CONFIG} ${PX4_BOARD_DIR}/default.px4board ${BOARD_DEFCONFIG}
                        WORKING_DIRECTORY ${PX4_SOURCE_DIR}
                        OUTPUT_VARIABLE DUMMY_RESULTS)
    endif()



    # determine which config flags are for directories


    # CONFIG_DRIVER
    # CONFIG_MODULE

    #   - collect defines per module
    #   - don't allow underscore in names
    #   - once established as a directory, everything below must be a directory or module
    #      - error if we end up with defines that don't map to an actual module
    #   -  these defines will then be limited in scope to that specific module




    # CONFIG_BOARD_
    # CONFIG_COMMON_



    # module_drivers_imu_invensense_icm20602
    #     # cmake properties per target
    #     # COMPILE_DEFINITIONS
    #
    #   px4_add_module MODULE

    set(module_dir_tree)

    # parse board config options for cmake
    file(STRINGS ${BOARD_CONFIG} config_contents)
    foreach(name_and_value ${config_contents})
        # Strip leading spaces
        string(REGEX REPLACE "^[ ]+" "" name_and_value ${name_and_value})

        # Find variable name
        string(REGEX MATCH "^CONFIG[^=]+" name ${name_and_value})

        if(name)
            # Find the value
            string(REPLACE "${name}=" "" value ${name_and_value})

            if(value)
                # remove extra quotes
                string(REPLACE "\"" "" value ${value})

                # Set the variable
                set(${name} ${value} CACHE INTERNAL "BOARD DEFCONFIG: ${name}" FORCE)

                message(STATUS "BOARD DEFCONFIG: ${name}=${value}")
            endif()



            # CONFIG_BOARD_
            # CONFIG_COMMON_


            # Find variable name
            string(REGEX MATCH "^CONFIG_BOARD_" board ${name_and_value})
            string(REGEX MATCH "^CONFIG_COMMON_" common ${name_and_value})

            if(board)
                string(REPLACE "CONFIG_BOARD_" "" config_key ${name})
                if(value)
                    set(${config_key} ${value})
                    message(STATUS "${config_key} ${value}")
                endif()
            elseif(common)
                # TODO:
            else()

                # everything else must be a module
                string(REPLACE "CONFIG_" "" module ${name})
                string(TOLOWER ${module} module)
                string(REPLACE "_" "/" module_path ${module})


                message(STATUS "module_path: ${module_path}")


                # drivers/px4io
                # drivers/lights/neopixel
                # drivers/imu/invensense/icm20602



                # Pattern 1 XXX / XXX
                string(REGEX REPLACE "(^[a-z]+)_([a-z]+[a-z0-9]+).*$" "\\1" p1_folder ${module})
                string(REGEX REPLACE "(^[a-z]+)_([a-z]+[a-z0-9]+).*$" "\\2" p1_subfolder ${module})

                # Pattern 2 XXX / XXX / XXX
                string(REGEX REPLACE "(^[a-z]+)_([a-z]+[a-z0-9]+)_([a-z]+[a-z0-9]+).*$" "\\1" p2_folder ${module})
                string(REGEX REPLACE "(^[a-z]+)_([a-z]+[a-z0-9]+)_([a-z]+[a-z0-9]+).*$" "\\2" p2_subfolder ${module})
                string(REGEX REPLACE "(^[a-z]+)_([a-z]+[a-z0-9]+)_([a-z]+[a-z0-9]+).*$" "\\3" p2_subsubfolder ${module})

                # Pattern 3 XXX / XXX / XXX
                string(REGEX REPLACE "(^[a-z]+)_([a-z]+[a-z0-9]+)_([a-z]+[a-z0-9]+)_([a-z]+[a-z0-9]+).*$" "\\1" p3_folder ${module})
                string(REGEX REPLACE "(^[a-z]+)_([a-z]+[a-z0-9]+)_([a-z]+[a-z0-9]+)_([a-z]+[a-z0-9]+).*$" "\\2" p3_subfolder ${module})
                string(REGEX REPLACE "(^[a-z]+)_([a-z]+[a-z0-9]+)_([a-z]+[a-z0-9]+)_([a-z]+[a-z0-9]+).*$" "\\3" p3_subsubfolder ${module})
                string(REGEX REPLACE "(^[a-z]+)_([a-z]+[a-z0-9]+)_([a-z]+[a-z0-9]+)_([a-z]+[a-z0-9]+).*$" "\\4" p3_subsubsubfolder ${module})

                # Pattern 4 XXX / XXX / XXX / XXX
                string(REGEX REPLACE "(^[a-z]+)_([a-z]+[a-z0-9]+)_([a-z]+[a-z0-9]+)_([a-z]+[a-z0-9]+)_([a-z]+[a-z0-9]+).*$" "\\1" p4_folder ${module})
                string(REGEX REPLACE "(^[a-z]+)_([a-z]+[a-z0-9]+)_([a-z]+[a-z0-9]+)_([a-z]+[a-z0-9]+)_([a-z]+[a-z0-9]+).*$" "\\2" p4_subfolder ${module})
                string(REGEX REPLACE "(^[a-z]+)_([a-z]+[a-z0-9]+)_([a-z]+[a-z0-9]+)_([a-z]+[a-z0-9]+)_([a-z]+[a-z0-9]+).*$" "\\3" p4_subsubfolder ${module})
                string(REGEX REPLACE "(^[a-z]+)_([a-z]+[a-z0-9]+)_([a-z]+[a-z0-9]+)_([a-z]+[a-z0-9]+)_([a-z]+[a-z0-9]+).*$" "\\4" p4_subsubsubfolder ${module})
                string(REGEX REPLACE "(^[a-z]+)_([a-z]+[a-z0-9]+)_([a-z]+[a-z0-9]+)_([a-z]+[a-z0-9]+)_([a-z]+[a-z0-9]+).*$" "\\4" p4_subsubsubsubfolder ${module})


                if(EXISTS "${PX4_SOURCE_DIR}/src/${p4_folder}/${p4_subfolder}/${p4_subsubfolder}/${p4_subsubsubfolder}/${p4_subsubsubsubfolder}/CMakeLists.txt")
                    message(STATUS "    p4: ${p4_folder} : ${p4_subfolder} : ${p4_subsubfolder} : ${p4_subsubsubfolder} : ${p4_subsubsubsubfolder}")

                elseif(EXISTS "${PX4_SOURCE_DIR}/src/${p3_folder}/${p3_subfolder}/${p3_subsubfolder}/${p3_subsubsubfolder}/CMakeLists.txt")
                    message(STATUS "    p3: ${p3_folder} : ${p3_subfolder} : ${p3_subsubfolder} : ${p3_subsubsubfolder}")

                elseif(EXISTS "${PX4_SOURCE_DIR}/src/${p2_folder}/${p2_subfolder}/${p2_subsubfolder}/CMakeLists.txt")
                    message(STATUS "    p2: ${p2_folder} : ${p2_subfolder} : ${p2_subsubfolder}")

                elseif(EXISTS "${PX4_SOURCE_DIR}/src/${p1_folder}/${p1_subfolder}/CMakeLists.txt")
                    message(STATUS "    p1: ${p1_folder} : ${p1_subfolder}")
                else()
                    message(FATAL_ERROR " ${module}????")
                endif()


                # if(EXISTS ${PX4_SOURCE_DIR}/src/${p1_folder}/${driver_path})
                #     list(APPEND config_module_list drivers/${driver_path})
                # elseif(EXISTS ${PX4_SOURCE_DIR}/src/drivers/${driver_p3_folder}/${driver_p3_subfolder}/${driver_p3_subsubfolder})
                #     list(APPEND config_module_list drivers/${driver_p3_folder}/${driver_p3_subfolder}/${driver_p3_subsubfolder})
                # elseif(EXISTS ${PX4_SOURCE_DIR}/src/drivers/${driver_p1_folder}/${driver_p1_subfolder})
                #     list(APPEND config_module_list drivers/${driver_p1_folder}/${driver_p1_subfolder})
                # elseif(EXISTS ${PX4_SOURCE_DIR}/src/drivers/${driver_p4_folder}/${driver_p4_subfolder})
                #     list(APPEND config_module_list drivers/${driver_p4_folder}/${driver_p4_subfolder})
                # elseif(EXISTS ${PX4_SOURCE_DIR}/src/drivers/${driver_p2_folder}/${driver_p2_subfolder})
                #     list(APPEND config_module_list drivers/${driver_p2_folder}/${driver_p2_subfolder})
                # elseif(EXISTS ${PX4_SOURCE_DIR}/src/drivers/${driver_p5_folder}/${driver_p5_subfolder}/${driver_p5_subsubfolder})
                #     list(APPEND config_module_list drivers/${driver_p5_folder}/${driver_p5_subfolder}/${driver_p5_subsubfolder})
                # else()
                #     message(FATAL_ERROR "Couldn't find path for ${driver}")
                # endif()








            endif()



        endif()



        # underscoe (_) is directory delimeter



    endforeach()

    if(PLATFORM)
        # set OS, and append specific platform module path
        set(PX4_PLATFORM ${PLATFORM} CACHE STRING "PX4 board OS" FORCE)
        list(APPEND CMAKE_MODULE_PATH ${PX4_SOURCE_DIR}/platforms/${PX4_PLATFORM}/cmake)

        # platform-specific include path
        include_directories(${PX4_SOURCE_DIR}/platforms/${PX4_PLATFORM}/src/px4/common/include)
    endif()

	if(ARCHITECTURE)
		set(CMAKE_SYSTEM_PROCESSOR ${ARCHITECTURE} CACHE INTERNAL "system processor" FORCE)
	endif()

	if(TOOLCHAIN)
		set(CMAKE_TOOLCHAIN_FILE Toolchain-${TOOLCHAIN} CACHE INTERNAL "toolchain file" FORCE)
	endif()

	set(romfs_extra_files)
	set(config_romfs_extra_dependencies)
	# additional embedded metadata
	if (NOT CONSTRAINED_FLASH AND NOT EXTERNAL_METADATA AND NOT ${PX4_BOARD_LABEL} STREQUAL "test")
		list(APPEND romfs_extra_files
			${PX4_BINARY_DIR}/parameters.json.xz
			${PX4_BINARY_DIR}/events/all_events.json.xz)
		list(APPEND romfs_extra_dependencies
			parameters_xml
			events_json)
	endif()
	list(APPEND romfs_extra_files ${PX4_BINARY_DIR}/component_general.json.xz)
	list(APPEND romfs_extra_dependencies component_general_json)
	set(config_romfs_extra_files ${romfs_extra_files} CACHE INTERNAL "extra ROMFS files" FORCE)
	set(config_romfs_extra_dependencies ${romfs_extra_dependencies} CACHE INTERNAL "extra ROMFS deps" FORCE)

	if(SERIAL_PORTS)
		set(board_serial_ports ${SERIAL_PORTS} PARENT_SCOPE)
	endif()

	# Serial ports
	set(board_serial_ports)
	if(SERIAL_GPS1)
        list(APPEND board_serial_ports GPS1:${SERIAL_GPS1})
	endif()
	if(SERIAL_GPS2)
        list(APPEND board_serial_ports GPS2:${SERIAL_GPS2})
	endif()
	if(SERIAL_GPS3)
        list(APPEND board_serial_ports GPS3:${SERIAL_GPS3})
	endif()
	if(SERIAL_GPS4)
        list(APPEND board_serial_ports GPS4:${SERIAL_GPS4})
	endif()
	if(SERIAL_GPS5)
        list(APPEND board_serial_ports GPS5:${SERIAL_GPS5})
	endif()
	if(SERIAL_TEL1)
        list(APPEND board_serial_ports TEL1:${SERIAL_TEL1})
	endif()
	if(SERIAL_TEL2)
        list(APPEND board_serial_ports TEL2:${SERIAL_TEL2})
	endif()
	if(SERIAL_TEL3)
        list(APPEND board_serial_ports TEL3:${SERIAL_TEL3})
	endif()
	if(SERIAL_TEL4)
        list(APPEND board_serial_ports TEL4:${SERIAL_TEL4})
	endif()
	if(SERIAL_TEL5)
        list(APPEND board_serial_ports TEL5:${SERIAL_TEL5})
	endif()
	if(SERIAL_WIFI)
        list(APPEND board_serial_ports WIFI:${SERIAL_WIFI})
	endif()


	# ROMFS
	if(ROMFSROOT)
		set(config_romfs_root ${ROMFSROOT} CACHE INTERNAL "ROMFS root" FORCE)

		if(BUILD_BOOTLOADER)
			set(config_build_bootloader "1" CACHE INTERNAL "build bootloader" FORCE)
		endif()

		# IO board (placed in ROMFS)
		if(IO)
			set(config_io_board ${IO} CACHE INTERNAL "IO" FORCE)
		endif()

		if(UAVCAN_PERIPHERALS)
			set(config_uavcan_peripheral_firmware ${UAVCAN_PERIPHERALS} CACHE INTERNAL "UAVCAN peripheral firmware" FORCE)
		endif()
	endif()

	if(UAVCAN_INTERFACES)
		set(config_uavcan_num_ifaces ${UAVCAN_INTERFACES} CACHE INTERNAL "UAVCAN interfaces" FORCE)
	endif()

	if(UAVCAN_TIMER_OVERRIDE)
		set(config_uavcan_timer_override ${UAVCAN_TIMER_OVERRIDE} CACHE INTERNAL "UAVCAN TIMER OVERRIDE" FORCE)
	endif()

	# OPTIONS

	if(CONSTRAINED_FLASH)
		set(px4_constrained_flash_build "1" CACHE INTERNAL "constrained flash build" FORCE)
		add_definitions(-DCONSTRAINED_FLASH)
	endif()

    if (NO_HELP)
		add_definitions(-DCONSTRAINED_FLASH_NO_HELP="https://docs.px4.io/master/en/modules/modules_main.html")
	endif()

	if(CONSTRAINED_MEMORY)
		set(px4_constrained_memory_build "1" CACHE INTERNAL "constrained memory build" FORCE)
		add_definitions(-DCONSTRAINED_MEMORY)
	endif()

	if(TESTING)
		set(PX4_TESTING "1" CACHE INTERNAL "testing enabled" FORCE)
	endif()

	if(ETHERNET)
		set(PX4_ETHERNET "1" CACHE INTERNAL "ethernet enabled" FORCE)
	endif()

	if(CRYPTO)
		set(PX4_CRYPTO ${CRYPTO} CACHE STRING "PX4 crypto implementation" FORCE)
		add_definitions(-DPX4_CRYPTO)
	endif()

	if(KEYSTORE)
		set(PX4_KEYSTORE ${KEYSTORE} CACHE STRING "PX4 keystore implementation" FORCE)
	endif()

	if(LINKER_PREFIX)
		set(PX4_BOARD_LINKER_PREFIX ${LINKER_PREFIX} CACHE STRING "PX4 board linker prefix" FORCE)
	else()
		set(PX4_BOARD_LINKER_PREFIX "" CACHE STRING "PX4 board linker prefix" FORCE)
	endif()

	if(COMPILE_DEFINITIONS)
        add_definitions( ${COMPILE_DEFINITIONS})
	endif()

	if(LINUX)
        add_definitions( "-D__PX4_LINUX" )
	endif()

	if(LOCKSTEP)
        set(ENABLE_LOCKSTEP_SCHEDULER yes)
	endif()

	if(NOLOCKSTEP)
        set(ENABLE_LOCKSTEP_SCHEDULER no)
	endif()

	if(FULL_OPTIMIZATION)
        set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)
	endif()

	include(px4_impl_os)
	px4_os_prebuild_targets(OUT prebuild_targets BOARD ${PX4_BOARD})

    # add board config directory src to build modules
	file(RELATIVE_PATH board_support_src_rel ${PX4_SOURCE_DIR}/src ${PX4_BOARD_DIR})
	list(APPEND config_module_list ${board_support_src_rel}/src)

	set(config_module_list ${config_module_list})

endif()


if(${LABEL} MATCHES "default" OR ${LABEL} MATCHES "bootloader" OR ${LABEL} MATCHES "canbootloader")
    add_custom_target(boardconfig
        ${CMAKE_COMMAND} -E env
        ${COMMON_KCONFIG_ENV_SETTINGS}
        ${MENUCONFIG_PATH} Kconfig
        COMMAND ${CMAKE_COMMAND} -E env ${COMMON_KCONFIG_ENV_SETTINGS} ${SAVEDEFCONFIG_PATH}
        COMMAND ${CMAKE_COMMAND} -E copy defconfig ${BOARD_DEFCONFIG}
        COMMAND ${CMAKE_COMMAND} -E remove defconfig
        COMMAND ${CMAKE_COMMAND} -E remove ${PX4_BINARY_DIR}/NuttX/apps_copy.stamp
        WORKING_DIRECTORY ${PX4_SOURCE_DIR}
        USES_TERMINAL
        COMMAND_EXPAND_LISTS
    )

    add_custom_target(boardguiconfig
        ${CMAKE_COMMAND} -E env
        ${COMMON_KCONFIG_ENV_SETTINGS}
        ${GUICONFIG_PATH} Kconfig
        COMMAND ${CMAKE_COMMAND} -E env ${COMMON_KCONFIG_ENV_SETTINGS} ${SAVEDEFCONFIG_PATH}
        COMMAND ${CMAKE_COMMAND} -E copy defconfig ${BOARD_DEFCONFIG}
        COMMAND ${CMAKE_COMMAND} -E remove defconfig
        COMMAND ${CMAKE_COMMAND} -E remove ${PX4_BINARY_DIR}/NuttX/apps_copy.stamp
        WORKING_DIRECTORY ${PX4_SOURCE_DIR}
        USES_TERMINAL
        COMMAND_EXPAND_LISTS
    )
else()
    add_custom_target(boardconfig
        ${CMAKE_COMMAND} -E env
        ${COMMON_KCONFIG_ENV_SETTINGS}
        ${MENUCONFIG_PATH} Kconfig
        COMMAND ${CMAKE_COMMAND} -E env ${COMMON_KCONFIG_ENV_SETTINGS} ${SAVEDEFCONFIG_PATH}
        COMMAND ${CMAKE_COMMAND} -E env ${COMMON_KCONFIG_ENV_SETTINGS} ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/Tools/kconfig/diffconfig.py -m ${PX4_BOARD_DIR}/default.px4board defconfig > ${BOARD_DEFCONFIG}
        COMMAND ${CMAKE_COMMAND} -E remove defconfig
        COMMAND ${CMAKE_COMMAND} -E remove ${PX4_BINARY_DIR}/NuttX/apps_copy.stamp
        WORKING_DIRECTORY ${PX4_SOURCE_DIR}
        USES_TERMINAL
        COMMAND_EXPAND_LISTS
    )

    add_custom_target(boardguiconfig
        ${CMAKE_COMMAND} -E env
        ${COMMON_KCONFIG_ENV_SETTINGS}
        ${GUICONFIG_PATH} Kconfig
        COMMAND ${CMAKE_COMMAND} -E env ${COMMON_KCONFIG_ENV_SETTINGS} ${SAVEDEFCONFIG_PATH}
        COMMAND ${CMAKE_COMMAND} -E env ${COMMON_KCONFIG_ENV_SETTINGS} ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/Tools/kconfig/diffconfig.py -m ${PX4_BOARD_DIR}/default.px4board defconfig > ${BOARD_DEFCONFIG}
        COMMAND ${CMAKE_COMMAND} -E remove defconfig
        COMMAND ${CMAKE_COMMAND} -E remove ${PX4_BINARY_DIR}/NuttX/apps_copy.stamp
        WORKING_DIRECTORY ${PX4_SOURCE_DIR}
        USES_TERMINAL
        COMMAND_EXPAND_LISTS
    )
endif()
