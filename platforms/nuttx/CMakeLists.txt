
include_directories(src/drivers)

add_subdirectory(boards/${VENDOR}/${BOARD}/src)
add_subdirectory(src)

# must be last
add_subdirectory(NuttX)


# add executable
set(fw_name ${CONFIG}.elf)
add_executable(${fw_name} boards/${VENDOR}/${BOARD}/nuttx-config/src/empty.c)
add_dependencies(${fw_name} git_nuttx)

get_property(module_libraries GLOBAL PROPERTY PX4_LIBRARIES)

target_link_libraries(${fw_name}
	-T${PX4_BINARY_DIR}/NuttX/nuttx/configs/${VENDOR}${BOARD}/scripts/ld.script
	-Wl,-Map=${PX4_BINARY_DIR}/${CONFIG}.map
	-Wl,--start-group
	${module_libraries}
	df_driver_framework
	nuttx_apps
	nuttx_arch
	nuttx_binfmt
	nuttx_binfmt
	nuttx_c
	nuttx_configs
	nuttx_cxx
	nuttx_drivers
	nuttx_fs
	nuttx_mm
	nuttx_sched
	-Wl,--end-group
	m
	gcc
	)

if (config_romfs_root)
	add_subdirectory(${PX4_SOURCE_DIR}/ROMFS ${PX4_BINARY_DIR}/ROMFS)
	target_link_libraries(${fw_name} romfs)
	if (config_io_board)
		add_dependencies(romfs copy_px4io_bin)
	endif()
endif()

# create px4 file (combined firmware and metadata)
set(fw_file ${PX4_BINARY_DIR}/${CONFIG}.px4)

add_custom_command(OUTPUT ${BOARD}.bin
	COMMAND ${OBJCOPY} -O binary ${PX4_BINARY_DIR}/${fw_name} ${BOARD}.bin
	DEPENDS ${fw_name}
	)

if (TARGET parameters_xml AND TARGET airframes_xml)
	add_custom_command(OUTPUT ${PX4_BINARY_DIR}/${CONFIG}.px4
		COMMAND ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/Tools/px_mkfw.py
			--prototype ${PX4_SOURCE_DIR}/platforms/${OS}/boards/${VENDOR}/${BOARD}/Image.prototype
			--git_identity ${PX4_SOURCE_DIR}
			--parameter_xml ${PX4_BINARY_DIR}/parameters.xml
			--airframe_xml ${PX4_BINARY_DIR}/airframes.xml
			--image ${BOARD}.bin > ${fw_file}
		DEPENDS ${BOARD}.bin parameters_xml airframes_xml
		COMMENT "Creating ${CONFIG}.px4"
		)

	add_custom_target(px4 ALL DEPENDS ${PX4_BINARY_DIR}/${CONFIG}.px4)

	# upload helper
	include(px4_upload)
	if (${BOARD} STREQUAL "aerofc-v1")		
		add_custom_target(upload
			COMMAND ${PX4_SOURCE_DIR}/Tools/aero_upload.sh ${fw_file}
			DEPENDS ${fw_file}
			COMMENT "uploading ${fw_file}"
			VERBATIM
			USES_TERMINAL
		)	
	else()

		set(serial_ports)
		if(${CMAKE_HOST_SYSTEM_NAME} STREQUAL "Linux")
			list(APPEND serial_ports
				/dev/serial/by-id/*_PX4_*
				/dev/serial/by-id/usb-3D_Robotics*
				/dev/serial/by-id/usb-The_Autopilot*
				/dev/serial/by-id/usb-Bitcraze*
				/dev/serial/by-id/pci-3D_Robotics*
				/dev/serial/by-id/pci-Bitcraze*
				/dev/serial/by-id/usb-Gumstix*
				)
		elseif(${CMAKE_HOST_SYSTEM_NAME} STREQUAL "Darwin")
			list(APPEND serial_ports
				/dev/tty.usbmodemPX*
				/dev/tty.usbmodem*
				)
		elseif(${CMAKE_HOST_SYSTEM_NAME} STREQUAL "Windows")
			foreach(port RANGE 32 0)
				list(APPEND serial_ports
					"COM${port}")
			endforeach()
		endif()
	
		string(REPLACE ";" "," serial_ports_csv "${serial_ports}")
		
		add_custom_target(upload
			COMMAND ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/Tools/px_uploader.py --port "${serial_ports_csv}" ${fw_file}
			DEPENDS ${fw_file}
			COMMENT "uploading ${fw_file}"
			VERBATIM
			USES_TERMINAL
			)

	endif()
endif()

# print size
add_custom_target(size
	COMMAND size ${fw_name}
	DEPENDS ${fw_name}
	WORKING_DIRECTORY ${PX4_BINARY_DIR}
	)

# print weak symbols
add_custom_target(check_weak
	COMMAND ${NM} ${fw_name} | ${GREP} " w " | cat
	DEPENDS ${fw_name}
	VERBATIM
	)

# debugger helpers
set(DEBUG_PORT "/dev/ttyACM0" CACHE STRING "debugging port")
configure_file(Debug/gdbinit.in .gdbinit)

add_custom_target(debug
	COMMAND ${GDB} $<TARGET_FILE:${fw_name}>
	DEPENDS ${fw_name} ${CMAKE_CURRENT_BINARY_DIR}/.gdbinit
	)

add_custom_target(debug_tui
	COMMAND ${GDBTUI} $<TARGET_FILE:${fw_name}>
	DEPENDS ${fw_name} ${CMAKE_CURRENT_BINARY_DIR}/.gdbinit
	)

add_custom_target(debug_ddd
	COMMAND ${DDD} --debugger ${GDB} $<TARGET_FILE:${fw_name}>
	DEPENDS ${fw_name} ${CMAKE_CURRENT_BINARY_DIR}/.gdbinit
	)

add_custom_target(debug_io
	COMMAND ${GDB} ${fw_io_path}
	DEPENDS ${fw_name} ${CMAKE_CURRENT_BINARY_DIR}/.gdbinit
	)

add_custom_target(debug_io_tui
	COMMAND ${GDBTUI} ${fw_io_path}
	DEPENDS ${fw_name} ${CMAKE_CURRENT_BINARY_DIR}/.gdbinit
	)

add_custom_target(debug_io_ddd
	COMMAND ${DDD} --debugger ${GDB} ${fw_io_path}}
	DEPENDS ${fw_io} ${CMAKE_CURRENT_BINARY_DIR}/.gdbinit
	)
