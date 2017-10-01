
#=============================================================================
# airframe generation
#=============================================================================
add_custom_command(OUTPUT ${PX4_BINARY_DIR}/airframes.xml
	COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tools/px_process_airframes.py
		-a ${PX4_SOURCE_DIR}/ROMFS/${config_romfs_root}/init.d
		--board CONFIG_ARCH_BOARD_${BOARD} --xml
	DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/tools/px_process_airframes.py
	COMMENT "Creating airframes.xml"
	WORKING_DIRECTORY ${PX4_BINARY_DIR}
	)
add_custom_target(airframes_xml DEPENDS ${PX4_BINARY_DIR}/airframes.xml)

add_custom_command(OUTPUT ${PX4_BINARY_DIR}/airframes.md
	COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tools/px_process_airframes.py
		-a ${PX4_SOURCE_DIR}/ROMFS/${config_romfs_root}/init.d
		--board CONFIG_ARCH_BOARD_${BOARD} --markdown
	DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/tools/px_process_airframes.py
	COMMENT "Creating airframes.md"
	WORKING_DIRECTORY ${PX4_BINARY_DIR}
	)
add_custom_target(airframes_markdown DEPENDS ${PX4_BINARY_DIR}/airframes.md)

#=============================================================================
# ROMFS generation
#=============================================================================

message(STATUS "ROMFS: ${config_romfs_root}")

set(romfs_temp_dir ${PX4_BINARY_DIR}/ROMFS/${config_romfs_root})
set(romfs_src_dir ${PX4_SOURCE_DIR}/ROMFS/${config_romfs_root})

# directory setup
# copy all romfs files, process airframes, prune comments
file(GLOB_RECURSE init_airframes ${PX4_SOURCE_DIR}/ROMFS/${config_romfs_root}/*/[1-9]*)
add_custom_command(OUTPUT ${romfs_temp_dir}/init.d/rcS ${romfs_temp_dir}/init.d/rc.autostart
	COMMAND cmake -E copy_directory ${romfs_src_dir} ${romfs_temp_dir}
	COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tools/px_process_airframes.py
		-a ${romfs_temp_dir}/init.d
		-s ${romfs_temp_dir}/init.d/rc.autostart
		--board ${BOARD}
	COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tools/px_romfs_pruner.py
		--folder ${romfs_temp_dir} --board ${BOARD}
	DEPENDS
		${romfs_src_files}
		${init_airframes}
		${PX4_SOURCE_DIR}/ROMFS/${config_romfs_root}/init.d/rcS
		${CMAKE_CURRENT_SOURCE_DIR}/tools/px_process_airframes.py
	)

set(romfs_dependencies)
list(APPEND romfs_dependencies
	${romfs_temp_dir}/init.d/rcS
	${romfs_temp_dir}/init.d/rc.autostart
	)
	
# create romfs.bin
add_custom_command(OUTPUT romfs.img
	COMMAND ${GENROMFS} -f romfs.img -d ${romfs_temp_dir} -V "NSHInitVol" -v > romfs.txt 2>&1
	DEPENDS ${romfs_dependencies}
	)

# create romfs.o
add_custom_command(OUTPUT nsh_romfsimg.c
	COMMAND xxd -i romfs.img nsh_romfsimg.c
	COMMAND sed 's/unsigned/const unsigned/g' nsh_romfsimg.c > nsh_romfsimg.c.tmp && mv nsh_romfsimg.c.tmp nsh_romfsimg.c
	DEPENDS romfs.img
	)

px4_add_library(romfs STATIC nsh_romfsimg.c)
set_target_properties(romfs PROPERTIES LINKER_LANGUAGE C)
