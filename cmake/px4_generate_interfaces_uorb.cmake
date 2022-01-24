
# headers
set(msg_out_path ${PX4_BINARY_DIR}/uORB/topics)

set(msg_files)
set(uorb_headers)
foreach(msg_file ${px4_msg_files})

	list(APPEND msg_files "${PX4_SOURCE_DIR}/${msg_file}")

	get_filename_component(msg ${msg_file} NAME_WE)

	# Pascal case to snake case (MsgFile -> msg_file)
	string(REGEX REPLACE "(.)([A-Z][a-z]+)" "\\1_\\2" msg "${msg}")
	string(REGEX REPLACE "([a-z0-9])([A-Z])" "\\1_\\2" msg "${msg}")
	string(TOLOWER "${msg}" msg)

	list(APPEND uorb_headers ${msg_out_path}/${msg}.h)
endforeach()

# Generate uORB headers
add_custom_command(OUTPUT ${uorb_headers}
	COMMAND ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/Tools/msg/tools/px_generate_uorb_topic_files.py
		-f ${msg_files}
		-i ${PX4_SOURCE_DIR}/msg
		-o ${msg_out_path}
		-e ${PX4_SOURCE_DIR}/Tools/msg/templates/uorb/msg.h.em
	DEPENDS
		${msg_files}
		${PX4_SOURCE_DIR}/Tools/msg/templates/uorb/msg.h.em
		${PX4_SOURCE_DIR}/Tools/msg/tools/px_generate_uorb_topic_files.py
		${PX4_SOURCE_DIR}/Tools/msg/tools/px_generate_uorb_topic_helper.py
	COMMENT "Generating uORB topic headers"
	WORKING_DIRECTORY ${PX4_SOURCE_DIR}/msg
	VERBATIM
	)
add_custom_target(uorb_headers DEPENDS ${uorb_headers})
