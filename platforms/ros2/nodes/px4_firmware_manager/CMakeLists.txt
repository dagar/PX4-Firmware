

# upload script

# build px4_fmu-v5x
#  node checks version and uploads

include(ExternalProject)

ExternalProject_Add(px4_firmware_build-px4_fmu-v5x_default
	SOURCE_DIR ${PX4_SOURCE_DIR}
	CMAKE_ARGS -DCONFIG=px4_fmu-v5x_default
	INSTALL_COMMAND ""
	#USES_TERMINAL_BUILD true
	EXCLUDE_FROM_ALL true
	#BUILD_ALWAYS true
)

ExternalProject_Add(px4_firmware_build-px4_sitl_default
	SOURCE_DIR ${PX4_SOURCE_DIR}
	CMAKE_ARGS -DCONFIG=px4_sitl_default
	INSTALL_COMMAND ""
	#USES_TERMINAL_BUILD true
	EXCLUDE_FROM_ALL true
	#BUILD_ALWAYS true
)


find_package(rclcpp REQUIRED)
add_executable(px4_firmware_node node.cpp)
add_dependencies(px4_firmware_node
	px4_firmware_build-px4_fmu-v5x_default
	px4_firmware_build-px4_sitl_default
)

ament_target_dependencies(px4_firmware_node
	rclcpp
)
install(
	TARGETS
		px4_firmware_node
	DESTINATION lib/${PROJECT_NAME}
)
