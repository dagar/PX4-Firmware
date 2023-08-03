

px4_add_git_submodule(TARGET git_micro-xrce-dds-agent PATH "Micro-XRCE-DDS-Agent")

include(ExternalProject)

ExternalProject_Add(micro-xrce-dds-agent_build
	#DOWNLOAD_COMMAND rsync -a --safe-links --delete --exclude=build --exclude=*.o --exclude=*.a --exclude=.config ${PX4_SOURCE_DIR}/ ${PX4_BINARY_DIR}/external/Source/px4io_firmware_build/
	#UPDATE_COMMAND   rsync -a --safe-links --delete --exclude=build --exclude=*.o --exclude=*.a --exclude=.config ${PX4_SOURCE_DIR}/ ${PX4_BINARY_DIR}/external/Source/px4io_firmware_build/
	DEPENDS git_micro-xrce-dds-agent
	SOURCE_DIR ${CMAKE_CURRENT_LIST_DIR}/Micro-XRCE-DDS-Agent
	CMAKE_ARGS
		-DUAGENT_SUPERBUILD=OFF
		-DUAGENT_BUILD_EXECUTABLE=ON
		-DBUILD_SHARED_LIBS=OFF
		-DUAGENT_FAST_PROFILE=ON
		-DUAGENT_CED_PROFILE=OFF
		-DUAGENT_DISCOVERY_PROFILE=OFF
		-DUAGENT_P2P_PROFILE=OFF
		-DUAGENT_SOCKETCAN_PROFILE=OFF
		-DUAGENT_LOGGER_PROFILE=ON # review
		-DUAGENT_USE_SYSTEM_FASTDDS=ON
		-DUAGENT_USE_SYSTEM_FASTCDR=ON
	INSTALL_COMMAND ""
	#USES_TERMINAL_BUILD true
	EXCLUDE_FROM_ALL true
	#BUILD_ALWAYS true
)


find_package(rclcpp REQUIRED)
add_executable(micro-xrce-dds-agent_node node.cpp)
add_dependencies(micro-xrce-dds-agent_node micro-xrce-dds-agent_build)

ament_target_dependencies(micro-xrce-dds-agent_node
	rclcpp
	#micro-xrce-dds-agent_build
)
install(
	TARGETS
		micro-xrce-dds-agent_node
	DESTINATION lib/${PROJECT_NAME}
)
