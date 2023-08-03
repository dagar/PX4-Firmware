############################################################################
#
#   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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

ExternalProject_Get_Property(micro-xrce-dds-agent_build BINARY_DIR)
install(
	PROGRAMS
		${BINARY_DIR}/MicroXRCEAgent
	DESTINATION
		lib/${PROJECT_NAME}
)
