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


# upload script

# build px4_fmu-v5x
#  node checks version and uploads

include(ExternalProject)

# ExternalProject_Add(px4_firmware_build-px4_fmu-v5x_default
# 	SOURCE_DIR ${PX4_SOURCE_DIR}
# 	CMAKE_ARGS -DCONFIG=px4_fmu-v5x_default
# 	INSTALL_COMMAND ""
# 	#USES_TERMINAL_BUILD true
# 	EXCLUDE_FROM_ALL true
# 	#BUILD_ALWAYS true
# )

ExternalProject_Add(px4_firmware_build-px4_sitl_default
	SOURCE_DIR ${PX4_SOURCE_DIR}
	CMAKE_ARGS -DCONFIG=px4_sitl_default -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}/lib
	#INSTALL_COMMAND ""
	#INSTALL_DIR
	INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
	#USES_TERMINAL_BUILD true
	EXCLUDE_FROM_ALL true
	#BUILD_ALWAYS true
)

#add_executable(px4 IMPORTED GLOBAL)
#ExternalProject_Get_Property(px4_firmware_build-px4_sitl_default BINARY_DIR)
#set_property(TARGET px4 PROPERTY IMPORTED_LOCATION ${BINARY_DIR}/bin/px4)


find_package(rclcpp REQUIRED)
add_executable(px4_firmware_node node.cpp)
add_dependencies(px4_firmware_node
	#px4_firmware_build-px4_fmu-v5x_default
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
