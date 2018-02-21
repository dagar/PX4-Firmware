############################################################################
#
#   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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


px4_add_git_submodule(TARGET git_smc PATH ${PX4_SOURCE_DIR}/Tools/smc)

add_custom_command(OUTPUT StateMachine_sm.cpp
	COMMAND
		java -jar ${PX4_SOURCE_DIR}/Tools/smc/bin/Smc.jar
			-nocatch -noex -c++ -crtp -cast static_cast -stack 10
			-d ${CMAKE_CURRENT_BINARY_DIR}
			${CMAKE_CURRENT_SOURCE_DIR}/StateMachine.sm
	DEPENDS
		StateMachine.sm
		git_smc
)
add_custom_target(generate_state_machine DEPENDS StateMachine_sm.cpp)

add_custom_command(OUTPUT StateMachine.dot
	COMMAND
		java -jar ${PX4_SOURCE_DIR}/Tools/smc/bin/Smc.jar
			-graph -glevel 1
			-d ${CMAKE_CURRENT_BINARY_DIR}
			${CMAKE_CURRENT_SOURCE_DIR}/StateMachine.sm
	COMMAND
		dot -T png -o StateMachine.png StateMachine.dot
	DEPENDS
		StateMachine.sm
)
add_custom_target(generate_state_machine_graph DEPENDS StateMachine.dot)

add_custom_command(OUTPUT StateMachine_sm.html
	COMMAND
		java -jar ${PX4_SOURCE_DIR}/Tools/smc/bin/Smc.jar
			-table
			-d ${CMAKE_CURRENT_BINARY_DIR}
			${CMAKE_CURRENT_SOURCE_DIR}/StateMachine.sm
	DEPENDS
		StateMachine.sm
	USES_TERMINAL
)
add_custom_target(generate_state_machine_table DEPENDS StateMachine_sm.html)

px4_add_module(
	MODULE modules__commander2
	MAIN commander2
	STACK_MAIN 4096
	STACK_MAX 2450
	INCLUDES
		${PX4_SOURCE_DIR}/Tools/smc/lib/C++
		${CMAKE_CURRENT_SOURCE_DIR}
		${CMAKE_CURRENT_BINARY_DIR}
	COMPILE_FLAGS
	SRCS
		Commander.cpp
		StateMachine.cpp
		${CMAKE_CURRENT_BINARY_DIR}/StateMachine_sm.cpp
	DEPENDS
		generate_state_machine
	)
