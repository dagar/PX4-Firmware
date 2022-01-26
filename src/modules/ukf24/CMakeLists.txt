############################################################################
#
#   Copyright (c) 2015-2022 PX4 Development Team. All rights reserved.
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
#############################################################################

add_subdirectory(Utility)

px4_add_module(
	MODULE modules__ukf24
	MAIN ukf24
	COMPILE_FLAGS
		#${MAX_CUSTOM_OPT_LEVEL}
		-fno-associative-math
		#-DDEBUG_BUILD
	INCLUDES
		UKF
	STACK_MAX
		3600
	SRCS
		UKF/RingBuffer.h
		UKF/airspeed_fusion.cpp
		UKF/common.h
		UKF/control.cpp
		UKF/covariance.cpp
		UKF/estimator_interface.cpp
		UKF/estimator_interface.h
		UKF/gps_checks.cpp
		UKF/mag_fusion.cpp
		UKF/terrain_estimator.cpp
		UKF/ukf.cpp
		UKF/ukf.h
		UKF/ukf_helper.cpp
		UKF/vel_pos_fusion.cpp

		UKF24.cpp
		UKF24.hpp
		#UKF24Selector.cpp
		#UKF24Selector.hpp

	DEPENDS
		geo
		hysteresis
		perf
		UKF24Utility
		px4_work_queue
		world_magnetic_model
	#UNITY_BUILD
	)

if(BUILD_TESTING)
	#add_subdirectory(EKF)
	#add_subdirectory(test)
endif()
