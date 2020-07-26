############################################################################
#
#   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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

px4_add_module(
	MODULE drivers__distance_sensor__vl53l1x
	MAIN vl53l1xMulti
	INCLUDES
		api/core
		api/platform
	SRCS
		vl53l1x_main.cpp
		VL53L1X.cpp
		VL53L1X.hpp

		api/core/vl53l1_error_exceptions.h
		api/core/vl53l1_register_settings.h
		api/core/vl53l1_wait.h
		api/core/vl53l1_api.c
		api/core/vl53l1_preset_setup.h
		api/core/vl53l1_error_strings.c
		api/core/vl53l1_core_support.c
		api/core/vl53l1_ll_device.h
		api/core/vl53l1_api_calibration.h
		api/core/vl53l1_silicon_core.c
		api/core/vl53l1_api_debug.c
		api/core/vl53l1_register_funcs.h
		api/core/vl53l1_api_strings.c
		api/core/vl53l1_register_map.h
		api/core/vl53l1_error_codes.h
		api/core/vl53l1_api_core.c
		api/core/vl53l1_core.h
		api/core/vl53l1_register_structs.h
		api/core/vl53l1_api_preset_modes.h
		api/core/vl53l1_nvm_map.h
		api/core/vl53l1_api_calibration.c
		api/core/vl53l1_core_support.h
		api/core/vl53l1_error_strings.h
		api/core/vl53l1_ll_def.h
		api/core/vl53l1_wait.c
		api/core/vl53l1_api.h
		api/core/vl53l1_api_strings.h
		api/core/vl53l1_register_funcs.c
		api/core/vl53l1_api_debug.h
		api/core/vl53l1_silicon_core.h
		api/core/vl53l1_def.h
		api/core/vl53l1_core.c
		api/core/vl53l1_api_preset_modes.c
		api/core/vl53l1_api_core.h
		api/core/vl53l1_tuning_parm_defaults.h
		api/platform/vl53l1_platform_log.h
		api/platform/vl53l1_types.h
		api/platform/vl53l1_platform.c
		api/platform/vl53l1_platform_user_data.h
		api/platform/vl53l1_platform_user_config.h
		api/platform/vl53l1_platform_user_defines.h
		api/platform/vl53l1_platform.h

		PCA9505.hpp
		PCA9505.cpp
	DEPENDS
		drivers_rangefinder
)
