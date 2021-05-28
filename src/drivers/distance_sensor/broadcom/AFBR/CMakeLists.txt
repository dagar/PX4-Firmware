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

add_library(afbrs50_m4_fpu_os STATIC IMPORTED GLOBAL)
set_property(TARGET afbrs50_m4_fpu_os PROPERTY IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/Lib/libafbrs50_m4_fpu_os.a)

px4_add_module(
	MODULE drivers__distance_sensor__afbrs50
	MAIN afbrs50
	COMPILE_FLAGS
#		-O0
		#-Wno-bad-function-cast # TODO: fix and enable
		-Wno-cast-align # TODO: fix and enable
	INCLUDES
		Include
		API/Inc
		Core/Inc
		Drivers/STM32F4xx_HAL_Driver/Inc
		Drivers/STM32F4xx_HAL_Driver/Inc/Legacy
		Drivers/CMSIS/Device/ST/STM32F4xx/Include
		Drivers/CMSIS/Include
		Platform
	SRCS
		API/Src/irq.c
		API/Src/s2pi.c
		API/Src/timer.c
		Core/Src/main.c
		Core/Src/gpio.c
		Core/Src/dma.c
		Core/Src/spi.c
		Core/Src/tim.c
		Core/Src/stm32f4xx_it.c
		Core/Src/stm32f4xx_hal_msp.c
		Core/Src/system_stm32f4xx.c
		Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_spi.c
		#Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c
		#Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c
		Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c
		Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c
		#Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c
		Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c
		Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c
		Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c
		#Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c
		#Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c
		Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c
		Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c
		Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_exti.c
		Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c
		Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c
		AFBRS50.cpp
		AFBRS50.hpp
		afbrs50_main.cpp
	DEPENDS
		px4_work_queue
		afbrs50_m4_fpu_os
		drivers_rangefinder
	)
