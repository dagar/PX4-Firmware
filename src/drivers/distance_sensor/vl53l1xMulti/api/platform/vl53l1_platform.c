
/*
* Copyright (c) 2017, STMicroelectronics - All Rights Reserved
*
* This file is part of VL53L1 Core and is dual licensed,
* either 'STMicroelectronics
* Proprietary license'
* or 'BSD 3-clause "New" or "Revised" License' , at your option.
*
********************************************************************************
*
* 'STMicroelectronics Proprietary license'
*
********************************************************************************
*
* License terms: STMicroelectronics Proprietary in accordance with licensing
* terms at www.st.com/sla0081
*
* STMicroelectronics confidential
* Reproduction and Communication of this document is strictly prohibited unless
* specifically authorized in writing by STMicroelectronics.
*
*
********************************************************************************
*
* Alternatively, VL53L1 Core may be distributed under the terms of
* 'BSD 3-clause "New" or "Revised" License', in which case the following
* provisions apply instead of the ones mentioned above :
*
********************************************************************************
*
* License terms: BSD 3-clause "New" or "Revised" License.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its contributors
* may be used to endorse or promote products derived from this software
* without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*
********************************************************************************
*
*/


#include "vl53l1_platform.h"
// #include "vl53l1_platform_log.h"
#include "vl53l1_api.h"

// #include "stm32xxx_hal.h"
#include <string.h>
// #include <time.h>
// #include <math.h>

#include <drivers/drv_hrt.h>
#include <px4_log.h>

// #define I2C_TIME_OUT_BASE   10
// #define I2C_TIME_OUT_BYTE   1

// #ifdef VL53L1_LOG_ENABLE
// #define trace_print(level, ...) VL53L1_trace_print_module_function(VL53L1_TRACE_MODULE_PLATFORM, level, VL53L1_TRACE_FUNCTION_NONE, ##__VA_ARGS__)
// #define trace_i2c(...) VL53L1_trace_print_module_function(VL53L1_TRACE_MODULE_NONE, VL53L1_TRACE_LEVEL_NONE, VL53L1_TRACE_FUNCTION_I2C, ##__VA_ARGS__)
// #endif

// #ifndef HAL_I2C_MODULE_ENABLED
// #warning "HAL I2C module must be enable "
// #endif

//extern I2C_HandleTypeDef hi2c1;
//#define VL53L0X_pI2cHandle    (&hi2c1)

/* when not customized by application define dummy one */
// #ifndef VL53L1_GetI2cBus
/** This macro can be overloaded by user to enforce i2c sharing in RTOS context
 */
// #   define VL53L1_GetI2cBus(...) (void)0
// #endif

// #ifndef VL53L1_PutI2cBus
/** This macro can be overloaded by user to enforce i2c sharing in RTOS context
 */
// #   define VL53L1_PutI2cBus(...) (void)0
// #endif

// uint8_t _I2CBuffer[256];

// int _I2CWrite(VL53L1_DEV Dev, uint8_t *pdata, uint32_t count) {
//     int status = 0;
//     return status;
// }

// int _I2CRead(VL53L1_DEV Dev, uint8_t *pdata, uint32_t count) {
//    int status = 0;
//    return Status;
// }

VL53L1_Error VL53L1_WriteMulti(VL53L1_DEV Dev, uint16_t index, uint8_t *pdata, uint32_t count)
{
	uint8_t tx_buf[256];

	if (count > sizeof(tx_buf) - 2) {
		return VL53L1_ERROR_INVALID_PARAMS;
	}

	tx_buf[0] = index >> 8;
	tx_buf[1] = index & 0xFF;
	memcpy(&tx_buf[2], pdata, count);

	//usleep(1300);

	if (Dev->dev == NULL) {
		PX4_ERR("I2C device not opened");
		return VL53L1_ERROR_CONTROL_INTERFACE;
	}

	struct i2c_msg_s msgv[1];

	{
		msgv[0].frequency = 400000;
		msgv[0].addr = Dev->I2cDevAddr;
		msgv[0].flags = 0;
		msgv[0].buffer = &tx_buf[0];
		msgv[0].length = count + 2;
	}

	if (I2C_TRANSFER(Dev->dev, &msgv[0], 1) == 0) {
		return VL53L1_ERROR_NONE;
	}

	return VL53L1_ERROR_CONTROL_INTERFACE;
}

// the ranging_sensor_comms.dll will take care of the page selection
VL53L1_Error VL53L1_ReadMulti(VL53L1_DEV Dev, uint16_t index, uint8_t *pdata, uint32_t count)
{
	uint8_t tx_buf[2] = {index >> 8, index & 0xFF};

	//usleep(1300);

	if (Dev->dev == NULL) {
		PX4_ERR("I2C device not opened");
		return VL53L1_ERROR_CONTROL_INTERFACE;
	}

	struct i2c_msg_s msgv[2];

	{
		msgv[0].frequency = 400000;
		msgv[0].addr = Dev->I2cDevAddr;
		msgv[0].flags = 0;
		msgv[0].buffer = &tx_buf[0];
		msgv[0].length = sizeof(tx_buf);
	}

	{
		msgv[1].frequency = 400000;
		msgv[1].addr = Dev->I2cDevAddr;
		msgv[1].flags = I2C_M_READ;
		msgv[1].buffer = pdata;
		msgv[1].length = count;
	}

	if (I2C_TRANSFER(Dev->dev, &msgv[0], 2) == 0) {
		return VL53L1_ERROR_NONE;
	}

	return VL53L1_ERROR_CONTROL_INTERFACE;
}

VL53L1_Error VL53L1_WrByte(VL53L1_DEV Dev, uint16_t index, uint8_t data)
{
	return VL53L1_WriteMulti(Dev, index, &data, 1);
}

VL53L1_Error VL53L1_RdByte(VL53L1_DEV Dev, uint16_t index, uint8_t *data)
{
	uint8_t tx_buf[2] = {index >> 8, index & 0xFF};

	//usleep(1300);

	if (Dev->dev == NULL) {
		PX4_ERR("I2C device not opened");
		return VL53L1_ERROR_CONTROL_INTERFACE;
	}

	struct i2c_msg_s msgv[2];

	{
		msgv[0].frequency = 400000;
		msgv[0].addr = Dev->I2cDevAddr;
		msgv[0].flags = 0;
		msgv[0].buffer = &tx_buf[0];
		msgv[0].length = 2;
	}

	{
		msgv[1].frequency = 400000;
		msgv[1].addr = Dev->I2cDevAddr;
		msgv[1].flags = I2C_M_READ;
		msgv[1].buffer = data;
		msgv[1].length = 1;
	}

	if (I2C_TRANSFER(Dev->dev, &msgv[0], 2) == 0) {
		return VL53L1_ERROR_NONE;
	}

	return VL53L1_ERROR_CONTROL_INTERFACE;
}

VL53L1_Error VL53L1_RdWord(VL53L1_DEV Dev, uint16_t index, uint16_t *data)
{
	uint8_t tx_buf[2] = {index >> 8, index & 0xFF};
	uint8_t rx_buf[2];

	//usleep(1300);

	if (Dev->dev == NULL) {
		PX4_ERR("I2C device not opened");
		return VL53L1_ERROR_CONTROL_INTERFACE;
	}

	struct i2c_msg_s msgv[2];

	{
		msgv[0].frequency = 400000;
		msgv[0].addr = Dev->I2cDevAddr;
		msgv[0].flags = 0;
		msgv[0].buffer = &tx_buf[0];
		msgv[0].length = sizeof(tx_buf);
	}

	{
		msgv[1].frequency = 400000;
		msgv[1].addr = Dev->I2cDevAddr;
		msgv[1].flags = I2C_M_READ;
		msgv[1].buffer = rx_buf;
		msgv[1].length = sizeof(rx_buf);
	}

	if (I2C_TRANSFER(Dev->dev, &msgv[0], 2) == 0) {
		*data = ((uint16_t)rx_buf[0] << 8) + (uint16_t)rx_buf[1];
		return VL53L1_ERROR_NONE;
	}

	return VL53L1_ERROR_CONTROL_INTERFACE;
}

VL53L1_Error VL53L1_GetTickCount(uint32_t *ptick_count_ms)
{
	VL53L1_Error status = VL53L1_ERROR_NONE;

	*ptick_count_ms = hrt_absolute_time() / 1000;

	return status;
}

VL53L1_Error VL53L1_GetTimerFrequency(int32_t *ptimer_freq_hz)
{
	VL53L1_Error status = VL53L1_ERROR_NONE;
	return status;
}

VL53L1_Error VL53L1_WaitMs(VL53L1_Dev_t *pdev, int32_t wait_ms)
{
	VL53L1_Error status = VL53L1_ERROR_NONE;
	usleep(wait_ms * 1000);
	return status;
}

VL53L1_Error VL53L1_WaitUs(VL53L1_Dev_t *pdev, int32_t wait_us)
{
	VL53L1_Error status = VL53L1_ERROR_NONE;
	usleep(wait_us);
	return status;
}

VL53L1_Error VL53L1_WaitValueMaskEx(
	VL53L1_Dev_t *pdev,
	uint32_t      timeout_ms,
	uint16_t      index,
	uint8_t       value,
	uint8_t       mask,
	uint32_t      poll_delay_ms)
{
	VL53L1_Error status  = VL53L1_ERROR_NONE;
	return status;
}




