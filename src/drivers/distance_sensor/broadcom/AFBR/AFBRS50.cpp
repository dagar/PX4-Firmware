/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/* Include Files */
#include "AFBRS50.hpp"
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <lib/drivers/device/Device.hpp>

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*!***************************************************************************
 * @brief	printf-like function to send print messages via UART.
 *
 * @details Defined in "driver/uart.c" source file.
 *
 * 			Open an UART connection with 115200 bps, 8N1, no handshake to
 * 			receive the data on a computer.
 *
 * @param	fmt_s The usual printf parameters.
 *
 * @return 	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
extern status_t print(const char  *fmt_s, ...);

/*!***************************************************************************
 * @brief	Initialization routine for board hardware and peripherals.
 *****************************************************************************/
static void hardware_init(void);

/*!***************************************************************************
 * @brief	Measurement data ready callback function.
 *
 * @details
 *
 * @param	status *
 * @param	data *
 * @return 	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t measurement_ready_callback(status_t status, void * data);

AFBRS50::AFBRS50(uint8_t device_orientation):
	ScheduledWorkItem(MODULE_NAME, px4::ins_instance_to_wq(0)),
	_px4_rangefinder(0, device_orientation)
{
	device::Device::DeviceId device_id;
	device_id.devid_s.bus_type = device::Device::DeviceBusType::DeviceBusType_SPI;

	uint8_t bus_num = 0;

	if (bus_num < 10) {
		device_id.devid_s.bus = bus_num;
	}

	_px4_rangefinder.set_device_id(device_id.devid);
	_px4_rangefinder.set_device_type(DRV_DIST_DEVTYPE_AFBRS50);

	_px4_rangefinder.set_max_distance(AFBRS50_MAX_DISTANCE);
	_px4_rangefinder.set_min_distance(AFBRS50_MIN_DISTANCE);
	_px4_rangefinder.set_fov(AFBRS50_FIELD_OF_VIEW);
}

AFBRS50::~AFBRS50()
{
	stop();

	perf_free(_comms_error);
	perf_free(_sample_perf);
}

int
AFBRS50::collect()
{
	// From example.c
	myData = 0;
	/* Triggers a single measurement.
		* Note that due to the laser safety algorithms, the method might refuse
		* to restart a measurement when the appropriate time has not been elapsed
		* right now. The function returns with status #STATUS_ARGUS_POWERLIMIT and
		* the function must be called again later. Use the frame time configuration
		* in order to adjust the timing between two measurement frames. */
	status = Argus_TriggerMeasurement(hnd, measurement_ready_callback);
	if (status == STATUS_ARGUS_POWERLIMIT)
	{
		/* Not ready (due to laser safety) to restart the measurement yet.
			* Come back later. */
		__asm("nop");
	}
	else if (status != STATUS_OK)
	{
		/* Error Handling ...*/
	}
	else
	{
		/* Wait until measurement data is ready. */
		do
		{
			status = Argus_GetStatus(hnd);
			__asm("nop");
		}
		while(status == STATUS_BUSY);

		if (status != STATUS_OK)
		{
			/* Error Handling ...*/
		}

		else
		{
			/* The measurement data structure. */
			argus_results_t res;

			/* Evaluate the raw measurement results. */
			status = Argus_EvaluateData(hnd, &res, (void*)myData);

			if (status != STATUS_OK)
			{
				/* Error Handling ...*/
			}

			else
			{
				/* Use the recent measurement results
					* (converting the Q9.22 value to float and print or display it). */
				float result = res.Bin.Range / (Q9_22_ONE / 1000);
				print("Range: %d mm\r\n", result);
				return result;
			}
		}
	}




	/*
	perf_begin(_sample_perf);

	//const int buffer_size = sizeof(_buffer);
	//const int message_size = sizeof(reading_msg);

	//int bytes_read = ::read(_file_descriptor, _buffer + _buffer_len, buffer_size - _buffer_len);

	//if (bytes_read < 1) {
		// Trigger a new measurement.
		// return measure();
	//}

	//_buffer_len += bytes_read;

	//if (_buffer_len < message_size) {
		// Return on next scheduled cycle to collect remaining data.
	//	return PX4_OK;
	//}

	// NOTE: little-endian support only.
	uint16_t distance_mm = 0;
	float distance_m = static_cast<float>(distance_mm) / 1000.0f;

	// @TODO - implement a meaningful signal quality value.
	int8_t signal_quality = -1;

	_px4_rangefinder.update(_measurement_time, distance_m, signal_quality);

	perf_end(_sample_perf);

	// Trigger the next measurement.
	return measure();

	*/
}

int
AFBRS50::init()
{
	// From example.c
	argus_hnd_t * hnd = Argus_CreateHandle();
	if (hnd == 0)
	{
		print("ERROR: Handle not initialized\r\n");
	}
	hardware_init();
	status_t status = Argus_Init(hnd, SPI_SLAVE);
	if (status != STATUS_OK)
	{
		print("ERROR: Init status not okay: %i\r\n", status);
	}
	uint32_t value = Argus_GetAPIVersion();
	uint8_t a = (value >> 24) & 0xFFU;
	uint8_t b = (value >> 16) & 0xFFU;
	uint8_t c = value & 0xFFFFU;
	uint32_t id = Argus_GetChipID(hnd);
	argus_module_version_t mv = Argus_GetModuleVersion(hnd);
	print("\n##### AFBR-S50 API - Simple Example ##############\n"
		  "  API Version: v%d.%d.%d\n"
		  "  Chip ID:     %d\n"
		  "  Module:      %s\n"
		  "##################################################\n",
		  a, b, c, id,
		  mv == AFBR_S50MV85G_V1 ? "AFBR-S50MV85G (v1)" :
		  mv == AFBR_S50MV85G_V2 ? "AFBR-S50MV85G (v2)" :
		  mv == AFBR_S50MV85G_V3 ? "AFBR-S50MV85G (v3)" :
		  mv == AFBR_S50LV85D_V1 ? "AFBR-S50LV85D (v1)" :
		  mv == AFBR_S50MV68B_V1 ? "AFBR-S50MV68B (v1)" :
		  mv == AFBR_S50MV85I_V1 ? "AFBR-S50MV85I (v1)" :
		  mv == AFBR_S50SV85K_V1 ? "AFBR-S50SV85K (v1)" :
		  "unknown");
	Argus_SetConfigurationFrameTime( hnd, 100000 ); // 0.1 second = 10 Hz


	hrt_abstime time_now = hrt_absolute_time();

	const hrt_abstime timeout_usec = time_now + 500000_us; // 0.5sec

	while (time_now < timeout_usec) {
		if (measure() == PX4_OK) {
			px4_usleep(AFBRS50_MEASURE_INTERVAL);

			if (collect() == PX4_OK) {
				// The file descriptor can only be accessed by the process that opened it,
				// so closing here allows the port to be opened from scheduled work queue.
				stop();
				return PX4_OK;
			}
		}

		px4_usleep(1000);
		time_now = hrt_absolute_time();
	}

	PX4_ERR("No readings from AFBRS50");
	return PX4_ERROR;
}

int
AFBRS50::measure()
{
	_measurement_time = hrt_absolute_time();
	return PX4_OK;
}

void
AFBRS50::print_info()
{
	perf_print_counter(_comms_error);
	perf_print_counter(_sample_perf);
}

void
AFBRS50::Run()
{
	collect();
}

void
AFBRS50::start()
{
	// Schedule the driver at regular intervals.
	ScheduleOnInterval(AFBRS50_MEASURE_INTERVAL, AFBRS50_MEASURE_INTERVAL);
}

void
AFBRS50::stop()
{
	// Clear the work queue schedule.
	ScheduleClear();
}

static void AFBRS50::hardware_init(void)
{
	/* Initialize the board with clocks. */
	BOARD_ClockInit();

	/* Disable the watchdog timer. */
	COP_Disable();

	/* Init GPIO ports. */
	GPIO_Init();

	/* Initialize timer required by the API. */
	Timer_Init();

	/* Initialize UART for print functionality. */
	UART_Init();

	/* Initialize the S2PI hardware required by the API. */
	S2PI_Init(SPI_SLAVE, SPI_BAUD_RATE);
}

status_t AFBRS50::measurement_ready_callback(status_t status, void * data)
{
	if (status != STATUS_OK)
	{
		/* Error Handling ...*/
	}
	else
	{
		/* Inform the main task about new data ready.
		 * Note: do not call the evaluate measurement method
		 * from within this callback since it is invoked in
		 * a interrupt service routine and should return as
		 * soon as possible. */
		assert(myData == 0);

		myData = data;
	}
	return status;
}
