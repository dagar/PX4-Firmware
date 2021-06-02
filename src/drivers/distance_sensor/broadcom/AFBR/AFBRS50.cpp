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

#include <lib/drivers/device/Device.hpp>

#define AFBRS50_FIELD_OF_VIEW        (0.105f) // 6 deg cone angle.
#define AFBRS50_MAX_DISTANCE         30.0f
#define AFBRS50_MIN_DISTANCE         0.01f
#define AFBRS50_MEASURE_INTERVAL     (1000000 / 10) // 10Hz

/*! Define the SPI slave (to be used in the SPI module). */
#define SPI_SLAVE 2
/*! Define the SPI baud rate (to be used in the SPI module). */
#define SPI_BAUD_RATE 5000000

#include "s2pi.h"
#include "timer.h"

static argus_hnd_t *_hnd{nullptr};
static volatile void *_myData{nullptr};

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

int AFBRS50::init()
{
	_hnd = Argus_CreateHandle();

	if (_hnd == 0) {
		PX4_ERR("ERROR: Handle not initialized\r\n");
		Argus_DestroyHandle(_hnd);
		return PX4_ERROR;
	}

	/* Initialize the S2PI hardware required by the API. */
	printf("S2PI_Init\n");
	S2PI_Init(SPI_SLAVE, SPI_BAUD_RATE);

	printf("Argus_Init\n");
	status_t status = Argus_Init(_hnd, SPI_SLAVE);

	if (status != STATUS_OK) {
		PX4_ERR("ERROR: Init status not okay: %i\r\n", status);
		Argus_Deinit(_hnd);
		Argus_DestroyHandle(_hnd);
		return PX4_ERROR;
	}

	uint32_t value = Argus_GetAPIVersion();
	PX4_INFO("AFBR API Verion %d", value);
	uint8_t a = (value >> 24) & 0xFFU;
	uint8_t b = (value >> 16) & 0xFFU;
	uint8_t c = value & 0xFFFFU;
	uint32_t id = Argus_GetChipID(_hnd);
	argus_module_version_t mv = Argus_GetModuleVersion(_hnd);

	PX4_INFO("\n##### AFBR-S50 API - Simple Example ##############\n"
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

	//Argus_SetConfigurationFrameTime( _hnd, 100000 ); // 0.1 second = 10 Hz

	// Schedule the driver at regular intervals.
	//ScheduleOnInterval(AFBRS50_MEASURE_INTERVAL);

	return PX4_OK;
}

static status_t measurement_ready_callback(status_t status, void *data)
{
	if (status != STATUS_OK) {
		/* Error Handling ...*/
	} else {
		/* Inform the main task about new data ready.
		 * Note: do not call the evaluate measurement method
		 * from within this callback since it is invoked in
		 * a interrupt service routine and should return as
		 * soon as possible. */
		assert(_myData == 0);

		_myData = data;
	}

	return status;
}

void AFBRS50::Run()
{
	// From example.c
	_myData = nullptr;

	/* Triggers a single measurement.
	* Note that due to the laser safety algorithms, the method might refuse
	* to restart a measurement when the appropriate time has not been elapsed
	* right now. The function returns with status #STATUS_ARGUS_POWERLIMIT and
	* the function must be called again later. Use the frame time configuration
	* in order to adjust the timing between two measurement frames.
	*/
	status_t status = Argus_TriggerMeasurement(_hnd, measurement_ready_callback);

	if (status == STATUS_ARGUS_POWERLIMIT) {
		/* Not ready (due to laser safety) to restart the measurement yet. Come back later. */

	} else if (status != STATUS_OK) {
		/* Error Handling ...*/
	} else {
		/* Wait until measurement data is ready. */
		do {
			status = Argus_GetStatus(_hnd);

		} while (status == STATUS_BUSY);

		if (status != STATUS_OK) {
			/* Error Handling ...*/
		} else {
			// Evaluate the raw measurement results.
			argus_results_t res{};
			status = Argus_EvaluateData(_hnd, &res, (void *)_myData);

			if (status != STATUS_OK) {
				/* Error Handling ...*/

			} else {
				// Use the recent measurement results
				// (converting the Q9.22 value to float and print or display it).
				float result = res.Bin.Range / (Q9_22_ONE / 1000);
				PX4_INFO("Range: %f mm\r\n", (double)result);
			}
		}
	}
}

void AFBRS50::stop()
{
	// Clear the work queue schedule.
	ScheduleClear();
}

void AFBRS50::print_info()
{
	perf_print_counter(_comms_error);
	perf_print_counter(_sample_perf);
}
