/*************************************************************************//**
 * @file
 * @brief    	This file is part of the AFBR-S50 SDK example application.
 *
 * @copyright	Copyright c 2016-2019, Avago Technologies GmbH.
 * 				All rights reserved.
 *****************************************************************************/


/*******************************************************************************
 * Include Files
 ******************************************************************************/
#include "argus.h"

#include "driver/gpio.h"
#include "driver/s2pi.h"
#include "driver/uart.h"
#include "driver/timer.h"

/*******************************************************************************
 * Defines
 ******************************************************************************/

/*! Define the SPI slave (to be used in the SPI module). */
#define SPI_SLAVE 1

/*! Define the SPI baud rate (to be used in the SPI module). */
#define SPI_BAUD_RATE 6000000

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*! Global raw data variable. */
static volatile void * myData = 0;

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

/*!***************************************************************************
 * @brief	Application entry point.
 *
 * @details	The main function of the program, called after startup code
 * 			This function should never be exited.
 *****************************************************************************/
int main(void)
{
	/* The API module handle that contains all data definitions that is
	 * required within the API module for the corresponding hardware device.
	 * Every call to an API function requires the passing of a pointer to this
	 * data structure. */
	argus_hnd_t * hnd = Argus_CreateHandle();

	if (hnd == 0)
	{
		/* Error Handling ...*/
	}

	/* Init GPIO ports. */
	GPIO_Init();

	/* Initialize timer required by the API. */
	Timer_Init();

	/* Initialize UART for print functionality. */
	UART_Init();

	/* Initialize the S2PI hardware required by the API. */
	S2PI_Init(SPI_SLAVE, SPI_BAUD_RATE);

	/* Initialize the API with default values.
	 * This implicitly calls the initialization functions
	 * of the underlying API modules.
	 *
	 * The second parameter is stored and passed to all function calls
	 * to the S2PI module. This piece of information can be utilized in
	 * order to determine the addressed SPI slave and enabled the usage
	 * of multiple devices on a single SPI peripheral. */
	status_t status = Argus_Init(hnd, SPI_SLAVE);

	if (status != STATUS_OK)
	{
		/* Error Handling ...*/
	}

	/* Print some information about current API and connected device. */
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

	/* Adjust some configuration parameters by invoking the dedicated API methods. */
	Argus_SetConfigurationFrameTime(hnd, 100000); // 0.1 second = 10 Hz

	/* The program loop ... */
	for(;;)
	{
		myData = 0;

		/* Triggers a single measurement.
		 * Note that due to the laser safety algorithms, the method might refuse
		 * to restart a measurement when the appropriate time has not been elapsed
		 * right now. The function returns with status #STATUS_ARGUS_POWERLIMIT and
		 * the function must be called again later. Use the frame time configuration
		 * in order to adjust the timing between two measurement frames. */
		status = Argus_TriggerMeasurement(hnd, measurement_ready_callback);
		if (status == STATUS_ARGUS_POWERLIMIT) {
			/* Not ready (due to laser safety) to restart the measurement yet.
			 * Come back later. */
			__asm("nop");
		} else if (status != STATUS_OK) {
			/* Error Handling ...*/
		} else {
			/* Wait until measurement data is ready. */
			do {
				status = Argus_GetStatus(hnd);
			} while(status == STATUS_BUSY);

			if (status != STATUS_OK) {
				/* Error Handling ...*/
			} else {
				/* The measurement data structure. */
				argus_results_t res = 0;

				/* Evaluate the raw measurement results. */
				status = Argus_EvaluateData(hnd, &res, (void*)myData);

				if (status != STATUS_OK) {
					/* Error Handling ...*/
				} else {
					/* Use the recent measurement results
					 * (converting the Q9.22 value to float and print or display it). */
					print("Range: %d mm\n", res.Bin.Range / (Q9_22_ONE / 1000));
				}
			}
		}
	}
}

status_t measurement_ready_callback(status_t status, void * data)
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
