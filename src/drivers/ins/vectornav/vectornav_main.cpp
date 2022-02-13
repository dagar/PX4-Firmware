/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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

#include "VectorNav.hpp"

#include <px4_platform_common/getopt.h>

namespace vectornav
{

VectorNav *g_dev{nullptr};

static int start(const char *port)
{
	if (g_dev != nullptr) {
		PX4_ERR("already started");
		return PX4_OK;
	}

	// Instantiate the driver.
	g_dev = new VectorNav(port);

	if (g_dev == nullptr) {
		PX4_ERR("driver start failed");
		return PX4_ERROR;
	}

	if (OK != g_dev->init()) {
		PX4_ERR("driver start failed");
		delete g_dev;
		g_dev = nullptr;
		return PX4_ERROR;
	}

	return PX4_OK;
}

static int status()
{
	if (g_dev == nullptr) {
		PX4_ERR("driver not running");
		return 1;
	}

	printf("state @ %p\n", g_dev);
	g_dev->print_info();

	return 0;
}

static int stop()
{
	if (g_dev != nullptr) {
		PX4_INFO("stopping driver");
		delete g_dev;
		g_dev = nullptr;
		PX4_INFO("driver stopped");

	} else {
		PX4_ERR("driver not running");
		return 1;
	}

	return PX4_OK;
}

static int usage()
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

Serial bus driver for the VectorNav VN-100, VN-200, VN-300.

Most boards are configured to enable/start the driver on a specified UART using the SENS_VN_CFG parameter.

Setup/usage information: https://docs.px4.io/master/en/sensor/vectornav.html

### Examples

Attempt to start driver on a specified serial device.
$ vectornav start -d /dev/ttyS1
Stop driver
$ vectornav stop
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("vectornav", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("distance_sensor");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start driver");
	PRINT_MODULE_USAGE_PARAM_STRING('d', nullptr, nullptr, "Serial device", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("status","Driver status");
	PRINT_MODULE_USAGE_COMMAND_DESCR("stop","Stop driver");
	PRINT_MODULE_USAGE_COMMAND_DESCR("status","Print driver status");
	return PX4_OK;
}

} // namespace





#include <stdio.h>
#include <inttypes.h>

/* Include files needed to use VnSensor. */
#include "vn/sensors.h"

void asciiAsyncMessageReceived(void *userData, VnUartPacket *packet, size_t runningIndex);
void asciiOrBinaryAsyncMessageReceived(void *userData, VnUartPacket *packet, size_t runningIndex);
int processErrorReceived(char *errorMessage, VnError errorCode);

extern "C" __EXPORT int vectornav_main(int argc, char *argv[])
{
	/* This example walks through using the VectorNav C Library to connect to
	 * and interact with a VectorNav sensor using the VnSensor structure. */

	/* First determine which COM port your sensor is attached to and update the
	 * constant below. Also, if you have changed your sensor from the factory
	 * default baudrate of 115200, you will need to update the baudrate
	 * constant below as well. */
	const char SENSOR_PORT[] = "COM1";	/* Windows format for physical and virtual (USB) serial port. */
	/*const char SENSOR_PORT[] = "/dev/ttyS1"; */ /* Linux format for physical serial port. */
	/*const char SENSOR_PORT[] = "/dev/ttyUSB0"; */ /* Linux format for virtual (USB) serial port. */
	/*const char SENSOR_PORT[] = "/dev/tty.usbserial-FTXXXXXX"; */ /* Mac OS X format for virtual (USB) serial port. */
	/*const char SENSOR_PORT[] = "/dev/ttyS0"; */ /* CYGWIN format. Usually the Windows COM port number minus 1. This would connect to COM1. */
	const uint32_t SENSOR_BAUDRATE = 115200;

	/* We first need to initialize our VnSensor structure. */
	VnSensor vs;
	VnSensor_initialize(&vs);

	VnError error;

	/* Now connect to our sensor. */
	if ((error = VnSensor_connect(&vs, SENSOR_PORT, SENSOR_BAUDRATE)) != E_NONE) {
		return processErrorReceived("Error connecting to sensor.", error);
	}

	/* Let's query the sensor's model number. */
	char modelNumber[30];

	if ((error = VnSensor_readModelNumber(&vs, modelNumber, sizeof(modelNumber))) != E_NONE) {
		return processErrorReceived("Error reading model number.", error);
	}

	printf("Model Number: %s\n", modelNumber);

	/* Get some orientation data from the sensor. */
	vec3f ypr;

	if ((error = VnSensor_readYawPitchRoll(&vs, &ypr)) != E_NONE) {
		return processErrorReceived("Error reading yaw pitch roll.", error);
	}

	char strConversions[50];
	str_vec3f(strConversions, ypr);
	printf("Current YPR: %s\n", strConversions);

	/* Get some orientation and IMU data. */
	YawPitchRollMagneticAccelerationAndAngularRatesRegister reg;

	if ((error = VnSensor_readYawPitchRollMagneticAccelerationAndAngularRates(&vs, &reg)) != E_NONE) {
		return processErrorReceived("Error reading orientation and IMU data.", error);
	}

	str_vec3f(strConversions, reg.yawPitchRoll);
	printf("Current YPR: %s\n", strConversions);
	str_vec3f(strConversions, reg.mag);
	printf("Current Magnetic: %s\n", strConversions);
	str_vec3f(strConversions, reg.accel);
	printf("Current Acceleration: %s\n", strConversions);
	str_vec3f(strConversions, reg.gyro);
	printf("Current Angular Rates: %s\n", strConversions);

	/* Let's do some simple reconfiguration of the sensor. As it comes from the
	 * factory, the sensor outputs asynchronous data at 40 Hz. We will change
	 * this to 2 Hz for demonstration purposes. */
	uint32_t oldHz;

	if ((error = VnSensor_readAsyncDataOutputFrequency(&vs, &oldHz)) != E_NONE) {
		return processErrorReceived("Error reading async data output frequency.", error);
	}

	if ((error = VnSensor_writeAsyncDataOutputFrequency(&vs, 2, true)) != E_NONE) {
		return processErrorReceived("Error writing async data output frequency.", error);
	}

	uint32_t newHz;

	if ((error = VnSensor_readAsyncDataOutputFrequency(&vs, &newHz)) != E_NONE) {
		return processErrorReceived("Error reading async data output frequency.", error);
	}

	printf("Old Async Frequency: %d Hz\n", oldHz);
	printf("New Async Frequency: %d Hz\n", newHz);

	/* For the registers that have more complex configuration options, it is
	 * convenient to read the current existing register configuration, change
	 * only the values of interest, and then write the configuration to the
	 * register. This allows preserving the current settings for the register's
	 * other fields. Below, we change the heading mode used by the sensor. */
	VpeBasicControlRegister vpeReg;

	if ((error = VnSensor_readVpeBasicControl(&vs, &vpeReg)) != E_NONE) {
		return processErrorReceived("Error reading VPE basic control.", error);
	}

	strFromHeadingMode(strConversions, (VnHeadingMode)vpeReg.headingMode);
	printf("Old Heading Mode: %s\n", strConversions);
	vpeReg.headingMode = VNHEADINGMODE_ABSOLUTE;

	if ((error = VnSensor_writeVpeBasicControl(&vs, vpeReg, true)) != E_NONE) {
		return processErrorReceived("Error writing VPE basic control.", error);
	}

	if ((error = VnSensor_readVpeBasicControl(&vs, &vpeReg)) != E_NONE) {
		return processErrorReceived("Error reading VPE basic control.", error);
	}

	strFromHeadingMode(strConversions, (VnHeadingMode)vpeReg.headingMode);
	printf("New Heading Mode: %s\n", strConversions);

	/* Up to now, we have shown some examples of how to configure the sensor
	 * and query for the latest measurements. However, this querying is a
	 * relatively slow method for getting measurements since the CPU has to
	 * send out the command to the sensor and also wait for the command
	 * response. An alternative way of receiving the sensor's latest
	 * measurements without the waiting for a query response, you can configure
	 * the library to alert you when new asynchronous data measurements are
	 * received. We will illustrate hooking up to our current VnSensor to
	 * receive these notifications of asynchronous messages. */

	/* First let's configure the sensor to output a known asynchronous data
	 * message type. */
	if ((error = VnSensor_writeAsyncDataOutputType(&vs, VNYPR, true)) != E_NONE) {
		return processErrorReceived("Error writing to async data output type.", error);
	}

	VnAsciiAsync asyncType;

	if ((error = VnSensor_readAsyncDataOutputType(&vs, &asyncType)) != E_NONE) {
		return processErrorReceived("Error reading async data output type.", error);
	}

	strFromVnAsciiAsync(strConversions, asyncType);
	printf("ASCII Async Type: %s\n", strConversions);

	/* You will need to define a method which has the appropriate
	 * signature for receiving notifications. This is implemented with the
	 * method asciiAsyncMessageReceived. Now we register the method with the
	 * VnSensor structure. */
	VnSensor_registerAsyncPacketReceivedHandler(&vs, asciiAsyncMessageReceived, NULL);

	/* Now sleep for 5 seconds so that our asynchronous callback method can
	 * receive and display receive yaw, pitch, roll packets. */
	printf("Starting sleep...\n");
	VnThread_sleepSec(5);

	/* Unregister our callback method. */
	VnSensor_unregisterAsyncPacketReceivedHandler(&vs);

	/* As an alternative to receiving notifications of new ASCII asynchronous
	 * messages, the binary output configuration of the sensor is another
	 * popular choice for receiving data since it is compact, fast to parse,
	 * and can be output at faster rates over the same connection baudrate.
	 * Here we will configure the binary output register and process packets
	 * with a new callback method that can handle both ASCII and binary
	 * packets. */

	/* First we create a structure for setting the configuration information
	 * for the binary output register to send yaw, pitch, roll data out at
	 * 4 Hz. */




	BinaryOutputRegister bor;
	//CommonGroup commonField = COMMONGROUP_TIMESTARTUP | COMMONGROUP_YAWPITCHROLL;
	BinaryOutputRegister_initialize(
		&bor,
		ASYNCMODE_BOTH,
		200,
		COMMONGROUP_YAWPITCHROLL,
		TIMEGROUP_NONE,
		IMUGROUP_NONE,
		GPSGROUP_NONE,
		ATTITUDEGROUP_NONE,
		INSGROUP_NONE,
		GPSGROUP_NONE);

	if ((error = VnSensor_writeBinaryOutput1(&vs, &bor, true)) != E_NONE) {
		return processErrorReceived("Error writing binary output 1.", error);
	}

	VnSensor_registerAsyncPacketReceivedHandler(&vs, asciiOrBinaryAsyncMessageReceived, NULL);

	printf("Starting sleep...\n");
	VnThread_sleepSec(5);

	VnSensor_unregisterAsyncPacketReceivedHandler(&vs);

	/* Now disconnect from the sensor since we are finished. */
	if ((error = VnSensor_disconnect(&vs)) != E_NONE) {
		return processErrorReceived("Error disconnecting from sensor.", error);
	}

	return 0;
}

/* This is our basic callback handler for notifications of new asynchronous
 * data packets received. The userData parameter is a pointer to the data we
 * supplied when we called registerAsyncPacketReceivedHandler. In this case
 * we didn't need any user data so we just set this to NULL. Alternatively you
 * can provide a pointer to user data which you can use in the callback method.
 * One use for this is help in calling back to a member method instead of just
 * a global or static method. The Packet p parameter is an encapsulation of
 * the data packet. At this state, it has already been validated and identified
 * as an asynchronous data message. However, some processing is required on the
 * user side to make sure it is the right type of asynchronous message type so
 * we can parse it correctly. The index parameter is an advanced usage item and
 * can be safely ignored for now. */
void asciiAsyncMessageReceived(void *userData, VnUartPacket *packet, size_t runningIndex)
{
	vec3f ypr;
	char strConversions[50];

	/* Silence 'unreferenced formal parameters' warning in Visual Studio. */
	(userData);
	(runningIndex);

	/* Make sure we have an ASCII packet and not a binary packet. */
	if (VnUartPacket_type(packet) != PACKETTYPE_ASCII) {
		return;
	}

	/* Make sure we have a VNYPR data packet. */
	if (VnUartPacket_determineAsciiAsyncType(packet) != VNYPR) {
		return;
	}

	/* We now need to parse out the yaw, pitch, roll data. */
	VnUartPacket_parseVNYPR(packet, &ypr);

	/* Now print out the yaw, pitch, roll measurements. */
	str_vec3f(strConversions, ypr);
	printf("ASCII Async YPR: %s\n", strConversions);
}

void asciiOrBinaryAsyncMessageReceived(void *userData, VnUartPacket *packet, size_t runningIndex)
{
	vec3f ypr;
	char strConversions[50];

	/* Silence 'unreferenced formal parameters' warning in Visual Studio. */
	(userData);
	(runningIndex);

	if (VnUartPacket_type(packet) == PACKETTYPE_ASCII && VnUartPacket_determineAsciiAsyncType(packet) == VNYPR) {
		VnUartPacket_parseVNYPR(packet, &ypr);
		str_vec3f(strConversions, ypr);
		printf("ASCII Async YPR: %s\n", strConversions);

		return;
	}

	if (VnUartPacket_type(packet) == PACKETTYPE_BINARY) {
		uint64_t timeStartup;

		/* First make sure we have a binary packet type we expect since there
		 * are many types of binary output types that can be configured. */
		// COMMONGROUP_TIMESTARTUP | COMMONGROUP_YAWPITCHROLL
		if (!VnUartPacket_isCompatible(packet,
					       COMMONGROUP_YAWPITCHROLL,
					       TIMEGROUP_NONE,
					       IMUGROUP_NONE,
					       GPSGROUP_NONE,
					       ATTITUDEGROUP_NONE,
					       INSGROUP_NONE,
					       GPSGROUP_NONE))
			/* Not the type of binary packet we are expecting. */
		{
			return;
		}

		/* Ok, we have our expected binary output packet. Since there are many
		 * ways to configure the binary data output, the burden is on the user
		 * to correctly parse the binary packet. However, we can make use of
		 * the parsing convenience methods provided by the Packet structure.
		 * When using these convenience methods, you have to extract them in
		 * the order they are organized in the binary packet per the User Manual. */
		timeStartup = VnUartPacket_extractUint64(packet);
		ypr = VnUartPacket_extractVec3f(packet);

		str_vec3f(strConversions, ypr);
		printf("Binary Async TimeStartup: %" PRIu64 "\n", timeStartup);
		printf("Binary Async YPR: %s\n", strConversions);

		return;
	}
}

int processErrorReceived(char *errorMessage, VnError errorCode)
{
	char errorCodeStr[100];
	strFromVnError(errorCodeStr, errorCode);
	printf("%s\nERROR: %s\n", errorMessage, errorCodeStr);
	return -1;
}







#if 0
extern "C" __EXPORT int vectornav_main(int argc, char *argv[])
{
	int ch = 0;
	const char *device_path = nullptr;
	int myoptind = 1;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "d:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			device_path = myoptarg;
			break;

		default:
			PX4_WARN("Unknown option!");
			return PX4_ERROR;
		}
	}

	if (myoptind >= argc) {
		PX4_ERR("unrecognized command");
		return vectornav::usage();
	}

	if (!strcmp(argv[myoptind], "start")) {
		if (strcmp(device_path, "") != 0) {
			return vectornav::start(device_path);

		} else {
			PX4_WARN("Please specify device path!");
			return vectornav::usage();
		}

	} else if (!strcmp(argv[myoptind], "stop")) {
		return vectornav::stop();

	} else if (!strcmp(argv[myoptind], "status")) {
		return vectornav::status();
	}

	return vectornav::usage();
}
#endif
