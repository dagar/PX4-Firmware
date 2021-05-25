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

/**
 * @file AFBRS50.hpp
 *
 * Driver for the Broadcom AFBR-S50 connected via SPI.
 *
 */

#ifndef _AFBRS50_H
#define _AFBRS50_H

#include <drivers/device/device.h>
#include <drivers/drv_hrt.h>

#include <uORB/topics/distance_sensor.h>

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/tasks.h>

#include <lib/drivers/rangefinder/PX4Rangefinder.hpp>

/*! Define the SPI slave (to be used in the SPI module). */
#define SPI_SLAVE 2

/*! Define the SPI baud rate (to be used in the SPI module). */
#define SPI_BAUD_RATE 5000000

#define MAX_DETECTABLE_DISTANCE          30.0f
#define MIN_DETECTABLE_DISTANCE          0.01f
#define NUM_SAMPLES_CONSISTENT           5
#define MAX_SAMPLE_DEVIATION             0.15f
class AFBRS50 : public ModuleBase<AFBRS50>
{
public:
	AFBRS50();
	~AFBRS50() override;

	/**
	 * @see ModuleBase::custom_command().
	 * @brief main Main entry point to the module that should be
	 *        called directly from the module's main method.
	 * @param argc The input argument count.
	 * @param argv Pointer to the input argument array.
	 * @return Returns 0 iff successful, -1 otherwise.
	 */
	static int custom_command(int argc, char *argv[]);

	/**
	 * @see ModuleBase::instantiate().
	 * @brief Instantiates the pga460 object.
	 * @param argc The input argument count.
	 * @param argv Pointer to the input argument array.
	 */
	static PGA460 *instantiate(int argc, char *argv[]);

	/**
	 * @see ModuleBase::print_usage().
	 * @brief Prints the module usage to the nuttshell console.
	 * @param reason The requested reason for printing to console.
	 */
	static int print_usage(const char *reason = nullptr);

	/**
	 * @see ModuleBase::task_spawn().
	 */
	static int task_spawn(int argc, char *argv[]);

	/**
	 * @see ModuleBase::run().
	 */
	void run() override;

private:

	/**
	 * @brief
	 * @return Returns PX4_OK upon success or PX4_ERROR on fail.
	 */
	int init();

	/**
	 * @brief Commands the device to perform an ultrasonic measurement.
	 */
	int take_measurement(const uint8_t mode);

	/**
	 * @brief Commands the device to publish the measurement results to uORB.
	 * @param dist The calculated distance to the object.
	 */
	void uORB_publish_results(const float dist);

	/** @orb_advert_t orb_advert_t uORB advertisement topic. */
	orb_advert_t _distance_sensor_topic{nullptr};

	/** @param _fd Returns the file descriptor from open(). */
	int _fd{-1};

	/** @param _start_loop The starting value for the loop time of the main loop. */
	uint64_t _start_loop{0};

	device::Device::DeviceId _device_id;
};

#endif
