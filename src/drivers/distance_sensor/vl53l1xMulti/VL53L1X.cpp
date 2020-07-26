/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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
 * @file vl53l1x.cpp
 *
 * Driver for the vl53l1x ToF Sensor from ST Microelectronics connected via I2C.
 */

#include "VL53L1X.hpp"

#define _BSD_SOURCE
#include <endian.h>

//#define VL53L1X_DEBUG 1

using namespace time_literals;

VL53L1X::VL53L1X(uint8_t rotation, int bus, int address) :
	I2C(DRV_DIST_DEVTYPE_VL53L1X, MODULE_NAME, bus, address, 400000),
	ScheduledWorkItem(MODULE_NAME, px4::device_bus_to_wq(get_device_id())),
	_pca9505(bus),
	_sample_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": read")),
	_measure_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": measure")),
	_no_measurement_perf(perf_alloc(PC_COUNT, MODULE_NAME": no measurement available")),
	_comms_errors(perf_alloc(PC_COUNT, MODULE_NAME": communication error"))
{
	//_debug_enabled = true;

	for (int i = 0; i < 12; i++) {
		_px4_dist[i] = nullptr;
	}

	// up the retries since the device misses the first measure attempts
	I2C::_retries = 3;
}

VL53L1X::~VL53L1X()
{
	// make sure we are truly inactive
	stop();

	// free perf counters
	perf_free(_measure_perf);
	perf_free(_sample_perf);
	perf_free(_no_measurement_perf);
	perf_free(_comms_errors);
}

int
VL53L1X::init()
{
	if (_pca9505.init() != PX4_OK) {
		PX4_ERR("PCA9505 init failed");
	}

	/* do I2C init (and probe) first */
	if (I2C::init() != OK) {
		return PX4_ERROR;
	}

	// initialize VL53L1 lidar
	printf("\n");

	for (int i = 0; i < 3; i++) {
		vl53l1_init(0 + i);
		vl53l1_init(3 + i);
		vl53l1_init(6 + i);
		vl53l1_init(9 + i);
		printf("\n");
	}

	printf("\n");

	PX4_INFO("enabling all arms");
	_pca9505.EnableArm(PCA9505::SensorArm::FrontRight | PCA9505::SensorArm::RearRight | PCA9505::SensorArm::RearLeft |
			   PCA9505::SensorArm::FrontLeft);

	start();

	return PX4_OK;
}

int VL53L1X::vl53l1_init(uint8_t instance)
{
	printf("configuring %d: ", instance);

	PCA9505::DIST_SENS sensor = static_cast<PCA9505::DIST_SENS>(instance);
	uint8_t rotation = 0;

	switch (sensor) {
	case PCA9505::DIST_SENS::Forward_0:	// front right arm
	case PCA9505::DIST_SENS::Forward_1:	// front left arm
		rotation = distance_sensor_s::ROTATION_FORWARD_FACING;
		printf("ROTATION_FORWARD_FACING: ");
		break;

	case PCA9505::DIST_SENS::ForwardRight:	// front right arm
		rotation = distance_sensor_s::ROTATION_YAW_45;
		printf("ROTATION_YAW_45: ");
		break;

	case PCA9505::DIST_SENS::Right_0:	// front right arm
	case PCA9505::DIST_SENS::Right_1:	// back right arm
		rotation = distance_sensor_s::ROTATION_RIGHT_FACING;
		printf("ROTATION_RIGHT_FACING: ");
		break;

	case PCA9505::DIST_SENS::BackwardRight:	// back right arm
		rotation = distance_sensor_s::ROTATION_YAW_135;
		printf("ROTATION_YAW_135: ");
		break;

	case PCA9505::DIST_SENS::Backward_0:	// back right arm
	case PCA9505::DIST_SENS::Backward_1:	// back left arm
		rotation = distance_sensor_s::ROTATION_BACKWARD_FACING;
		printf("ROTATION_BACKWARD_FACING: ");
		break;

	case PCA9505::DIST_SENS::BackwardLeft:	// back left arm
		rotation = distance_sensor_s::ROTATION_YAW_225;
		printf("ROTATION_YAW_225: ");
		break;

	case PCA9505::DIST_SENS::Left_0:		// back left arm
	case PCA9505::DIST_SENS::Left_1:		// front left arm
		rotation = distance_sensor_s::ROTATION_LEFT_FACING;
		printf("ROTATION_LEFT_FACING: ");
		break;

	case PCA9505::DIST_SENS::ForwardLeft:	// front left arm
		rotation = distance_sensor_s::ROTATION_YAW_315;
		printf("ROTATION_YAW_315: ");
		break;
	}

	_pca9505.EnableSensor(sensor);
	usleep(5000);

	vl53l1x[instance].dev = _dev;

	set_device_address(VL53L1X_BASEADDR);
	vl53l1x[instance].I2cDevAddr = VL53L1X_BASEADDR;

	VL53L1_Error status = 0;

	// wait device booted
	status = VL53L1_WaitDeviceBooted(&vl53l1x[instance]);

	if (status != VL53L1_ERROR_NONE) {
		printf("WaitDeviceBooted(%d) status: %d\n", instance, status);
		//return PX4_ERROR;
	}

	// update I2C address
	// this function MUST be called prior to VL53L1_DataInit()
	uint8_t final_i2c_addr = VL53L1X_BASEADDR + instance + 1;
	status = VL53L1_SetDeviceAddress(&vl53l1x[instance], final_i2c_addr);

	if (status != VL53L1_ERROR_NONE) {
		//PX4_WARN("VL53L1_SetDeviceAddress(%d) status: %d", instance, status);
	}

	vl53l1x[instance].I2cDevAddr = final_i2c_addr;

	// data init
	status = VL53L1_DataInit(&vl53l1x[instance]);

	if (status != VL53L1_ERROR_NONE) {
		printf(" DataInit(%d) status: %d ", instance, status);
		//return PX4_ERROR;
	}

	// init
	status = VL53L1_StaticInit(&vl53l1x[instance]);

	if (status != VL53L1_ERROR_NONE) {
		printf(" StaticInit(%d) status: %d ", instance, status);
		//return PX4_ERROR;
	}

	// distance mode
	status = VL53L1_SetDistanceMode(&vl53l1x[instance], VL53L1_DISTANCEMODE_SHORT);

	if (status != VL53L1_ERROR_NONE) {
		printf(" SetDistanceMode(%d) status: %d ", instance, status);
		//return PX4_ERROR;
	}

	// measurement timing budget
	// 20 ms is the minimum timing budget and can be used only in Short distance mode.
	// 33 ms is the minimum timing budget which can work for all distance modes.
	// 140 ms is the timing budget which allows the maximum distance of 4 m (in the dark on a white chart) to be reached under Long distance mode
	status = VL53L1_SetMeasurementTimingBudgetMicroSeconds(&vl53l1x[instance], 20000);

	if (status != VL53L1_ERROR_NONE) {
		printf(" SetMeasurementTimingBudgetMicroSeconds(%d) status: %d ", instance, status);
		//return PX4_ERROR;
	}

	// set intermeasurement period in milliseconds (should be greater than timing budget above)
	status = VL53L1_SetInterMeasurementPeriodMilliSeconds(&vl53l1x[instance], 24);

	if (status != VL53L1_ERROR_NONE) {
		printf(" SetInterMeasurementPeriodMilliSeconds(%d) status: %d ", instance, status);
		//return PX4_ERROR;
	}

	// start measurement
	status = VL53L1_StartMeasurement(&vl53l1x[instance]);

	if (status != VL53L1_ERROR_NONE) {
		printf(" StartMeasurement(%d) status: %d ", instance, status);
		//return PX4_ERROR;
	}

	bool success = false;

	// Wait for a successful measurement before creating the
	//  PX4Rangefinder (distance_sensor publication) instance, but don't publish
	for (int retry = 0; retry < 10; retry++) {
		if (measure(sensor, false)) {
			success = true;
			_px4_dist[instance] = new PX4Rangefinder(get_device_id(), ORB_PRIO_DEFAULT, rotation);
			_px4_dist[instance]->set_min_distance(VL53L1X_MIN_RANGING_DISTANCE);
			_px4_dist[instance]->set_max_distance(VL53L1X_MAX_RANGING_DISTANCE);
			_px4_dist[instance]->set_hfov(math::radians(27.0f));
			_px4_dist[instance]->set_vfov(math::radians(27.0f));
			break;

		} else {
			usleep(10000);
		}
	}

	if (success) {
		printf(": success!\n", instance);

	} else {
		printf(": FAILED!\n", instance);
	}

	return PX4_OK;
}

int
VL53L1X::probe()
{
	// TODO: check for existance?

	return PX4_OK;
}

void
VL53L1X::start()
{
	// schedule a cycle to start things
	ScheduleOnInterval(1000);
}

void
VL53L1X::stop()
{
	ScheduleClear();

	for (int i = 0; i < 12; i++) {
		VL53L1_StopMeasurement(&vl53l1x[0]);
	}
}

void
VL53L1X::Run()
{
	perf_begin(_sample_perf);

	int instance = static_cast<int>(_current_sensor);

	if (_px4_dist[instance] != nullptr) {
		// inter-measurement period is 24 milliseconds
		if (hrt_elapsed_time(&_last_update[instance]) >= 24_ms) {
			perf_begin(_measure_perf);
			measure(_current_sensor);
			perf_end(_measure_perf);
		}
	}

	switch (_current_sensor) {
	case PCA9505::DIST_SENS::Forward_0:
		_current_sensor = PCA9505::DIST_SENS::ForwardRight;
		break;

	case PCA9505::DIST_SENS::ForwardRight:
		_current_sensor = PCA9505::DIST_SENS::Right_0;
		break;

	case PCA9505::DIST_SENS::Right_0:
		_current_sensor = PCA9505::DIST_SENS::Backward_0;
		break;

	case PCA9505::DIST_SENS::Backward_0:
		_current_sensor = PCA9505::DIST_SENS::Left_0;
		break;

	case PCA9505::DIST_SENS::Left_0:
		_current_sensor = PCA9505::DIST_SENS::ForwardLeft;
		break;

	case PCA9505::DIST_SENS::ForwardLeft:
		_current_sensor = PCA9505::DIST_SENS::Forward_0; // back to Forward 0
		break;

	default:
		_current_sensor = PCA9505::DIST_SENS::Forward_0;
		break;
	}

	perf_end(_sample_perf);
}

bool
VL53L1X::measure(PCA9505::DIST_SENS sensor, bool publish)
{
	int instance = static_cast<int>(sensor);

	// get VL53L1 range measurement
	uint8_t measurement_available = 0;
	const hrt_abstime timestamp_sample = hrt_absolute_time();
	const VL53L1_Error status = VL53L1_GetMeasurementDataReady(&vl53l1x[instance], &measurement_available);

	int ret = (status == VL53L1_ERROR_NONE);

	if ((status == VL53L1_ERROR_NONE) && (measurement_available == 1)) {
		VL53L1_RangingMeasurementData_t meas_data{};
		const VL53L1_Error status_meas = VL53L1_GetRangingMeasurementData(&vl53l1x[instance], &meas_data);

		if (status_meas == VL53L1_ERROR_NONE) {
			const VL53L1_Error status_clear = VL53L1_ClearInterruptAndStartMeasurement(&vl53l1x[instance]);

			if (status_clear != VL53L1_ERROR_NONE) {
				PX4_ERR("clear error %d", status_clear);
			}

			_last_update[instance] = timestamp_sample;

			// Range Status for the current measurement.
			//  Value = 0 means value is valid.
			switch (meas_data.RangeStatus) {
			case VL53L1_RANGESTATUS_RANGE_VALID: {
					const float distance_m = meas_data.RangeMilliMeter / 1000.0f;

					if (publish) {
						// only publish if our last valid reading was less than 100 milliseconds ago
						if (hrt_elapsed_time(&_last_valid_data[instance]) < 100_ms) {
							if (_px4_dist[instance] != nullptr) {
								_px4_dist[instance]->update(timestamp_sample, distance_m);
							}
						}
					}

					_last_valid_data[instance] = hrt_absolute_time();

					return true;
				}
				break;

			case VL53L1_RANGESTATUS_OUTOFBOUNDS_FAIL:
				return true;
				break;

			default:
				return true;
				break;
			}

		} else {
			perf_count(_comms_errors);
			//PX4_ERR("%d measurement error %d", instance, status_meas);
		}

	} else {
		perf_count(_no_measurement_perf);
	}

	return ret;
}

void
VL53L1X::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_measure_perf);
	perf_print_counter(_no_measurement_perf);
	perf_print_counter(_comms_errors);

	_pca9505.print_info();

	for (int i = 0; i < 12; i++) {
		if (_px4_dist[i] != nullptr) {
			//_px4_dist[i]->print_status();
		}
	}
}
