#pragma once

#include <string.h>

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <drivers/device/i2c.h>
#include <lib/perf/perf_counter.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_range_finder.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/distance_sensor.h>
#include <lib/drivers/rangefinder/PX4Rangefinder.hpp>

/* Configuration Constants */
#define VL53L1X_BUS_DEFAULT 4

#define VL53L1X_BASEADDR 0x29 // 7-bit address 0x29

#define VL53L1X_MAX_RANGING_DISTANCE 1.3f
#define VL53L1X_MIN_RANGING_DISTANCE 0.04f

#include "api/core/vl53l1_api.h"

#include "PCA9505.hpp"

// Short  136 cm
// Medium 290 cm
// Long   360 cm

// Model ID 0x010F 0xEA
// Module Type 0x0110 0xCC
// Mask Revision 0x0111 0x10

//  I2C read/writes can be 8,16 or 32-bit

// The minimum and maximum timing budgets are [20 ms, 1000 ms]

class VL53L1X : public device::I2C, public px4::ScheduledWorkItem
{
public:
	VL53L1X(uint8_t rotation = distance_sensor_s::ROTATION_FORWARD_FACING, int bus = 4, int address = VL53L1X_BASEADDR);

	virtual ~VL53L1X();

	virtual int init() override;

	void print_info();

protected:
	virtual int probe() override;

private:

	int vl53l1_init(uint8_t instance);

	bool measure(PCA9505::DIST_SENS sensor, bool publish = true);

	VL53L1_Dev_t	vl53l1x[12];

	PCA9505		_pca9505;

	PCA9505::DIST_SENS _current_sensor{PCA9505::DIST_SENS::Forward_0};

	PX4Rangefinder *_px4_dist[12];

	hrt_abstime	_last_update[12] {};
	hrt_abstime	_last_valid_data[12] {};

	perf_counter_t	_sample_perf;
	perf_counter_t	_measure_perf;
	perf_counter_t	_no_measurement_perf;
	perf_counter_t	_comms_errors;

	void start();
	void stop();
	void Run() override;

};
