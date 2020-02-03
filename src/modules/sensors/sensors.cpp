/****************************************************************************
 *
 *   Copyright (c) 2012-2018 PX4 Development Team. All rights reserved.
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
 * @file sensors.cpp
 *
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Julian Oes <julian@oes.ch>
 * @author Thomas Gubler <thomas@px4.io>
 * @author Anton Babushkin <anton@px4.io>
 * @author Beat Küng <beat-kueng@gmx.net>
 */

#include <drivers/drv_adc.h>
#include <drivers/drv_hrt.h>
#include <lib/conversion/rotation.h>
#include <lib/mathlib/mathlib.h>
#include <lib/parameters/param.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/time.h>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/differential_pressure.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_preflight.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_magnetometer.h>

#include "parameters.h"
#include "voted_sensors_update.h"

#include "airspeed/VehicleAirspeed.hpp"
#include "vehicle_acceleration/VehicleAcceleration.hpp"
#include "vehicle_angular_velocity/VehicleAngularVelocity.hpp"
#include "vehicle_air_data/VehicleAirData.hpp"
#include "vehicle_imu/VehicleIMU.hpp"

using namespace sensors;
using namespace time_literals;

class Sensors : public ModuleBase<Sensors>, public ModuleParams
{
public:
	explicit Sensors(bool hil_enabled);
	~Sensors() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static Sensors *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;

private:
	const bool	_hil_enabled;			/**< if true, HIL is active */
	bool		_armed{false};				/**< arming status of the vehicle */

	uORB::Subscription	_actuator_ctrl_0_sub{ORB_ID(actuator_controls_0)};		/**< attitude controls sub */
	uORB::Subscription	_parameter_update_sub{ORB_ID(parameter_update)};				/**< notification of parameter updates */
	uORB::Subscription	_vcontrol_mode_sub{ORB_ID(vehicle_control_mode)};		/**< vehicle control mode subscription */

	uORB::Publication<sensor_combined_s>		_sensor_pub{ORB_ID(sensor_combined)};			/**< combined sensor data topic */
	uORB::Publication<sensor_preflight_s>		_sensor_preflight{ORB_ID(sensor_preflight)};		/**< sensor preflight topic */
	uORB::Publication<vehicle_magnetometer_s>	_magnetometer_pub{ORB_ID(vehicle_magnetometer)};	/**< combined sensor data topic */

	perf_counter_t	_loop_perf;			/**< loop performance counter */

#ifdef ADC_AIRSPEED_VOLTAGE_CHANNEL
	int	_adc_fd {-1};				/**< ADC driver handle */

	hrt_abstime	_last_adc{0};			/**< last time we took input from the ADC */

	differential_pressure_s	_diff_pres {};
	uORB::PublicationMulti<differential_pressure_s>	_diff_pres_pub{ORB_ID(differential_pressure)};		/**< differential_pressure */
#endif /* ADC_AIRSPEED_VOLTAGE_CHANNEL */

	Parameters		_parameters{};			/**< local copies of interesting parameters */
	ParameterHandles	_parameter_handles{};		/**< handles for interesting parameters */

	VotedSensorsUpdate _voted_sensors_update;

	VehicleAcceleration	_vehicle_acceleration;
	VehicleAngularVelocity	_vehicle_angular_velocity;
	VehicleAirData          _vehicle_air_data;
	VehicleAirspeed         *_vehicle_airspeed{nullptr};

	static constexpr int MAX_SENSOR_COUNT = 3;
	VehicleIMU      *_vehicle_imu_list[MAX_SENSOR_COUNT] {};


	/**
	 * Update our local parameter cache.
	 */
	int		parameters_update();

	/**
	 * Check for changes in parameters.
	 */
	void 		parameter_update_poll(bool forced = false);

	/**
	 * Do adc-related initialisation.
	 */
	int		adc_init();

	/**
	 * Poll the ADC and update readings to suit.
	 *
	 * @param raw			Combined sensor data structure into which
	 *				data should be returned.
	 */
	void		adc_poll();

	void		InitializeVehicleAirspeed();
	void		InitializeVehicleIMU();

};

Sensors::Sensors(bool hil_enabled) :
	ModuleParams(nullptr),
	_hil_enabled(hil_enabled),
	_loop_perf(perf_alloc(PC_ELAPSED, "sensors")),
	_voted_sensors_update(_parameters, hil_enabled)
{
	initialize_parameter_handles(_parameter_handles);

	_vehicle_acceleration.Start();
	_vehicle_angular_velocity.Start();
	_vehicle_air_data.Start();

	InitializeVehicleAirspeed();
	InitializeVehicleIMU();
}

Sensors::~Sensors()
{
	_vehicle_acceleration.Stop();
	_vehicle_angular_velocity.Stop();
	_vehicle_air_data.Stop();

	if (_vehicle_airspeed != nullptr) {
		_vehicle_airspeed->Stop();
		delete _vehicle_airspeed;
	}

	for (auto &i : _vehicle_imu_list) {
		if (i != nullptr) {
			i->Stop();
			delete i;
		}
	}
}

int
Sensors::parameters_update()
{
	if (_armed) {
		return 0;
	}

	/* read the parameter values into _parameters */
	update_parameters(_parameter_handles, _parameters);

	_voted_sensors_update.parametersUpdate();

	return PX4_OK;
}

int
Sensors::adc_init()
{
	if (!_hil_enabled) {
#ifdef ADC_AIRSPEED_VOLTAGE_CHANNEL
		_adc_fd = px4_open(ADC0_DEVICE_PATH, O_RDONLY);

		if (_adc_fd == -1) {
			PX4_ERR("no ADC found: %s", ADC0_DEVICE_PATH);
			return PX4_ERROR;
		}

#endif // ADC_AIRSPEED_VOLTAGE_CHANNEL
	}

	return OK;
}

void
Sensors::parameter_update_poll(bool forced)
{
	// check for parameter updates
	if (_parameter_update_sub.updated() || forced) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		parameters_update();
		updateParams();
	}
}

void
Sensors::adc_poll()
{
#ifdef ADC_AIRSPEED_VOLTAGE_CHANNEL

	/* only read if not in HIL mode */
	if (_hil_enabled) {
		return;
	}

	if (_parameters.diff_pres_analog_scale > 0.0f) {

		hrt_abstime t = hrt_absolute_time();

		/* rate limit to 100 Hz */
		if (t - _last_adc >= 10000) {
			/* make space for a maximum of twelve channels (to ensure reading all channels at once) */
			px4_adc_msg_t buf_adc[PX4_MAX_ADC_CHANNELS];
			/* read all channels available */
			int ret = px4_read(_adc_fd, &buf_adc, sizeof(buf_adc));

			if (ret >= (int)sizeof(buf_adc[0])) {

				/* Read add channels we got */
				for (unsigned i = 0; i < ret / sizeof(buf_adc[0]); i++) {
					if (ADC_AIRSPEED_VOLTAGE_CHANNEL == buf_adc[i].am_channel) {

						/* calculate airspeed, raw is the difference from */
						const float voltage = (float)(buf_adc[i].am_data) * 3.3f / 4096.0f * 2.0f;  // V_ref/4096 * (voltage divider factor)

						/**
						 * The voltage divider pulls the signal down, only act on
						 * a valid voltage from a connected sensor. Also assume a non-
						 * zero offset from the sensor if its connected.
						 */
						if (voltage > 0.4f) {
							const float diff_pres_pa_raw = voltage * _parameters.diff_pres_analog_scale - _parameters.diff_pres_offset_pa;

							_diff_pres.timestamp = t;
							_diff_pres.differential_pressure_raw_pa = diff_pres_pa_raw;
							_diff_pres.differential_pressure_filtered_pa = (_diff_pres.differential_pressure_filtered_pa * 0.9f) +
									(diff_pres_pa_raw * 0.1f);
							_diff_pres.temperature = -1000.0f;

							_diff_pres_pub.publish(_diff_pres);
						}
					}
				}

				_last_adc = t;
			}
		}
	}

#endif /* ADC_AIRSPEED_VOLTAGE_CHANNEL */
}

void Sensors::InitializeVehicleAirspeed()
{
	// create a VehicleAirspeed instance only if differential pressure sensors exist
	if (_vehicle_airspeed == nullptr) {
		uORB::Subscription differential_pressure_sub{ORB_ID(differential_pressure)};
		differential_pressure_s dpres{};
		differential_pressure_sub.copy(&dpres);

		if ((dpres.timestamp > 0) && PX4_ISFINITE(dpres.differential_pressure_raw_pa)) {
			VehicleAirspeed *airspeed = new VehicleAirspeed();

			if (airspeed != nullptr) {
				// Start VehicleIMU instance and store
				if (airspeed->Start()) {
					_vehicle_airspeed = airspeed;

				} else {
					delete airspeed;
				}
			}
		}
	}
}

void Sensors::InitializeVehicleIMU()
{
	// create a VehicleIMU instance for each accel/gyro pair
	for (uint8_t i = 0; i < MAX_SENSOR_COUNT; i++) {
		if (_vehicle_imu_list[i] == nullptr) {

			uORB::Subscription accel_sub{ORB_ID(sensor_accel_integrated), i};
			sensor_accel_integrated_s accel{};
			accel_sub.copy(&accel);

			uORB::Subscription gyro_sub{ORB_ID(sensor_gyro_integrated), i};
			sensor_gyro_integrated_s gyro{};
			gyro_sub.copy(&gyro);

			if (accel.device_id > 0 && gyro.device_id > 0) {
				VehicleIMU *imu = new VehicleIMU(i, i);

				if (imu != nullptr) {
					// Start VehicleIMU instance and store
					if (imu->Start()) {
						_vehicle_imu_list[i] = imu;

					} else {
						delete imu;
					}
				}
			}
		}
	}
}

void
Sensors::run()
{
	adc_init();

	sensor_combined_s raw = {};
	sensor_preflight_s preflt = {};
	vehicle_magnetometer_s magnetometer = {};

	_voted_sensors_update.init(raw);

	/* (re)load params and calibration */
	parameter_update_poll(true);

	/* get a set of initial values */
	_voted_sensors_update.sensorsPoll(raw, magnetometer);

	/* wakeup source */
	px4_pollfd_struct_t poll_fds = {};
	poll_fds.events = POLLIN;

	uint64_t last_config_update = hrt_absolute_time();

	while (!should_exit()) {

		/* use the best-voted gyro to pace output */
		poll_fds.fd = _voted_sensors_update.bestGyroFd();

		/* wait for up to 50ms for data (Note that this implies, we can have a fail-over time of 50ms,
		 * if a gyro fails) */
		int pret = px4_poll(&poll_fds, 1, 50);

		/* If pret == 0 it timed out but we should still do all checks and potentially copy
		 * other gyros. */

		/* this is undesirable but not much we can do - might want to flag unhappy status */
		if (pret < 0) {
			/* if the polling operation failed because no gyro sensor is available yet,
			 * then attempt to subscribe once again
			 */
			if (_voted_sensors_update.numGyros() == 0) {
				_voted_sensors_update.initializeSensors();
			}

			px4_usleep(1000);
			continue;
		}

		perf_begin(_loop_perf);

		/* check vehicle status for changes to publication state */
		if (_vcontrol_mode_sub.updated()) {
			vehicle_control_mode_s vcontrol_mode{};
			_vcontrol_mode_sub.copy(&vcontrol_mode);
			_armed = vcontrol_mode.flag_armed;
		}

		/* the timestamp of the raw struct is updated by the gyroPoll() method (this makes the gyro
		 * a mandatory sensor) */
		const uint64_t magnetometer_prev_timestamp = magnetometer.timestamp;

		_voted_sensors_update.sensorsPoll(raw, magnetometer);

		/* check analog airspeed */
		adc_poll();

		if (raw.timestamp > 0) {

			_voted_sensors_update.setRelativeTimestamps(raw);

			_sensor_pub.publish(raw);

			if (magnetometer.timestamp != magnetometer_prev_timestamp) {
				_magnetometer_pub.publish(magnetometer);
			}

			_voted_sensors_update.checkFailover();

			/* If the the vehicle is disarmed calculate the length of the maximum difference between
			 * IMU units as a consistency metric and publish to the sensor preflight topic
			*/
			if (!_armed) {
				preflt.timestamp = hrt_absolute_time();
				_voted_sensors_update.calcAccelInconsistency(preflt);
				_voted_sensors_update.calcGyroInconsistency(preflt);
				_voted_sensors_update.calcMagInconsistency(preflt);

				_sensor_preflight.publish(preflt);
			}
		}

		/* keep adding sensors as long as we are not armed,
		 * when not adding sensors poll for param updates
		 */
		if (!_armed && hrt_elapsed_time(&last_config_update) > 500_ms) {
			_voted_sensors_update.initializeSensors();
			InitializeVehicleAirspeed();
			InitializeVehicleIMU();

			last_config_update = hrt_absolute_time();

		} else {

			/* check parameters for updates */
			parameter_update_poll();
		}

		perf_end(_loop_perf);
	}

	_voted_sensors_update.deinit();
}

int Sensors::task_spawn(int argc, char *argv[])
{
	/* start the task */
	_task_id = px4_task_spawn_cmd("sensors",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_SENSOR_HUB,
				      2000,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

int Sensors::print_status()
{
	_voted_sensors_update.printStatus();

	PX4_INFO_RAW("\n");

	PX4_INFO_RAW("\n");
	_vehicle_acceleration.PrintStatus();

	PX4_INFO_RAW("\n");
	_vehicle_angular_velocity.PrintStatus();

	PX4_INFO_RAW("\n");
	_vehicle_air_data.PrintStatus();

	if (_vehicle_airspeed != nullptr) {
		PX4_INFO_RAW("\n");
		_vehicle_airspeed->PrintStatus();
	}

	PX4_INFO_RAW("\n");

	for (auto &i : _vehicle_imu_list) {
		if (i != nullptr) {
			i->PrintStatus();
		}
	}

	return 0;
}

int Sensors::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int Sensors::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
The sensors module is central to the whole system. It takes low-level output from drivers, turns
it into a more usable form, and publishes it for the rest of the system.

The provided functionality includes:
- Read the output from the sensor drivers (`sensor_gyro`, etc.).
  If there are multiple of the same type, do voting and failover handling.
  Then apply the board rotation and temperature calibration (if enabled). And finally publish the data; one of the
  topics is `sensor_combined`, used by many parts of the system.
- Make sure the sensor drivers get the updated calibration parameters (scale & offset) when the parameters change or
  on startup. The sensor drivers use the ioctl interface for parameter updates. For this to work properly, the
  sensor drivers must already be running when `sensors` is started.
- Do preflight sensor consistency checks and publish the `sensor_preflight` topic.

### Implementation
It runs in its own thread and polls on the currently selected gyro topic.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("sensors", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_FLAG('h', "Start in HIL mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

Sensors *Sensors::instantiate(int argc, char *argv[])
{
	bool hil_enabled = false;
	bool error_flag = false;

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "h", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'h':
			hil_enabled = true;
			break;

		case '?':
			error_flag = true;
			break;

		default:
			PX4_WARN("unrecognized flag");
			error_flag = true;
			break;
		}
	}

	if (error_flag) {
		return nullptr;
	}

	return new Sensors(hil_enabled);
}

extern "C" __EXPORT int sensors_main(int argc, char *argv[])
{
	return Sensors::main(argc, argv);
}
