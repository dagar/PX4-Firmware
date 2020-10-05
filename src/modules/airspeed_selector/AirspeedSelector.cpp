/****************************************************************************
 *
 *   Copyright (c) 2018-2020 PX4 Development Team. All rights reserved.
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

#include "AirspeedSelector.hpp"

AirspeedSelector::AirspeedSelector():
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	// initialise parameters
	update_params();
}

AirspeedSelector::~AirspeedSelector()
{
	ScheduleClear();

	perf_free(_perf_elapsed);
}

int AirspeedSelector::task_spawn(int argc, char *argv[])
{
	AirspeedSelector *dev = new AirspeedSelector();

	// check if the trampoline is called for the first time
	if (!dev) {
		PX4_ERR("alloc failed");
		return PX4_ERROR;
	}

	_object.store(dev);

	dev->ScheduleOnInterval(SCHEDULE_INTERVAL, 10000);
	_task_id = task_id_is_work_queue;
	return PX4_OK;
}

void AirspeedSelector::init()
{
	check_for_connected_airspeed_sensors();

	/* Set the default sensor */
	if (_param_airspeed_primary_index.get() > _number_of_airspeed_sensors) {
		/* constrain the index to the number of sensors connected*/
		_valid_airspeed_index = math::min(_param_airspeed_primary_index.get(), _number_of_airspeed_sensors);

		if (_number_of_airspeed_sensors == 0) {
			mavlink_log_info(&_mavlink_log_pub,
					 "No airspeed sensor detected. Switch to non-airspeed mode.");

		} else {
			mavlink_log_info(&_mavlink_log_pub,
					 "Primary airspeed index bigger than number connected sensors. Take last sensor.");
		}

	} else {
		_valid_airspeed_index =
			_param_airspeed_primary_index.get(); // set index to the one provided in the parameter ASPD_PRIMARY
	}

	_prev_airspeed_index = _valid_airspeed_index; // needed to detect a switching
}

void AirspeedSelector::check_for_connected_airspeed_sensors()
{
	/* check for new connected airspeed sensor */
	int detected_airspeed_sensors = 0;

	if (_param_airspeed_primary_index.get() > 0) {

		for (int i = 0; i < _airspeed_subs.size(); i++) {
			if (!_airspeed_subs[i].advertised()) {
				break;
			}

			detected_airspeed_sensors = i + 1;
		}

	} else {
		detected_airspeed_sensors = 0; //user has selected groundspeed-windspeed as primary source, or disabled airspeed
	}

	_number_of_airspeed_sensors = detected_airspeed_sensors;
}

void AirspeedSelector::Run()
{
	if (should_exit()) {
		exit_and_cleanup();
	}

	_time_now_usec = hrt_absolute_time(); //hrt time of the current cycle

	/* do not run the airspeed selector until 2s after system boot, as data from airspeed sensor
	and estimator may not be valid yet*/
	if (hrt_absolute_time() < 2_s) {
		return;
	}

	perf_begin(_perf_elapsed);

	if (!_initialized) {
		init(); // initialize airspeed validator instances
		_initialized = true;
	}

	if (_param_sub.updated()) {
		parameter_update_s update;
		_param_sub.copy(&update);

		update_params();
	}

	vehicle_air_data_s vehicle_air_data{};

	if (_vehicle_air_data_sub.copy(&vehicle_air_data)) {
		_initial_input_data.air_pressure_pa = vehicle_air_data.baro_pressure_pa;
	}

	if (_vehicle_acceleration_sub.updated()) {
		vehicle_acceleration_s accel;

		if (_vehicle_acceleration_sub.copy(&accel)) {
			_initial_input_data.accel_z = accel.xyz[2];
		}
	}

	if (_vehicle_land_detected_sub.updated()) {
		vehicle_land_detected_s vehicle_land_detected;

		if (_vehicle_land_detected_sub.copy(&vehicle_land_detected)) {
			_landed = vehicle_land_detected.landed;
		}
	}

	if (_vehicle_status_sub.updated()) {
		vehicle_status_s vehicle_status;

		if (_vehicle_status_sub.copy(&vehicle_status)) {
			_armed = (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);
		}
	}

	if (_vtol_vehicle_status_sub.updated()) {
		vtol_vehicle_status_s vtol_vehicle_status;

		if (_vtol_vehicle_status_sub.copy(&vtol_vehicle_status)) {
			_fixed_wing = !vtol_vehicle_status.vtol_in_rw_mode;
		}
	}

	// estimator status
	if (_estimator_status_sub.updated()) {
		estimator_status_s estimator_status;

		if (_estimator_status_sub.update(&estimator_status)) {
			_initial_input_data.vel_test_ratio = estimator_status.vel_test_ratio;
			_initial_input_data.mag_test_ratio = estimator_status.mag_test_ratio;
		}
	}

	// vehicle attitude
	if (_vehicle_attitude_sub.updated()) {
		vehicle_attitude_s vehicle_attitude;

		if (_vehicle_attitude_sub.copy(&vehicle_attitude)) {
			_initial_input_data.att_q[0] = vehicle_attitude.q[0];
			_initial_input_data.att_q[1] = vehicle_attitude.q[1];
			_initial_input_data.att_q[2] = vehicle_attitude.q[2];
			_initial_input_data.att_q[3] = vehicle_attitude.q[3];
		}
	}

	// vehicle local position
	if (_vehicle_local_position_sub.updated()) {
		vehicle_local_position_s vehicle_local_position;

		if (_vehicle_local_position_sub.copy(&vehicle_local_position)) {
			_initial_input_data.lpos_vx = vehicle_local_position.vx;
			_initial_input_data.lpos_vy = vehicle_local_position.vy;
			_initial_input_data.lpos_vz = vehicle_local_position.vz;
			_initial_input_data.lpos_evh = vehicle_local_position.evh;
			_initial_input_data.lpos_evv = vehicle_local_position.evv;

			_vehicle_local_position_valid = (vehicle_local_position.timestamp > 0) && vehicle_local_position.v_xy_valid;

			const Vector3f velocity{vehicle_local_position.vx, vehicle_local_position.vy, vehicle_local_position.vz};

			update_wind_estimator_sideslip(vehicle_local_position.timestamp_sample, Quatf{_initial_input_data.att_q}, velocity);
			update_ground_minus_wind_airspeed(vehicle_air_data.baro_pressure_pa, vehicle_air_data.baro_temp_celcius, velocity);
		}
	}

	const bool in_air = !_landed;

	// reset takeoff_situation to true when not in air or not in fixed-wing mode
	if (!in_air || !_fixed_wing) {
		_in_takeoff_situation = true;
	}

	// Check for new connected airspeed sensors as long as we're disarmed
	if (!_armed) {
		check_for_connected_airspeed_sensors();
	}

	// iterate through all airspeed sensors, poll new data from them and update their validators
	for (int i = 0; i < MAX_NUM_AIRSPEED_SENSORS; i++) {
		// poll raw airspeed topic of the i-th sensor
		airspeed_s airspeed_raw;

		if (_airspeed_subs[i].update(&airspeed_raw)) {
			// Prepare data for airspeed_validator
			airspeed_validator_update_data input_data{_initial_input_data};
			input_data.timestamp = airspeed_raw.timestamp;
			input_data.lpos_valid = _vehicle_local_position_valid;

			input_data.airspeed_indicated_raw = airspeed_raw.indicated_airspeed_m_s;
			input_data.airspeed_true_raw = airspeed_raw.true_airspeed_m_s;
			input_data.airspeed_timestamp = airspeed_raw.timestamp;
			input_data.air_temperature_celsius = airspeed_raw.air_temperature_celsius;

			// update in_fixed_wing_flight for the current airspeed sensor validator
			// takeoff situation is active from start till one of the sensors' IAS or groundspeed_CAS is above stall speed
			if (airspeed_raw.indicated_airspeed_m_s > _airspeed_stall.get() || _ground_minus_wind_CAS > _airspeed_stall.get()) {
				_in_takeoff_situation = false;
			}

			input_data.in_fixed_wing_flight = (_armed && _fixed_wing && in_air && !_in_takeoff_situation);

			// push input data into airspeed validator
			_airspeed_validator[i].update_airspeed_validator(input_data);
		}
	}

	select_airspeed_and_publish();

	perf_end(_perf_elapsed);
}

void AirspeedSelector::update_params()
{
	updateParams();

	/* update wind estimator (sideslip fusion only) parameters */
	_wind_estimator_sideslip.set_wind_p_noise(_param_west_w_p_noise.get());
	_wind_estimator_sideslip.set_tas_scale_p_noise(_param_west_sc_p_noise.get());
	_wind_estimator_sideslip.set_tas_noise(_param_west_tas_noise.get());
	_wind_estimator_sideslip.set_beta_noise(_param_west_beta_noise.get());
	_wind_estimator_sideslip.set_tas_gate(_param_west_tas_gate.get());
	_wind_estimator_sideslip.set_beta_gate(_param_west_beta_gate.get());

	/* update airspeedValidator parameters */
	for (int i = 0; i < MAX_NUM_AIRSPEED_SENSORS; i++) {
		_airspeed_validator[i].set_wind_estimator_wind_p_noise(_param_west_w_p_noise.get());
		_airspeed_validator[i].set_wind_estimator_tas_scale_p_noise(_param_west_sc_p_noise.get());
		_airspeed_validator[i].set_wind_estimator_tas_noise(_param_west_tas_noise.get());
		_airspeed_validator[i].set_wind_estimator_beta_noise(_param_west_beta_noise.get());
		_airspeed_validator[i].set_wind_estimator_tas_gate(_param_west_tas_gate.get());
		_airspeed_validator[i].set_wind_estimator_beta_gate(_param_west_beta_gate.get());
		_airspeed_validator[i].set_wind_estimator_scale_estimation_on(_param_west_scale_estimation_on.get());

		/* Only apply manual entered airspeed scale to first airspeed measurement */
		// TODO: enable multiple airspeed sensors
		_airspeed_validator[0].set_airspeed_scale_manual(_param_west_airspeed_scale.get());

		_airspeed_validator[i].set_tas_innov_threshold(_tas_innov_threshold.get());
		_airspeed_validator[i].set_tas_innov_integ_threshold(_tas_innov_integ_threshold.get());
		_airspeed_validator[i].set_checks_fail_delay(_checks_fail_delay.get());
		_airspeed_validator[i].set_checks_clear_delay(_checks_clear_delay.get());
		_airspeed_validator[i].set_airspeed_stall(_airspeed_stall.get());
	}

	/* when airspeed scale estimation is turned on and the airspeed is valid, then set the scale inside the wind estimator to -1 such that it starts to estimate it */
	if (!_scale_estimation_previously_on && _param_west_scale_estimation_on.get()) {
		if (_valid_airspeed_index > 0) {
			// set it to a negative value to start estimation inside wind estimator
			_airspeed_validator[0].set_airspeed_scale_manual(-1.0f);

		} else {
			mavlink_log_info(&_mavlink_log_pub, "Airspeed: can't estimate scale as no valid sensor.");
			_param_west_scale_estimation_on.set(0); // reset this param to 0 as estimation was not turned on
			_param_west_scale_estimation_on.commit_no_notification();
		}

		/* If one sensor is valid and we switched out of scale estimation, then publish message and change the value of param ASPD_ASPD_SCALE */

	} else if (_scale_estimation_previously_on && !_param_west_scale_estimation_on.get()) {
		if (_valid_airspeed_index > 0) {
			_param_west_airspeed_scale.set(_airspeed_validator[_valid_airspeed_index - 1].get_CAS_scale());
			_param_west_airspeed_scale.commit_no_notification();
			_airspeed_validator[_valid_airspeed_index - 1].set_airspeed_scale_manual(_param_west_airspeed_scale.get());

			mavlink_log_info(&_mavlink_log_pub, "Airspeed: estimated scale (ASPD_ASPD_SCALE): %0.2f",
					 (double)_airspeed_validator[_valid_airspeed_index - 1].get_CAS_scale());

		} else {
			mavlink_log_info(&_mavlink_log_pub, "Airspeed: can't estimate scale as no valid sensor.");
		}
	}

	_scale_estimation_previously_on = _param_west_scale_estimation_on.get();
}

void AirspeedSelector::update_wind_estimator_sideslip(const hrt_abstime &timestamp, const matrix::Quatf &q,
		const matrix::Vector3f &velocity)
{
	// update wind and airspeed estimator
	_wind_estimator_sideslip.update(timestamp);

	// sideslip fusion
	_wind_estimator_sideslip.fuse_beta(timestamp, velocity, q);

	/* fill message for publishing later */
	_wind_estimate_sideslip.timestamp = timestamp;
	float wind[2];
	_wind_estimator_sideslip.get_wind(wind);
	_wind_estimate_sideslip.windspeed_north = wind[0];
	_wind_estimate_sideslip.windspeed_east = wind[1];
	float wind_cov[2];
	_wind_estimator_sideslip.get_wind_var(wind_cov);
	_wind_estimate_sideslip.variance_north = wind_cov[0];
	_wind_estimate_sideslip.variance_east = wind_cov[1];
	_wind_estimate_sideslip.tas_innov = _wind_estimator_sideslip.get_tas_innov();
	_wind_estimate_sideslip.tas_innov_var = _wind_estimator_sideslip.get_tas_innov_var();
	_wind_estimate_sideslip.beta_innov = _wind_estimator_sideslip.get_beta_innov();
	_wind_estimate_sideslip.beta_innov_var = _wind_estimator_sideslip.get_beta_innov_var();
	_wind_estimate_sideslip.tas_scale = _wind_estimator_sideslip.get_tas_scale();
}

void AirspeedSelector::update_ground_minus_wind_airspeed(float baro_pressure_pa, float baro_temp_celcius,
		const matrix::Vector3f &velocity)
{
	/* calculate airspeed estimate based on groundspeed-windspeed to use as fallback */
	float TAS_north = velocity(0) - _wind_estimate_sideslip.windspeed_north;
	float TAS_east = velocity(1) - _wind_estimate_sideslip.windspeed_east;
	float TAS_down = velocity(2); // no wind estimate in z
	_ground_minus_wind_TAS = sqrtf(TAS_north * TAS_north + TAS_east * TAS_east + TAS_down * TAS_down);
	_ground_minus_wind_CAS = calc_CAS_from_TAS(_ground_minus_wind_TAS, baro_pressure_pa, baro_temp_celcius);
}

void AirspeedSelector::select_airspeed_and_publish()
{
	/* Find new valid index if airspeed currently is invalid, but we have sensors, primary sensor is real sensor and checks are enabled or new sensor was added. */
	bool airspeed_sensor_switching_necessary = _prev_airspeed_index < airspeed_index::FIRST_SENSOR_INDEX ||
			!_airspeed_validator[_prev_airspeed_index - 1].get_airspeed_valid();
	bool airspeed_sensor_switching_allowed = _number_of_airspeed_sensors > 0 &&
			_param_airspeed_primary_index.get() > airspeed_index::GROUND_MINUS_WIND_INDEX && _param_airspeed_checks_on.get();
	bool airspeed_sensor_added = _prev_number_of_airspeed_sensors < _number_of_airspeed_sensors;

	if (airspeed_sensor_switching_necessary && (airspeed_sensor_switching_allowed || airspeed_sensor_added)) {

		_valid_airspeed_index = airspeed_index::DISABLED_INDEX; // set to disabled

		/* Loop through all sensors and take the first valid one */
		for (int i = 0; i < _number_of_airspeed_sensors; i++) {
			if (_airspeed_validator[i].get_airspeed_valid()) {
				_valid_airspeed_index = i + 1;
				break;
			}
		}
	}

	/* Airspeed enabled by user (Primary set to > -1), and no valid airspeed sensor available or primary set to 0. Thus set index to ground-wind one (if position is valid), otherwise to disabled*/
	if (_param_airspeed_primary_index.get() > airspeed_index::DISABLED_INDEX &&
	    (_valid_airspeed_index < airspeed_index::FIRST_SENSOR_INDEX
	     || _param_airspeed_primary_index.get() == airspeed_index::GROUND_MINUS_WIND_INDEX)) {

		/* _vehicle_local_position_valid determines if ground-wind estimate is valid */
		/* To use ground-windspeed as airspeed source, either the primary has to be set this way or fallback be enabled */
		if (_vehicle_local_position_valid && (_param_airspeed_fallback.get()
						      || _param_airspeed_primary_index.get() == airspeed_index::GROUND_MINUS_WIND_INDEX)) {
			_valid_airspeed_index = airspeed_index::GROUND_MINUS_WIND_INDEX;

		} else {
			_valid_airspeed_index = airspeed_index::DISABLED_INDEX;
		}
	}

	/* publish critical message (and log) in index has changed */
	/* Suppress log message if still on the ground and no airspeed sensor connected */
	if (_valid_airspeed_index != _prev_airspeed_index && (_number_of_airspeed_sensors > 0 || !_landed)) {
		mavlink_log_critical(&_mavlink_log_pub, "Airspeed: switched from sensor %i to %i", _prev_airspeed_index,
				     _valid_airspeed_index);
	}

	_prev_airspeed_index = _valid_airspeed_index;
	_prev_number_of_airspeed_sensors = _number_of_airspeed_sensors;

	/* fill out airspeed_validated message for publishing it */
	airspeed_validated_s airspeed_validated{};
	airspeed_validated.true_ground_minus_wind_m_s = NAN;
	airspeed_validated.calibrated_ground_minus_wind_m_s = NAN;
	airspeed_validated.indicated_airspeed_m_s = NAN;
	airspeed_validated.calibrated_airspeed_m_s = NAN;
	airspeed_validated.true_airspeed_m_s = NAN;
	airspeed_validated.airspeed_sensor_measurement_valid = false;
	airspeed_validated.selected_airspeed_index = _valid_airspeed_index;

	switch (_valid_airspeed_index) {
	case airspeed_index::DISABLED_INDEX:
		break;

	case airspeed_index::GROUND_MINUS_WIND_INDEX:
		/* Take IAS, CAS, TAS from groundspeed-windspeed */
		airspeed_validated.indicated_airspeed_m_s = _ground_minus_wind_CAS;
		airspeed_validated.calibrated_airspeed_m_s = _ground_minus_wind_CAS;
		airspeed_validated.true_airspeed_m_s = _ground_minus_wind_TAS;
		airspeed_validated.calibrated_ground_minus_wind_m_s = _ground_minus_wind_CAS;
		airspeed_validated.true_ground_minus_wind_m_s = _ground_minus_wind_TAS;

		break;

	default:
		airspeed_validated.indicated_airspeed_m_s = _airspeed_validator[_valid_airspeed_index - 1].get_IAS();
		airspeed_validated.calibrated_airspeed_m_s = _airspeed_validator[_valid_airspeed_index - 1].get_CAS();
		airspeed_validated.true_airspeed_m_s = _airspeed_validator[_valid_airspeed_index - 1].get_TAS();
		airspeed_validated.calibrated_ground_minus_wind_m_s = _ground_minus_wind_CAS;
		airspeed_validated.true_ground_minus_wind_m_s = _ground_minus_wind_TAS;
		airspeed_validated.airspeed_sensor_measurement_valid = true;
		break;
	}

	// publish airspeed validated topic
	airspeed_validated.timestamp = hrt_absolute_time();
	_airspeed_validated_pub.publish(airspeed_validated);

	// publish sideslip-only-fusion wind topic
	_wind_est_pub[0].publish(_wind_estimate_sideslip);

	// publish the wind estimator states from all airspeed validators
	for (int i = 0; i < _number_of_airspeed_sensors; i++) {
		wind_estimate_s wind_est = _airspeed_validator[i].get_wind_estimator_states();
		wind_est.timestamp = hrt_absolute_time();
		_wind_est_pub[i + 1].publish(wind_est);
	}
}

int AirspeedSelector::custom_command(int argc, char *argv[])
{
	if (!is_running()) {
		int ret = AirspeedSelector::task_spawn(argc, argv);

		if (ret) {
			return ret;
		}
	}

	return print_usage("unknown command");
}

int AirspeedSelector::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This module provides a single airspeed_validated topic, containing indicated (IAS),
calibrated (CAS), true airspeed (TAS) and the information if the estimation currently
is invalid and if based sensor readings or on groundspeed minus windspeed.
Supporting the input of multiple "raw" airspeed inputs, this module automatically switches
to a valid sensor in case of failure detection. For failure detection as well as for
the estimation of a scale factor from IAS to CAS, it runs several wind estimators
and also publishes those.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("airspeed_estimator", "estimator");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int airspeed_selector_main(int argc, char *argv[])
{
	return AirspeedSelector::main(argc, argv);
}
