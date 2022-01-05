/****************************************************************************
 *
 *   Copyright (c) 2020-2021 PX4 Development Team. All rights reserved.
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

#include "VehicleAirData.hpp"

#include <px4_platform_common/log.h>
#include <lib/geo/geo.h>

namespace sensors
{

using namespace matrix;

static constexpr uint32_t SENSOR_TIMEOUT{300_ms};

VehicleAirData::VehicleAirData() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	_voter.set_timeout(SENSOR_TIMEOUT);

	ParametersUpdate(true);
}

VehicleAirData::~VehicleAirData()
{
	Stop();
	perf_free(_cycle_perf);
}

bool VehicleAirData::Start()
{
	ScheduleNow();
	return true;
}

void VehicleAirData::Stop()
{
	Deinit();

	// clear all registered callbacks
	for (auto &sub : _sensor_sub) {
		sub.unregisterCallback();
	}
}

void VehicleAirData::AirTemperatureUpdate()
{
	differential_pressure_s differential_pressure;

	static constexpr float temperature_min_celsius = -20.f;
	static constexpr float temperature_max_celsius = 35.f;

	// update air temperature if data from differential pressure sensor is finite and not exactly 0
	// limit the range to max 35°C to limt the error due to heated up airspeed sensors prior flight
	if (_differential_pressure_sub.update(&differential_pressure) && PX4_ISFINITE(differential_pressure.temperature)
	    && fabsf(differential_pressure.temperature) > FLT_EPSILON) {

		_air_temperature_celsius = math::constrain(differential_pressure.temperature, temperature_min_celsius,
					   temperature_max_celsius);
	}
}

void VehicleAirData::ParametersUpdate(bool force)
{
	// Check if parameters have changed
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		updateParams();

		// update priority
		for (int instance = 0; instance < MAX_SENSOR_COUNT; instance++) {
			const int32_t priority_old = _calibration[instance].priority();
			_calibration[instance].ParametersUpdate();
			const int32_t priority_new = _calibration[instance].priority();

			if (priority_old != priority_new) {
				if (_priority[instance] == priority_old) {
					_priority[instance] = priority_new;

				} else {
					// change relative priority to incorporate any sensor faults
					int priority_change = priority_new - priority_old;
					_priority[instance] = math::constrain(_priority[instance] + priority_change, 1, 100);
				}
			}
		}
	}
}

void VehicleAirData::SensorCalibrationUpdate()
{
	// State variance assumed for barometer bias storage.
	// This is a reference variance used to calculate the fraction of learned barometer bias that will be used to update the stored value.
	// Larger values cause a larger fraction of the learned biases to be used.
	static constexpr float barob_vref = 2.5e-7f;
	static constexpr float min_var_allowed = barob_vref * 0.01f;
	//static constexpr float max_var_allowed = barob_vref * 100.f;

	if (_armed) {
		static constexpr uint8_t baro_cal_size = sizeof(_baro_cal) / sizeof(_baro_cal[0]);

		for (int i = 0; i < math::min(_estimator_baro_bias_subs.size(), baro_cal_size); i++) {
			estimator_baro_bias_s estimator_baro_bias;

			if (_estimator_baro_bias_subs[i].update(&estimator_baro_bias)) {

				const float bias{estimator_baro_bias.bias};
				const float bias_variance{estimator_baro_bias.bias_var};

				const bool valid = (hrt_elapsed_time(&estimator_baro_bias.timestamp) < 1_s)
						   && (estimator_baro_bias.baro_device_id != 0) &&
						   //estimator_baro_bias.bias_valid &&
						   // estimator_baro_bias.bias_stable &&
						   (bias_variance > min_var_allowed);

				if (valid) {
					// find corresponding mag calibration
					for (int baro_index = 0; baro_index < MAX_SENSOR_COUNT; baro_index++) {
						if (_calibration[baro_index].device_id() == estimator_baro_bias.baro_device_id) {

							_baro_cal[i].device_id = estimator_baro_bias.baro_device_id;

							// readd estimated bias that was removed before publishing vehicle_magnetometer
							//_baro_cal[i].offset = _calibration[baro_index].BiasCorrectedSensorOffset(bias) +
							//		     _calibration_estimator_bias[baro_index];

							_baro_cal[i].offset = bias; // TODO



							// TODO: use altitude bias to determine barometer bias


							// pressure_pa + pressure_bias
							//  altitude - bias


							// float altitude = PressureToAltitude(pressure_pa, temperature);




							_baro_cal[i].variance = bias_variance;
							//_baro_cal[i].temperature = _last_data[baro_index].temperature;

							_in_flight_cal_available = true;
							break;
						}
					}
				}
			}
		}

	} else if (_in_flight_cal_available) {
		// not armed and mag cal available
		bool calibration_param_save_needed = false;
		// iterate through available bias estimates and fuse them sequentially using a Kalman Filter scheme
		float state_variance{barob_vref};

		for (int baro_index = 0; baro_index < MAX_SENSOR_COUNT; baro_index++) {
			// apply all valid saved offsets
			for (int i = 0; i < ORB_MULTI_MAX_INSTANCES; i++) {
				if ((_calibration[baro_index].device_id() != 0) && (_baro_cal[i].device_id == _calibration[baro_index].device_id())) {
					const float baro_cal_orig{_calibration[baro_index].offset()};

					// calculate weighting using ratio of variances and update stored bias values
					const float &observation{_baro_cal[i].offset};
					const float &obs_variance{_baro_cal[i].variance};

					const float innovation{baro_cal_orig - observation};
					const float innovation_variance{state_variance + obs_variance};
					const float kalman_gain{state_variance / innovation_variance};

					// new offset
					const float baro_cal_offset{baro_cal_orig - innovation * kalman_gain};

					state_variance = fmaxf(state_variance * (1.f - kalman_gain), 0.f);

					if (_calibration[baro_index].set_offset(baro_cal_offset)) {

						PX4_INFO("%d (%" PRIu32 ") EST:%d offset: %.2f->%.2f (full %.3f)",
							 baro_index, _calibration[baro_index].device_id(), i,
							 (double)baro_cal_orig, (double)baro_cal_offset, (double)_baro_cal[i].offset);

						//_calibration[baro_index].set_temperature(_last_data[baro_index].temperature);

						_calibration[baro_index].ParametersSave();

						calibration_param_save_needed = true;
					}

				}

				// clear
				_baro_cal[i] = {};
			}
		}

		if (calibration_param_save_needed) {
			param_notify_changes();
			//_last_calibration_update = hrt_absolute_time();
		}

		_in_flight_cal_available = false;
	}
}

void VehicleAirData::Run()
{
	perf_begin(_cycle_perf);

	// check vehicle status for changes to armed state
	if (_vehicle_control_mode_sub.updated()) {
		vehicle_control_mode_s vehicle_control_mode;

		if (_vehicle_control_mode_sub.copy(&vehicle_control_mode)) {
			_armed = vehicle_control_mode.flag_armed;
		}
	}

	ParametersUpdate();


	float gps_altitude = NAN;
	float gps_alt_var = NAN;

	if (_sensor_gps_sub.updated()) {
		sensor_gps_s sensor_gps;

		if (_sensor_gps_sub.copy(&sensor_gps)) {
			if (sensor_gps.fix_type >= 3 && sensor_gps.epv < 10) {
				gps_altitude = sensor_gps.alt * 0.001f;
				gps_alt_var = sensor_gps.epv * sensor_gps.epv;
			}
		}
	}


	AirTemperatureUpdate();

	bool updated[MAX_SENSOR_COUNT] {};

	for (int uorb_index = 0; uorb_index < MAX_SENSOR_COUNT; uorb_index++) {

		if (!_calibration[uorb_index].enabled()) {
			continue;
		}

		if (!_advertised[uorb_index]) {
			// use data's timestamp to throttle advertisement checks
			if ((_last_publication_timestamp[uorb_index] == 0)
			    || (hrt_elapsed_time(&_last_publication_timestamp[uorb_index]) > 1_s)) {

				if (_sensor_sub[uorb_index].advertised()) {
					if (uorb_index > 0) {
						/* the first always exists, but for each further sensor, add a new validator */
						if (!_voter.add_new_validator()) {
							PX4_ERR("failed to add validator for %s %i", _calibration[uorb_index].SensorString(), uorb_index);
						}
					}

					_advertised[uorb_index] = true;

					// advertise outputs in order if publishing all
					if (!_param_sens_baro_mode.get()) {
						for (int instance = 0; instance < uorb_index; instance++) {
							_vehicle_air_data_pub[instance].advertise();
						}
					}

					if (_selected_sensor_sub_index < 0) {
						_sensor_sub[uorb_index].registerCallback();
					}

				} else {
					_last_publication_timestamp[uorb_index] = hrt_absolute_time();
				}
			}
		}

		if (_advertised[uorb_index]) {
			sensor_baro_s report;

			while (_sensor_sub[uorb_index].update(&report)) {
				updated[uorb_index] = true;

				if (_calibration[uorb_index].device_id() != report.device_id) {
					_calibration[uorb_index].set_device_id(report.device_id, report.is_external);
					_priority[uorb_index] = _calibration[uorb_index].priority();
				}

				// millibar to Pa
				const float raw_pressure_pascals = report.pressure * 100.f;

				// pressure corrected with offset (if available)
				const float pressure_corrected = _calibration[uorb_index].Correct(raw_pressure_pascals);

				if (_calibration[uorb_index].enabled()) {
					float data_array[3] {pressure_corrected, report.temperature, 0.f};
					_voter.put(uorb_index, report.timestamp_sample, data_array, report.error_count, _priority[uorb_index]);

					_timestamp_sample_sum[uorb_index] += report.timestamp_sample;
					_data_sum[uorb_index] += pressure_corrected;
					_temperature_sum[uorb_index] += report.temperature;
					_data_sum_count[uorb_index]++;
				}

				_last_data[uorb_index] = pressure_corrected;

			}
		}
	}

	// check for the current best sensor
	int best_index = 0;
	_voter.get_best(hrt_absolute_time(), &best_index);

	if (best_index >= 0) {
		if (_selected_sensor_sub_index != best_index) {
			// clear all registered callbacks
			for (auto &sub : _sensor_sub) {
				sub.unregisterCallback();
			}

			if (_param_sens_baro_mode.get()) {
				if (_selected_sensor_sub_index >= 0) {
					PX4_INFO("%s switch from #%" PRId8 " -> #%d", _calibration[_selected_sensor_sub_index].SensorString(),
						 _selected_sensor_sub_index, best_index);
				}
			}

			_selected_sensor_sub_index = best_index;
			_sensor_sub[_selected_sensor_sub_index].registerCallback();
		}
	}







	for (int instance = 0; instance < MAX_SENSOR_COUNT; instance++) {
		// publish all sensors as separate instances
		if (updated[instance]) {

			const hrt_abstime timestamp_sample = _timestamp_sample_sum[instance] / _data_sum_count[instance];

			if ((_param_sens_baro_rate.get() > 0) && (_data_sum_count[instance] > 0)
			    && ((timestamp_sample - _last_publication_timestamp[instance]) >= (1e6f / _param_sens_baro_rate.get()))
			   ) {

				const float pressure_pa = _data_sum[instance] / _data_sum_count[instance];
				const float temperature = _temperature_sum[instance] / _data_sum_count[instance];

				float altitude = PressureToAltitude(pressure_pa, temperature);

				// Baro bias estimation using GPS altitude
				const float dt = math::constrain(1e-6f * (timestamp_sample - _last_publication_timestamp[instance]), 0.f, 1.f);
				_baro_b_est[instance].setMaxStateNoise(2.f); // TODO: baro_noise
				_baro_b_est[instance].setProcessNoiseStdDev(0.005f); // TODO
				_baro_b_est[instance].predict(dt);


				if (PX4_ISFINITE(gps_altitude)) {
					// Use GPS altitude as a reference to compute the baro bias measurement
					const float baro_bias = altitude - gps_altitude;
					const float baro_bias_var = gps_alt_var + (2.f * 2.f); // TODO: baro_noise sq(baro_noise)

					_baro_b_est[instance].fuseBias(baro_bias, baro_bias_var);
				}

				const BaroBiasEstimator::status &status = _baro_b_est[instance].getStatus();

				estimator_baro_bias_s baro_bias{};
				baro_bias.timestamp_sample = timestamp_sample;
				baro_bias.baro_device_id = 0;//_device_id_baro;
				baro_bias.bias = status.bias;
				baro_bias.bias_var = status.bias_var;
				baro_bias.innov = status.innov;
				baro_bias.innov_var = status.innov_var;
				baro_bias.innov_test_ratio = status.innov_test_ratio;
				baro_bias.timestamp = hrt_absolute_time();
				_estimator_baro_bias_pub[instance].publish(baro_bias);


				// calculate air density
				float air_density = pressure_pa / (CONSTANTS_AIR_GAS_CONST * (_air_temperature_celsius -
								   CONSTANTS_ABSOLUTE_NULL_CELSIUS));

				// populate vehicle_air_data with and publish
				vehicle_air_data_s out{};
				out.timestamp_sample = timestamp_sample;
				out.baro_device_id = _calibration[instance].device_id();
				out.baro_alt_meter = altitude;
				out.baro_temp_celcius = temperature;
				out.baro_pressure_pa = pressure_pa;
				out.rho = air_density;
				out.calibration_count = _calibration[instance].calibration_count();
				out.timestamp = hrt_absolute_time();


				if (_param_sens_baro_mode.get()
				    && (_selected_sensor_sub_index == instance)
				    && (_voter.get_sensor_state(_selected_sensor_sub_index) == DataValidator::ERROR_FLAG_NO_ERROR)
				   ) {
					// otherwise only ever publish the first instance
					_vehicle_air_data_pub[0].publish(out);

				} else {
					// publish all
					_vehicle_air_data_pub[instance].publish(out);
				}

				_last_publication_timestamp[instance] = timestamp_sample;

				// reset
				_timestamp_sample_sum[instance] = 0;
				_data_sum[instance] = 0;
				_temperature_sum[instance] = 0;
				_data_sum_count[instance] = 0;
			}
		}
	}


	// check failover and report
	if (_param_sens_baro_mode.get() == 1) {
		if (_last_failover_count != _voter.failover_count()) {
			uint32_t flags = _voter.failover_state();
			int failover_index = _voter.failover_index();

			if (flags != DataValidator::ERROR_FLAG_NO_ERROR) {
				if (failover_index != -1) {
					const hrt_abstime now = hrt_absolute_time();

					if (now - _last_error_message > 3_s) {
						mavlink_log_emergency(&_mavlink_log_pub, "%s #%i failed: %s%s%s%s%s!",
								      _calibration[0].SensorString(),
								      failover_index,
								      ((flags & DataValidator::ERROR_FLAG_NO_DATA) ? " OFF" : ""),
								      ((flags & DataValidator::ERROR_FLAG_STALE_DATA) ? " STALE" : ""),
								      ((flags & DataValidator::ERROR_FLAG_TIMEOUT) ? " TIMEOUT" : ""),
								      ((flags & DataValidator::ERROR_FLAG_HIGH_ERRCOUNT) ? " ERR CNT" : ""),
								      ((flags & DataValidator::ERROR_FLAG_HIGH_ERRDENSITY) ? " ERR DNST" : ""));
						_last_error_message = now;
					}

					// reduce priority of failed sensor to the minimum
					_priority[failover_index] = 1;
				}
			}

			_last_failover_count = _voter.failover_count();
		}
	}

	PublishStatus();

	// reschedule timeout
	ScheduleDelayed(100_ms);

	perf_end(_cycle_perf);
}

float VehicleAirData::PressureToAltitude(float pressure_pa, float temperature) const
{
	// calculate altitude using the hypsometric equation
	static constexpr float T1 = 15.f - CONSTANTS_ABSOLUTE_NULL_CELSIUS; // temperature at base height in Kelvin
	static constexpr float a = -6.5f / 1000.f; // temperature gradient in degrees per metre

	// current pressure at MSL in kPa (QNH in hPa)
	const float p1 = _param_sens_baro_qnh.get() * 0.1f;

	// measured pressure in kPa
	const float p = pressure_pa * 0.001f;

	/*
	 * Solve:
	 *
	 *     /        -(aR / g)     \
	 *    | (p / p1)          . T1 | - T1
	 *     \                      /
	 * h = -------------------------------  + h1
	 *                   a
	 */
	float altitude = (((powf((p / p1), (-(a * CONSTANTS_AIR_GAS_CONST) / CONSTANTS_ONE_G))) * T1) - T1) / a;

	return altitude;
}

void VehicleAirData::Publish(uint8_t instance, bool multi)
{

}

void VehicleAirData::PublishStatus()
{
	if (_selected_sensor_sub_index >= 0) {
		sensors_status_s sensors_status{};
		sensors_status.device_id_primary = _calibration[_selected_sensor_sub_index].device_id();

		float mean{};
		int sensor_count = 0;

		for (int sensor_index = 0; sensor_index < MAX_SENSOR_COUNT; sensor_index++) {
			if ((_calibration[sensor_index].device_id() != 0) && (_calibration[sensor_index].enabled())) {
				sensor_count++;
				mean += _last_data[sensor_index];
			}
		}

		if (sensor_count > 0) {
			mean /= sensor_count;
		}

		for (int sensor_index = 0; sensor_index < MAX_SENSOR_COUNT; sensor_index++) {
			if (_calibration[sensor_index].device_id() != 0) {

				_sensor_diff[sensor_index] = 0.95f * _sensor_diff[sensor_index] + 0.05f * (_last_data[sensor_index] - mean);

				sensors_status.device_ids[sensor_index] = _calibration[sensor_index].device_id();
				sensors_status.inconsistency[sensor_index] = _sensor_diff[sensor_index];
				sensors_status.healthy[sensor_index] = (_voter.get_sensor_state(sensor_index) == DataValidator::ERROR_FLAG_NO_ERROR);
				sensors_status.priority[sensor_index] = _voter.get_sensor_priority(sensor_index);
				sensors_status.enabled[sensor_index] = _calibration[sensor_index].enabled();
				sensors_status.external[sensor_index] = _calibration[sensor_index].external();

			} else {
				sensors_status.inconsistency[sensor_index] = NAN;
			}
		}

		sensors_status.timestamp = hrt_absolute_time();
		_sensors_status_baro_pub.publish(sensors_status);
	}
}

void VehicleAirData::PrintStatus()
{
	if (_selected_sensor_sub_index >= 0) {
		PX4_INFO("selected %s: %" PRIu32 " (%" PRId8 ")", _calibration[_selected_sensor_sub_index].SensorString(),
			 _calibration[_selected_sensor_sub_index].device_id(), _selected_sensor_sub_index);
	}

	_voter.print();

	for (int i = 0; i < MAX_SENSOR_COUNT; i++) {
		if (_advertised[i] && (_priority[i] > 0)) {
			_calibration[i].PrintStatus();
		}
	}
}

}; // namespace sensors
