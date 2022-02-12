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

#include "OutputPredictor.hpp"

OutputPredictor::OutputPredictor() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
{
	_lp_filter_angular_velocity.set_cutoff_frequency(kInitialRateHz, _param_imu_gyro_cutoff.get());
	_notch_filter_angular_velocity.setParameters(kInitialRateHz, _param_imu_gyro_nf_freq.get(),
			_param_imu_gyro_nf_bw.get());

	_lp_filter_angular_acceleration.set_cutoff_frequency(kInitialRateHz, _param_imu_dgyro_cutoff.get());

	_lp_filter_linear_acceleration.set_cutoff_frequency(kInitialRateHz, _param_imu_accel_cutoff.get());
	_lp_filter_velocity_body.set_cutoff_frequency(kInitialRateHz, _param_imu_accel_cutoff.get());
	_lp_filter_velocity_ned.set_cutoff_frequency(kInitialRateHz, _param_imu_accel_cutoff.get());

	_output_new.quat_nominal.setIdentity();
}

OutputPredictor::~OutputPredictor()
{
	perf_free(_loop_perf);
}

bool OutputPredictor::init()
{
	if (!_sensor_gyro_sub.registerCallback()) {
		ScheduleDelayed(10_ms);
	}

	return true;
}

void OutputPredictor::Run()
{
	if (should_exit()) {
		_sensor_gyro_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	// backup schedule
	ScheduleDelayed(10_ms);

	perf_begin(_loop_perf);

	// update corrections first to set _selected_sensor
	bool selection_updated = SensorSelectionUpdate();

	_calibration_accel.SensorCorrectionsUpdate(selection_updated);
	_calibration_gyro.SensorCorrectionsUpdate(selection_updated);
	SensorBiasUpdate(selection_updated);
	//ParametersUpdate();

	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		updateParams();
	}

	control_state_s control_state{};


	const float dt = (accel.timestamp_sample - _accel_timestamp_sample_last) * 1e-6f;
	_accel_timestamp_sample_last = accel.timestamp_sample;

	const Vector3f accel_raw{accel.x, accel.y, accel.z};
	// _accel_integrator.put(accel_raw, dt);


	const float dt = (gyro.timestamp_sample - _gyro_timestamp_sample_last) * 1e-6f;
	_gyro_timestamp_sample_last = gyro.timestamp_sample;

	const Vector3f gyro_raw{gyro.x, gyro.y, gyro.z};
	_gyro_integrator.put(gyro_raw, dt);


	sensor_gyro_s gyro;

	while (_sensor_gyro_sub.update(&gyro)) {

		// collect sample interval average for filters
		if ((_timestamp_sample_last_gyro > 0) && (gyro.timestamp_sample > _timestamp_sample_last_gyro)) {
			_interval_sum_gyro += (gyro.timestamp_sample - _timestamp_sample_last_gyro);
			_interval_count_gyro++;

		} else {
			_interval_sum_gyro = 0.f;
			_interval_count_gyro = 0.f;
		}

		_timestamp_sample_last_gyro = gyro.timestamp_sample;

		const float dt = math::constrain((gyro.timestamp_sample - _timestamp_sample_last_gyro) * 1e-6f, 0.0001f, 0.1f);
		_timestamp_sample_prev_gyro = gyro.timestamp_sample;

		// delta angle: apply offsets, scale, and board rotation
		_calibration_gyro.SensorCorrectionsUpdate();
		const Vector3f angular_velocity_raw{_calibration_gyro.Correct(Vector3f{gyro.x, gyro.y, gyro.z} - _bias_gyro)};

		const Vector3f delta_angle{angular_velocity_raw / dt};
		CalculateQuaternionOutput(gyro.timestamp_sample, delta_angle, dt);

		// Gyro filtering:
		// - Apply general notch filter (IMU_GYRO_NF_FREQ)
		const Vector3f angular_velocity_notched{_notch_filter_angular_velocity.apply(angular_velocity_raw)};

		// - Apply general low-pass filter (IMU_GYRO_CUTOFF)
		const Vector3f angular_velocity_filtered{_lp_filter_angular_velocity.apply(angular_velocity_notched)};

		// - Differentiate & apply specific angular acceleration (D-term) low-pass (IMU_DGYRO_CUTOFF)
		const Vector3f angular_acceleration_raw = (angular_velocity_filtered - _angular_velocity_prev) / dt;
		_angular_velocity_prev = angular_velocity_filtered;
		_angular_acceleration_prev = angular_acceleration_raw;
		const Vector3f angular_acceleration_filtered{_lp_filter_angular_acceleration.apply(angular_acceleration_raw)};

		// calculate the velocity of the IMU relative to the body origin
		const Vector3f vel_imu_rel_body = angular_acceleration_filtered % _imu_pos_body;

		// rotate the relative velocity into earth frame
		_vel_imu_rel_body_ned = Dcmf{_output_new.quat_nominal} * vel_imu_rel_body;

		CheckFiltersGyro();

		// publish once all new samples are processed
		if (!_sensor_gyro_sub.updated()) {

			bool publish = true;

			if (_param_imu_gyro_ratemax.get() > 0) {
				const uint64_t interval = 1e6f / _param_imu_gyro_ratemax.get();

				if (hrt_elapsed_time(&_last_angular_velocity_publish) < interval) {
					publish = false;
				}
			}

			if (publish) {
				// Publish vehicle_angular_acceleration
				vehicle_angular_acceleration_s v_angular_acceleration;
				v_angular_acceleration.timestamp_sample = gyro.timestamp_sample;
				angular_acceleration_filtered.copyTo(v_angular_acceleration.xyz);
				v_angular_acceleration.timestamp = hrt_absolute_time();
				_vehicle_angular_acceleration_pub.publish(v_angular_acceleration);

				// Publish vehicle_angular_velocity
				vehicle_angular_velocity_s v_angular_velocity;
				v_angular_velocity.timestamp_sample = gyro.timestamp_sample;
				angular_velocity_filtered.copyTo(v_angular_velocity.xyz);
				v_angular_velocity.timestamp = hrt_absolute_time();
				_vehicle_angular_velocity_pub.publish(v_angular_velocity);

				_last_angular_velocity_publish = v_angular_velocity.timestamp_sample;
			}

			control_state.timestamp_sample = gyro.timestamp_sample;
			angular_acceleration_filtered.copyTo(control_state.angular_acceleration);
			angular_velocity_filtered.copyTo(control_state.angular_velocity);
		}
	}


	// update accel, stopping once caught up to the last gyro sample
	sensor_accel_s accel;

	while (_sensor_accel_sub.update(&accel)) {

		_calibration_accel.SensorCorrectionsUpdate();
		const Vector3f accel_corrected{_calibration_accel.Correct(Vector3f{accel.x, accel.y, accel.z} - _bias_accel)};

		// Apply calibration and filter
		//  - calibration offsets, scale factors, and thermal scale (if available)
		//  - estimated in run bias (if available)
		//  - biquad low-pass filter
		const Vector3f accel_filtered = _lp_filter_linear_acceleration.apply(accel_corrected);
		_linear_acceleration_prev = accel_corrected;

		CheckFiltersAccel();

		const float dt = math::constrain((accel.timestamp_sample - _timestamp_sample_last_accel) * 1e-6f, 0.0001f, 0.1f);
		const Vector3f delta_velocity{accel_corrected / dt};
		CalculateOutputStates(accel.timestamp_sample, delta_velocity, dt);

		// publish once all new samples are processed
		if (!_sensor_accel_sub.updated()) {

			// Publish vehicle_acceleration
			vehicle_acceleration_s v_acceleration;
			v_acceleration.timestamp_sample = accel.timestamp_sample;
			accel_filtered.copyTo(v_acceleration.xyz);
			v_acceleration.timestamp = hrt_absolute_time();
			_vehicle_acceleration_pub.publish(v_acceleration);

			// control state updates
			const matrix::Dcmf R_to_body(_output_new.quat_nominal.inversed());

			// body frame velocity
			const Vector3f velocity_NED{_output_new.velocity_NED - _vel_imu_rel_body_ned};
			const Vector3f velocity_body{R_to_body * velocity_NED};
			const Vector3f velocity_body_filtered = _lp_filter_velocity_body.apply(velocity_body);
			velocity_body_filtered.copyTo(control_state.velocity_body);

			// get the velocity of the body frame origin in local NED earth frame
			const Vector3f velocity_ned{_output_new.velocity_NED - _vel_imu_rel_body_ned};
			const Vector3f velocity_ned_filtered = _lp_filter_velocity_ned.apply(velocity_ned);
			velocity_ned_filtered.copyTo(control_state.velocity_ned);

			// body frame acceleration without gravity
			const Vector3f gravity_body{R_to_body *Vector3f{0, 0, CONSTANTS_ONE_G}};
			const Vector3f linear_acceleration = accel_filtered + gravity_body;
			linear_acceleration.copyTo(control_state.linear_acceleration);
		}
	}

	// find corresponding estimated sensor bias
	if (_estimator_selector_status_sub.updated()) {
		estimator_selector_status_s estimator_selector_status;

		if (_estimator_selector_status_sub.copy(&estimator_selector_status)) {
			_estimator_sensor_bias_sub.ChangeInstance(estimator_selector_status.primary_instance);
			_estimator_states_sub.ChangeInstance(estimator_selector_status.primary_instance);
			_estimator_status_sub.ChangeInstance(estimator_selector_status.primary_instance);
		}
	}

	// first get latest estimates
	if (_estimator_states_sub.updated()) {
		estimator_states_s estimator_states;

		if (_estimator_states_sub.copy(&estimator_states)) {
			const Quatf q{estimator_states.states[0], estimator_states.states[1], estimator_states.states[2], estimator_states.states[3]};
			const Vector3f velocity{estimator_states.states[4], estimator_states.states[5], estimator_states.states[6]};
			const Vector3f position{estimator_states.states[7], estimator_states.states[8], estimator_states.states[9]};

			if (!_output_predictor_aligned) {
				alignOutputFilter(q, velocity, position);
				_output_predictor_aligned = true;
			}

			CorrectOutputStates(gyro.timestamp_sample, estimator_states.timestamp_sample, q, velocity, position);
		}
	}

	if (control_state.timestamp_sample != 0) {
		// get the position of the body frame origin in local earth frame
		// rotate the position of the IMU relative to the boy origin into earth frame
		const Vector3f pos_offset_earth = Dcmf(_output_new.quat_nominal) * _imu_pos_body;
		// subtract from the EKF position (which is at the IMU) to get position at the body origin
		const Vector3f position{_output_new.position - pos_offset_earth};
		position.copyTo(control_state.position);
		control_state.position_valid = true;

		control_state.timestamp = hrt_absolute_time();
		_control_state_pub.publish(control_state);
	}

	perf_end(_loop_perf);
}

void OutputPredictor::SensorBiasUpdate(bool force)
{
	if (_estimator_sensor_bias_sub.updated() || force) {
		estimator_sensor_bias_s bias;

		if (_estimator_sensor_bias_sub.copy(&bias)) {
			if (bias.accel_device_id == _selected_device_id_accel) {
				_bias_accel = Vector3f{bias.accel_bias};

			} else {
				_bias_accel.zero();
			}

			if (bias.gyro_device_id == _selected_device_id_gyro) {
				_bias_gyro = Vector3f{bias.gyro_bias};

			} else {
				_bias_gyro.zero();
			}
		}
	}
}

bool OutputPredictor::SensorSelectionUpdate(bool force)
{
	bool ret = false;

	if (_sensor_selection_sub.updated() || (_selected_device_id_accel == 0) || (_selected_device_id_gyro == 0) || force) {
		sensor_selection_s sensor_selection{};
		_sensor_selection_sub.copy(&sensor_selection);

		if ((_selected_device_id_accel != sensor_selection.accel_device_id)
		    || (_selected_device_id_gyro != sensor_selection.gyro_device_id)) {
			for (uint8_t i = 0; i < MAX_SENSOR_COUNT; i++) {
				uORB::SubscriptionData<sensor_accel_s> sensor_accel_sub{ORB_ID(sensor_accel), i};

				if ((sensor_accel_sub.get().device_id != 0) && (sensor_accel_sub.get().device_id == sensor_selection.accel_device_id)) {

					if (_sensor_accel_sub.ChangeInstance(i) && _sensor_accel_sub.registerCallback()) {
						PX4_INFO("selected accel changed %d -> %d", _selected_sub_index_accel, i);

						// record selected sensor (array index)
						_selected_sub_index_accel = i;
						_selected_device_id_accel = sensor_selection.accel_device_id;

						// clear bias and corrections
						_bias_accel.zero();
						_calibration_accel.set_device_id(sensor_accel_sub.get().device_id);

						// reset sample interval accumulator on sensor change
						_timestamp_sample_last_accel = 0;
						_required_sample_updates_accel = 0;

						ret = true;
					}
				}

				uORB::SubscriptionData<sensor_gyro_s> sensor_gyro_sub{ORB_ID(sensor_gyro), i};

				if ((sensor_gyro_sub.get().device_id != 0) && (sensor_gyro_sub.get().device_id == sensor_selection.gyro_device_id)) {

					if (_sensor_gyro_sub.ChangeInstance(i) && _sensor_gyro_sub.registerCallback()) {
						PX4_INFO("selected gyro changed %d -> %d", _selected_sub_index_gyro, i);

						// record selected sensor (array index)
						_selected_sub_index_gyro = i;
						_selected_device_id_gyro = sensor_selection.accel_device_id;

						// clear bias and corrections
						_bias_gyro.zero();
						_calibration_gyro.set_device_id(sensor_gyro_sub.get().device_id);

						// reset sample interval accumulator on sensor change
						_timestamp_sample_last_gyro = 0;
						_required_sample_updates_gyro = 0;

						ret = true;
					}
				}
			}
		}
	}

	return ret;
}

void OutputPredictor::CheckFiltersAccel()
{
	if (_interval_count_accel > 1000) {
		bool reset_filters = false;

		// calculate sensor update rate
		const float sample_interval_avg = _interval_sum_accel / _interval_count_accel;

		if (PX4_ISFINITE(sample_interval_avg) && (sample_interval_avg > 0.0f)) {

			_update_rate_hz_accel = 1.e6f / sample_interval_avg;

			// check if sample rate error is greater than 1%
			if ((fabsf(_update_rate_hz_accel - _filter_sample_rate_accel) / _filter_sample_rate_accel) > 0.01f) {
				reset_filters = true;
			}

			if (reset_filters || (_required_sample_updates_gyro == 0)) {
				if (_param_imu_gyro_ratemax.get() > 0) {
					// determine number of sensor samples that will get closest to the desired rate
					const float configured_interval_us = 1e6f / _param_imu_gyro_ratemax.get();
					const uint8_t samples = math::constrain(roundf(configured_interval_us / sample_interval_avg), 1.f,
										(float)sensor_accel_s::ORB_QUEUE_LENGTH);

					_sensor_accel_sub.set_required_updates(samples);
					_required_sample_updates_accel = samples;

				} else {
					_sensor_accel_sub.set_required_updates(1);
					_required_sample_updates_accel = 1;
				}
			}
		}

		if (!reset_filters) {
			// accel low pass cutoff frequency changed
			if (fabsf(_lp_filter_linear_acceleration.get_cutoff_freq() - _param_imu_accel_cutoff.get()) > 0.01f) {
				reset_filters = true;
			}
		}

		if (reset_filters) {
			PX4_DEBUG("resetting accel filters, accel sample rate: %.3f Hz -> %.3f Hz", (double)_filter_sample_rate_accel,
				  (double)_update_rate_hz_accel);
			_filter_sample_rate_accel = _update_rate_hz_accel;

			_lp_filter_linear_acceleration.set_cutoff_frequency(_filter_sample_rate_accel, _param_imu_accel_cutoff.get());
			_lp_filter_linear_acceleration.reset(_linear_acceleration_prev);

			_lp_filter_velocity_body.set_cutoff_frequency(_filter_sample_rate_accel, _param_imu_accel_cutoff.get());
			//_lp_filter_velocity_body.reset(_velocity_body_prev);

			_lp_filter_velocity_ned.set_cutoff_frequency(_filter_sample_rate_accel, _param_imu_accel_cutoff.get());
			//_lp_filter_velocity_ned.reset(_velocity_ned_prev);
		}

		// reset sample interval accumulator
		_timestamp_sample_last_accel = 0;
	}
}

void OutputPredictor::CheckFiltersGyro()
{
	if (_interval_count_gyro > 1000) {
		bool reset_filters = false;

		// calculate sensor update rate
		const float sample_interval_avg = _interval_sum_gyro / _interval_count_gyro;

		if (PX4_ISFINITE(sample_interval_avg) && (sample_interval_avg > 0.0f)) {

			_update_rate_hz_gyro = 1.e6f / sample_interval_avg;

			// check if sample rate error is greater than 1%
			if ((fabsf(_update_rate_hz_gyro - _filter_sample_rate_gyro) / _filter_sample_rate_gyro) > 0.01f) {
				reset_filters = true;
			}

			if (reset_filters || (_required_sample_updates_gyro == 0)) {
				if (_param_imu_gyro_ratemax.get() > 0) {
					// determine number of sensor samples that will get closest to the desired rate
					const float configured_interval_us = 1e6f / _param_imu_gyro_ratemax.get();
					const uint8_t samples = math::constrain(roundf(configured_interval_us / sample_interval_avg), 1.f,
										(float)sensor_gyro_s::ORB_QUEUE_LENGTH);

					_sensor_gyro_sub.set_required_updates(samples);
					_required_sample_updates_gyro = samples;

				} else {
					_sensor_gyro_sub.set_required_updates(1);
					_required_sample_updates_gyro = 1;
				}
			}
		}

		if (!reset_filters) {
			// gyro low pass cutoff frequency changed
			if (fabsf(_lp_filter_angular_velocity.get_cutoff_freq() - _param_imu_gyro_cutoff.get()) > 0.01f) {
				reset_filters = true;
			}

			// gyro notch filter frequency or bandwidth changed
			if ((fabsf(_notch_filter_angular_velocity.getNotchFreq() - _param_imu_gyro_nf_freq.get()) > 0.01f)
			    || (fabsf(_notch_filter_angular_velocity.getBandwidth() - _param_imu_gyro_nf_bw.get()) > 0.01f)) {
				reset_filters = true;
			}

			// gyro derivative low pass cutoff changed
			if (fabsf(_lp_filter_angular_acceleration.get_cutoff_freq() - _param_imu_dgyro_cutoff.get()) > 0.01f) {
				reset_filters = true;
			}
		}

		if (reset_filters) {
			PX4_DEBUG("resetting filters, accel sample rate: %.3f Hz -> %.3f Hz", (double)_filter_sample_rate_gyro,
				  (double)_update_rate_hz_gyro);
			_filter_sample_rate_gyro = _update_rate_hz_gyro;

			_lp_filter_angular_velocity.set_cutoff_frequency(_filter_sample_rate_gyro, _param_imu_gyro_cutoff.get());
			_lp_filter_angular_velocity.reset(_angular_velocity_prev);

			_notch_filter_angular_velocity.setParameters(_filter_sample_rate_gyro, _param_imu_gyro_nf_freq.get(),
					_param_imu_gyro_nf_bw.get());
			_notch_filter_angular_velocity.reset(_angular_velocity_prev);

			_lp_filter_angular_acceleration.set_cutoff_frequency(_filter_sample_rate_gyro, _param_imu_dgyro_cutoff.get());
			_lp_filter_angular_acceleration.reset(_angular_acceleration_prev);
		}

		// reset sample interval accumulator
		_timestamp_sample_last_gyro = 0;
	}
}

// align output filter states to match EKF states at the fusion time horizon
void OutputPredictor::alignOutputFilter(const Quatf &quat_nominal, const Vector3f &vel, const Vector3f &pos)
{
	const outputSample &output_delayed = _output_buffer.get_oldest();

	// calculate the quaternion rotation delta from the EKF to output observer states at the EKF fusion time horizon
	Quatf q_delta{quat_nominal * output_delayed.quat_nominal.inversed()};
	q_delta.normalize();

	// calculate the velocity and position deltas between the output and EKF at the EKF fusion time horizon
	const Vector3f vel_ned_delta = vel - output_delayed.velocity_NED;
	const Vector3f vel_body_delta = vel - output_delayed.velocity_body; // TODO:
	const Vector3f pos_delta = pos - output_delayed.position;

	// loop through the output filter state history and add the deltas
	for (uint8_t i = 0; i < _output_buffer.get_length(); i++) {
		_output_buffer[i].quat_nominal = Quatf(q_delta * _output_buffer[i].quat_nominal).normalized();
		_output_buffer[i].velocity_NED += vel_ned_delta;
		_output_buffer[i].velocity_body += vel_body_delta;
		_output_buffer[i].position += pos_delta;
	}

	_output_new = _output_buffer.get_newest();
}

void OutputPredictor::CalculateQuaternionOutput(const hrt_abstime &timestamp_sample, const Vector3f &imu_delta_angle,
		const float dt)
{
	// Apply corrections to the delta angle required to track the quaternion states at the EKF fusion time horizon
	const Vector3f delta_angle(imu_delta_angle + _delta_angle_corr);

	// rotate the previous INS quaternion by the delta quaternions
	// the quaternions must always be normalised after modification
	_output_new.time_us = timestamp_sample;
	_output_new.quat_nominal = Quatf(_output_new.quat_nominal * Quatf{AxisAnglef{delta_angle}}).normalized();

	// correct velocity for IMU offset
	// TODO: calculate angular acceleration
}

void OutputPredictor::CalculateOutputStates(const hrt_abstime &timestamp_sample, const Vector3f &delta_velocity,
		const float dt)
{
	// save the previous velocity so we can use trapezoidal integration
	const Vector3f vel_last(_output_new.velocity_NED);

	// rotate the delta velocity to earth frame
	Vector3f delta_vel_earth{Dcmf{_output_new.quat_nominal} *delta_velocity}; // TODO: use quat synchronized with delta velocity
	delta_vel_earth(2) += CONSTANTS_ONE_G * dt; // correct for measured acceleration due to gravity

	// increment the INS velocity states by the measurement plus corrections
	// do the same for vertical state used by alternative correction algorithm
	_output_new.velocity_NED += delta_vel_earth;
	_output_new.velocity_body += delta_velocity;
	_output_new.vertical_velocity += delta_vel_earth(2);

	// use trapezoidal integration to calculate the INS position states
	// do the same for vertical state used by alternative correction algorithm
	const Vector3f delta_pos_NED = (_output_new.velocity_NED + vel_last) * (dt * 0.5f);
	_output_new.position += delta_pos_NED;
	_output_new.vertical_velocity_integ += delta_pos_NED(2);

	// store the INS states in a ring buffer with the same length and time coordinates as the IMU data buffer
	_output_buffer.push(_output_new);
}

void OutputPredictor::CorrectOutputStates(const hrt_abstime &imu_timestamp_sample,
		const hrt_abstime &timestamp_estimate, const Quatf &q, const Vector3f &velocity, const Vector3f &position)
{
	_dt_imu_avg = 1.f / _filter_sample_rate_accel;
	_dt_ekf_avg = 0.99f * _dt_ekf_avg + 0.01f * (timestamp_estimate * 1e-6f);

	// Complementary filter gains
	const float vel_gain = _dt_ekf_avg / math::constrain(_param_vel_tau, _dt_ekf_avg, 10.0f);
	const float pos_gain = _dt_ekf_avg / math::constrain(_param_pos_tau, _dt_ekf_avg, 10.0f);

	// get the oldest INS state data from the ring buffer
	// this data will be at the EKF fusion time horizon
	outputSample output_delayed;

	if (_output_buffer.pop_first_older_than(timestamp_estimate, output_delayed)) {
		// calculate the quaternion delta between the INS and EKF quaternions at the EKF fusion time horizon
		const Quatf q_error((q.inversed() * output_delayed.quat_nominal).normalized());

		// convert the quaternion delta to a delta angle
		const float scalar = (q_error(0) >= 0.0f) ? -2.f : 2.f;
		const Vector3f delta_ang_error{scalar * q_error(1), scalar * q_error(2), scalar * q_error(3)};

		// calculate a gain that provides tight tracking of the estimator attitude states and
		// adjust for changes in time delay to maintain consistent damping ratio of ~0.7
		const float time_delay = fmaxf((imu_timestamp_sample - timestamp_estimate) * 1e-6f, _dt_imu_avg);
		const float att_gain = 0.5f * _dt_imu_avg / time_delay;

		// calculate a corrrection to the delta angle
		// that will cause the INS to track the EKF quaternions
		_delta_angle_corr = delta_ang_error * att_gain;

		/*
		* Loop through the output filter state history and apply the corrections to the velocity and position states.
		* This method is too expensive to use for the attitude states due to the quaternion operations required
		* but because it eliminates the time delay in the 'correction loop' it allows higher tracking gains
		* to be used and reduces tracking error relative to EKF states.
		*/

		// calculate velocity and position tracking errors
		const Vector3f vel_err(velocity - output_delayed.velocity_NED);
		// calculate a velocity correction that will be applied to the output state history
		_velocity_error_integ += vel_err;
		const Vector3f vel_correction = vel_err * vel_gain + _velocity_error_integ * sq(vel_gain) * 0.1f;

		const Vector3f velocity_body{Dcmf(q.inversed()) *velocity};
		const Vector3f vel_body_err(velocity_body - output_delayed.velocity_body);
		_velocity_body_error_integ += vel_body_err;
		const Vector3f vel_body_correction = vel_body_err * vel_gain + _velocity_body_error_integ * sq(vel_gain) * 0.1f;

		const Vector3f pos_err(position - output_delayed.position);
		// calculate a position correction that will be applied to the output state history
		_position_error_integ += pos_err;
		const Vector3f pos_correction = pos_err * pos_gain + _position_error_integ * sq(pos_gain) * 0.1f;

		/*
		* Calculate corrections to be applied to vel and pos output state history.
		* The vel and pos state history are corrected individually so they track the EKF states at
		* the fusion time horizon. This option provides the most accurate tracking of EKF states.
		*/
		for (uint8_t index = 0; index < _output_buffer.get_length(); index++) {
			// a constant velocity correction is applied
			_output_buffer[index].velocity_NED += vel_correction;

			_output_buffer[index].velocity_body += vel_body_correction;

			// a constant position correction is applied
			_output_buffer[index].position += pos_correction;
		}

		/*
		* Calculate a correction to be applied to vert_vel that casues vert_vel_integ to track the EKF
		* down position state at the fusion time horizon using an alternative algorithm to what
		* is used for the vel and pos state tracking. The algorithm applies a correction to the vert_vel
		* state history and propagates vert_vel_integ forward in time using the corrected vert_vel history.
		* This provides an alternative vertical velocity output that is closer to the first derivative
		* of the position but does degrade tracking relative to the EKF state.
		*/

		// calculate down velocity and position tracking errors
		const float vert_vel_err = (velocity(2) - output_delayed.vertical_velocity);
		const float vert_vel_integ_err = (position(2) - output_delayed.vertical_velocity);

		// calculate a velocity correction that will be applied to the output state history
		// using a PD feedback tuned to a 5% overshoot
		const float vert_vel_correction = vert_vel_integ_err * pos_gain + vert_vel_err * vel_gain * 1.1f;

		// loop through the vertical output filter state history starting at the oldest and apply the corrections to the
		// vert_vel states and propagate vert_vel_integ forward using the corrected vert_vel
		uint8_t index = _output_buffer.get_oldest_index();
		const uint8_t size = _output_buffer.get_length();

		for (uint8_t counter = 0; counter < (size - 1); counter++) {
			const uint8_t index_next = (index + 1) % size;
			outputSample &current = _output_buffer[index];
			outputSample &next = _output_buffer[index_next];

			// correct the velocity
			if (counter == 0) {
				current.vertical_velocity += vert_vel_correction;
			}

			next.vertical_velocity += vert_vel_correction;

			// position is propagated forward using the corrected velocity and a trapezoidal integrator
			const float dt = (next.time_us - current.time_us) * 1e-6f;
			next.vertical_velocity_integ = current.vertical_velocity_integ + (current.vertical_velocity + next.vertical_velocity) *
						       0.5f * dt;

			// advance the index
			index = (index + 1) % size;
		}

		// update output state to corrected values
		_output_new = _output_buffer.get_newest();

		output_predictor_status_s output_predictor_status{};
		output_predictor_status.timestamp_sample = imu_timestamp_sample;

		output_predictor_status.angle_tracking_error = delta_ang_error.norm();
		vel_err.copyTo(output_predictor_status.velocity_tracking_error);
		vel_body_err.copyTo(output_predictor_status.velocity_body_tracking_error);
		pos_err.copyTo(output_predictor_status.position_tracking_error);

		output_predictor_status.timestamp = hrt_absolute_time();
		_output_predictor_status_pub.publish(output_predictor_status);
	}
}

int OutputPredictor::task_spawn(int argc, char *argv[])
{
	OutputPredictor *instance = new OutputPredictor();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int OutputPredictor::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int OutputPredictor::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description


)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("output_predictor", "estimator");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int output_predictor_main(int argc, char *argv[])
{
	return OutputPredictor::main(argc, argv);
}
