#include "output_predictor.h"

#include <lib/mathlib/mathlib.h>

#include <lib/geo/geo.h>

using matrix::Quatf;
using matrix::Vector2f;
using matrix::Vector3f;
using matrix::Dcmf;
using matrix::AxisAnglef;

bool OutputPredictor::reset(int imu_buffer_length)
{
	// TODO: who resets the output buffer content?
	_output_new.vel.setZero();
	_output_new.pos.setZero();
	_output_new.quat_nominal.setIdentity();

	// TODO: dagar output predictor reset
	_delta_angle_corr.setZero();

	if (_output_buffer.allocate(imu_buffer_length) && _output_vert_buffer.allocate(imu_buffer_length)) {
		return true;
	}

	return false;
}

void OutputPredictor::alignOutputFilter(const estimator::stateSample& state)
{
	const outputSample &output_delayed = _output_buffer.get_oldest();

	// calculate the quaternion rotation delta from the EKF to output observer states at the EKF fusion time horizon
	Quatf q_delta{state.quat_nominal * output_delayed.quat_nominal.inversed()};
	q_delta.normalize();

	// calculate the velocity and position deltas between the output and EKF at the EKF fusion time horizon
	const Vector3f vel_delta = state.vel - output_delayed.vel;
	const Vector3f pos_delta = state.pos - output_delayed.pos;

	// loop through the output filter state history and add the deltas
	for (uint8_t i = 0; i < _output_buffer.get_length(); i++) {
		_output_buffer[i].quat_nominal = q_delta * _output_buffer[i].quat_nominal;
		_output_buffer[i].quat_nominal.normalize();
		_output_buffer[i].vel += vel_delta;
		_output_buffer[i].pos += pos_delta;
	}

	_output_new = _output_buffer.get_newest();
}

void OutputPredictor::resetQuatState(const Quatf& quat_change)
{
	// add the reset amount to the output observer buffered data
	for (uint8_t i = 0; i < _output_buffer.get_length(); i++) {
		_output_buffer[i].quat_nominal = quat_change * _output_buffer[i].quat_nominal;
	}

	// apply the change in attitude quaternion to our newest quaternion estimate
	// which was already taken out from the output buffer
	_output_new.quat_nominal = quat_change * _output_new.quat_nominal;
}

void OutputPredictor::calculateOutputStates(uint64_t time_imu_delayed, const estimator::imuSample &imu_latest, const estimator::stateSample& state, float dt_ekf_avg, bool imu_updated)
{
	const float dt = math::constrain((imu_latest.time_us - _time_newest_imu_sample) / 1e6f, 0.0001f, 0.02f);
	_time_newest_imu_sample = imu_latest.time_us;

	if (_time_newest_imu_sample > 0) {
		_dt_imu_avg = 0.8f * _dt_imu_avg + 0.2f * dt;
	}

	// Use full rate IMU data at the current time horizon

	// correct delta angles for bias offsets
	const float dt_scale_correction = _dt_imu_avg / dt_ekf_avg;

	// Apply corrections to the delta angle required to track the quaternion states at the EKF fusion time horizon
	const Vector3f delta_angle(imu_latest.delta_ang - state.delta_ang_bias * dt_scale_correction + _delta_angle_corr);

	// calculate a yaw change about the earth frame vertical
	const float spin_del_ang_D = delta_angle.dot(Vector3f(_R_to_earth_now.row(2)));
	_yaw_delta_ef += spin_del_ang_D;

	// Calculate filtered yaw rate to be used by the magnetometer fusion type selection logic
	// Note fixed coefficients are used to save operations. The exact time constant is not important.
	_yaw_rate_lpf_ef = 0.95f * _yaw_rate_lpf_ef + 0.05f * spin_del_ang_D / imu_latest.delta_ang_dt;


	_output_new.time_us = imu_latest.time_us;
	_output_vert_new.time_us = imu_latest.time_us;

	const Quatf dq(AxisAnglef{delta_angle});

	// rotate the previous INS quaternion by the delta quaternions
	_output_new.quat_nominal = _output_new.quat_nominal * dq;

	// the quaternions must always be normalised after modification
	_output_new.quat_nominal.normalize();

	// calculate the rotation matrix from body to earth frame
	_R_to_earth_now = Dcmf(_output_new.quat_nominal);

	// correct delta velocity for bias offsets
	const Vector3f delta_vel_body{imu_latest.delta_vel - state.delta_vel_bias * dt_scale_correction};

	// rotate the delta velocity to earth frame
	Vector3f delta_vel_earth{_R_to_earth_now * delta_vel_body};

	// correct for measured acceleration due to gravity
	delta_vel_earth(2) += CONSTANTS_ONE_G * imu_latest.delta_vel_dt;

	// calculate the earth frame velocity derivatives
	if (imu_latest.delta_vel_dt > 1e-4f) {
		_vel_deriv = delta_vel_earth * (1.0f / imu_latest.delta_vel_dt);
	}

	// save the previous velocity so we can use trapezoidal integration
	const Vector3f vel_last(_output_new.vel);

	// increment the INS velocity states by the measurement plus corrections
	// do the same for vertical state used by alternative correction algorithm
	_output_new.vel += delta_vel_earth;
	_output_vert_new.vert_vel += delta_vel_earth(2);

	// use trapezoidal integration to calculate the INS position states
	// do the same for vertical state used by alternative correction algorithm
	const Vector3f delta_pos_NED = (_output_new.vel + vel_last) * (imu_latest.delta_vel_dt * 0.5f);
	_output_new.pos += delta_pos_NED;
	_output_vert_new.vert_vel_integ += delta_pos_NED(2);

	// accumulate the time for each update
	_output_vert_new.dt += imu_latest.delta_vel_dt;

	// correct velocity for IMU offset
	if (imu_latest.delta_ang_dt > 1e-4f) {
		// TODO: dagar angular acceleration

		// calculate the average angular rate across the last IMU update
		const Vector3f ang_rate = imu_latest.delta_ang * (1.f / imu_latest.delta_ang_dt);

		// calculate the velocity of the IMU relative to the body origin
		const Vector3f vel_imu_rel_body = ang_rate % _imu_pos_body;

		// rotate the relative velocity into earth frame
		_vel_imu_rel_body_ned = _R_to_earth_now * vel_imu_rel_body;
	}

	// store the INS states in a ring buffer with the same length and time coordinates as the IMU data buffer
	if (imu_updated) {
		_output_buffer.push(_output_new);
		_output_vert_buffer.push(_output_vert_new);

		// get the oldest INS state data from the ring buffer
		// this data will be at the EKF fusion time horizon
		// TODO: there is no guarantee that data is at delayed fusion horizon
		//       Shouldnt we use pop_first_older_than?
		const outputSample &output_delayed = _output_buffer.get_oldest();
		const outputVert &output_vert_delayed = _output_vert_buffer.get_oldest();

		// calculate the quaternion delta between the INS and EKF quaternions at the EKF fusion time horizon
		const Quatf q_error((state.quat_nominal.inversed() * output_delayed.quat_nominal).normalized());

		// convert the quaternion delta to a delta angle
		const float scalar = (q_error(0) >= 0.0f) ? -2.f : 2.f;

		const Vector3f delta_ang_error{scalar * q_error(1), scalar * q_error(2), scalar * q_error(3)};

		// calculate a gain that provides tight tracking of the estimator attitude states and
		// adjust for changes in time delay to maintain consistent damping ratio of ~0.7
		const float time_delay = fmaxf((imu_latest.time_us - time_imu_delayed) * 1e-6f, _dt_imu_avg);
		const float att_gain = 0.5f * _dt_imu_avg / time_delay;

		// calculate a corrrection to the delta angle
		// that will cause the INS to track the EKF quaternions
		_delta_angle_corr = delta_ang_error * att_gain;
		_output_tracking_error(0) = delta_ang_error.norm();

		/*
		 * Loop through the output filter state history and apply the corrections to the velocity and position states.
		 * This method is too expensive to use for the attitude states due to the quaternion operations required
		 * but because it eliminates the time delay in the 'correction loop' it allows higher tracking gains
		 * to be used and reduces tracking error relative to EKF states.
		 */

		// Complementary filter gains
		const float vel_gain = dt_ekf_avg / math::constrain(_vel_Tau, dt_ekf_avg, 10.0f);
		const float pos_gain = dt_ekf_avg / math::constrain(_pos_Tau, dt_ekf_avg, 10.0f);

		// calculate down velocity and position tracking errors
		const float vert_vel_err = (state.vel(2) - output_vert_delayed.vert_vel);
		const float vert_vel_integ_err = (state.pos(2) - output_vert_delayed.vert_vel_integ);

		// calculate a velocity correction that will be applied to the output state history
		// using a PD feedback tuned to a 5% overshoot
		const float vert_vel_correction = vert_vel_integ_err * pos_gain + vert_vel_err * vel_gain * 1.1f;

		applyCorrectionToVerticalOutputBuffer(vert_vel_correction);

		// calculate velocity and position tracking errors
		const Vector3f vel_err(state.vel - output_delayed.vel);
		const Vector3f pos_err(state.pos - output_delayed.pos);

		_output_tracking_error(1) = vel_err.norm();
		_output_tracking_error(2) = pos_err.norm();

		// calculate a velocity correction that will be applied to the output state history
		_vel_err_integ += vel_err;
		const Vector3f vel_correction = vel_err * vel_gain + _vel_err_integ * sq(vel_gain) * 0.1f;

		// calculate a position correction that will be applied to the output state history
		_pos_err_integ += pos_err;
		const Vector3f pos_correction = pos_err * pos_gain + _pos_err_integ * sq(pos_gain) * 0.1f;

		applyCorrectionToOutputBuffer(vel_correction, pos_correction);
	}
}

void OutputPredictor::applyCorrectionToVerticalOutputBuffer(float vert_vel_correction)
{
	// loop through the vertical output filter state history starting at the oldest and apply the corrections to the
	// vert_vel states and propagate vert_vel_integ forward using the corrected vert_vel
	uint8_t index = _output_vert_buffer.get_oldest_index();

	const uint8_t size = _output_vert_buffer.get_length();

	for (uint8_t counter = 0; counter < (size - 1); counter++) {
		const uint8_t index_next = (index + 1) % size;
		outputVert &current_state = _output_vert_buffer[index];
		outputVert &next_state = _output_vert_buffer[index_next];

		// correct the velocity
		if (counter == 0) {
			current_state.vert_vel += vert_vel_correction;
		}

		next_state.vert_vel += vert_vel_correction;

		// position is propagated forward using the corrected velocity and a trapezoidal integrator
		next_state.vert_vel_integ = current_state.vert_vel_integ + (current_state.vert_vel + next_state.vert_vel) * 0.5f * next_state.dt;

		// advance the index
		index = (index + 1) % size;
	}

	// update output state to corrected values
	_output_vert_new = _output_vert_buffer.get_newest();

	// reset time delta to zero for the next accumulation of full rate IMU data
	_output_vert_new.dt = 0.0f;
}

void OutputPredictor::applyCorrectionToOutputBuffer(const Vector3f &vel_correction, const Vector3f &pos_correction)
{
	// loop through the output filter state history and apply the corrections to the velocity and position states
	for (uint8_t index = 0; index < _output_buffer.get_length(); index++) {
		// a constant velocity correction is applied
		_output_buffer[index].vel += vel_correction;

		// a constant position correction is applied
		_output_buffer[index].pos += pos_correction;
	}

	// update output state to corrected values
	_output_new = _output_buffer.get_newest();
}

Quatf OutputPredictor::calculate_quaternion(const estimator::imuSample& imu_sample, const estimator::stateSample& state, const float dt_ekf_avg) const
{
	// Correct delta angle data for bias errors using bias state estimates from the EKF and also apply
	// corrections required to track the EKF quaternion states
	const Vector3f delta_angle{imu_sample.delta_ang - state.delta_ang_bias * (_dt_imu_avg / dt_ekf_avg) + _delta_angle_corr};

	// increment the quaternions using the corrected delta angle vector
	// the quaternions must always be normalised after modification
	return Quatf{_output_new.quat_nominal * AxisAnglef{delta_angle}}.unit();
}

void OutputPredictor::resetHorizontalVelocity(const Vector2f& delta_horz_vel)
{
	for (uint8_t index = 0; index < _output_buffer.get_length(); index++) {
		_output_buffer[index].vel.xy() += delta_horz_vel;
	}

	_output_new.vel.xy() += delta_horz_vel;
}

void OutputPredictor::resetVerticalVelocity(const float delta_vert_vel)
{
	for (uint8_t index = 0; index < _output_buffer.get_length(); index++) {
		_output_buffer[index].vel(2) += delta_vert_vel;
		_output_vert_buffer[index].vert_vel += delta_vert_vel;
	}

	_output_new.vel(2) += delta_vert_vel;
	_output_vert_new.vert_vel += delta_vert_vel;
}

void OutputPredictor::resetHorizontalPosition(const Vector2f& delta_horz_pos)
{
	for (uint8_t index = 0; index < _output_buffer.get_length(); index++) {
		_output_buffer[index].pos.xy() += delta_horz_pos;
	}

	_output_new.pos.xy() += delta_horz_pos;
}

void OutputPredictor::resetVerticalPosition(const float delta_vert_pos)
{
	// apply the change in height / height rate to our newest height / height rate estimate
	// which have already been taken out from the output buffer
	_output_new.pos(2) += delta_vert_pos;

	// add the reset amount to the output observer buffered data
	for (uint8_t i = 0; i < _output_buffer.get_length(); i++) {
		_output_buffer[i].pos(2) += delta_vert_pos;
		_output_vert_buffer[i].vert_vel_integ += delta_vert_pos;
	}
}
