
#ifndef EKF_OUTPUT_PREDICTOR_H
#define EKF_OUTPUT_PREDICTOR_H

#include <matrix/math.hpp>

#include "common.h"
#include "RingBuffer.h"

class OutputPredictor
{
public:

	OutputPredictor() = default;
	~OutputPredictor() = default;

	bool reset(int imu_buffer_length);

	// align output filter states to match EKF states at the fusion time horizon
	void alignOutputFilter(const estimator::stateSample& state);

	void resetQuatState(const matrix::Quatf& quat_change);

	/*
	* Implement a strapdown INS algorithm using the latest IMU data at the current time horizon.
	* Buffer the INS states and calculate the difference with the EKF states at the delayed fusion time horizon.
	* Calculate delta angle, delta velocity and velocity corrections from the differences and apply them at the
	* current time horizon so that the INS states track the EKF states at the delayed fusion time horizon.
	* The inspiration for using a complementary filter to correct for time delays in the EKF
	* is based on the work by A Khosravian:
	* “Recursive Attitude Estimation in the Presence of Multi-rate and Multi-delay Vector Measurements”
	* A Khosravian, J Trumpf, R Mahony, T Hamel, Australian National University
	*/
	void calculateOutputStates(uint64_t time_imu_delayed, const estimator::imuSample &imu_latest, const estimator::stateSample& state, float dt_ekf_avg, bool imu_updated);

	void resetHorizontalVelocity(const matrix::Vector2f& delta_horz_vel);
	void resetVerticalVelocity(const float delta_vert_vel);

	void resetHorizontalPosition(const matrix::Vector2f& delta_horz_pos);
	void resetVerticalPosition(const float delta_vert_pos);

	void resetVerticalVelocityIntegrator(const float vert_pos) { _output_vert_new.vert_vel_integ = vert_pos; }

	/*
	* Predict the previous quaternion output state forward using the latest IMU delta angle data.
	*/
	matrix::Quatf calculate_quaternion(const estimator::imuSample& imu_sample, const estimator::stateSample& state, const float dt_ekf_avg) const;

	const matrix::Quatf &quat_nominal() const { return _output_new.quat_nominal; }

	// get the velocity of the body frame origin in local NED earth frame
	matrix::Vector3f velocity() const { return _output_new.vel - _vel_imu_rel_body_ned; }

	// get the velocity derivative in earth frame
	const matrix::Vector3f& velocity_derivative() const { return _vel_deriv; }

	// get the derivative of the vertical position of the body frame origin in local NED earth frame
	float vertical_position_derivative() const { return _output_vert_new.vert_vel - _vel_imu_rel_body_ned(2); }

	// get the position of the body frame origin in local earth frame
	matrix::Vector3f position() const
	{
		// rotate the position of the IMU relative to the boy origin into earth frame
		const matrix::Vector3f pos_offset_earth = _R_to_earth_now * _imu_pos_body;
		// subtract from the EKF position (which is at the IMU) to get position at the body origin
		return _output_new.pos - pos_offset_earth;
	}

	// return an array containing the output predictor angular, velocity and position tracking
	// error magnitudes (rad), (m/sec), (m)
	const matrix::Vector3f &output_tracking_error() const { return _output_tracking_error; }

	float getYawDeltaEf()
	{
		float yaw_delta_ef = _yaw_delta_ef;
		_yaw_delta_ef = 0.f;
		return yaw_delta_ef;
	}

	float getYawRateLpf() const { return _yaw_rate_lpf_ef; }


	// TODO:
	// set _imu_pos_body
	// set _vel_Tau
	// set _pos_Tau


	// TODO: print status
	//printf("output buffer: %d/%d (%d Bytes)\n", _output_buffer.entries(), _output_buffer.get_length(), _output_buffer.get_total_size());
	// printf("output vert buffer: %d/%d (%d Bytes)\n", _output_vert_buffer.entries(), _output_vert_buffer.get_length(), _output_vert_buffer.get_total_size());

private:

	// return the square of two floating point numbers - used in auto coded sections
	static constexpr inline float sq(float var) { return var * var; }

	/*
	* Calculate a correction to be applied to vert_vel that casues vert_vel_integ to track the EKF
	* down position state at the fusion time horizon using an alternative algorithm to what
	* is used for the vel and pos state tracking. The algorithm applies a correction to the vert_vel
	* state history and propagates vert_vel_integ forward in time using the corrected vert_vel history.
	* This provides an alternative vertical velocity output that is closer to the first derivative
	* of the position but does degrade tracking relative to the EKF state.
	*/
	void applyCorrectionToVerticalOutputBuffer(float vert_vel_correction);

	/*
	* Calculate corrections to be applied to vel and pos output state history.
	* The vel and pos state history are corrected individually so they track the EKF states at
	* the fusion time horizon. This option provides the most accurate tracking of EKF states.
	*/
	void applyCorrectionToOutputBuffer(const matrix::Vector3f &vel_correction, const matrix::Vector3f &pos_correction);


	struct outputSample {
		uint64_t    time_us{};          ///< timestamp of the measurement (uSec)
		matrix::Quatf       quat_nominal{};     ///< nominal quaternion describing vehicle attitude
		matrix::Vector3f    vel{};              ///< NED velocity estimate in earth frame (m/sec)
		matrix::Vector3f    pos{};              ///< NED position estimate in earth frame (m/sec)
	};

	struct outputVert {
		uint64_t    time_us{};          ///< timestamp of the measurement (uSec)
		float       vert_vel{};         ///< Vertical velocity calculated using alternative algorithm (m/sec)
		float       vert_vel_integ{};   ///< Integral of vertical velocity (m)
		float       dt{};               ///< delta time (sec)
	};

	uint64_t _time_newest_imu_sample{0};
	float _dt_imu_avg{0.005f};	// average imu update period in s

	RingBuffer<outputSample> _output_buffer{12};
	RingBuffer<outputVert> _output_vert_buffer{12};

	// output predictor states
	matrix::Vector3f _delta_angle_corr{};	///< delta angle correction vector (rad)
	matrix::Vector3f _vel_err_integ{};	///< integral of velocity tracking error (m)
	matrix::Vector3f _pos_err_integ{};	///< integral of position tracking error (m.s)
	matrix::Vector3f _output_tracking_error{}; ///< contains the magnitude of the angle, velocity and position track errors (rad, m/s, m)

	outputSample _output_new{};		// filter output on the non-delayed time horizon
	outputVert _output_vert_new{};		// vertical filter output on the non-delayed time horizon
	matrix::Matrix3f _R_to_earth_now{};		// rotation matrix from body to earth frame at current time
	matrix::Vector3f _vel_imu_rel_body_ned{};		// velocity of IMU relative to body origin in NED earth frame
	matrix::Vector3f _vel_deriv{};		// velocity derivative at the IMU in NED earth frame (m/s/s)


	// output complementary filter tuning
	// TODO: dagar copy from EKF
	matrix::Vector3f _imu_pos_body{};                ///< xyz position of IMU in body frame (m)
	float _vel_Tau{0.25f};                   ///< velocity state correction time constant (1/sec)
	float _pos_Tau{0.25f};                   ///< position state correction time constant (1/sec)

	float _yaw_delta_ef{0.0f};		///< Recent change in yaw angle measured about the earth frame D axis (rad)
	float _yaw_rate_lpf_ef{0.0f};		///< Filtered angular rate about earth frame D axis (rad/sec)
};

#endif // !EKF_OUTPUT_PREDICTOR_H
