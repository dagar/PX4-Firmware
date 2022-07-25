
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
	void alignOutputFilter(const matrix::Quatf &quat_nominal, const matrix::Vector3f &vel, const matrix::Vector3f &pos);

	void resetQuatState(const matrix::Quatf &quat_change);

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
	void calculateOutputStates(const uint64_t &time_us, const matrix::Vector3f &delta_ang, const float delta_ang_dt,
				   const matrix::Vector3f &delta_vel, const float delta_vel_dt);

	void correctOutputStates(const uint64_t &time_imu_delayed, const matrix::Quatf &quat_delayed,
				 const matrix::Vector3f &velocity_delayed, const matrix::Vector3f &position_delayed);

	void resetHorizontalVelocity(const matrix::Vector2f &delta_horz_vel);
	void resetVerticalVelocity(const float delta_vert_vel);

	void resetHorizontalPosition(const matrix::Vector2f &delta_horz_pos);
	void resetVerticalPosition(const float delta_vert_pos);

	void resetVerticalVelocityIntegrator(const float vert_pos)
	{
		//_output_vert_buffer.get_newest().vert_vel_integ = vert_pos;
		int i = _output_vert_buffer.get_oldest_index();
		_output_vert_buffer[i].vert_vel_integ = vert_pos;
	}

	const matrix::Quatf &quat_nominal() const { return _output_buffer.get_newest().quat_nominal; }

	// get the velocity of the body frame origin in local NED earth frame
	matrix::Vector3f velocity() const { return _output_buffer.get_newest().vel - _vel_imu_rel_body_ned; }

	// get the velocity derivative in earth frame
	const matrix::Vector3f &velocity_derivative() const { return _vel_deriv; }

	// get the derivative of the vertical position of the body frame origin in local NED earth frame
	float vertical_position_derivative() const { return _output_vert_buffer.get_newest().vert_vel - _vel_imu_rel_body_ned(2); }

	// get the position of the body frame origin in local earth frame
	matrix::Vector3f position() const
	{
		// rotate the position of the IMU relative to the boy origin into earth frame
		const matrix::Vector3f pos_offset_earth = matrix::Dcmf{_output_buffer.get_newest().quat_nominal} * _imu_pos_body;
		// subtract from the EKF position (which is at the IMU) to get position at the body origin
		return _output_buffer.get_newest().pos - pos_offset_earth;
	}

	// return an array containing the output predictor angular, velocity and position tracking
	// error magnitudes (rad), (m/sec), (m)
	matrix::Vector3f output_tracking_error() const
	{
		// contains the magnitude of the angle, velocity and position track errors (rad, m/s, m)
		return matrix::Vector3f{_output_tracking_delta_angle_error, _output_tracking_vel_error, _output_tracking_pos_error};
	}

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

	struct outputSample {
		uint64_t    time_us{};          ///< timestamp of the measurement (uSec)
		matrix::Quatf    quat_nominal{};     ///< nominal quaternion describing vehicle attitude
		matrix::Vector3f vel{};              ///< NED velocity estimate in earth frame (m/sec)
		matrix::Vector3f pos{};              ///< NED position estimate in earth frame (m/sec)
	};

	struct outputVert {
		uint64_t    time_us{};          ///< timestamp of the measurement (uSec)
		float       vert_vel{};         ///< Vertical velocity calculated using alternative algorithm (m/sec)
		float       vert_vel_integ{};   ///< Integral of vertical velocity (m)
		float       dt{};               ///< delta time (sec)
	};

	// TODO: dagar update these
	matrix::Vector3f _accel_bias{};
	matrix::Vector3f _gyro_bias{};

	uint64_t _time_newest_imu_sample{0};
	uint64_t _time_delayed_imu_sample{0};
	float _dt_imu_avg{0.005f};	// average IMU update period in s
	float _dt_ekf_avg{0.010f};	// average EKF update period in s

	RingBuffer<outputSample> _output_buffer{24};
	RingBuffer<outputVert> _output_vert_buffer{24};

	// output predictor states
	matrix::Vector3f _delta_angle_corr{};	///< delta angle correction vector (rad)
	matrix::Vector3f _vel_err_integ{};	///< integral of velocity tracking error (m)
	matrix::Vector3f _pos_err_integ{};	///< integral of position tracking error (m.s)

	float _output_tracking_delta_angle_error{};
	float _output_tracking_vel_error{};
	float _output_tracking_pos_error{};

	matrix::Vector3f _vel_imu_rel_body_ned{};		// velocity of IMU relative to body origin in NED earth frame
	matrix::Vector3f _vel_deriv{};		// velocity derivative at the IMU in NED earth frame (m/s/s)

	// output complementary filter tuning
	// TODO: dagar copy from EKF
	matrix::Vector3f _imu_pos_body{};                ///< xyz position of IMU in body frame (m)
	float _vel_Tau{0.25f};                   ///< velocity state correction time constant (1/sec)
	float _pos_Tau{0.25f};                   ///< position state correction time constant (1/sec)
};

#endif // !EKF_OUTPUT_PREDICTOR_H
