/****************************************************************************
 *
 *   Copyright (c) 2015-2023 PX4 Development Team. All rights reserved.
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

#ifndef EKF_EXTENDED_KALMAN_FILTER_HPP
#define EKF_EXTENDED_KALMAN_FILTER_HPP

#include <stdint.h>
#include "derivation/generated/state.h"

#include <lib/mathlib/mathlib.h>
#include <lib/matrix/matrix/math.hpp>


#include "../common.h"

using matrix::AxisAnglef;
using matrix::Dcmf;
using matrix::Eulerf;
using matrix::Matrix3f;
using matrix::Quatf;
using matrix::Vector2f;
using matrix::Vector3f;
using matrix::wrap_pi;

using estimator::imuSample;

class ExtendedKalmanFilter
{
public:
	typedef matrix::Vector<float, State::size> VectorState;
	typedef matrix::SquareMatrix<float, State::size> SquareMatrixState;
	typedef matrix::SquareMatrix<float, 2> Matrix2f;

	ExtendedKalmanFilter() = default;
	~ExtendedKalmanFilter() = default;

	bool init(const Vector3f &accel);

	// initialise ekf covariance matrix
	// Sets initial values for the covariance matrix
	// Do not call before quaternion states have been initialised
	void initialiseCovariance();

	void reset();

	bool update(const imuSample &imu);

	template <const IdxDof &S>
	matrix::Vector<float, S.dof>getStateVariance() const { return P.slice<S.dof, S.dof>(S.idx, S.idx).diag(); } // calling getStateCovariance().diag() uses more flash space

	template <const IdxDof &S>
	matrix::SquareMatrix<float, S.dof>getStateCovariance() const { return P.slice<S.dof, S.dof>(S.idx, S.idx); }

	const matrix::SquareMatrix<float, State::size> &covariances() const { return P; }

	const StateSample &state() const { return _state; }
	const VectorState &state_vector() const { return _state.vector(); }

	// orientation
	const Quatf &getStateOrientation() const { return _state.quat_nominal; }
	bool fuseYaw(const float obs_var, const float innov);
	bool fuseYaw(const float innov, const float innov_var, const VectorState &H_YAW);
	void computeYawInnovVarAndH(float variance, float &innovation_variance, VectorState &H_YAW) const;

	void resetQuatCov(const Vector3f &euler_noise_ned);
	// reset the quaternion states and covariances to the new yaw value, preserving the roll and pitch
	void resetQuatStateYaw(float yaw, float yaw_variance);
	float getYawVar() const;

	// velocity
	bool fuseVelocity(const float innov, const float innov_var, const int axis);
	void resetVelocityTo(const Vector3f &vel, const Vector3f &vel_var);
	void resetHorizontalVelocityTo(const Vector2f &vel_xy, const Vector2f &vel_xy_var);
	void resetVerticalVelocityTo(const float vel_z, const float vel_z_var);

	struct CorrectionResult3
	{
		Vector3f innovation{};
		Vector3f innovation_variance{};
		Vector3f test_ratio{};
		bool rejected[3]{};
	};

	CorrectionResult3 velocityCorrection(const Vector3f &observation, const Vector3f &observation_variance, float innovation_gate = 1.f);
	bool velocityReset(const Vector3f& observation, const Vector3f& observation_variance);
	bool velocityXYReset(const Vector2f& observation, const Vector2f& observation_variance);
	bool velocityZReset(const float& observation, const float& observation_variance);


	// position
	bool fusePosition(const float innov, const float innov_var, const int axis);
	void resetPositionTo(const Vector3f &pos, const Vector3f &pos_var);
	void resetHorizontalPositionTo(const Vector2f &xy, const Vector2f &xy_var);
	void resetVerticalPositionTo(const float z, const float z_var);

	// gyro bias
	void resetGyroBias();
	void resetGyroBiasCov();
	void resetGyroBiasZCov();
	void inhibitGyroBiasAxis(int axis);
	void uninhibitGyroBiasAxis(int axis);

	// accel bias
	void resetAccelBias();
	void resetAccelBiasCov();
	void inhibitAccelBiasAxis(int axis);
	void uninhibitAccelBiasAxis(int axis);

#if defined(CONFIG_EKF2_MAGNETOMETER)
	// mag
	void resetMagBias();
	void resetMagCov();

	bool setMagEarth(const Vector3f &mag_earth_field);
	bool setMagBias(const Vector3f &mag_bias);
#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_WIND)
	// wind
	void resetWind();
	void resetWindToZero();
	void resetWindCov();
#endif // CONFIG_EKF2_WIND

	bool measurementUpdate(VectorState &K, float innovation_variance, float innovation);



	bool gyro_bias_inhibited() const { return _gyro_bias_inhibit[0] || _gyro_bias_inhibit[1] || _gyro_bias_inhibit[2]; }
	bool accel_bias_inhibited() const { return _accel_bias_inhibit[0] || _accel_bias_inhibit[1] || _accel_bias_inhibit[2]; }


	template <const IdxDof &S>
	void resetStateCovariance(const matrix::SquareMatrix<float, S.dof> &cov)
	{
		P.uncorrelateCovarianceSetVariance<S.dof>(S.idx, 0.f);
		P.slice<S.dof, S.dof>(S.idx, S.idx) = cov;
	}


	// bool horizontalVelocityValid() const { return !_vel_xy_deadreckon_time_exceeded; }
	// bool verticalVelocityValid() const { return !_vel_z_deadreckon_time_exceeded; }

	// bool horizontalPositionValid() const { return !_pos_xy_deadreckon_time_exceeded; }
	// bool verticalPositionValid() const { return !_pos_z_deadreckon_time_exceeded; }


	// TODO:
	//  status flags for core states
	//    - bad vel, bad pos, bad yaw?
	//    - bad mag, bad wind


	// TODO:

	// inhibitGyro(int axis)
	// inhibitAccel(int axis)

	// inhibitMag()

	// enableMag()
	// disableMag()

	// enableWind()
	// disableWind()

private:
	// return the square of two floating point numbers - used in auto coded sections
	static constexpr float sq(float var) { return var * var; }

	// predict
	void predictCovariance(const imuSample &imu);
	void predictState(const imuSample &imu);

	// correct

	// generic function which will perform a fusion step given a kalman gain K
	// and a scalar innovation value
	void fuse(const VectorState &K, float innovation);

	void clearInhibitedStateKalmanGains(VectorState &K) const;

	// if the covariance correction will result in a negative variance, then
	// the covariance matrix is unhealthy and must be corrected
	bool checkAndFixCovarianceUpdate(const SquareMatrixState &KHP);

	// limit the diagonal of the covariance matrix
	// force symmetry when the argument is true
	void fixCovarianceErrors(bool force_symmetry);

	// constrain the ekf states
	void constrainStates();
	void constrainStateVar(const IdxDof &state, float min, float max);

	SquareMatrixState P{}; ///< state covariance matrix
	StateSample _state{};  ///< state struct of the ekf running at the delayed time horizon

	float _dt_ekf_avg{0.010f}; ///< average update rate of the ekf in s

	bool _filter_initialised{false}; ///< true when the EKF sttes and covariances been initialised

	uint64_t _time_delayed_us{0}; // captures the imu sample on the delayed time horizon

	uint64_t _time_last_yaw_fuse{0};
	uint64_t _time_last_pos_xy_fuse{0};     ///< time the last fusion of horizontal position measurements was performed (uSec)
	uint64_t _time_last_pos_z_fuse{0};      ///< time the last fusion of vertical position measurements was performed (uSec)
	uint64_t _time_last_vel_xy_fuse{0};     ///< time the last fusion of horizontal velocity measurements was performed (uSec)
	uint64_t _time_last_vel_z_fuse{0};      ///< time the last fusion of verticalvelocity measurements was performed (uSec)
#if defined(CONFIG_EKF2_MAGNETOMETER)
	uint64_t _time_last_mag_fuse {0};       ///< time the last fusion of magnetometer measurements was performed (uSec)
#endif // CONFIG_EKF2_MAGNETOMETER
#if defined(CONFIG_EKF2_WIND)
	uint64_t _time_last_wind_fuse {0};      ///< time the last fusion of wind measurements was performed (uSec)
#endif // CONFIG_EKF2_WIND

	// variables used to inhibit accel bias learning
	bool _gyro_bias_inhibit[3] {};		///< true when the gyro bias learning is being inhibited for the specified axis
	bool _accel_bias_inhibit[3] {};		///< true when the accel bias learning is being inhibited for the specified axis

	Vector3f _prev_gyro_bias_var{};         ///< saved gyro XYZ bias variances
	Vector3f _prev_accel_bias_var{};        ///< saved accel XYZ bias variances

	Vector3f _earth_rate_NED{};	///< earth rotation vector (NED) in rad/s

	float _height_rate_lpf{0.0f}; // TODO: move outside?

	struct {
		bool gyro_bias_inhibit[3] {false, false, false}; ///< true when the gyro bias learning is being inhibited for the specified axis

		bool accel_bias_inhibit[3] {false, false, false}; ///< true when the accel bias learning is being inhibited for the specified axis

		bool bad_acc_vertical{false};

		bool mag{false};  ///< true if 3-axis magnetometer measurement fusion (mag states only) is intended

		bool wind{false}; ///< true when wind velocity is being estimated

	} _control_flags{};

	struct parameters {
		// input noise
		float gyro_noise{0.015f};               ///< IMU angular rate noise used for covariance prediction (rad/sec)
		float accel_noise{0.35f};               ///< IMU acceleration noise use for covariance prediction (m/sec**2)

		float initial_tilt_err{0.1f};           ///< 1-sigma tilt error after initial alignment using gravity vector (rad)
		float initial_yaw_uncertainty{0.5f};    ///< 1-sigma initial uncertainty in yaw (rad)
		float initial_vel_uncertainty{0.5f};    ///< 1-sigma initial uncertainty in velocity (m/sec)
		float initial_pos_uncertainty{0.5f};    ///< 1-sigma initial uncertainty in position (m)

		// gyro bias
		float switch_on_gyro_bias{0.1f};        ///< 1-sigma gyro bias uncertainty at switch on (rad/sec)
		float gyro_bias_p_noise{1.e-3f};        ///< process noise for IMU rate gyro bias prediction (rad/sec**2)
		float gyro_bias_lim{0.4f};              ///< maximum gyro bias magnitude (rad/sec)

		// accel bias
		float switch_on_accel_bias{0.2f};       ///< 1-sigma accelerometer bias uncertainty at switch on (m/sec**2)
		float accel_bias_p_noise{1.e-2f};       ///< process noise for IMU accelerometer bias prediction (m/sec**3)
		float acc_bias_lim{0.4f};               ///< maximum accel bias magnitude (m/sec**2)

#if defined(CONFIG_EKF2_MAGNETOMETER)
		// mag
		float initial_mag_uncertainty{5.e-2f};  ///< measurement noise used for 3-axis magnetometer fusion (Gauss)
		float mage_p_noise{1.e-3f};             ///< process noise for earth magnetic field prediction (Gauss/sec)
		float magb_p_noise{1.e-4f};             ///< process noise for body magnetic field prediction (Gauss/sec)
		float mag_bias_lim{0.5f};               ///< maximum mag bias magnitude (Gauss)
#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_WIND)
		// wind
		float initial_wind_uncertainty{1.f};    ///< 1-sigma initial uncertainty in wind velocity (m/sec)
		float wind_vel_nsd{1.e-2f};             ///< process noise spectral density for wind velocity prediction (m/sec**2/sqrt(Hz))
		float wind_vel_nsd_scaler{0.5f};        ///< scaling of wind process noise with vertical velocity
#endif // CONFIG_EKF2_WIND

	} _params{};

	struct StateResetCounts {
		uint8_t quat{0};   ///< number of quaternion reset events (allow to wrap if count exceeds 255)
		uint8_t vel_xy{0}; ///< number of horizontal position reset events (allow to wrap if count exceeds 255)
		uint8_t vel_z{0};  ///< number of vertical velocity reset events (allow to wrap if count exceeds 255)
		uint8_t pos_xy{0}; ///< number of horizontal position reset events (allow to wrap if count exceeds 255)
		uint8_t pos_z{0};  ///< number of vertical position reset events (allow to wrap if count exceeds 255)
	};

	struct StateResets {
		Quatf quat_change{1.f, 0.f, 0.f, 0.f}; ///< quaternion delta due to last reset - multiply pre-reset quaternion by this to get post-reset quaternion
		Vector2f vel_xy_change{};              ///< North East velocity change due to last reset (m)
		float vel_z_change{0.f};               ///< Down velocity change due to last reset (m/sec)
		Vector2f pos_xy_change{};              ///< North, East position change due to last reset (m)
		float pos_z_change{0.f};               ///< Down position change due to last reset (m)

		StateResetCounts reset_count{};
	};

	StateResets _state_reset_status{};	///< reset event monitoring structure containing velocity, position, height and yaw reset information
	StateResetCounts _state_reset_count_prev{};

};

#endif // !EKF_EXTENDED_KALMAN_FILTER_HPP
