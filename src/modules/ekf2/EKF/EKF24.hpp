

#ifndef EKF_EKF24_H
#define EKF_EKF24_H

#include "common.h"

#include <mathlib/math/filter/AlphaFilter.hpp>
#include <lib/geo/geo.h>

using namespace estimator;

class EKF24
{
public:
	EKF24() = default;
	~EKF24() = default;


	static constexpr uint8_t kNumStates{24}; ///< number of EKF states

	using Vector24f = matrix::Vector<float, kNumStates>;
	using SquareMatrix24f = matrix::SquareMatrix<float, kNumStates>;

	// Sets initial values for the covariance matrix
	// Do not call before quaternion states have been initialised
	void initialiseCovariance();
	bool initialiseFilter(const imuSample &imu_init);

	void reset();
	void resetQuatCov();

	// reset the quaternion states and covariances to the new yaw value, preserving the roll and pitch
	// yaw : Euler yaw angle (rad)
	// yaw_variance : yaw error variance (rad^2)
	void resetQuatStateYaw(float yaw, float yaw_variance);

	// Reset all IMU bias states and covariances to initial alignment values.
	void resetImuBias();
	void resetGyroBias();
	void resetAccelBias();

	void resetMagCov();
	void clearMagCov();

	void resetGyroBiasZCov();

	bool update(const imuSample &imu_sample_delayed);






	// get the state vector at the delayed time horizon
	inline matrix::Vector<float, 24> getStateAtFusionHorizonAsVector() const
	{
		matrix::Vector<float, 24> state;
		state.slice<4, 1>(0, 0) = _state.quat_nominal;
		state.slice<3, 1>(4, 0) = _state.vel;
		state.slice<3, 1>(7, 0) = _state.pos;
		state.slice<3, 1>(10, 0) = _state.gyro_bias;
		state.slice<3, 1>(13, 0) = _state.accel_bias;
		state.slice<3, 1>(16, 0) = _state.mag_I;
		state.slice<3, 1>(19, 0) = _state.mag_B;
		state.slice<2, 1>(22, 0) = _state.wind_vel;

		return state;
	}


	const auto& P() const { return P; }

	float getVariance(uint8_t index) { return P(index, index); }



	// orientation (states 0, 1, 2, 3)
	const Quatf& getQuaternion() const { return _state.quat_nominal; }

	// velocity (states 4, 5, 6)
	const Vector3f& getVelocity() const { return _state.vel; }
	Vector3f getVelocityVariance() const { return Vector3f{P(4, 4), P(5, 5), P(6, 6)}; }

	// position (states 7, 8, 9)
	const Vector3f& getPosition() const { return _state.pos; }
	Vector3f getPositionVariance() const { return Vector3f{P(7, 7), P(8, 8), P(9, 9)}; }

	// gyro bias (states 10, 11, 12)
	const Vector3f &getGyroBias() const { return _state.gyro_bias; } // get the gyroscope bias in rad/s
	Vector3f getGyroBiasVariance() const { return Vector3f{P(10, 10), P(11, 11), P(12, 12)}; } // get the gyroscope bias variance in rad/s
	float getGyroBiasLimit() const { return _params.gyro_bias_lim; }

	// accel bias (states 13, 14, 15)
	const Vector3f &getAccelBias() const { return _state.accel_bias; } // get the accelerometer bias in m/s**2
	Vector3f getAccelBiasVariance() const { return Vector3f{P(13, 13), P(14, 14), P(15, 15)}; } // get the accelerometer bias variance in m/s**2
	float getAccelBiasLimit() const { return _params.acc_bias_lim; }

	// mag bias (states 19, 20, 21)
	const Vector3f &getMagBias() const { return _state.mag_B; }
	Vector3f getMagBiasVariance() const
	{
		// TODO:
		// if (_control_status.flags.mag_3D) {
		// 	return Vector3f{P(19, 19), P(20, 20), P(21, 21)};
		// }

		return _saved_mag_bf_variance;
	}
	float getMagBiasLimit() const { return 0.5f; } // 0.5 Gauss

	// wind velocity (states 22, 23)
	const Vector2f &getWindVelocity() const { return _state.wind_vel; };
	Vector2f getWindVelocityVariance() const { return P.slice<2, 2>(22, 22).diag(); }


	bool measurementUpdate(Vector24f &K, float innovation_variance, float innovation);

	// fuse single velocity and position measurement
	bool fuseVelPosHeight(const float innov, const float innov_var, const int obs_index);





	// rotate quaternion covariances into variances for an equivalent rotation vector
	Vector3f calcRotVecVariances() const;


	// TODO:
	//  - externally set accel bias inhibited
	//  - calcEarthRateNED -> setEarthRateNED?
	//  - externally set _fault_status.flags.bad_acc_vertical


private:


	using Matrix2f = matrix::SquareMatrix<float, 2>;

	// return the square of two floating point numbers - used in auto coded sections
	static constexpr float sq(float var) { return var * var; }


	bool initialiseTilt();



	// initialise the quaternion covariances using rotation vector variances
	// do not call before quaternion states are initialised
	void initialiseQuatCovariances(Vector3f &rot_vec_var);

	// uncorrelate quaternion states from other states
	void uncorrelateQuatFromOtherStates();

	// Increase the yaw error variance of the quaternions
	// Argument is additional yaw variance in rad**2
	void increaseQuatYawErrVariance(float yaw_variance);

	// save covariance data for re-use when auto-switching between heading and 3-axis fusion
	void saveMagCovData();
	// load and save mag field state covariance data for re-use
	void loadMagCovData();

	void predictCovariance(const imuSample &imu_delayed);
	void predictState(const imuSample &imu_delayed);

	// constrain the ekf states
	void constrainStates();

	// if the covariance correction will result in a negative variance, then
	// the covariance matrix is unhealthy and must be corrected
	bool checkAndFixCovarianceUpdate(const SquareMatrix24f &KHP);

	void fuse(const Vector24f &K, float innovation);

	// limit the diagonal of the covariance matrix
	// force symmetry when the argument is true
	void fixCovarianceErrors(bool force_symmetry);

	void clearInhibitedStateKalmanGains(Vector24f &K) const;

	SquareMatrix24f P{};	///< state covariance matrix

	stateSample _state{};		///< state struct of the ekf running at the delayed time horizon
	Dcmf _R_to_earth{};	///< transformation matrix from body frame to earth frame from last EKF prediction

	float _dt_ekf_avg{0.010f}; ///< average update rate of the ekf in s


	bool _is_first_imu_sample{true};
	AlphaFilter<Vector3f> _accel_lpf{0.1f};	///< filtered accelerometer measurement used to align tilt (m/s/s)
	AlphaFilter<Vector3f> _gyro_lpf{0.1f};	///< filtered gyro measurement used for alignment excessive movement check (rad/sec)

	// variables used to inhibit accel bias learning
	bool _accel_bias_inhibit[3] {};		///< true when the accel bias learning is being inhibited for the specified axis
	bool _gyro_bias_inhibit[3] {};		///< true when the gyro bias learning is being inhibited for the specified axis

	Vector3f _ang_rate_delayed_raw{};	///< uncorrected angular rate vector at fusion time horizon (rad/sec)

	Vector2f _accel_lpf_NE{};			///< Low pass filtered horizontal earth frame acceleration (m/sec**2)
	float _yaw_delta_ef{0.0f};		///< Recent change in yaw angle measured about the earth frame D axis (rad)
	float _yaw_rate_lpf_ef{0.0f};		///< Filtered angular rate about earth frame D axis (rad/sec)

	Vector3f _accel_vec_filt{};		///< acceleration vector after application of a low pass filter (m/sec**2)
	float _accel_magnitude_filt{0.0f};	///< acceleration magnitude after application of a decaying envelope filter (rad/sec)
	float _ang_rate_magnitude_filt{0.0f};		///< angular rate magnitude after application of a decaying envelope filter (rad/sec)

	float _height_rate_lpf{0.0f};

	Vector3f _prev_gyro_bias_var{};  ///< saved gyro XYZ bias variances
	Vector3f _prev_accel_bias_var{}; ///< saved accel XYZ bias variances



	Vector3f _saved_mag_bf_variance {}; ///< magnetic field state variances that have been saved for use at the next initialisation (Gauss**2)
	Matrix2f _saved_mag_ef_ne_covmat{}; ///< NE magnetic field state covariance sub-matrix saved for use at the next initialisation (Gauss**2)
	float _saved_mag_ef_d_variance{};   ///< D magnetic field state variance saved for use at the next initialisation (Gauss**2)

	bool _mag_decl_cov_reset{false}; ///< true after the fuseDeclination() function has been used to modify the earth field covariances after a magnetic field reset event.


	Vector3f _earth_rate_NED{};	///< earth rotation vector (NED) in rad/s


	struct {
		int32_t filter_update_interval_us{10000}; ///< filter update interval in microseconds

		int32_t imu_ctrl{static_cast<int32_t>(ImuCtrl::GyroBias) | static_cast<int32_t>(ImuCtrl::AccelBias)};

		// initialization errors
		float switch_on_gyro_bias{0.1f};        ///< 1-sigma gyro bias uncertainty at switch on (rad/sec)
		float switch_on_accel_bias{0.2f};       ///< 1-sigma accelerometer bias uncertainty at switch on (m/sec**2)
		float initial_tilt_err{0.1f};           ///< 1-sigma tilt error after initial alignment using gravity vector (rad)
		const float initial_wind_uncertainty{1.0f};     ///< 1-sigma initial uncertainty in wind velocity (m/sec)

		// input noise
		float gyro_noise{1.5e-2f};              ///< IMU angular rate noise used for covariance prediction (rad/sec)
		float accel_noise{3.5e-1f};             ///< IMU acceleration noise use for covariance prediction (m/sec**2)

		// accel bias learning control
		float acc_bias_lim{0.4f};               ///< maximum accel bias magnitude (m/sec**2)
		float acc_bias_learn_acc_lim{25.0f};    ///< learning is disabled if the magnitude of the IMU acceleration vector is greater than this (m/sec**2)
		float acc_bias_learn_gyr_lim{3.0f};     ///< learning is disabled if the magnitude of the IMU angular rate vector is greater than this (rad/sec)
		float acc_bias_learn_tc{0.5f};          ///< time constant used to control the decaying envelope filters applied to the accel and gyro magnitudes (sec)

		float gyro_bias_lim{0.4f};              ///< maximum gyro bias magnitude (rad/sec)

		// magnetometer fusion
		float mag_noise{5.0e-2f};               ///< measurement noise used for 3-axis magnetoemeter fusion (Gauss)


		// process noise
		float gyro_bias_p_noise{1.0e-3f};       ///< process noise for IMU rate gyro bias prediction (rad/sec**2)
		float accel_bias_p_noise{1.0e-2f};      ///< process noise for IMU accelerometer bias prediction (m/sec**3)
		float mage_p_noise{1.0e-3f};            ///< process noise for earth magnetic field prediction (Gauss/sec)
		float magb_p_noise{1.0e-4f};            ///< process noise for body magnetic field prediction (Gauss/sec)
		float wind_vel_nsd{1.0e-2f};        ///< process noise spectral density for wind velocity prediction (m/sec**2/sqrt(Hz))
		const float wind_vel_nsd_scaler{0.5f};      ///< scaling of wind process noise with vertical velocity


	} _params{};



	// TODO: control status flags sync?
	bool _mag_states_enabled{false};
	bool _wind_states_enabled{false};

};


#endif // EKF_EKF24_H
