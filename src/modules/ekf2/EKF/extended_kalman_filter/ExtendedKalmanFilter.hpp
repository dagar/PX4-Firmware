


#ifndef EXTENDED_KALMAN_FILTER_HPP
#define EXTENDED_KALMAN_FILTER_HPP

#include <lib/mathlib/math/filter/AlphaFilter.hpp>
#include <lib/matrix/matrix/math.hpp>

#include "derivation/generated/state.h"

class ExtendedKalmanFilter
{
public:
	typedef matrix::Vector<float, State::size> VectorState;
	typedef matrix::SquareMatrix<float, State::size> SquareMatrixState;

	ExtendedKalmanFilter();
	~ExtendedKalmanFilter() = default;



	const StateSample &state() const { return _state; }

	template <const IdxDof &S>
	matrix::Vector<float, S.dof>getStateVariance() const { return P.slice<S.dof, S.dof>(S.idx, S.idx).diag(); } // calling getStateCovariance().diag() uses more flash space

	template <const IdxDof &S>
	matrix::SquareMatrix<float, S.dof>getStateCovariance() const { return P.slice<S.dof, S.dof>(S.idx, S.idx); }

	// get the full covariance matrix
	const matrix::SquareMatrix<float, State::size> &covariances() const { return P; }
	float stateCovariance(unsigned r, unsigned c) const { return P(r, c); }

	// get the diagonal elements of the covariance matrix
	matrix::Vector<float, State::size> covariances_diagonal() const { return P.diag(); }

	matrix::Vector3f getRotVarBody() const;
	matrix::Vector3f getRotVarNed() const;
	float getYawVar() const;
	float getTiltVariance() const;


	// gyro bias
	float getGyroBiasLimit() const { return _params.gyro_bias_lim; }

	// accel bias
	float getAccelBiasLimit() const { return _params.acc_bias_lim; }

#if defined(CONFIG_EKF2_MAGNETOMETER)
	float getMagBiasLimit() const { return 0.5f; } // 0.5 Gauss
#endif // CONFIG_EKF2_MAGNETOMETER




	bool update(const uint64_t time_us,
		    const matrix::Vector3f &delta_angle, const float delta_angle_dt,
		    const matrix::Vector3f &delta_velocity, const float delta_velocity_dt);

	// set the internal states and status to their default value
	void reset();


	// fuse single direct state measurement (eg NED velocity, NED position, mag earth field, etc)
	bool fuseDirectStateMeasurement(const float innov, const float innov_var, const float R, const int state_index);

	bool measurementUpdate(VectorState &K, const VectorState &H, const float R, const float innovation);


	void computeYawInnovVarAndH(float variance, float &innovation_variance, VectorState &H_YAW) const;


	// velocity
	bool fuseHorizontalVelocity(const matrix::Vector2f &innovation, const matrix::Vector2f &innovation_variance,
				    const matrix::Vector2f &observation_variance);
	bool fuseVelocity(const matrix::Vector3f &innovation, const matrix::Vector3f &innovation_variance,
			  const matrix::Vector3f &observation_variance);
	void resetHorizontalVelocityTo(const matrix::Vector2f &new_horz_vel, const matrix::Vector2f &new_horz_vel_var);
	void resetVerticalVelocityTo(float new_vert_vel, float new_vert_vel_var);
	void resetVelocityTo(const matrix::Vector3f &new_vel, const matrix::Vector3f &new_vel_var);


	// position
	bool fuseHorizontalPosition(const matrix::Vector2f &innovation, const matrix::Vector2f &innovation_variance,
				    const matrix::Vector2f &observation_variance);
	bool fuseVerticalPosition(const float innovation, const float innovation_variance, const float observation_variance);
	void resetHorizontalPositionTo(const matrix::Vector2f &new_horz_pos, const matrix::Vector2f &new_horz_pos_var);
	void resetVerticalPositionTo(const float new_vert_pos, const float new_vert_pos_var);




	template <const IdxDof &S>
	void resetStateCovariance(const matrix::SquareMatrix<float, S.dof> &cov)
	{
		P.uncorrelateCovarianceSetVariance<S.dof>(S.idx, 0.0f);
		P.slice<S.dof, S.dof>(S.idx, S.idx) = cov;
	}


	void resetQuatCov(const float yaw_noise = NAN);
	void resetQuatCov(const matrix::Vector3f &rot_var_ned);

	// reset the quaternion states and covariances to the new yaw value, preserving the roll and pitch
	// yaw : Euler yaw angle (rad)
	// yaw_variance : yaw error variance (rad^2)
	void resetQuatStateYaw(float yaw, float yaw_variance);

	void resetGyroBias();
	void resetGyroBiasCov();
	void resetGyroBiasZCov();

	void resetAccelBias();
	void resetAccelBiasCov();


#if defined(CONFIG_EKF2_MAGNETOMETER)
	void resetMagCov();
#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_WIND)
	// perform a reset of the wind states and related covariances
	void resetWind();
	void resetWindCov();
	void resetWindToZero();
#endif // CONFIG_EKF2_WIND


	bool accel_bias_inhibited() const { return _accel_bias_inhibit[0] || _accel_bias_inhibit[1] || _accel_bias_inhibit[2]; }
	bool gyro_bias_inhibited() const { return _gyro_bias_inhibit[0] || _gyro_bias_inhibit[1] || _gyro_bias_inhibit[2]; }

	const auto &state_reset_status() const { return _state_reset_status; }

private:

	// initialise ekf covariance matrix
	void initialiseCovariance();
	bool initialiseTilt(const matrix::Vector3f &accel_body);

	// predict ekf state
	void predictState(const uint64_t time_us,
			  const matrix::Vector3f &delta_angle, const float delta_angle_dt,
			  const matrix::Vector3f &delta_velocity, const float delta_velocity_dt);

	// predict ekf covariance
	void predictCovariance(const uint64_t time_us,
			       const matrix::Vector3f &delta_angle, const float delta_angle_dt,
			       const matrix::Vector3f &delta_velocity, const float delta_velocity_dt);

	inline float sq(float x) const { return x * x; };

	void clearInhibitedStateKalmanGains(VectorState &K) const;

	// limit the diagonal of the covariance matrix
	void constrainStateVariances();

	void constrainStateVar(const IdxDof &state, float min, float max);
	void constrainStateVarLimitRatio(const IdxDof &state, float min, float max, float max_ratio = 1.e6f);

	void fuse(const VectorState &K, float innovation);

	static constexpr float kGyroBiasVarianceMin{1e-9f};
	static constexpr float kAccelBiasVarianceMin{1e-9f};

#if defined(CONFIG_EKF2_MAGNETOMETER)
	static constexpr float kMagVarianceMin = 1e-6f;
#endif // CONFIG_EKF2_MAGNETOMETER


	StateSample _state{};       ///< state struct of the ekf running
	SquareMatrixState P{};	    ///< state covariance matrix

	matrix::Dcmf _R_to_earth{}; ///< transformation matrix from body frame to earth frame from last EKF prediction

	struct parameters {
		// input noise
		float gyro_noise{1.5e-2f};              ///< IMU angular rate noise used for covariance prediction (rad/sec)
		float accel_noise{3.5e-1f};             ///< IMU acceleration noise use for covariance prediction (m/sec**2)


		float initial_tilt_err{0.1f};           ///< 1-sigma tilt error after initial alignment using gravity vector (rad)

		float switch_on_gyro_bias{0.1f};        ///< 1-sigma gyro bias uncertainty at switch on (rad/sec)
		float gyro_bias_p_noise{1.0e-3f};       ///< process noise for IMU rate gyro bias prediction (rad/sec**2)
		float gyro_bias_lim{0.4f};              ///< maximum gyro bias magnitude (rad/sec)

		float switch_on_accel_bias{0.2f};       ///< 1-sigma accelerometer bias uncertainty at switch on (m/sec**2)
		float accel_bias_p_noise{1.0e-2f};      ///< process noise for IMU accelerometer bias prediction (m/sec**3)
		float acc_bias_lim{0.4f};               ///< maximum accel bias magnitude (m/sec**2)

#if defined(CONFIG_EKF2_MAGNETOMETER)
		float switch_on_mag_bias {0.5f};        ///< 1-sigma magnetometer bias uncertainty at switch on (gauss)
		float mag_noise{5.0e-2f};               ///< measurement noise used for 3-axis magnetometer fusion (Gauss)
		float mage_p_noise {1.0e-3f};           ///< process noise for earth magnetic field prediction (Gauss/sec)
		float magb_p_noise{1.0e-4f};            ///< process noise for body magnetic field prediction (Gauss/sec)
#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_WIND)
		const float initial_wind_uncertainty {1.0f};  ///< 1-sigma initial uncertainty in wind velocity (m/sec)
		float wind_vel_nsd{1.0e-2f};        ///< process noise spectral density for wind velocity prediction (m/sec**2/sqrt(Hz))
		const float wind_vel_nsd_scaler{0.5f};      ///< scaling of wind process noise with vertical velocity
#endif // CONFIG_EKF2_WIND

#if defined(CONFIG_EKF2_TERRAIN)
		float terrain_p_noise {5.0f};           ///< process noise for terrain offset (m/sec)
		float terrain_gradient{0.5f};           ///< gradient of terrain used to estimate process noise due to changing position (m/m)

		float rng_gnd_clearance{0.1f};         ///< minimum valid value for range when on ground (m)
#endif // CONFIG_EKF2_TERRAIN

	} _params{};



	struct StateResetCounts {
		uint8_t velNE{0};	///< number of horizontal position reset events (allow to wrap if count exceeds 255)
		uint8_t velD{0};	///< number of vertical velocity reset events (allow to wrap if count exceeds 255)
		uint8_t posNE{0};	///< number of horizontal position reset events (allow to wrap if count exceeds 255)
		uint8_t posD{0};	///< number of vertical position reset events (allow to wrap if count exceeds 255)
		uint8_t quat{0};	///< number of quaternion reset events (allow to wrap if count exceeds 255)
	};

	struct StateResets {
		matrix::Vector2f velNE_change;  ///< North East velocity change due to last reset (m)
		float velD_change;	///< Down velocity change due to last reset (m/sec)
		matrix::Vector2f posNE_change;	///< North, East position change due to last reset (m)
		float posD_change;	///< Down position change due to last reset (m)
		matrix::Quatf
		quat_change; ///< quaternion delta due to last reset - multiply pre-reset quaternion by this to get post-reset quaternion

		StateResetCounts reset_count{};
	};

	StateResets _state_reset_status{};	///< reset event monitoring structure containing velocity, position, height and yaw reset information
	StateResetCounts _state_reset_count_prev{};


	bool _filter_initialised{false};	///< true when the EKF sttes and covariances been initialised

	bool _accel_bias_inhibit[3] {};		///< true when the accel bias learning is being inhibited for the specified axis
	bool _gyro_bias_inhibit[3] {};		///< true when the gyro bias learning is being inhibited for the specified axis

#if defined(CONFIG_EKF2_MAGNETOMETER)
	bool _mag_enabled {false};
#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_WIND)
	bool _wind_enabled {false};
#endif // CONFIG_EKF2_WIND

#if defined(CONFIG_EKF2_TERRAIN)
	bool _terrain_enabled {false};
#endif // CONFIG_EKF2_TERRAIN

	matrix::Vector3f _earth_rate_NED{};     ///< earth rotation vector (NED) in rad/s

	float _height_rate_lpf{0.f};
};

#endif // EXTENDED_KALMAN_FILTER_HPP
