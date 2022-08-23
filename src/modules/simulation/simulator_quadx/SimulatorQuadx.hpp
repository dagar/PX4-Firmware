/****************************************************************************
*
*   Copyright (c) 2019-2022 PX4 Development Team. All rights reserved.
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
 * @file sih.hpp
 * Simulator in Hardware
 *
 * @author Romain Chiappinelli      <romain.chiap@gmail.com>
 *
 * Coriolis g Corporation - January 2019
 */

// The sensor signals reconstruction and noise levels are from [1]
// [1] Bulka E, and Nahon M, "Autonomous fixed-wing aerobatics: from theory to flight."
//     In 2018 IEEE International Conference on Robotics and Automation (ICRA), pp. 6573-6580. IEEE, 2018.
// The aerodynamic model is from [2]
// [2] Khan W, supervised by Nahon M, "Dynamics modeling of agile fixed-wing unmanned aerial vehicles."
//     McGill University (Canada), PhD thesis, 2016.
// The quaternion integration are from [3]
// [3] Sveier A, Sjøberg AM, Egeland O. "Applied Runge–Kutta–Munthe-Kaas Integration for the Quaternion Kinematics."
//     Journal of Guidance, Control, and Dynamics. 2019 Dec;42(12):2747-54.
// The tailsitter model is from [4]
// [4] Chiappinelli R, supervised by Nahon M, "Modeling and control of a flying wing tailsitter unmanned aerial vehicle."
//     McGill University (Canada), Masters Thesis, 2018.

#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>

#include <matrix/matrix/math.hpp>   // matrix, vectors, dcm, quaterions
#include <conversion/rotation.h>    // math::radians,
#include <lib/geo/geo.h>        // to get the physical constants
#include <drivers/drv_hrt.h>        // to get the real time
#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>
#include <perf/perf_counter.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/vehicle_angular_velocity.h>   // to publish groundtruth
#include <uORB/topics/vehicle_attitude.h>           // to publish groundtruth
#include <uORB/topics/vehicle_global_position.h>    // to publish groundtruth
#include <uORB/topics/distance_sensor.h>

#if defined(ENABLE_LOCKSTEP_SCHEDULER)
#include <sys/time.h>
#endif

using namespace time_literals;

class SimulatorQuadx : public ModuleBase<SimulatorQuadx>, public ModuleParams
{
public:
	SimulatorQuadx();

	virtual ~SimulatorQuadx();

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static SimulatorQuadx *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase::print_status() */
	int print_status() override;

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

	static float generate_wgn();    // generate white Gaussian noise sample

	// generate white Gaussian noise sample as a 3D vector with specified std
	static matrix::Vector3f noiseGauss3f(float stdx, float stdy, float stdz);

	// timer called periodically to post the semaphore
	static void timer_callback(void *sem);

private:
	void parameters_updated();

	// simulated sensor instances
	PX4Accelerometer _px4_accel{1310988}; // 1310988: DRV_IMU_DEVTYPE_SIM, BUS: 1, ADDR: 1, TYPE: SIMULATION
	PX4Gyroscope     _px4_gyro{1310988};  // 1310988: DRV_IMU_DEVTYPE_SIM, BUS: 1, ADDR: 1, TYPE: SIMULATION

	// to publish the distance sensor
	distance_sensor_s                    _distance_snsr{};
	uORB::Publication<distance_sensor_s> _distance_snsr_pub{ORB_ID(distance_sensor)};

	// angular velocity groundtruth
	uORB::Publication<vehicle_angular_velocity_s>	_vehicle_angular_velocity_gt_pub{ORB_ID(vehicle_angular_velocity_groundtruth)};

	// attitude groundtruth
	uORB::Publication<vehicle_attitude_s> _att_gt_pub{ORB_ID(vehicle_attitude_groundtruth)};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::Subscription _actuator_out_sub{ORB_ID(actuator_outputs)};

	// hard constants
	static constexpr uint16_t NB_MOTORS = 4;
	static constexpr float T1_C = 15.0f;                        // ground temperature in Celsius
	static constexpr float T1_K = T1_C - CONSTANTS_ABSOLUTE_NULL_CELSIUS;   // ground temperature in Kelvin
	static constexpr float TEMP_GRADIENT  = -6.5f / 1000.0f;    // temperature gradient in degrees per metre

	void init_variables();
	void read_motors();
	void generate_force_and_torques();
	void equations_of_motion();
	void reconstruct_sensors_signals();
	void send_dist_snsr();
	void publish_sih();
	void sensor_step();

#if defined(ENABLE_LOCKSTEP_SCHEDULER)
	void lockstep_loop();
	uint64_t _current_simulation_time_us{0};
	float _achieved_speedup{0.f};
#endif

	void realtime_loop();
	px4_sem_t       _data_semaphore;
	hrt_call 	_timer_call;

	perf_counter_t  _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t  _loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": cycle interval")};

	hrt_abstime _last_run{0};
	hrt_abstime _last_actuator_output_time{0};
	hrt_abstime _dist_snsr_time{0};
	hrt_abstime _now{0};
	float       _dt{0};         // sampling time [s]
	bool        _grounded{true};// whether the vehicle is on the ground

	matrix::Vector3f    _T_B;           // thrust force in body frame [N]
	matrix::Vector3f    _Fa_I;          // aerodynamic force in inertial frame [N]
	matrix::Vector3f    _Mt_B;          // thruster moments in the body frame [Nm]
	matrix::Vector3f    _Ma_B;          // aerodynamic moments in the body frame [Nm]
	matrix::Vector3f    _p_I;           // inertial position [m]
	matrix::Vector3f    _v_I;           // inertial velocity [m/s]
	matrix::Vector3f    _v_B;           // body frame velocity [m/s]
	matrix::Vector3f    _p_I_dot;       // inertial position differential
	matrix::Vector3f    _v_I_dot;       // inertial velocity differential
	matrix::Quatf       _q;             // quaternion attitude
	matrix::Dcmf        _C_IB;          // body to inertial transformation
	matrix::Vector3f    _w_B;           // body rates in body frame [rad/s]
	matrix::Quatf       _dq;            // quaternion differential
	matrix::Vector3f    _w_B_dot;       // body rates differential
	float       _u[NB_MOTORS];          // thruster signals

	// sensors reconstruction
	matrix::Vector3f    _acc;
	matrix::Vector3f    _gyro;

	float       _temp_c;   // reconstructed (simulated) barometer temperature in degrees Celsius

	// parameters
	float _MASS, _T_MAX, _Q_MAX, _L_ROLL, _L_PITCH, _KDV, _KDW, _H0, _T_TAU;

	matrix::Vector3f _W_I;  // weight of the vehicle in inertial frame [N]
	matrix::Matrix3f _I;    // vehicle inertia matrix
	matrix::Matrix3f _Im1;  // inverse of the inertia matrix

	float _distance_snsr_min, _distance_snsr_max, _distance_snsr_override;

	// parameters defined in sih_params.c
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::IMU_GYRO_RATEMAX>) _imu_gyro_ratemax,
		(ParamInt<px4::params::IMU_INTEG_RATE>) _imu_integration_rate,
		(ParamFloat<px4::params::SIM_QX_MASS>) _sih_mass,
		(ParamFloat<px4::params::SIM_QX_IXX>) _sih_ixx,
		(ParamFloat<px4::params::SIM_QX_IYY>) _sih_iyy,
		(ParamFloat<px4::params::SIM_QX_IZZ>) _sih_izz,
		(ParamFloat<px4::params::SIM_QX_IXY>) _sih_ixy,
		(ParamFloat<px4::params::SIM_QX_IXZ>) _sih_ixz,
		(ParamFloat<px4::params::SIM_QX_IYZ>) _sih_iyz,
		(ParamFloat<px4::params::SIM_QX_T_MAX>) _sih_t_max,
		(ParamFloat<px4::params::SIM_QX_Q_MAX>) _sih_q_max,
		(ParamFloat<px4::params::SIM_QX_L_ROLL>) _sih_l_roll,
		(ParamFloat<px4::params::SIM_QX_L_PITCH>) _sih_l_pitch,
		(ParamFloat<px4::params::SIM_QX_KDV>) _sih_kdv,
		(ParamFloat<px4::params::SIM_QX_KDW>) _sih_kdw,
		(ParamFloat<px4::params::SIM_QX_LOC_H0>) _sih_h0,
		(ParamFloat<px4::params::SIM_QX_DSNSR_MIN>) _sih_distance_snsr_min,
		(ParamFloat<px4::params::SIM_QX_DSNSR_MAX>) _sih_distance_snsr_max,
		(ParamFloat<px4::params::SIM_QX_DSNSR_OVR>) _sih_distance_snsr_override,
		(ParamFloat<px4::params::SIM_QX_T_TAU>) _sih_thrust_tau
	)
};
