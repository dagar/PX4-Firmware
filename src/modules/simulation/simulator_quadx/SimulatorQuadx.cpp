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
 * @file sih.cpp
 * Simulator in Hardware
 *
 * @author Romain Chiappinelli      <romain.chiap@gmail.com>
 *
 * Coriolis g Corporation - January 2019
 */

#include "SimulatorQuadx.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>

#include <drivers/drv_pwm_output.h>         // to get PWM flags
#include <lib/drivers/device/Device.hpp>

using namespace math;
using namespace matrix;
using namespace time_literals;

SimulatorQuadx::SimulatorQuadx() :
	ModuleParams(nullptr)
{}

SimulatorQuadx::~SimulatorQuadx()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

void SimulatorQuadx::run()
{
	_px4_accel.set_temperature(T1_C);
	_px4_gyro.set_temperature(T1_C);

	parameters_updated();
	init_variables();

	const hrt_abstime task_start = hrt_absolute_time();
	_last_run = task_start;
	_dist_snsr_time = task_start;

	_actuator_out_sub = uORB::Subscription{ORB_ID(actuator_outputs_sim)};

#if defined(ENABLE_LOCKSTEP_SCHEDULER)
	lockstep_loop();
#else
	realtime_loop();
#endif
	exit_and_cleanup();
}

#if defined(ENABLE_LOCKSTEP_SCHEDULER)
// Get current timestamp in microseconds
static uint64_t micros()
{
	struct timeval t;
	gettimeofday(&t, nullptr);
	return t.tv_sec * ((uint64_t)1000000) + t.tv_usec;
}

void SimulatorQuadx::lockstep_loop()
{
	int rate = math::min(_imu_gyro_ratemax.get(), _imu_integration_rate.get());

	// default to 400Hz (2500 us interval)
	if (rate <= 0) {
		rate = 400;
	}

	// 200 - 2000 Hz
	int sim_interval_us = math::constrain(int(roundf(1e6f / rate)), 500, 5000);

	float speed_factor = 1.f;
	const char *speedup = getenv("PX4_SIM_SPEED_FACTOR");

	if (speedup) {
		speed_factor = atof(speedup);
	}

	int rt_interval_us = int(roundf(sim_interval_us / speed_factor));

	PX4_INFO("Simulation loop with %d Hz (%d us sim time interval)", rate, sim_interval_us);
	PX4_INFO("Simulation with %.1fx speedup. Loop with (%d us wall time interval)", (double)speed_factor, rt_interval_us);
	uint64_t pre_compute_wall_time_us;

	while (!should_exit()) {
		pre_compute_wall_time_us = micros();
		perf_count(_loop_interval_perf);

		_current_simulation_time_us += sim_interval_us;
		struct timespec ts;
		abstime_to_ts(&ts, _current_simulation_time_us);
		px4_clock_settime(CLOCK_MONOTONIC, &ts);

		perf_begin(_loop_perf);
		sensor_step();
		perf_end(_loop_perf);

		// Only do lock-step once we received the first actuator output
		int sleep_time;
		uint64_t current_wall_time_us;

		if (_last_actuator_output_time <= 0) {
			PX4_DEBUG("SIH starting up - no lockstep yet");
			current_wall_time_us = micros();
			sleep_time = math::max(0, sim_interval_us - (int)(current_wall_time_us - pre_compute_wall_time_us));

		} else {
			px4_lockstep_wait_for_components();
			current_wall_time_us = micros();
			sleep_time = math::max(0, rt_interval_us - (int)(current_wall_time_us - pre_compute_wall_time_us));
		}

		_achieved_speedup = 0.99f * _achieved_speedup + 0.01f * ((float)sim_interval_us / (float)(
					    current_wall_time_us - pre_compute_wall_time_us + sleep_time));
		usleep(sleep_time);
	}
}
#endif

void SimulatorQuadx::realtime_loop()
{
	int rate = _imu_gyro_ratemax.get();

	// default to 250 Hz (4000 us interval)
	if (rate <= 0) {
		rate = 250;
	}

	// 200 - 2000 Hz
	int interval_us = math::constrain(int(roundf(1e6f / rate)), 500, 5000);

	px4_sem_init(&_data_semaphore, 0, 0);
	hrt_call_every(&_timer_call, interval_us, interval_us, timer_callback, &_data_semaphore);

	while (!should_exit()) {
		px4_sem_wait(&_data_semaphore);     // periodic real time wakeup
		perf_begin(_loop_perf);
		sensor_step();
		perf_end(_loop_perf);
	}

	hrt_cancel(&_timer_call);
	px4_sem_destroy(&_data_semaphore);
}


void SimulatorQuadx::timer_callback(void *sem)
{
	px4_sem_post((px4_sem_t *)sem);
}

void SimulatorQuadx::sensor_step()
{
	// check for parameter updates
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		updateParams();
		parameters_updated();
	}

	perf_begin(_loop_perf);

	_now = hrt_absolute_time();
	_dt = (_now - _last_run) * 1e-6f;
	_last_run = _now;

	read_motors();

	generate_force_and_torques();

	equations_of_motion();

	reconstruct_sensors_signals();

	// update IMU every iteration
	_px4_accel.update(_now, _acc(0), _acc(1), _acc(2));
	_px4_gyro.update(_now, _gyro(0), _gyro(1), _gyro(2));

	// distance sensor published at 50 Hz
	if (_now - _dist_snsr_time >= 20_ms
	    && fabs(_distance_snsr_override) < 10000) {
		_dist_snsr_time = _now;
		send_dist_snsr();
	}

	// send groundtruth
	publish_sih();

	perf_end(_loop_perf);
}

// store the parameters in a more convenient form
void SimulatorQuadx::parameters_updated()
{
	_T_MAX = _sih_t_max.get();
	_Q_MAX = _sih_q_max.get();
	_L_ROLL = _sih_l_roll.get();
	_L_PITCH = _sih_l_pitch.get();
	_KDV = _sih_kdv.get();
	_KDW = _sih_kdw.get();
	_H0 = _sih_h0.get();

	_MASS = _sih_mass.get();

	_W_I = Vector3f(0.0f, 0.0f, _MASS * CONSTANTS_ONE_G);

	_I = diag(Vector3f(_sih_ixx.get(), _sih_iyy.get(), _sih_izz.get()));
	_I(0, 1) = _I(1, 0) = _sih_ixy.get();
	_I(0, 2) = _I(2, 0) = _sih_ixz.get();
	_I(1, 2) = _I(2, 1) = _sih_iyz.get();

	// guards against too small determinants
	_Im1 = 100.0f * inv(static_cast<typeof _I>(100.0f * _I));

	_distance_snsr_min = _sih_distance_snsr_min.get();
	_distance_snsr_max = _sih_distance_snsr_max.get();
	_distance_snsr_override = _sih_distance_snsr_override.get();

	_T_TAU = _sih_thrust_tau.get();
}

// initialization of the variables for the simulator
void SimulatorQuadx::init_variables()
{
	srand(1234);    // initialize the random seed once before calling generate_wgn()

	_p_I = Vector3f(0.0f, 0.0f, 0.0f);
	_v_I = Vector3f(0.0f, 0.0f, 0.0f);
	_q = Quatf(1.0f, 0.0f, 0.0f, 0.0f);
	_w_B = Vector3f(0.0f, 0.0f, 0.0f);

	_u[0] = _u[1] = _u[2] = _u[3] = 0.0f;
}

// read the motor signals outputted from the mixer
void SimulatorQuadx::read_motors()
{
	actuator_outputs_s actuators_out;

	if (_actuator_out_sub.update(&actuators_out)) {
		_last_actuator_output_time = actuators_out.timestamp;

		for (int i = 0; i < NB_MOTORS; i++) { // saturate the motor signals

			float u_sp = actuators_out.output[i];
			_u[i] = _u[i] + _dt / _T_TAU * (u_sp - _u[i]); // first order transfer function with time constant tau
		}
	}
}

// generate the motors thrust and torque in the body frame
void SimulatorQuadx::generate_force_and_torques()
{
	_T_B = Vector3f(0.0f, 0.0f, -_T_MAX * (+_u[0] + _u[1] + _u[2] + _u[3]));
	_Mt_B = Vector3f(_L_ROLL * _T_MAX * (-_u[0] + _u[1] + _u[2] - _u[3]),
			 _L_PITCH * _T_MAX * (+_u[0] - _u[1] + _u[2] - _u[3]),
			 _Q_MAX * (+_u[0] + _u[1] - _u[2] - _u[3]));
	_Fa_I = -_KDV * _v_I;   // first order drag to slow down the aircraft
	_Ma_B = -_KDW * _w_B;   // first order angular damper
}

// apply the equations of motion of a rigid body and integrate one step
void SimulatorQuadx::equations_of_motion()
{
	_C_IB = matrix::Dcm<float>(_q); // body to inertial transformation

	// Equations of motion of a rigid body
	_p_I_dot = _v_I;                        // position differential
	_v_I_dot = (_W_I + _Fa_I + _C_IB * _T_B) / _MASS;   // conservation of linear momentum
	// _q_dot = _q.derivative1(_w_B);              // attitude differential
	_dq = Quatf::expq(0.5f * _dt * _w_B);
	_w_B_dot = _Im1 * (_Mt_B + _Ma_B - _w_B.cross(_I * _w_B)); // conservation of angular momentum

	// fake ground, avoid free fall
	if (_p_I(2) > 0.0f && (_v_I_dot(2) > 0.0f || _v_I(2) > 0.0f)) {
		if (!_grounded) {    // if we just hit the floor
			// for the accelerometer, compute the acceleration that will stop the vehicle in one time step
			_v_I_dot = -_v_I / _dt;

		} else {
			_v_I_dot.setZero();
		}

		_v_I.setZero();
		_w_B.setZero();
		_grounded = true;

	} else {
		// integration: Euler forward
		_p_I = _p_I + _p_I_dot * _dt;
		_v_I = _v_I + _v_I_dot * _dt;
		_q = _q * _dq;
		_q.normalize();
		// integration Runge-Kutta 4
		// rk4_update(_p_I, _v_I, _q, _w_B);
		_w_B = constrain(_w_B + _w_B_dot * _dt, -6.0f * M_PI_F, 6.0f * M_PI_F);
		_grounded = false;
	}
}

// SimulatorQuadx::States SimulatorQuadx::eom_f(States x) 	// equations of motion f: x'=f(x)
// {
// 	States x_dot{}; 	// dx/dt

// 	Dcmf C_IB = matrix::Dcm<float>(x.q); // body to inertial transformation
// 	// Equations of motion of a rigid body
// 	x_dot.p_I = x.v_I;                        // position differential
// 	x_dot.v_I = (_W_I + _Fa_I + C_IB * _T_B) / _MASS;   // conservation of linear momentum
// 	x_dot.q = x.q.derivative1(x.w_B);              // attitude differential
// 	x_dot.w_B = _Im1 * (_Mt_B + _Ma_B - x.w_B.cross(_I * x.w_B)); // conservation of angular momentum

// 	return x_dot;
// }

// reconstruct the noisy sensor signals
void SimulatorQuadx::reconstruct_sensors_signals()
{
	// The sensor signals reconstruction and noise levels are from [1]
	// [1] Bulka, Eitan, and Meyer Nahon. "Autonomous fixed-wing aerobatics: from theory to flight."
	//     In 2018 IEEE International Conference on Robotics and Automation (ICRA), pp. 6573-6580. IEEE, 2018.

	// IMU
	_acc = _C_IB.transpose() * (_v_I_dot - Vector3f(0.0f, 0.0f, CONSTANTS_ONE_G)) + noiseGauss3f(0.5f, 1.7f, 1.4f);
	_gyro = _w_B + noiseGauss3f(0.14f, 0.07f, 0.03f);

	// temperature
	float altitude = (_H0 - _p_I(2)) + generate_wgn() * 0.14f; // altitude with noise
	_temp_c = T1_K + CONSTANTS_ABSOLUTE_NULL_CELSIUS + TEMP_GRADIENT * altitude; // reconstructed temperture in Celsius
}

void SimulatorQuadx::send_dist_snsr()
{
	_distance_snsr.timestamp = _now;
	_distance_snsr.type = distance_sensor_s::MAV_DISTANCE_SENSOR_LASER;
	_distance_snsr.orientation = distance_sensor_s::ROTATION_DOWNWARD_FACING;
	_distance_snsr.min_distance = _distance_snsr_min;
	_distance_snsr.max_distance = _distance_snsr_max;
	_distance_snsr.signal_quality = -1;
	_distance_snsr.device_id = 0;

	if (_distance_snsr_override >= 0.f) {
		_distance_snsr.current_distance = _distance_snsr_override;

	} else {
		_distance_snsr.current_distance = -_p_I(2) / _C_IB(2, 2);

		if (_distance_snsr.current_distance > _distance_snsr_max) {
			// this is based on lightware lw20 behaviour
			_distance_snsr.current_distance = UINT16_MAX / 100.f;

		}
	}

	_distance_snsr_pub.publish(_distance_snsr);
}

void SimulatorQuadx::publish_sih()
{
	// publish angular velocity groundtruth
	vehicle_angular_velocity_s vehicle_angular_velocity_gt{};
	vehicle_angular_velocity_gt.xyz[0] = _w_B(0); // rollspeed;
	vehicle_angular_velocity_gt.xyz[1] = _w_B(1); // pitchspeed;
	vehicle_angular_velocity_gt.xyz[2] = _w_B(2); // yawspeed;
	vehicle_angular_velocity_gt.timestamp = hrt_absolute_time();
	_vehicle_angular_velocity_gt_pub.publish(vehicle_angular_velocity_gt);


	// publish attitude groundtruth
	vehicle_attitude_s att_gt{};
	att_gt.q[0] = _q(0);
	att_gt.q[1] = _q(1);
	att_gt.q[2] = _q(2);
	att_gt.q[3] = _q(3);
	att_gt.timestamp = hrt_absolute_time();
	_att_gt_pub.publish(att_gt);

	// TODO: publish vehicle_local_position groundtruth
}

float SimulatorQuadx::generate_wgn()   // generate white Gaussian noise sample with std=1
{
	// algorithm 1:
	// float temp=((float)(rand()+1))/(((float)RAND_MAX+1.0f));
	// return sqrtf(-2.0f*logf(temp))*cosf(2.0f*M_PI_F*rand()/RAND_MAX);
	// algorithm 2: from BlockRandGauss.hpp
	static float V1, V2, S;
	static bool phase = true;
	float X;

	if (phase) {
		do {
			float U1 = (float)rand() / (float)RAND_MAX;
			float U2 = (float)rand() / (float)RAND_MAX;
			V1 = 2.0f * U1 - 1.0f;
			V2 = 2.0f * U2 - 1.0f;
			S = V1 * V1 + V2 * V2;
		} while (S >= 1.0f || fabsf(S) < 1e-8f);

		X = V1 * float(sqrtf(-2.0f * float(logf(S)) / S));

	} else {
		X = V2 * float(sqrtf(-2.0f * float(logf(S)) / S));
	}

	phase = !phase;
	return X;
}

// generate white Gaussian noise sample vector with specified std
Vector3f SimulatorQuadx::noiseGauss3f(float stdx, float stdy, float stdz)
{
	return Vector3f(generate_wgn() * stdx, generate_wgn() * stdy, generate_wgn() * stdz);
}

int SimulatorQuadx::print_status()
{
#if defined(ENABLE_LOCKSTEP_SCHEDULER)
	PX4_INFO("Running in lockstep mode");
	PX4_INFO("Achieved speedup: %.2fX", (double)_achieved_speedup);
#endif

	PX4_INFO("vehicle landed: %d", _grounded);
	PX4_INFO("dt [us]: %d", (int)(_dt * 1e6f));
	PX4_INFO("inertial position NED (m)");
	_p_I.print();
	PX4_INFO("inertial velocity NED (m/s)");
	_v_I.print();
	PX4_INFO("attitude roll-pitch-yaw (deg)");
	(Eulerf(_q) * 180.0f / M_PI_F).print();
	PX4_INFO("angular acceleration roll-pitch-yaw (deg/s)");
	(_w_B * 180.0f / M_PI_F).print();
	PX4_INFO("actuator signals");
	Vector<float, 8> u = Vector<float, 8>(_u);
	u.transpose().print();
	PX4_INFO("Aerodynamic forces NED inertial (N)");
	_Fa_I.print();
	PX4_INFO("Aerodynamic moments body frame (Nm)");
	_Ma_B.print();
	PX4_INFO("Thruster moments in body frame (Nm)");
	_Mt_B.print();
	return 0;
}

int SimulatorQuadx::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("simulator_quadx",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_MAX,
				      1250,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

SimulatorQuadx *SimulatorQuadx::instantiate(int argc, char *argv[])
{
	SimulatorQuadx *instance = new SimulatorQuadx();

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

int SimulatorQuadx::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int SimulatorQuadx::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This module provide a simulator for a quadrotor running fully
inside the hardware autopilot.

This simulator subscribes to "actuator_outputs" which are the actuator pwm
signals given by the mixer.

This simulator publishes the sensors signals corrupted with realistic noise
in order to incorporate the state estimator in the loop.

### Implementation
The simulator implements the equations of motion using matrix algebra.
Quaternion representation is used for the attitude.
Forward Euler is used for integration.
Most of the variables are declared global in the .hpp file to avoid stack overflow.


)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("simulator_quadx", "simulation");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}

extern "C" __EXPORT int simulator_quadx_main(int argc, char *argv[])
{
	return SimulatorQuadx::main(argc, argv);
}
