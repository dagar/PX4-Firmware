/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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

#include "MulticopterIndiAttRateControl.hpp"

#include <drivers/drv_hrt.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>

using namespace matrix;
using namespace time_literals;
using math::radians;

MulticopterIndiAttRateControl::MulticopterIndiAttRateControl(bool vtol) :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
	_actuators_0_pub(vtol ? ORB_ID(actuator_controls_virtual_mc) : ORB_ID(actuator_controls_0)),
	_loop_perf()
{
	_vehicle_status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;

	parameters_updated();
}

MulticopterIndiAttRateControl::~MulticopterIndiAttRateControl()
{
	perf_free(_loop_perf);
}

bool MulticopterIndiAttRateControl::init()
{
	if (!_vehicle_angular_velocity_sub.registerCallback()) {
		PX4_ERR("vehicle_angular_velocity callback registration failed!");
		return false;
	}

	return true;
}

void MulticopterIndiAttRateControl::parameters_updated()
{
	// rate control parameters
	// The controller gain K is used to convert the parallel (P + I/s + sD) form
	// to the ideal (K * [1 + 1/sTi + sTd]) form
	const Vector3f rate_k = Vector3f(_param_mc_rollrate_k.get(), _param_mc_pitchrate_k.get(), _param_mc_yawrate_k.get());

	// manual rate control acro mode rate limits
	_acro_rate_max = Vector3f(radians(_param_mc_acro_r_max.get()), radians(_param_mc_acro_p_max.get()),
				  radians(_param_mc_acro_y_max.get()));
}

void MulticopterIndiAttRateControl::indi_init_filters()
{
	// tau = 1/(2*pi*Fc)
	float tau = 1.0f / (2.0f * M_PI_F * indi.filt_cutoff);
	float tau_r = 1.0f / (2.0f * M_PI_F * indi.filt_cutoff_r);
	float tau_axis[3] = {tau, tau, tau_r};
	float sample_time = 1.0f / ATTITUDE_RATE;

	// Filtering of gyroscope and actuators
	for (int8_t i = 0; i < 3; i++) {
		init_butterworth_2_low_pass(&indi.u[i], tau_axis[i], sample_time, 0.0f);
		init_butterworth_2_low_pass(&indi.rate[i], tau_axis[i], sample_time, 0.0f);
	}
}

void MulticopterIndiAttRateControl::finite_difference_from_filter(float *output, Butterworth2LowPass *filter)
{
	for (int8_t i = 0; i < 3; i++) {
		output[i] = (filter[i].o[0] - filter[i].o[1]) * ATTITUDE_RATE;
	}
}

void MulticopterIndiAttRateControl::controllerINDIInit()
{
	/*
	 * TODO
	 * Can this also be called during flight, for instance when switching controllers?
	 * Then the filters should not be reset to zero but to the current values of sensors and actuators.
	 */
	float_rates_zero(&indi.angular_accel_ref);
	float_rates_zero(&indi.u_act_dyn);
	float_rates_zero(&indi.u_in);

	// Re-initialize filters
	indi_init_filters();

	attitudeControllerInit(ATTITUDE_UPDATE_DT);
	positionControllerInit();
	positionControllerINDIInit();
}

void MulticopterIndiAttRateControl::filter_pqr(Butterworth2LowPass *filter, struct FloatRates *new_values)
{
	update_butterworth_2_low_pass(&filter[0], new_values->p);
	update_butterworth_2_low_pass(&filter[1], new_values->q);
	update_butterworth_2_low_pass(&filter[2], new_values->r);
}

void MulticopterIndiAttRateControl::controllerINDI(control_t *control, setpoint_t *setpoint,
		const sensorData_t *sensors, const state_t *state, const uint32_t tick)
{
	// 1 - Update the gyro filter with the new measurements.
	float stateAttitudeRateRoll = radians(sensors->gyro.x);
	float stateAttitudeRatePitch = -radians(sensors->gyro.y); // Account for Crazyflie coordinate system
	float stateAttitudeRateYaw = radians(sensors->gyro.z);

	matrix::Vector3f body_rates{stateAttitudeRateRoll, stateAttitudeRatePitch, stateAttitudeRateYaw};
	filter_pqr(_indi.rate, &body_rates);

	// _indi.rate   = vehicle_angular_velocity
	// _indi.rate_d = vehicle_angular_acceleration

	// 2 - Calculate the derivative with finite difference.
	finite_difference_from_filter(_indi.rate_d, _indi.rate);

	// 3 - same filter on the actuators (or control_t values), using the commands from the previous timestep.
	// TODO: filter actuator controls using same lowpass at rates?
	// indi.u_act_dyn
	filter_pqr(_indi.u, &_indi.u_act_dyn);

	// 4 - Calculate the desired angular acceleration by:
	//  4.1 - Rate_reference = P * attitude_error, where attitude error can be calculated with your favorite
	//        algorithm. You may even use a function that is already there, such as attitudeControllerCorrectAttitudePID(),
	//        though this will be inaccurate for large attitude errors, but it will be ok for now.
	//  4.2 - Angular_acceleration_reference = D * (rate_reference – rate_measurement)
	matrix::Vector3f attitude_error{rate_sp - state_attitude_rates}; // state_attitude_rates = sensor_gyro

	_indi.angular_accel_ref = _indi.reference_acceleration_err * attitude_error;


	// 5. Update the For each axis: delta_command = 1/control_effectiveness * (angular_acceleration_reference – angular_acceleration)
	//    Increment in angular acceleration requires increment in control input
	//    G1 is the control effectiveness. In the yaw axis, we need something additional: G2.
	//    It takes care of the angular acceleration caused by the change in rotation rate of the propellers
	//    (they have significant inertia, see the paper mentioned in the header for more explanation)
	_indi.du(0) = 1.f / _indi.g1(0) * (_indi.angular_accel_ref(0) - _indi.rate_d(0));
	_indi.du(1) = 1.f / _indi.g1(1) * (_indi.angular_accel_ref(1) - _indi.rate_d(1));
	_indi.du(2) = 1.f / (_indi.g1(2) - _indi.g2) * (_indi.angular_accel_ref(2) - _indi.rate_d(2) - _indi.g2 * _indi.du(2));


	// 6. Add delta_commands to commands and bound to allowable values
	//  - u is filtered actuator controls from prevoius iteration
	//  - du
	_indi.u_in = _indi.u + _indi.du;

	// bound the total control input
	_indi.u_in(0) = constrain(indi.u_in(0), -bound_control_input, bound_control_input);
	_indi.u_in(1) = constrain(indi.u_in(1), -bound_control_input, bound_control_input);
	_indi.u_in(2) = constrain(indi.u_in(2), -bound_control_input, bound_control_input);

	// Propagate input filters
	// first order actuator dynamics
	_indi.u_act_dyn = _indi.u_act_dyn + _indi.act_dyn * (_indi.u_in - _indi.u_act_dyn);

	// virtual control is the desired angular acceleration

	// the moments of inertia of the vehicle and the propellers as well as the thrust and drag coefficients of the rotors



	// input: the virtual control ν
	// output: angular acceleration of the system omega_dot

	// The angular velocity measurement from the gyroscope is fed back through the
	// differentiating second order filter and subtracted from the virtual control to give the angular acceleration error omega_dot_err



	//Don't increment if thrust is off
	// TODO: this should be something more elegant, but without this the inputs will increment to the maximum before even getting in the air.
	if (_indi.thrust < thrust_threshold) {
		_indi.angular_accel_ref.zero();
		_indi.u_act_dyn.zero();
		_indi.u_in.zero();

		if (_indi.thrust == 0) {
			attitudeControllerResetAllPID();

			// Reset the calculated YAW angle for rate control
			attitudeDesired.yaw = state->attitude.yaw;
		}
	}

	/* INDI feedback */
	_actuator_controls.control[0] = _indi.u_in(0);
	_actuator_controls.control[1] = _indi.u_in(1);
	_actuator_controls.control[2] = _indi.u_in(2);
	_actuator_controls.control[3] = _indi.thrust;
}

void MulticopterIndiAttRateControl::Run()
{
	if (should_exit()) {
		_vehicle_angular_velocity_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		updateParams();
		parameters_updated();
	}

	/* run controller on gyro changes */
	vehicle_angular_velocity_s angular_velocity;

	if (_vehicle_angular_velocity_sub.update(&angular_velocity)) {

		// grab corresponding vehicle_angular_acceleration immediately after vehicle_angular_velocity copy
		vehicle_angular_acceleration_s v_angular_acceleration{};
		_vehicle_angular_acceleration_sub.copy(&v_angular_acceleration);

		const hrt_abstime now = angular_velocity.timestamp_sample;

		// Guard against too small (< 0.125ms) and too large (> 20ms) dt's.
		const float dt = math::constrain(((now - _last_run) * 1e-6f), 0.000125f, 0.02f);
		_last_run = now;

		const Vector3f angular_accel{v_angular_acceleration.xyz};
		const Vector3f rates{angular_velocity.xyz};

		/* check for updates in other topics */
		_v_control_mode_sub.update(&_v_control_mode);

		if (_vehicle_land_detected_sub.updated()) {
			vehicle_land_detected_s vehicle_land_detected;

			if (_vehicle_land_detected_sub.copy(&vehicle_land_detected)) {
				_landed = vehicle_land_detected.landed;
				_maybe_landed = vehicle_land_detected.maybe_landed;
			}
		}

		_vehicle_status_sub.update(&_vehicle_status);

		if (_landing_gear_sub.updated()) {
			landing_gear_s landing_gear;

			if (_landing_gear_sub.copy(&landing_gear)) {
				if (landing_gear.landing_gear != landing_gear_s::GEAR_KEEP) {
					_landing_gear = landing_gear.landing_gear;
				}
			}
		}

		if (_v_control_mode.flag_control_manual_enabled && !_v_control_mode.flag_control_attitude_enabled) {
			// generate the rate setpoint from sticks
			manual_control_setpoint_s manual_control_setpoint;

			if (_manual_control_setpoint_sub.update(&manual_control_setpoint)) {
				// manual rates control - ACRO mode
				const Vector3f man_rate_sp{
					math::superexpo(manual_control_setpoint.y, _param_mc_acro_expo.get(), _param_mc_acro_supexpo.get()),
					math::superexpo(-manual_control_setpoint.x, _param_mc_acro_expo.get(), _param_mc_acro_supexpo.get()),
					math::superexpo(manual_control_setpoint.r, _param_mc_acro_expo_y.get(), _param_mc_acro_supexpoy.get())};

				_rates_sp = man_rate_sp.emult(_acro_rate_max);
				_thrust_sp = math::constrain(manual_control_setpoint.z, 0.0f, 1.0f);

				// publish rate setpoint
				vehicle_rates_setpoint_s v_rates_sp{};
				v_rates_sp.roll = _rates_sp(0);
				v_rates_sp.pitch = _rates_sp(1);
				v_rates_sp.yaw = _rates_sp(2);
				v_rates_sp.thrust_body[0] = 0.0f;
				v_rates_sp.thrust_body[1] = 0.0f;
				v_rates_sp.thrust_body[2] = -_thrust_sp;
				v_rates_sp.timestamp = hrt_absolute_time();

				_v_rates_sp_pub.publish(v_rates_sp);
			}

		} else {
			// use rates setpoint topic
			vehicle_rates_setpoint_s v_rates_sp;

			if (_v_rates_sp_sub.update(&v_rates_sp)) {
				_rates_sp(0) = PX4_ISFINITE(v_rates_sp.roll)  ? v_rates_sp.roll  : rates(0);
				_rates_sp(1) = PX4_ISFINITE(v_rates_sp.pitch) ? v_rates_sp.pitch : rates(1);
				_rates_sp(2) = PX4_ISFINITE(v_rates_sp.yaw)   ? v_rates_sp.yaw   : rates(2);
				_thrust_sp = -v_rates_sp.thrust_body[2];
			}
		}

		// run the rate controller
		if (_v_control_mode.flag_control_rates_enabled) {

			// reset integral if disarmed
			if (!_v_control_mode.flag_armed || _vehicle_status.vehicle_type != vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
				//_rate_control.resetIntegral();
			}

			// update saturation status from mixer feedback
			if (_motor_limits_sub.updated()) {
				multirotor_motor_limits_s motor_limits;

				if (_motor_limits_sub.copy(&motor_limits)) {
					//MultirotorMixer::saturation_status saturation_status;
					//saturation_status.value = motor_limits.saturation_status;

					//_rate_control.setSaturationStatus(saturation_status);
				}
			}

			// run rate controller
			const Vector3f att_control{};// = _rate_control.update(rates, _rates_sp, angular_accel, dt, _maybe_landed || _landed);

			// publish rate controller status
			rate_ctrl_status_s rate_ctrl_status{};
			//_rate_control.getRateControlStatus(rate_ctrl_status);
			rate_ctrl_status.timestamp = hrt_absolute_time();
			_controller_status_pub.publish(rate_ctrl_status);

			// publish actuator controls
			actuator_controls_s actuators{};
			actuators.control[actuator_controls_s::INDEX_ROLL] = PX4_ISFINITE(att_control(0)) ? att_control(0) : 0.0f;
			actuators.control[actuator_controls_s::INDEX_PITCH] = PX4_ISFINITE(att_control(1)) ? att_control(1) : 0.0f;
			actuators.control[actuator_controls_s::INDEX_YAW] = PX4_ISFINITE(att_control(2)) ? att_control(2) : 0.0f;
			actuators.control[actuator_controls_s::INDEX_THROTTLE] = PX4_ISFINITE(_thrust_sp) ? _thrust_sp : 0.0f;
			actuators.control[actuator_controls_s::INDEX_LANDING_GEAR] = _landing_gear;
			actuators.timestamp_sample = angular_velocity.timestamp_sample;

			// scale effort by battery status if enabled
			if (_param_mc_bat_scale_en.get()) {
				if (_battery_status_sub.updated()) {
					battery_status_s battery_status;

					if (_battery_status_sub.copy(&battery_status)) {
						_battery_status_scale = battery_status.scale;
					}
				}

				if (_battery_status_scale > 0.0f) {
					for (int i = 0; i < 4; i++) {
						actuators.control[i] *= _battery_status_scale;
					}
				}
			}

			actuators.timestamp = hrt_absolute_time();
			_actuators_0_pub.publish(actuators);

		} else if (_v_control_mode.flag_control_termination_enabled) {
			if (!_vehicle_status.is_vtol) {
				// publish actuator controls
				actuator_controls_s actuators{};
				actuators.timestamp = hrt_absolute_time();
				_actuators_0_pub.publish(actuators);
			}
		}
	}

	perf_end(_loop_perf);
}

int MulticopterIndiAttRateControl::task_spawn(int argc, char *argv[])
{
	bool vtol = false;

	if (argc > 1) {
		if (strcmp(argv[1], "vtol") == 0) {
			vtol = true;
		}
	}

	MulticopterIndiAttRateControl *instance = new MulticopterIndiAttRateControl(vtol);

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

int MulticopterIndiAttRateControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int MulticopterIndiAttRateControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This implements the a incremental non-linear dynamic inversion attitude & rate controller or multicopters.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("mc_indi_att_rate_control", "controller");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int mc_indi_att_rate_control_main(int argc, char *argv[])
{
	return MulticopterIndiAttRateControl::main(argc, argv);
}
