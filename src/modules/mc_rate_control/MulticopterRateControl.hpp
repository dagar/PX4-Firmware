/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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
 * @file mc_rate_control_main.cpp
 * Multicopter rate controller.
 *
 * Publication for the desired attitude tracking:
 * Daniel Mellinger and Vijay Kumar. Minimum Snap Trajectory Generation and Control for Quadrotors.
 * Int. Conf. on Robotics and Automation, Shanghai, China, May 2011.
 *
 * @author Lorenz Meier		<lorenz@px4.io>
 * @author Anton Babushkin	<anton.babushkin@me.com>
 * @author Sander Smeets	<sander@droneslab.com>
 *
 * The controller one loop PD loop for angular rate error.
 * Desired rotation calculated keeping in mind that yaw response is normally slower than roll/pitch.
 * For small deviations controller rotates copter to have shortest path of thrust vector and independently rotates around yaw,
 * so actual rotation axis is not constant. For large deviations controller rotates copter around fixed axis.
 * These two approaches fused seamlessly with weight depending on angular error.
 * When thrust vector directed near-horizontally (e.g. roll ~= PI/2) yaw setpoint ignored because of singularity.
 * Controller doesn't use Euler angles for work, they generated only for more human-friendly control and logging.
 * If rotation matrix setpoint is invalid it will be generated from Euler angles for compatibility with old position controllers.
 */

#include <conversion/rotation.h>
#include <drivers/drv_hrt.h>
#include <lib/geo/geo.h>
#include <lib/mathlib/mathlib.h>
#include <px4_config.h>
#include <px4_defines.h>
#include <px4_posix.h>
#include <px4_tasks.h>
#include <systemlib/circuit_breaker.h>
#include <systemlib/err.h>
#include <systemlib/param/param.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/control_state.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/mc_att_ctrl_status.h>
#include <uORB/topics/multirotor_motor_limits.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_correction.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/uORB.h>

/**
 * Multicopter attitude control app start / stop handling function
 *
 * @ingroup apps
 */

static constexpr float YAW_DEADZONE = 0.05f;
static constexpr float MIN_TAKEOFF_THRUST = 0.2f;
static constexpr float TPA_RATE_LOWER_LIMIT = 0.05f;
static constexpr float MANUAL_THROTTLE_MAX_MULTICOPTER = 0.9f;
static constexpr float ATTITUDE_TC_DEFAULT = 0.2f;

static constexpr uint8_t AXIS_INDEX_ROLL = 0;
static constexpr uint8_t AXIS_INDEX_PITCH = 1;
static constexpr uint8_t AXIS_INDEX_YAW = 2;
static constexpr uint8_t AXIS_COUNT = 3;

static constexpr int MAX_GYRO_COUNT = 3;

using uORB::Subscription;

class MulticopterRateControl
{
public:
	MulticopterRateControl();
	~MulticopterRateControl();

	// no copy, assignment, move, move assignment
	MulticopterRateControl(const MulticopterRateControl &) = delete;
	MulticopterRateControl &operator=(const MulticopterRateControl &) = delete;
	MulticopterRateControl(MulticopterRateControl &&) = delete;
	MulticopterRateControl &operator=(MulticopterRateControl &&) = delete;

	/**
	 * Start the multicopter attitude control task.
	 *
	 * @return		OK on success.
	 */
	int		start();

private:

	bool	_task_should_exit{false};	/**< if true, task_main() should exit */
	int	_control_task{-1};		/**< task handle */

	int	_ctrl_state_sub{-1};		/**< control state subscription */
	int	_v_rates_sp_sub{-1};		/**< vehicle rates setpoint subscription */
	int	_v_control_mode_sub{-1};	/**< vehicle control mode subscription */
	int	_params_sub{-1};		/**< parameter updates subscription */
	int	_manual_control_sp_sub{-1};	/**< manual control setpoint subscription */
	int	_vehicle_status_sub{-1};	/**< vehicle status subscription */
	int	_motor_limits_sub{-1};		/**< motor limits subscription */
	int	_battery_status_sub{-1};	/**< battery status subscription */
	int	_sensor_gyro_sub[MAX_GYRO_COUNT];	/**< gyro data subscription */
	int	_sensor_correction_sub{-1};	/**< sensor thermal correction subscription */

	unsigned _gyro_count{MAX_GYRO_COUNT};
	int _selected_gyro{0};

	orb_advert_t	_v_rates_sp_pub{nullptr};		/**< rate setpoint publication */
	orb_advert_t	_actuators_0_pub{nullptr};		/**< attitude actuator controls publication */
	orb_advert_t	_controller_status_pub{nullptr};	/**< controller status publication */

	orb_id_t _rates_sp_id{nullptr};		/**< pointer to correct rates setpoint uORB metadata structure */
	orb_id_t _actuators_id{nullptr};	/**< pointer to correct actuator controls0 uORB metadata structure */

	bool		_actuators_0_circuit_breaker_enabled{false};	/**< circuit breaker to suppress output */

	control_state_s			_ctrl_state{};		/**< control state */
	vehicle_rates_setpoint_s	_v_rates_sp{};		/**< vehicle rates setpoint */
	manual_control_setpoint_s	_manual_control_sp{};	/**< manual control setpoint */
	vehicle_control_mode_s		_v_control_mode{};	/**< vehicle control mode */
	vehicle_status_s		_vehicle_status{};	/**< vehicle status */
	multirotor_motor_limits_s	_motor_limits{};	/**< motor limits */
	mc_att_ctrl_status_s 		_controller_status{}; 	/**< controller status */
	battery_status_s		_battery_status{};	/**< battery status */
	sensor_gyro_s			_sensor_gyro{};		/**< gyro data before thermal correctons and ekf bias estimates are applied */
	sensor_correction_s		_sensor_correction{};	/**< sensor thermal corrections */

	union {
		struct {
			uint16_t motor_pos	: 1; // 0 - true when any motor has saturated in the positive direction
			uint16_t motor_neg	: 1; // 1 - true when any motor has saturated in the negative direction
			uint16_t roll_pos	: 1; // 2 - true when a positive roll demand change will increase saturation
			uint16_t roll_neg	: 1; // 3 - true when a negative roll demand change will increase saturation
			uint16_t pitch_pos	: 1; // 4 - true when a positive pitch demand change will increase saturation
			uint16_t pitch_neg	: 1; // 5 - true when a negative pitch demand change will increase saturation
			uint16_t yaw_pos	: 1; // 6 - true when a positive yaw demand change will increase saturation
			uint16_t yaw_neg	: 1; // 7 - true when a negative yaw demand change will increase saturation
			uint16_t thrust_pos	: 1; // 8 - true when a positive thrust demand change will increase saturation
			uint16_t thrust_neg	: 1; // 9 - true when a negative thrust demand change will increase saturation
		} flags;
		uint16_t value;
	} _saturation_status{};

	perf_counter_t	_loop_perf;			/**< loop performance counter */
	perf_counter_t	_controller_latency_perf;

	math::Vector<3>		_rates_prev;		/**< angular rates on previous step */
	math::Vector<3>		_rates_sp;		/**< angular rates setpoint */
	math::Vector<3>		_rates_int;		/**< angular rates integral error */
	float			_thrust_sp{0.0f};	/**< thrust setpoint */
	math::Vector<3>		_att_control;		/**< attitude control vector */

	math::Matrix<3, 3>	_I;			/**< identity matrix */

	math::Matrix<3, 3>	_board_rotation = {};	/**< rotation matrix for the orientation that the board is mounted */

	struct {
		param_t roll_rate_p;
		param_t roll_rate_i;
		param_t roll_rate_integ_lim;
		param_t roll_rate_d;
		param_t roll_rate_ff;

		param_t pitch_rate_p;
		param_t pitch_rate_i;
		param_t pitch_rate_integ_lim;
		param_t pitch_rate_d;
		param_t pitch_rate_ff;

		param_t yaw_rate_p;
		param_t yaw_rate_i;
		param_t yaw_rate_integ_lim;
		param_t yaw_rate_d;
		param_t yaw_rate_ff;

		param_t tpa_breakpoint_p;
		param_t tpa_breakpoint_i;
		param_t tpa_breakpoint_d;
		param_t tpa_rate_p;
		param_t tpa_rate_i;
		param_t tpa_rate_d;

		param_t acro_roll_max;
		param_t acro_pitch_max;
		param_t acro_yaw_max;

		param_t rattitude_thres;

		param_t bat_scale_en;

		param_t board_rotation;
		param_t board_offset[3];

	} _params_handles{};		/**< handles for interesting parameters */

	struct {
		math::Vector<3> rate_p;				/**< P gain for angular rate error */
		math::Vector<3> rate_i;				/**< I gain for angular rate error */
		math::Vector<3> rate_int_lim;			/**< integrator state limit for rate loop */
		math::Vector<3> rate_d;				/**< D gain for angular rate error */
		math::Vector<3>	rate_ff;			/**< Feedforward gain for desired rates */

		float tpa_breakpoint_p;				/**< Throttle PID Attenuation breakpoint */
		float tpa_breakpoint_i;				/**< Throttle PID Attenuation breakpoint */
		float tpa_breakpoint_d;				/**< Throttle PID Attenuation breakpoint */
		float tpa_rate_p;					/**< Throttle PID Attenuation slope */
		float tpa_rate_i;					/**< Throttle PID Attenuation slope */
		float tpa_rate_d;					/**< Throttle PID Attenuation slope */

		math::Vector<3> acro_rate_max;		/**< max attitude rates in acro mode */

		float rattitude_thres;

		int32_t bat_scale_en;

		int32_t board_rotation;
		float board_offset[3];

	} _params{};

	/**
	 * Update our local parameter cache.
	 */
	int			parameters_update();

	/**
	 * Check for changes in vehicle control mode.
	 */
	void		battery_status_poll();
	void		vehicle_control_mode_poll();
	void		vehicle_manual_poll();
	void		vehicle_rates_setpoint_poll();
	void		vehicle_status_poll();
	void		vehicle_motor_limits_poll();
	void		parameter_update_poll();
	void		sensor_correction_poll();
	void		control_state_poll();

	void		control_attitude_rates(float dt);

	/**
	 * Throttle PID attenuation.
	 */
	math::Vector<3> pid_attenuations(float tpa_breakpoint, float tpa_rate);

	/**
	 * Shim for calling task_main from task_create.
	 */
	static void	task_main_trampoline(int argc, char *argv[]);

	/**
	 * Main attitude control task.
	 */
	void		task_main();
};

namespace mc_rate_control
{
extern MulticopterRateControl	*g_control;
}
