/****************************************************************************
 *
 *   Copyright (c) 2012-2015, 2017 PX4 Development Team. All rights reserved.
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

#ifndef DRIVERS_PX4FMU_PX4FMU_HPP_
#define DRIVERS_PX4FMU_PX4FMU_HPP_

/**
 * @file PX4FMU.hpp
 *
 * Driver/configurator for the PX4 FMU
 */

#include <cfloat>

#include <board_config.h>
#include <drivers/device/device.h>
#include <drivers/drv_gpio.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_input_capture.h>
#include <drivers/drv_mixer.h>
#include <drivers/drv_pwm_output.h>
#include <px4_config.h>
#include <px4_getopt.h>
#include <px4_log.h>
#include <px4_module.h>
#include <px4_workqueue.h>
#include <systemlib/board_serial.h>
#include <systemlib/mixer/mixer.h>
#include <systemlib/param/param.h>
#include <systemlib/perf_counter.h>
#include <systemlib/pwm_limit/pwm_limit.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/multirotor_motor_limits.h>
#include <uORB/topics/parameter_update.h>

#define SCHEDULE_INTERVAL	2000	/**< The schedule interval in usec (500 Hz) */

/** Mode given via CLI */
enum PortMode {
	PORT_MODE_UNSET = 0,
	PORT_FULL_GPIO,
	PORT_FULL_PWM,
	PORT_PWM4,
	PORT_PWM3,
	PORT_PWM2,
	PORT_PWM1,
	PORT_PWM3CAP1,
	PORT_PWM2CAP2,
	PORT_CAPTURE,
};

#if !defined(BOARD_HAS_PWM)
#  error "board_config.h needs to define BOARD_HAS_PWM"
#endif

class PX4FMU : public device::CDev, public ModuleBase<PX4FMU>
{
public:
	enum Mode {
		MODE_NONE,
		MODE_1PWM,
		MODE_2PWM,
		MODE_2PWM2CAP,
		MODE_3PWM,
		MODE_3PWM1CAP,
		MODE_4PWM,
		MODE_6PWM,
		MODE_8PWM,
		MODE_4CAP,
		MODE_5CAP,
		MODE_6CAP,
	};
	PX4FMU(bool run_as_task);
	virtual ~PX4FMU();

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static PX4FMU *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

	/**
	 * run the main loop: if running as task, continuously iterate, otherwise execute only one single cycle
	 */
	void cycle();

	/** @see ModuleBase::print_status() */
	int print_status() override;

	/** change the FMU mode of the running module */
	static int fmu_new_mode(PortMode new_mode);

	static int test();

	virtual int	ioctl(file *filp, int cmd, unsigned long arg);

	virtual int	init();

	int		set_mode(Mode mode);
	Mode		get_mode() { return _mode; }

	static void	capture_trampoline(void *context, uint32_t chan_index,
					   hrt_abstime edge_time, uint32_t edge_state,
					   uint32_t overflow);

private:
	hrt_abstime _time_last_mix{0};

	static const unsigned MAX_ACTUATORS = DIRECT_PWM_OUTPUT_CHANNELS;

	Mode		_mode{MODE_NONE};

	unsigned	_pwm_default_rate{50};
	unsigned	_pwm_alt_rate{50};
	uint32_t	_pwm_alt_rate_channels{0};

	unsigned	_current_update_rate{0};

	const bool 		_run_as_task;
	static struct work_s	_work;

	int		_armed_sub{-1};
	int		_param_sub{-1};

	orb_advert_t	_outputs_pub{nullptr};
	unsigned	_num_outputs{0};

	int		_class_instance{0};

	bool		_throttle_armed{false};

	bool		_pwm_on{false};
	uint32_t	_pwm_mask{0};
	bool		_pwm_initialized{false};

	MixerGroup	*_mixers{nullptr};

	uint32_t	_groups_required{0};
	uint32_t	_groups_subscribed{0};

	int		_control_subs[actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS] {};
	actuator_controls_s _controls[actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS] {};
	orb_id_t	_control_topics[actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS] {};
	pollfd	_poll_fds[actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS] {};
	unsigned	_poll_fds_num{0};

	static pwm_limit_t	_pwm_limit;
	static actuator_armed_s	_armed;

	uint16_t	_failsafe_pwm[MAX_ACTUATORS] {};
	uint16_t	_disarmed_pwm[MAX_ACTUATORS] {};
	uint16_t	_min_pwm[MAX_ACTUATORS] {};
	uint16_t	_max_pwm[MAX_ACTUATORS] {};
	uint16_t	_trim_pwm[MAX_ACTUATORS] {};

	uint16_t	_reverse_pwm_mask{0};

	unsigned	_num_disarmed_set{0};

	orb_advert_t      _to_mixer_status{nullptr}; 	///< mixer status flags

	float _mot_t_max{0.0f};	// maximum rise time for motor (slew rate limiting)
	float _thr_mdl_fac{0.0f};	// thrust to pwm modeling factor

	perf_counter_t	_ctl_latency;

	static bool	arm_nothrottle()
	{
		return ((_armed.prearmed && !_armed.armed) || _armed.in_esc_calibration_mode);
	}

	static void	cycle_trampoline(void *arg);

	static int	control_callback(uintptr_t handle, uint8_t control_group, uint8_t control_index, float &input);
	void		capture_callback(uint32_t chan_index, hrt_abstime edge_time, uint32_t edge_state, uint32_t overflow);

	void		subscribe();

	int		set_pwm_rate(unsigned rate_map, unsigned default_rate, unsigned alt_rate);
	int		pwm_ioctl(file *filp, int cmd, unsigned long arg);

	void		update_pwm_out_state(bool on);

	void		update_params(bool force = false);
	void		update_pwm_rev_mask();
	void		update_pwm_trims();

#if defined(BOARD_HAS_FMU_GPIO)

	struct GPIOConfig {
		uint32_t	input;
		uint32_t	output;
		uint32_t	alt;
	};

	static const GPIOConfig	_gpio_tab[];
	static const unsigned _ngpio;
#endif /* BOARD_HAS_FMU_GPIO */

	int		gpio_reset(void);
	int		gpio_set_function(uint32_t gpios, int function);
	int		gpio_write(uint32_t gpios, int function);
	int		gpio_read(uint32_t *value);
	int		gpio_ioctl(file *filp, int cmd, unsigned long arg);

	int		capture_ioctl(file *filp, int cmd, unsigned long arg);

	PX4FMU(const PX4FMU &) = delete;
	PX4FMU operator=(const PX4FMU &) = delete;
};



#endif /* DRIVERS_PX4FMU_PX4FMU_HPP_ */
