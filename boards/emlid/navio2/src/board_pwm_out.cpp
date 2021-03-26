/****************************************************************************
 *
 *   Copyright (c) 2017-2021 PX4 Development Team. All rights reserved.
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

#include <drivers/drv_pwm_output.h>

static const int MAX_NUM_PWM = 14;
static const int FREQUENCY_PWM = 400;

int _pwm_fd[MAX_NUM_PWM];
int _pwm_num;

// ::snprintf(path, sizeof(path), "%s/export", _device);
// ::snprintf(path, sizeof(path), "%s/pwm%u/enable", _device, i);
// ::snprintf(path, sizeof(path), "%s/pwm%u/period", _device, i);
// ::snprintf(path, sizeof(path), "%s/pwm%u/duty_cycle", _device, i);

// /sys/class/pwm/pwmchip0/
// /sys/class/pwm/pwmchip0/device
// /sys/class/pwm/pwmchip0/subsystem
// /sys/class/pwm/pwmchip0/export
// /sys/class/pwm/pwmchip0/unexport
// /sys/class/pwm/pwmchip0/power
// /sys/class/pwm/pwmchip0/power/runtime_suspended_time
// /sys/class/pwm/pwmchip0/power/autosuspend_delay_ms
// /sys/class/pwm/pwmchip0/power/runtime_active_time
// /sys/class/pwm/pwmchip0/power/control
// /sys/class/pwm/pwmchip0/power/runtime_status
// /sys/class/pwm/pwmchip0/uevent
// /sys/class/pwm/pwmchip0/npwm

int up_pwm_servo_init(uint32_t channel_mask)
{

	// /sys/class/pwm/pwmchip0/export

	// enable
	//  /sys/class/pwm/pwmchip0  pwm%u/enable

	for (int i = 0; i < MAX_NUM_PWM; ++i) {
		_pwm_fd[i] = -1;
	}




	int i;
	char path[128];

	for (i = 0; i < _pwm_num; ++i) {
		::snprintf(path, sizeof(path), "%s/export", _device);

		if (pwm_write_sysfs(path, i) < 0) {
			PX4_ERR("PWM export failed");
		}
	}

	for (i = 0; i < _pwm_num; ++i) {
		::snprintf(path, sizeof(path), "%s/pwm%u/enable", _device, i);

		if (pwm_write_sysfs(path, 1) < 0) {
			PX4_ERR("PWM enable failed");
		}
	}

	for (i = 0; i < _pwm_num; ++i) {
		::snprintf(path, sizeof(path), "%s/pwm%u/period", _device, i);

		if (pwm_write_sysfs(path, (int)1e9 / FREQUENCY_PWM)) {
			PX4_ERR("PWM period failed");
		}
	}

	for (i = 0; i < _pwm_num; ++i) {
		::snprintf(path, sizeof(path), "%s/pwm%u/duty_cycle", _device, i);
		_pwm_fd[i] = ::open(path, O_WRONLY | O_CLOEXEC);

		if (_pwm_fd[i] == -1) {
			PX4_ERR("PWM: Failed to open duty_cycle.");
			return -errno;
		}
	}

	return 0;
}

void up_pwm_servo_deinit()
{
	for (int i = 0; i < MAX_NUM_PWM; ++i) {
		if (_pwm_fd[i] != -1) {
			::close(_pwm_fd[i]);
		}
	}
}

int up_pwm_servo_set(unsigned channel, servo_position_t value)
{
	// write to %s/pwm%u/duty_cycle
	return 0;






	char data[16];

	if (num_outputs > _pwm_num) {
		num_outputs = _pwm_num;
	}

	int ret = 0;

	//convert this to duty_cycle in ns
	for (int i = 0; i < num_outputs; ++i) {
		int n = ::snprintf(data, sizeof(data), "%u", pwm[i] * 1000);
		int write_ret = ::write(_pwm_fd[i], data, n);

		if (n != write_ret) {
			ret = -1;
		}
	}

	return ret;
}

servo_position_t up_pwm_servo_get(unsigned channel)
{
	// read from // write to %s/pwm%u/duty_cycle
	return 0;
}

void up_pwm_update()
{
	// Trigger all timer's channels in Oneshot mode to fire the oneshots with updated values.
}

uint32_t up_pwm_servo_get_rate_group(unsigned group)
{
	// %s/pwm%u/period
	//  (int)1e9 / FREQUENCY_PWM
	return 0;
}

int up_pwm_servo_set_rate_group_update(unsigned group, unsigned rate)
{
	// %s/pwm%u/period
	//  (int)1e9 / FREQUENCY_PWM
	return 0;
}

void up_pwm_servo_arm(bool armed)
{
	// %s/pwm%u/enable
	// %s/export
}
