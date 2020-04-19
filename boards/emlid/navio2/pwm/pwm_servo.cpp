
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
