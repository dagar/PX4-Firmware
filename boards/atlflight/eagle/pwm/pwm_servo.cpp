
#include <drivers/drv_pwm_output.h>

#include <dev_fs_lib_pwm.h>

static const int NUM_PWM = 4;
static char _device[] = "/dev/pwm-1";

// snapdragon pins 27,28,29,30 configured for pwm (port J13)
static const int PIN_GPIO = 27;
static struct ::dspal_pwm_ioctl_signal_definition _signal_definition;
static struct ::dspal_pwm _pwm_gpio[NUM_PWM];
static struct ::dspal_pwm_ioctl_update_buffer *_update_buffer;
static struct ::dspal_pwm *_pwm;

static constexpr int FREQUENCY_PWM_HZ = 400;

int _fd{-1};

int up_pwm_servo_init(uint32_t channel_mask)
{
	// open PWM device
	_fd = open(_device, 0);

	if (_fd < 0) {
		PX4_ERR("failed to open PWM device!");
		return -1;
	}

	// configure PWM
	for (int i = 0; i < NUM_PWM; i++) {
		_pwm_gpio[i].gpio_id = PIN_GPIO + i;
	}

	// description of signal
	_signal_definition.num_gpios = NUM_PWM;
	_signal_definition.period_in_usecs = 1000000 / FREQUENCY_PWM_HZ;
	_signal_definition.pwm_signal = _pwm_gpio;

	// send signal definition to DSP
	if (::ioctl(_fd, PWM_IOCTL_SIGNAL_DEFINITION, &_signal_definition) != 0) {
		PX4_ERR("failed to send signal to DSP");
		return -1;
	}

	// retrieve shared buffer which will be used to update desired pulse width
	if (::ioctl(_fd, PWM_IOCTL_GET_UPDATE_BUFFER, &_update_buffer) != 0) {
		PX4_ERR("failed to receive update buffer ");
		return -1;
	}

	_pwm = _update_buffer->pwm_signal;

	return 0;
}

void up_pwm_servo_deinit()
{
	close(_fd);
}

int up_pwm_servo_set(unsigned channel, servo_position_t value)
{
	_pwm[channel].pulse_width_in_usecs = value;
	return 0;
}

servo_position_t up_pwm_servo_get(unsigned channel)
{
	return _pwm[channel].pulse_width_in_usecs;
}

void up_pwm_update()
{
	// Trigger all timer's channels in Oneshot mode to fire the oneshots with updated values.
}

uint32_t up_pwm_servo_get_rate_group(unsigned group)
{
	return 0;
}

int up_pwm_servo_set_rate_group_update(unsigned group, unsigned rate)
{
	return 0;
}

void up_pwm_servo_arm(bool armed)
{
}
