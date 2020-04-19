
#include <drivers/drv_pwm_output.h>

#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

// https://github.com/StrawsonDesign/librobotcontrol/blob/master/library/src/io/pwm.c

static const int MAX_NUM_PWM = 8;
static const int MIN_FREQUENCY_PWM = 40;

int _num_outputs;

#define MIN_HZ 1
#define MAX_HZ 1000000000
#define MAXBUF 128
#define SYS_DIR "/sys/class/pwm"

// ocp only used for exporting right now, everything else through SYS_DIR
// to allow for shorter strings and neater code.
#define OCP_DIR "/sys/devices/platform/ocp/4830%d000.epwmss/4830%d200.pwm/pwm"
#define OCP_OFFSET	66

int up_pwm_servo_init(uint32_t channel_mask)
{

	// EXPORT
	// /sys/devices/platform/ocp/4830%d000.epwmss/4830%d200.pwm/pwm
	//   len = snprintf(buf, sizeof(buf), OCP_DIR "/pwmchip*/export", ss*2, ss*2);


	//system("ls -la /sys/class/pwm/pwmchip4/pwm-4:0/");
	//system("udevadm trigger");
	//rc_usleep(1000000);
	//system("ls -la /sys/class/pwm/pwmchip4/pwm-4:0/");

	// open file descriptors for duty cycles
	//  len = snprintf(buf, sizeof(buf), SYS_DIR "/pwmchip%d/pwm0/duty_cycle", ss*2); // mode 0
	//  len = snprintf(buf, sizeof(buf), SYS_DIR "/pwm-%d:0/duty_cycle", ssindex[ss]); // mode 1



	return 0;
}

void up_pwm_servo_deinit()
{
	// 	len = snprintf(buf, sizeof(buf), OCP_DIR "/pwmchip*/unexport", ss*2, ss*2);

}

int up_pwm_servo_set(unsigned channel, servo_position_t value)
{

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
	return 0;
}

int up_pwm_servo_set_rate_group_update(unsigned group, unsigned rate)
{
	return 0;
}

void up_pwm_servo_arm(bool armed)
{
}
