
#include <drivers/drv_pwm_output.h>

#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

static constexpr int MAX_ZYNQ_PWMS = 8;	/**< maximum number of pwm channels */

// Period|Hi 32 bits each
struct s_period_hi {
	uint32_t period;
	uint32_t hi;
};

struct pwm_cmd {
	struct s_period_hi periodhi[MAX_ZYNQ_PWMS];
};

volatile struct pwm_cmd *_shared_mem_cmd = nullptr;
static constexpr const char *_device = "/dev/mem";
int _num_outputs;


#define RCOUT_ZYNQ_PWM_BASE	    0x43c00000
static const int TICK_PER_US   =  50;
static const int FREQUENCY_PWM = 400;
static const int TICK_PER_S  = 50000000;

static unsigned long freq2tick(uint16_t freq_hz)
{
	unsigned long duty = TICK_PER_S / (unsigned long)freq_hz;
	return duty;
}

int up_pwm_servo_init(uint32_t channel_mask)
{
	_num_outputs = max_num_outputs;

	if (_num_outputs > MAX_ZYNQ_PWMS) {
		PX4_WARN("number of outputs too large. Setting to %i", MAX_ZYNQ_PWMS);
		_num_outputs = MAX_ZYNQ_PWMS;
	}

	uint32_t mem_fd = open(_device, O_RDWR | O_SYNC);
	_shared_mem_cmd = (struct pwm_cmd *) mmap(0, 0x1000, PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, RCOUT_ZYNQ_PWM_BASE);
	close(mem_fd);

	if (_shared_mem_cmd == nullptr) {
		PX4_ERR("initialize pwm pointer failed.");
		return -1;
	}

	for (int i = 0; i < _num_outputs; ++i) {
		_shared_mem_cmd->periodhi[i].period   =  freq2tick(FREQUENCY_PWM);
		_shared_mem_cmd->periodhi[i].hi = freq2tick(FREQUENCY_PWM) / 2;
		PX4_DEBUG("Output values: %d, %d", _shared_mem_cmd->periodhi[i].period, _shared_mem_cmd->periodhi[i].hi);
	}

	return 0;
}

void up_pwm_servo_deinit()
{
	if (_shared_mem_cmd) {
		munmap((void *)_shared_mem_cmd, 0x1000);
	}
}

int up_pwm_servo_set(unsigned channel, servo_position_t value)
{
	if (num_outputs > _num_outputs) {
		num_outputs = _num_outputs;
	}

	//convert this to duty_cycle in ns
	for (int i = 0; i < num_outputs; ++i) {
		//n = ::asprintf(&data, "%u", pwm[i] * 1000);
		//::write(_pwm_fd[i], data, n);
		_shared_mem_cmd->periodhi[i].hi = TICK_PER_US * pwm[i];
		//printf("ch:%d, val:%d*%d ", ch, period_us, TICK_PER_US);
	}

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
