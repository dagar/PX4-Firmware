#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/miscdevice.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/time.h>
#include <linux/platform_device.h>
#include <linux/version.h>

#define ST_TOF_IOCTL_WFI 1

#define IRQ_GPIO 17

static struct miscdevice st_tof_miscdev;
static wait_queue_head_t wq;
static int intr_ready_flag = -1;
static int intr_gpio_nb = IRQ_GPIO;
static unsigned int st_tof_irq_num;

module_param(intr_gpio_nb, int, 0000);
MODULE_PARM_DESC(intr_gpio_nb, "select gpio# to use for vl53l1X interrupt");

static int st_tof_dev_open(struct inode *inode, struct file *file)
{
	pr_debug("%s(%d)\n", __func__, __LINE__);
	return 0;
}

static int st_tof_dev_release(struct inode *inode, struct file *file)
{
	pr_debug("%s(%d)\n", __func__, __LINE__);
	return 0;

}

static long st_tof_dev_ioctl(struct file *file,
			     unsigned int cmd, unsigned long arg)
{

	/* pr_debug("st_tof_dev_ioctl : cmd = %u\n", cmd); */
	switch (cmd) {
	case ST_TOF_IOCTL_WFI:
		pr_debug("%s(%d)\n", __func__, __LINE__);
		wait_event_interruptible(wq, intr_ready_flag != 0);
		/* TODO :
		 * use wait_event_interruptible_timeout(timingBudget + margin)
		 */
		intr_ready_flag = 0;
		break;

	default:
		return -EINVAL;

	}
	return 0;
}

static const struct file_operations st_tof_dev_ranging_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = st_tof_dev_ioctl,
	.open = st_tof_dev_open,
	.release = st_tof_dev_release,
};

static irqreturn_t st_tof_intr_handler(int st_tof_irq_num, void *dev_id)
{
	intr_ready_flag = 1;
	wake_up_interruptible(&wq);
	return IRQ_HANDLED;
}

static int __init st_tof_module_init(void)
{

	unsigned long irqflags;
	int ret;

	st_tof_irq_num = gpio_to_irq(intr_gpio_nb);
	pr_err("IRQ = %u, GPIO = %u\n", st_tof_irq_num, intr_gpio_nb);
	/* irqflags = IRQF_TRIGGER_FALLING|IRQF_ONESHOT; */
	irqflags = IRQF_TRIGGER_RISING | IRQF_ONESHOT;

	init_waitqueue_head(&wq);

	ret = request_irq(st_tof_irq_num, st_tof_intr_handler, irqflags,
			"st_tof_sensor", NULL);
	if (ret) {
		pr_err("Failed to register IRQ handler %d\n", ret);
		return -EPERM;
	}

	st_tof_miscdev.minor = MISC_DYNAMIC_MINOR;
	st_tof_miscdev.name = "st_tof_dev";
	st_tof_miscdev.fops = &st_tof_dev_ranging_fops;

	ret = misc_register(&st_tof_miscdev);
	if (ret) {
		pr_err("Failed to create misc device, err = %d\n", ret);
		return -EPERM;
	}

	pr_debug("%s(%d)\n", __func__, __LINE__);
	return 0;
}

static void __exit st_tof_module_exit(void)
{
	free_irq(st_tof_irq_num, NULL);
	misc_deregister(&st_tof_miscdev);
	pr_debug("%s(%d)\n", __func__, __LINE__);
}

MODULE_AUTHOR("STMicroelectronics Imaging Division");
MODULE_DESCRIPTION("ST VL53L1X sensor IT driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0.0");

module_init(st_tof_module_init);
module_exit(st_tof_module_exit);
