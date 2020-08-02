
Compiling the kernel module:
	Set KDIR and CCPREFIX variables in the env or in the makefile
	Change the GPIO number (IRQ_GPIO) for interrupt in module file st_tof_module.c
	It can be also modified at module insertion with 'intr_gpio_nb' param
		ex: sudo insmod st_tof_module.ko intr_gpio_nb=17

Compiling the user space app:
	To compile for RPi, use Makefile. Set CC value to gcc, if compiling natively on Pi
	make builds the application in "waiting for end of ranging interrupts" operation
	make POLLING=1 builds the application in polling mode operation, there is there no need for st_tof_module kernel module

