
obj-m += st_tof_module.o

KDIR ?= /usr/src/linux-headers-`uname -r`

CCPREFIX=''

#Force user to set the variables or modify the makefile
ifndef KDIR
$(error Please set KDIR value - Path to Kernel output dir value)
endif

ifndef CCPREFIX
$(error Please define CCPREFIX environment variable - Path to Compiler)
endif




#For Snapdragon
#ARCH_ARM ?= arm64


#For Raspberry Pi
ARCH_ARM ?= arm



all:
		$(MAKE) -C $(KDIR) M=$(PWD)/ modules ARCH=$(ARCH_ARM) CROSS_COMPILE=$(CCPREFIX)
		
clean:
		$(MAKE) -C $(KDIR) M=$(PWD)/ modules ARCH=$(ARCH_ARM) CROSS_COMPILE=$(CCPREFIX) clean





