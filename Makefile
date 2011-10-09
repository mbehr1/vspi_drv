# if KERNELRELEASE is defined it's invoked from the
# kernel build system
ifneq ($(KERNELRELEASE),)

obj-m := vspi_drv.o
#otherwise we're called e.g. from the command
# line; invoke the kernel build system.
else

	KERNELDIR ?= /lib/modules/$(shell uname -r)/build
	PWD := $(shell pwd)
	
default:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules
	
clean:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) clean
		
endif