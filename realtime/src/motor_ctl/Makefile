local_include	:= $(PWD)/../../include/
obj-m	:= motor_ctl.o
KDIR	:= /lib/modules/$(shell uname -r)/build
PWD	:= $(shell pwd)
UNAMER	:= $(shell uname -r)
EXTRA_CFLAGS 	:= -DCLKDIR -iquote$(local_include) -I/usr/local/include/ -I/usr/realtime/include/ -I/usr/include/ -ffast-math -mhard-float
#EXTRA_CFLAGS 	:= -DPWM -DTRIG -iquote$(local_include) -I/usr/local/include/ -I/usr/realtime/include/ -I/usr/include/ -ffast-math -mhard-float


default: 
	$(MAKE) -C $(KDIR) SUBDIRS=$(PWD) modules

.PHONY: clean install

install:
	cp motor_ctl.ko /usr/local/lib/modules/$(UNAMER)/motor_ctl.ko
	cp $(PWD)/scripts/motor-ctl /usr/local/bin/motor-ctl	
clean:
	rm motor_ctl.o motor_ctl.mod.o motor_ctl.ko

