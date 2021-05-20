obj-m += hal_ext_gpio.o
hal_ext_gpio-objs := cpuinfo.o hal_ext_gpio.o
LDFLAGS += -lgpiod
include /usr/share/linuxcnc/Makefile.modinc

clean:
	rm *.sym *.ver *.so *.mod.c *.o *.tmp
