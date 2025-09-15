# Makefile for BMP390 kernel module

obj-m += bmp390.o bmp390_core.o bmp390_iio.o bmp390_i2c.o bmp390_spi.o

KERNEL_DIR ?= /lib/modules/$(shell uname -r)/build
PWD := $(shell pwd)

all:
	make -C $(KERNEL_DIR) M=$(PWD) modules

clean:
	make -C $(KERNEL_DIR) M=$(PWD) clean

install:
	make -C $(KERNEL_DIR) M=$(PWD) modules_install
	depmod -a

.PHONY: all clean install