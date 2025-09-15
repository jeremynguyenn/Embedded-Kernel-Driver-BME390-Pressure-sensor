# Makefile for BMP390 kernel module and test program
#
# Usage:
#   make - Build kernel modules and test program
#   make install - Install modules and device tree overlay
#   make clean - Clean build artifacts
#   make doc - Generate kernel documentation
#
# Variables:
#   KERNEL_DIR: Kernel build directory (default: running kernel)
#   CROSS_COMPILE: Cross-compiler prefix (optional)
#   SIGN_KEY: Path to module signing key (optional)

obj-m += bmp390.o bmp390_core.o bmp390_iio.o bmp390_i2c.o bmp390_spi.o

KERNEL_DIR ?= /lib/modules/$(shell uname -r)/build
PWD := $(shell pwd)
DTC := dtc
INSTALL_MOD_DIR := kernel/drivers/iio/pressure
INSTALL_DTB_DIR := /boot/overlays

# Compiler flags
ccflags-y := -Wall -Wextra

# Module signing
SIGN_KEY ?=

all: modules bmp390_test

modules:
	$(MAKE) -C $(KERNEL_DIR) M=$(PWD) modules
ifdef SIGN_KEY
	$(KERNEL_DIR)/scripts/sign-file sha256 $(SIGN_KEY) $(PWD)/bmp390.ko
	$(KERNEL_DIR)/scripts/sign-file sha256 $(SIGN_KEY) $(PWD)/bmp390_core.ko
	$(KERNEL_DIR)/scripts/sign-file sha256 $(SIGN_KEY) $(PWD)/bmp390_iio.ko
	$(KERNEL_DIR)/scripts/sign-file sha256 $(SIGN_KEY) $(PWD)/bmp390_i2c.ko
	$(KERNEL_DIR)/scripts/sign-file sha256 $(SIGN_KEY) $(PWD)/bmp390_spi.ko
endif

bmp390_test: bmp390_test.c
	$(CROSS_COMPILE)gcc -o bmp390_test bmp390_test.c -Wall -Wextra

dtb: bmp390_driver.dts
	$(DTC) -@ -I dts -O dtb -o bmp390_driver.dtbo bmp390_driver.dts

clean:
	$(MAKE) -C $(KERNEL_DIR) M=$(PWD) clean
	rm -f bmp390_test *.dtbo *.o *.mod.c *.symvers *.order

install: modules dtb
	$(MAKE) -C $(KERNEL_DIR) M=$(PWD) modules_install INSTALL_MOD_PATH=/usr/lib/modules
	mkdir -p $(INSTALL_DTB_DIR)
	cp bmp390_driver.dtbo $(INSTALL_DTB_DIR)/
	depmod -a

doc:
	$(KERNEL_DIR)/scripts/kernel-doc -rst -enable-linenos \
		$(PWD)/*.c $(PWD)/*.h > bmp390_doc.rst

.PHONY: all modules bmp390_test dtb clean install doc