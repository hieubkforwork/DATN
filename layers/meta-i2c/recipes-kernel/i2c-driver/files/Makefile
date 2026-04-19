# 1. Khai báo file đích (Chỉ cần 1 dòng này cho module 1 file)
obj-m := my_i2c_soft.o

# 2. Giữ lại các cờ kiểm tra lỗi code (rất chuẩn chỉ)
ccflags-y += -std=gnu11
ccflags-y += -Wall
ccflags-y += -Wextra
ccflags-y += -Wmissing-prototypes
ccflags-y += -Wmissing-declarations
ccflags-y += -Wno-unused-parameter

KERNEL_SRC ?= $(KERNEL_PATH)

ifeq ($(strip $(KERNEL_SRC)),)
    KERNEL_SRC := /lib/modules/$(shell uname -r)/build
endif

ARCH          ?= arm64
CROSS_COMPILE ?= aarch64-linux-gnu-

all:
	$(MAKE) -C $(KERNEL_SRC) \
	        M=$(PWD)           \
	        ARCH=$(ARCH)       \
	        CROSS_COMPILE=$(CROSS_COMPILE) \
	        modules

clean:
	$(MAKE) -C $(KERNEL_SRC) \
	        M=$(PWD)           \
	        ARCH=$(ARCH)       \
	        CROSS_COMPILE=$(CROSS_COMPILE) \
	        clean

install:
	$(MAKE) -C $(KERNEL_SRC) \
	        M=$(PWD)           \
	        ARCH=$(ARCH)       \
	        CROSS_COMPILE=$(CROSS_COMPILE) \
	        modules_install
	depmod -a

help:
	@echo "Targets:"
	@echo "  all      - Build my_i2c_soft.ko (default)"
	@echo "  clean    - Remove build artefacts"
	@echo "  install  - Install .ko (root required, on-target only)"