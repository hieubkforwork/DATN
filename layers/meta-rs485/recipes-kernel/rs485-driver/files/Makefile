# 1. Đổi tên Module (Thêm chữ _mod vào để khác với file .c)
MODULE_NAME := rs485_sensor_mod

# 2. Khai báo tạo module từ 2 file thành phần
obj-m := $(MODULE_NAME).o

# Dùng cú pháp -y thay cho -objs (chuẩn Kbuild hiện đại)
$(MODULE_NAME)-y := \
    rs485_modbus.o \
    rs485_rtu.o

ccflags-y += -std=gnu11
ccflags-y += -Wall
ccflags-y += -Wextra
ccflags-y += -Wmissing-prototypes
ccflags-y += -Wmissing-declarations
ccflags-y += -Wno-unused-parameter
# ccflags-y += -DDEBUG     # Uncomment to enable dev_dbg() output in dmesg
# ccflags-y += -Werror     # Uncomment for zero-warning CI gate
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
	@echo "  all      - Build rs485_modbus.ko (default)"
	@echo "  clean    - Remove build artefacts"
	@echo "  install  - Install .ko (root required, on-target only)"
	@echo ""
	@echo "Variables:"
	@echo "  KERNEL_SRC    = $(KERNEL_SRC)"
	@echo "  ARCH          = $(ARCH)"
	@echo "  CROSS_COMPILE = $(CROSS_COMPILE)"
.PHONY: all clean install help
