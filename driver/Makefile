# =============================================================================
# Makefile — RS485 Sensor Hub Kernel Module (Serdev / Raspberry Pi 4)
#
# Compatible with:
#   - Yocto Project (Kirkstone) kernel-module.bbclass
#   - Cross-compilation for BCM2711 (arm64 / aarch64)
#   - Direct out-of-tree build against an installed sysroot
#
# Yocto build variables (exported automatically by module.bbclass):
#   KERNEL_SRC    — Staging kernel build tree   (${STAGING_KERNEL_DIR})
#   CROSS_COMPILE — Toolchain prefix             (aarch64-poky-linux-)
#   ARCH          — Target architecture          (arm64)
#
# Manual build (outside Yocto):
#   export KERNEL_SRC=/path/to/linux-5.15 ARCH=arm64 \
#          CROSS_COMPILE=aarch64-linux-gnu-
#   make
#
# =============================================================================

# ---------------------------------------------------------------------------
# 1. Module composition
#    Two C files linked into a single .ko
# ---------------------------------------------------------------------------
MODULE_NAME := rs485_modbus

obj-m := $(MODULE_NAME).o

$(MODULE_NAME)-objs := \
    rs485_modbus.o \
    rs485_rtu.o

# ---------------------------------------------------------------------------
# 2. Compiler flags
#    -std=gnu11  : required for kernel C11 constructs (static_assert, etc.)
#    -Wall / -Wextra: catch all common issues
#    -Wno-unused-parameter: serdev callbacks have unused params by convention
#    Remove -DDEBUG for release builds (silences all sh_dbg / dev_dbg calls)
# ---------------------------------------------------------------------------
ccflags-y += -std=gnu11
ccflags-y += -Wall
ccflags-y += -Wextra
ccflags-y += -Wmissing-prototypes
ccflags-y += -Wmissing-declarations
ccflags-y += -Wno-unused-parameter
# ccflags-y += -DDEBUG     # Uncomment to enable dev_dbg() output in dmesg
# ccflags-y += -Werror     # Uncomment for zero-warning CI gate

# ---------------------------------------------------------------------------
# 3. Kernel build tree resolution
#    Yocto exports KERNEL_SRC; allow KERNEL_PATH as fallback alias.
#    If neither is set, fall back to the running kernel's build directory
#    (only useful for native x86 development/testing).
# ---------------------------------------------------------------------------
KERNEL_SRC ?= $(KERNEL_PATH)

ifeq ($(strip $(KERNEL_SRC)),)
    KERNEL_SRC := /lib/modules/$(shell uname -r)/build
endif

# ---------------------------------------------------------------------------
# 4. Architecture / cross-compile defaults
#    Yocto sets ARCH and CROSS_COMPILE in the environment.
#    Provide sensible defaults so a developer can also build manually.
# ---------------------------------------------------------------------------
ARCH          ?= arm64
CROSS_COMPILE ?= aarch64-linux-gnu-

# ---------------------------------------------------------------------------
# 5. Default target
# ---------------------------------------------------------------------------
all:
	$(MAKE) -C $(KERNEL_SRC) \
	        M=$(PWD)           \
	        ARCH=$(ARCH)       \
	        CROSS_COMPILE=$(CROSS_COMPILE) \
	        modules

# ---------------------------------------------------------------------------
# 6. Clean
# ---------------------------------------------------------------------------
clean:
	$(MAKE) -C $(KERNEL_SRC) \
	        M=$(PWD)           \
	        ARCH=$(ARCH)       \
	        CROSS_COMPILE=$(CROSS_COMPILE) \
	        clean

# ---------------------------------------------------------------------------
# 7. Install (for manual on-target testing via NFS or SSH)
#    Not invoked by Yocto's bbclass — do_install handles that.
# ---------------------------------------------------------------------------
install:
	$(MAKE) -C $(KERNEL_SRC) \
	        M=$(PWD)           \
	        ARCH=$(ARCH)       \
	        CROSS_COMPILE=$(CROSS_COMPILE) \
	        modules_install
	depmod -a

# ---------------------------------------------------------------------------
# 8. Help
# ---------------------------------------------------------------------------
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

# ---------------------------------------------------------------------------
# 9. Phony targets
# ---------------------------------------------------------------------------
.PHONY: all clean install help
