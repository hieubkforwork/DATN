SUMMARY = "Recipe for RS485 Kernel Module"
LICENSE = "GPLv2"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/GPL-2.0-only;md5=801f80980d171dd6425610833a22dbe6"

inherit module

SRC_URI = "file://rs485_driver.c \
           file://Makefile"

S = "${WORKDIR}"

# Tự động nạp driver khi boot máy ảo
KERNEL_MODULE_AUTOLOAD += "rs485_driver"
