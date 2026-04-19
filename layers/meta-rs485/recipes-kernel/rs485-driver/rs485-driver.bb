SUMMARY = "RS485 Modbus Serdev Driver for RPi4"

LICENSE = "GPL-2.0-only"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/GPL-2.0-only;md5=801f80980d171dd6425610833a22dbe6"

COMPATIBLE_MACHINE = "^rpi$"

inherit module

SRC_URI = "file://rs485_modbus.c \
           file://rs485_modbus.h \
           file://rs485_rtu.c \
           file://rs485_rtu.h \
           file://rs485-pi4.dtbo \
           file://Makefile"

S = "${WORKDIR}"

# Auto load module
KERNEL_MODULE_AUTOLOAD += "rs485_sensor_mod"

# Không dùng default install
MODULES_INSTALL_TARGET = ""

# =========================================================================
# INSTALL
# =========================================================================
do_install() {
    # 1. Install kernel module
    install -d ${D}${nonarch_base_libdir}/modules/${KERNEL_VERSION}/extra
    install -m 0644 ${B}/rs485_sensor_mod.ko \
        ${D}${nonarch_base_libdir}/modules/${KERNEL_VERSION}/extra/

    # 2. sysctl config
    install -d ${D}${sysconfdir}
    echo "kernel.printk = 3 4 1 3" >> ${D}${sysconfdir}/sysctl.conf
}

# =========================================================
# INSTALL DTBO 
# =========================================================
do_install:append() {
    install -d ${D}/boot/overlays
    install -m 0644 ${S}/rs485-pi4.dtbo ${D}/boot/overlays/
}

FILES:${PN} += "/boot/overlays/rs485-pi4.dtbo"

# =========================================================================
# PACKAGE
# =========================================================================
FILES:${PN} += " \
    ${nonarch_base_libdir}/modules/${KERNEL_VERSION}/extra/rs485_sensor_mod.ko \
    ${sysconfdir}/sysctl.conf \
"