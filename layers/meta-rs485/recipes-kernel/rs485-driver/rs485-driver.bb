SUMMARY = "RS485 Modbus Serdev Driver for RPi4"
LICENSE = "GPL-2.0-only"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/GPL-2.0-only;md5=801f80980d171dd6425610833a22dbe6"

inherit module

SRC_URI = "file://rs485_modbus.c \
           file://rs485_modbus.h \
           file://rs485_rtu.c \
           file://rs485_rtu.h \
           file://rs485-pi4.dts \
           file://Makefile"

S = "${WORKDIR}"

# Tên module phải khớp với MODULE_NAME trong Makefile của Duy
KERNEL_MODULE_AUTOLOAD += "rs485_sensor_mod"

# Tắt lệnh install mặc định của Yocto để mình tự làm tay cho chắc
MODULES_INSTALL_TARGET = ""

do_install() {
    # 1. Cài đặt file Driver (.ko) vào đúng thư mục modules của Kernel
    install -d ${D}${nonarch_base_libdir}/modules/${KERNEL_VERSION}/extra
    install -m 0644 ${S}/rs485_sensor_mod.ko ${D}${nonarch_base_libdir}/modules/${KERNEL_VERSION}/extra/

    # 2. Cài đặt file cấu hình Device Tree (.dts) vào thư mục overlays
    install -d ${D}/boot/overlays
    install -m 0644 ${WORKDIR}/rs485-pi4.dts ${D}/boot/overlays/
}

# Khai báo các file sẽ xuất hiện trong Image cuối cùng
FILES:${PN} += "${nonarch_base_libdir}/modules/${KERNEL_VERSION}/extra/rs485_sensor_mod.ko"
FILES:${PN} += "/boot/overlays/rs485-pi4.dts"