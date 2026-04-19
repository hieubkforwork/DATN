SUMMARY = "Dynamic Software I2C Bit-banging Driver for RPi4"
LICENSE = "GPL-2.0-only"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/GPL-2.0-only;md5=801f80980d171dd6425610833a22dbe6"

inherit module

# CHỈ CÓ 2 FILE: Code C và Makefile. Không cần file .h hay .dts nữa!
SRC_URI = "file://my_i2c_soft.c \
           file://Makefile"

S = "${WORKDIR}"

# Tên module để Linux tự động load khi khởi động
KERNEL_MODULE_AUTOLOAD += "my_i2c_soft"

# Tắt lệnh install mặc định để tự copy bằng tay
MODULES_INSTALL_TARGET = ""

do_install() {
    # 1. Cài đặt file Driver (.ko) vào đúng thư mục modules của Kernel
    install -d ${D}${nonarch_base_libdir}/modules/${KERNEL_VERSION}/extra
    install -m 0644 ${S}/my_i2c_soft.ko ${D}${nonarch_base_libdir}/modules/${KERNEL_VERSION}/extra/
    
    # ĐÃ XÓA PHẦN COPY DEVICE TREE (.dts) VÌ KHÔNG CÒN DÙNG NỮA
}

# Khai báo file .ko sẽ xuất hiện trong Image cuối cùng
FILES:${PN} += "${nonarch_base_libdir}/modules/${KERNEL_VERSION}/extra/my_i2c_soft.ko"