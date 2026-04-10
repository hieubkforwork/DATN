SUMMARY = "Python test application for custom I2C Bit-banging"
LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://${COREBASE}/meta/files/common-licenses/MIT;md5=0835ade698e0bcf8506ecda2f7b4f302"

SRC_URI = "file://test_i2c.py"

S = "${WORKDIR}"

# Script Python nên không cần biên dịch C/C++
do_compile[noexec] = "1"

do_install() {
    # Tạo thư mục /usr/bin/ trong RootFS
    install -d ${D}${bindir}
    
    # Copy file test_i2c.py vào /usr/bin/ và đổi tên cho gọn
    install -m 0755 ${S}/test_i2c.py ${D}${bindir}/test_i2c
}

# Khai báo ràng buộc: Muốn chạy file này thì hệ điều hành phải có cài Python3
RDEPENDS:${PN} += "python3-core python3-fcntl"