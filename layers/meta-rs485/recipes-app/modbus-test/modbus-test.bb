SUMMARY = "User-space application to test RS485 Modbus driver"
LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/MIT;md5=0835ade698e0bcf8506ecda2f7b4f302"

SRC_URI = "file://modbus-test.c"

S = "${WORKDIR}"

do_compile() {
    ${CC} ${CFLAGS} ${LDFLAGS} modbus-test.c -o modbus-test
}

do_install() {
    install -d ${D}${bindir}
    install -m 0755 modbus-test ${D}${bindir}
}
