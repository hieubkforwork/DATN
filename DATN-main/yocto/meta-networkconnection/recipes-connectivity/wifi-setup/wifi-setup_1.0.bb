SUMMARY = "Script tu dong ket noi WiFi cho DATN"
LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/MIT;md5=0835ade698e0bcf8506ecda2f7b4f302"

# Khai bao file nguon
SRC_URI = "file://wifi-setup.sh"

S = "${WORKDIR}"

# Quy trinh cai dat vao RootFS
do_install() {
    # Tao thu muc /usr/bin tren Pi
    install -d ${D}${bindir}
    # Copy script vao va cap quyen thuc thi
    install -m 0755 wifi-setup.sh ${D}${bindir}/wifi-setup
}

FILES:${PN} += "${bindir}/wifi-setup"
