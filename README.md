# XÂY DỰNG KERNEL MODULE RS485 TRÊN YOCTO PROJECT

Chi tiết cách thiết lập, biên dịch và kiểm tra Driver RS485 (Character Device) dưới dạng một Custom Layer trong hệ sinh thái Yocto Project.

---

## 1. Cấu trúc thư mục dự án (Directory Structure)

Để đảm bảo BitBake có thể tìm thấy các tệp tin, cấu trúc thư mục được thiết lập như sau tại `/workspace/poky/`:

```text
meta-rs485-driver/
├── conf/
│   └── layer.conf                 # Cấu hình quét recipe của Layer
├── recipes-kernel/
│   └── rs485-driver/
│       ├── files/                 # Thư mục chứa mã nguồn thô
│       │   ├── Makefile           # File điều khiển biên dịch
│       │   └── rs485_driver.c     # Mã nguồn C của Driver
│       └── rs485-driver_1.0.bb    # Công thức (Recipe) BitBake
```

---

## 2. Chi tiết các file thành phần

### 2.1. Cấu hình Layer (`conf/layer.conf`)
Thông báo cho BitBake biết sự hiện diện của Layer và thứ tự ưu tiên.
```bitbake
BBPATH .= ":${LAYERDIR}"
BBFILES += "${LAYERDIR}/recipes-*/*/*.bb \
            ${LAYERDIR}/recipes-*/*/*.bbappend"

BBFILE_COLLECTIONS += "meta-rs485-driver"
BBFILE_PATTERN_meta-rs485-driver = "^${LAYERDIR}/"
BBFILE_PRIORITY_meta-rs485-driver = "6"
LAYERSERIES_COMPAT_meta-rs485-driver = "kirkstone jammy"
```

### 2.2. File điều khiển biên dịch (`files/Makefile`)
*Lưu ý: Các dòng lệnh sau `all:`, `clean:`, `modules_install:` phải bắt đầu bằng 1 phím Tab.*
```makefile
obj-m := rs485_driver.o
SRC := $(shell pwd)

all:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) modules

modules_install:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) modules_install

clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) clean
```

### 2.3. Công thức BitBake (`rs485-driver_1.0.bb`)
```bitbake
SUMMARY = "Recipe for RS485 Kernel Module"
LICENSE = "GPLv2"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/GPL-2.0-only;md5=801f80980d171dd6425610833a22dbe6"

inherit module

SRC_URI = "file://rs485_driver.c \
           file://Makefile"

S = "${WORKDIR}"

# Tự động nạp driver khi boot máy
KERNEL_MODULE_AUTOLOAD += "rs485_driver"
```

---

## 3. Quy trình thực hiện (Workflow)

### Bước 1: Khởi tạo môi trường
```bash
cd /workspace/poky
source oe-init-build-env
```

### Bước 2: Đăng ký Layer
```bash
bitbake-layers add-layer ../meta-rs485-driver
```

### Bước 3: Cấu hình cài đặt vào Image
Thêm dòng sau vào cuối file `conf/local.conf`:
```bitbake
IMAGE_INSTALL:append = " rs485-driver"
```

### Bước 4: Biên dịch và đóng gói
```bash
bitbake -c cleanall rs485-driver
bitbake rs485-driver
bitbake core-image-minimal
```

---

## 4. Kiểm tra và Xác nhận (Verification)

### 4.1. Khởi chạy QEMU
```bash
runqemu qemux86-64 nographic slirp
```

### 4.2. Kiểm tra Runtime (User: root)
1. **Kiểm tra nạp module:**
   `lsmod | grep rs485` -> Phải hiển thị `rs485_driver`.
2. **Kiểm tra file thiết bị:**
   `ls -l /dev/rs485_sensor` -> Phải thấy file `crw-------`.
3. **Đọc dữ liệu từ sensor:**
   `cat /dev/rs485_sensor` -> Kết quả: `Sensor ID: 0x15 Status: Active`.
4. **Xem Log Kernel:**
   `dmesg | grep RS485`
