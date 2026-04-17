# Hướng dẫn build image Raspberry Pi 4 với dm-verity và SELinux

Tài liệu này mô tả quy trình build image cho Raspberry Pi 4 theo flow:

1. `bitbake core-image-minimal`
2. `wic create secure-rpi4 -e core-image-minimal`

Flow này phù hợp với trường hợp dùng `dm-verity`, vì file `.verity` cần được tạo ra trước khi dùng `wic` để đóng gói image hoàn chỉnh.

---

## 1. Môi trường

### Yêu cầu
- Yocto/Poky branch `kirkstone`
- Máy build Linux
- Raspberry Pi 4
- Thẻ SD để flash image

### Các layer đang dùng
Ví dụ:

- `meta`
- `meta-poky`
- `meta-yocto-bsp`
- `meta-openembedded/meta-oe`
- `meta-openembedded/meta-python`
- `meta-security`
- `meta-selinux`
- `meta-raspberrypi`
- `meta-secure-rpi`

---

## 2. Cấu hình `bblayers.conf`

Đảm bảo đã add đầy đủ các layer cần thiết, đặc biệt:

- `meta-oe`
- `meta-python`
- `meta-security`
- `meta-selinux`
- `meta-raspberrypi`
- `meta-secure-rpi`

Có thể add bằng lệnh:

```bash
bitbake-layers add-layer ~/Yocto/poky/meta-openembedded/meta-oe
bitbake-layers add-layer ~/Yocto/poky/meta-openembedded/meta-python
bitbake-layers add-layer ~/Yocto/poky/meta-security
bitbake-layers add-layer ~/Yocto/poky/meta-selinux
bitbake-layers add-layer ~/Yocto/poky/meta-raspberrypi
bitbake-layers add-layer ~/Yocto/poky/meta-secure-rpi
