#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/slab.h>

#define DEVICE_NAME "rs485_sensor"
#define CLASS_NAME  "rs485_class"

/* Cấu hình chân GPIO - Thay đổi số này theo sơ đồ thực tế của Board */
#define RS485_DIR_PIN 17 

/* Thông tin Sensor ES_INTEGRATE_ODR_01 */
#define SLAVE_ID 0x15
static unsigned char temp_req[] = {0x15, 0x03, 0x01, 0xF5, 0x00, 0x01, 0x96, 0xD0};

static int major_number;
static struct class* rs485_class  = NULL;
static struct device* rs485_device = NULL;

/* -------------------------------------------------------------------------
 * 3. ĐIỀU KHIỂN RS485 DIRECTION (DE / RE pin)
 * ------------------------------------------------------------------------- */
static void rs485_tx_enable(void) {
    gpio_set_value(RS485_DIR_PIN, 1); // Mức cao để TRUYỀN
    udelay(50); 
}

static void rs485_rx_enable(void) {
    gpio_set_value(RS485_DIR_PIN, 0); // Mức thấp để NHẬN
}

/* -------------------------------------------------------------------------
 * 2. GIAO TIẾP UART (Mockup cho ttyAMA0)
 * ------------------------------------------------------------------------- */
static int uart_send(unsigned char *cmd, int size) {
    rs485_tx_enable();
    
    // Trong thực tế Yocto, bạn sẽ gọi tty_write hoặc tương tác thanh ghi UART
    pr_info("RS485: Sending Modbus command (Size: %d)\n", size);
    
    // Giả lập thời gian truyền dữ liệu
    msleep(10); 
    
    rs485_rx_enable();
    return 0;
}

static int uart_receive(unsigned char *res, int size) {
    // Giả lập nhận dữ liệu từ UART buffer
    // Thực tế: Đọc từ FIFO của UART AMA0
    return size; 
}

/* -------------------------------------------------------------------------
 * 5. SENSOR AUTO-DETECT (Probe flow)
 * ------------------------------------------------------------------------- */
static int probe_sensor(void) {
    unsigned char response[7] = {0};
    int raw_val;

    pr_info("RS485: Starting Auto-detect Sensor ID 0x%02X...\n", SLAVE_ID);
    
    uart_send(temp_req, sizeof(temp_req));
    msleep(150); // delay(100) như code mẫu + buffer
    
    uart_receive(response, 7);

    // Kiểm tra Function Code 0x03 như code mẫu bạn gửi
    if (response[1] == 0x03) {
        raw_val = (response[3] << 8) | response[4];
        pr_info("RS485: Sensor detected! Initial Temp: %d.%d\n", raw_val/10, raw_val%10);
        return 0;
    }

    pr_warn("RS485: Sensor not found during probe.\n");
    return -ENODEV;
}

/* -------------------------------------------------------------------------
 * 4. CHARACTER DEVICE INTERFACE (/dev/rs485_sensor)
 * ------------------------------------------------------------------------- */
static int dev_open(struct inode *inodep, struct file *filep) {
    pr_info("RS485: Device opened by User-space\n");
    return 0;
}

static ssize_t dev_read(struct file *filep, char *buffer, size_t len, loff_t *offset) {
    // Khi App đọc /dev/rs485_sensor, trả về dữ liệu sensor dạng chuỗi
    char message[64];
    int size;
    
    // Giả lập lấy dữ liệu từ biến toàn cục (đã được cập nhật từ UART)
    size = snprintf(message, sizeof(message), "Sensor ID: 0x%02X Status: Active\n", SLAVE_ID);
    
    if (copy_to_user(buffer, message, size)) return -EFAULT;
    return size;
}

static ssize_t dev_write(struct file *filep, const char *buffer, size_t len, loff_t *offset) {
    unsigned char k_buf[32];
    if (len > 32) len = 32;

    if (copy_from_user(k_buf, buffer, len)) return -EFAULT;
    
    // Cho phép User-space gửi lệnh Modbus tùy chỉnh xuống sensor
    uart_send(k_buf, len);
    return len;
}

static struct file_operations fops = {
    .open = dev_open,
    .read = dev_read,
    .write = dev_write,
};

/* -------------------------------------------------------------------------
 * 1. KERNEL MODULE SKELETON (init / exit)
 * ------------------------------------------------------------------------- */
static int __init rs485_init(void) {
    int ret;
    pr_info("RS485: Module Loading...\n");

    // 1. Đăng ký Major
    major_number = register_chrdev(0, DEVICE_NAME, &fops);
    if (major_number < 0) return major_number;

    // 2. Tạo Class
    rs485_class = class_create(THIS_MODULE, CLASS_NAME);
    if (IS_ERR(rs485_class)) {
        unregister_chrdev(major_number, DEVICE_NAME);
        return PTR_ERR(rs485_class);
    }

    // 3. Tạo Device
    rs485_device = device_create(rs485_class, NULL, MKDEV(major_number, 0), NULL, DEVICE_NAME);
    if (IS_ERR(rs485_device)) {
        class_destroy(rs485_class);
        unregister_chrdev(major_number, DEVICE_NAME);
        return PTR_ERR(rs485_device);
    }

    // 4. Request GPIO (SỬA ĐOẠN NÀY)
    ret = gpio_request(RS485_DIR_PIN, "RS485_DIR");
    if (ret) {
        // Chỉ in thông báo, KHÔNG dọn dẹp, KHÔNG return lỗi
        pr_err("RS485: GPIO %d not available (QEMU), skipping GPIO config...\n", RS485_DIR_PIN);
    } else {
        // Chỉ cấu hình nếu request thành công
        gpio_direction_output(RS485_DIR_PIN, 0);
    }

    probe_sensor();
    
    return 0; 
}

static void __exit rs485_exit(void) {
    device_destroy(rs485_class, MKDEV(major_number, 0));
    class_unregister(rs485_class);
    class_destroy(rs485_class);
    unregister_chrdev(major_number, DEVICE_NAME);
    gpio_free(RS485_DIR_PIN);
    pr_info("RS485: Module Unloaded\n");
}

module_init(rs485_init);
module_exit(rs485_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("JamesDo");
MODULE_DESCRIPTION("RS485 Driver for Yocto Project");
