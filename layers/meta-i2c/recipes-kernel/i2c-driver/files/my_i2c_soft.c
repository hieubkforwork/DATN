#include <linux/module.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/ioctl.h>

#define DRIVER_NAME "my_i2c_soft"
#define I2C_DELAY 5

// Định nghĩa mã IOCTL cho SDA và SCL
#define I2C_IOCTL_MAGIC 'k'
#define SET_SDA_PIN _IOW(I2C_IOCTL_MAGIC, 1, int)
#define SET_SCL_PIN _IOW(I2C_IOCTL_MAGIC, 2, int)

static int current_sda = -1;
static int current_scl = -1;
static int major;

// --- Helper: Giải phóng chân cũ và xin chân mới ---
static int update_gpio(int *current_pin, int new_pin, const char *label) {
    if (*current_pin != -1) gpio_free(*current_pin);
    if (gpio_request(new_pin, label)) return -EBUSY;
    *current_pin = new_pin;
    gpio_direction_output(new_pin, 1);
    return 0;
}

// --- Hàm xử lý lệnh từ User Space ---
static long my_ioctl(struct file *file, unsigned int cmd, unsigned long arg) {
    int pin_val;
    if (copy_from_user(&pin_val, (int __user *)arg, sizeof(pin_val))) return -EFAULT;

    switch(cmd) {
        case SET_SDA_PIN:
            if (update_gpio(&current_sda, pin_val, "I2C_SDA_DYN")) return -EBUSY;
            printk(KERN_INFO "I2C_Soft: SDA set to GPIO %d\n", pin_val);
            break;
        case SET_SCL_PIN:
            if (update_gpio(&current_scl, pin_val, "I2C_SCL_DYN")) return -EBUSY;
            printk(KERN_INFO "I2C_Soft: SCL set to GPIO %d\n", pin_val);
            break;
        default: return -EINVAL;
    }
    return 0;
}

// --- Logic I2C (Duy dùng current_sda và current_scl thay vì macro cố định) ---
static void i2c_start(void) {
    gpio_set_value(current_sda, 1); gpio_set_value(current_scl, 1); udelay(I2C_DELAY);
    gpio_set_value(current_sda, 0); udelay(I2C_DELAY);
    gpio_set_value(current_scl, 0); udelay(I2C_DELAY);
}

// (Tương tự cho i2c_stop và i2c_write_byte sử dụng current_sda/scl...)

static ssize_t dev_write(struct file *file, const char __user *buf, size_t len, loff_t *off) {
    if (current_sda == -1 || current_scl == -1) return -ENODEV; // Chưa cấu hình chân
    // Thực hiện logic gửi data như cũ...
    return len;
}

static struct file_operations fops = {
    .unlocked_ioctl = my_ioctl,
    .write = dev_write,
};

static int __init my_init(void) {
    major = register_chrdev(0, DRIVER_NAME, &fops);
    return 0;
}

static void __exit my_exit(void) {
    if (current_sda != -1) gpio_free(current_sda);
    if (current_scl != -1) gpio_free(current_scl);
    unregister_chrdev(major, DRIVER_NAME);
}

module_init(my_init); module_exit(my_exit);
MODULE_LICENSE("GPL");
