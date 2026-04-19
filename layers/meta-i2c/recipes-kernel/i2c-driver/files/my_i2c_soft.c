#include <linux/module.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/ioctl.h>
#include <linux/device.h>
#include <linux/slab.h>

#define DRIVER_NAME "my_i2c_soft"
#define CLASS_NAME  "i2c_soft_class"
#define I2C_DELAY   5 

/* IOCTL Commands */
#define I2C_IOCTL_MAGIC 'k'
#define SET_SDA_PIN    _IOW(I2C_IOCTL_MAGIC, 1, int)
#define SET_SCL_PIN    _IOW(I2C_IOCTL_MAGIC, 2, int)
#define SET_SLAVE_ADDR _IOW(I2C_IOCTL_MAGIC, 3, int)

static int current_sda = -1, current_scl = -1, slave_addr = 0x3C;
static int major;
static struct class*  i2c_class  = NULL;
static struct device* i2c_device = NULL;

/* GPIO Helper */
static int update_gpio(int *current_pin, int new_pin, const char *label) {
    if (*current_pin != -1) gpio_free(*current_pin);
    if (gpio_request(new_pin, label)) return -EBUSY;
    *current_pin = new_pin;
    gpio_direction_output(new_pin, 1);
    return 0;
}

/* --- I2C CORE LOGIC (Timing Optimized) --- */

static void i2c_start(void) {
    gpio_direction_output(current_sda, 1);
    gpio_set_value(current_scl, 1); udelay(I2C_DELAY);
    gpio_set_value(current_sda, 0); udelay(I2C_DELAY);
    gpio_set_value(current_scl, 0); udelay(I2C_DELAY);
}

static void i2c_stop(void) {
    gpio_direction_output(current_sda, 0); udelay(I2C_DELAY);
    gpio_set_value(current_scl, 1); udelay(I2C_DELAY);
    gpio_set_value(current_sda, 1); udelay(I2C_DELAY);
}

static int i2c_write_byte(unsigned char byte) {
    int i, ack;
    for (i = 7; i >= 0; i--) {
        gpio_set_value(current_scl, 0);
        udelay(I2C_DELAY / 2); // Wait for SCL to fall
        gpio_set_value(current_sda, (byte >> i) & 1);
        udelay(I2C_DELAY / 2); // Data setup (Fixes jagged strokes)
        gpio_set_value(current_scl, 1);
        udelay(I2C_DELAY);
    }
    // ACK Phase
    gpio_set_value(current_scl, 0);
    gpio_direction_input(current_sda);
    udelay(I2C_DELAY);
    gpio_set_value(current_scl, 1);
    udelay(I2C_DELAY);
    ack = gpio_get_value(current_sda);
    gpio_set_value(current_scl, 0);
    gpio_direction_output(current_sda, 1);
    udelay(I2C_DELAY);
    return ack;
}

static unsigned char i2c_read_byte(int send_ack) {
    int i;
    unsigned char byte = 0;
    gpio_direction_input(current_sda);
    for (i = 7; i >= 0; i--) {
        gpio_set_value(current_scl, 0); udelay(I2C_DELAY);
        gpio_set_value(current_scl, 1); udelay(I2C_DELAY);
        if (gpio_get_value(current_sda)) byte |= (1 << i);
    }
    gpio_set_value(current_scl, 0);
    gpio_direction_output(current_sda, send_ack ? 0 : 1);
    udelay(I2C_DELAY);
    gpio_set_value(current_scl, 1); udelay(I2C_DELAY);
    gpio_set_value(current_scl, 0); udelay(I2C_DELAY);
    gpio_direction_output(current_sda, 1);
    return byte;
}

/* --- File Operations --- */

static long my_ioctl(struct file *file, unsigned int cmd, unsigned long arg) {
    int val;
    if (get_user(val, (int __user *)arg)) return -EFAULT;
    switch(cmd) {
        case SET_SDA_PIN:    return update_gpio(&current_sda, val, "I2C_SDA");
        case SET_SCL_PIN:    return update_gpio(&current_scl, val, "I2C_SCL");
        case SET_SLAVE_ADDR: slave_addr = val; return 0;
        default: return -EINVAL;
    }
}

static ssize_t dev_write(struct file *file, const char __user *buf, size_t len, loff_t *off) {
    unsigned char *kbuf;
    int i, ret = 0;
    if (current_sda == -1 || current_scl == -1) return -ENODEV;
    kbuf = kmalloc(len, GFP_KERNEL);
    if (!kbuf) return -ENOMEM;
    if (copy_from_user(kbuf, buf, len)) { kfree(kbuf); return -EFAULT; }

    i2c_start();
    if (i2c_write_byte(slave_addr << 1)) { ret = -EIO; goto out; }
    for (i = 0; i < len; i++) i2c_write_byte(kbuf[i]);
    ret = len;
out:
    i2c_stop();
    kfree(kbuf);
    return ret;
}

static ssize_t dev_read(struct file *file, char __user *buf, size_t len, loff_t *off) {
    unsigned char *kbuf;
    int i;
    if (current_sda == -1 || current_scl == -1) return -ENODEV;
    kbuf = kmalloc(len, GFP_KERNEL);
    if (!kbuf) return -ENOMEM;

    i2c_start();
    if (i2c_write_byte((slave_addr << 1) | 1)) { i2c_stop(); kfree(kbuf); return -EIO; }
    for (i = 0; i < len; i++) kbuf[i] = i2c_read_byte(i < (len - 1));
    i2c_stop();

    if (copy_to_user(buf, kbuf, len)) { kfree(kbuf); return -EFAULT; }
    kfree(kbuf);
    return len;
}

static struct file_operations fops = {
    .unlocked_ioctl = my_ioctl,
    .write = dev_write,
    .read = dev_read,
    .owner = THIS_MODULE,
};

/* --- Module Init/Exit --- */

static int __init my_init(void) {
    major = register_chrdev(0, DRIVER_NAME, &fops);
    i2c_class = class_create(THIS_MODULE, CLASS_NAME);
    i2c_device = device_create(i2c_class, NULL, MKDEV(major, 0), NULL, DRIVER_NAME);
    pr_info("I2C_Soft: Driver ready with Fixed Timing & Read Support.\n");
    return 0;
}

static void __exit my_exit(void) {
    device_destroy(i2c_class, MKDEV(major, 0));
    class_unregister(i2c_class);
    class_destroy(i2c_class);
    unregister_chrdev(major, DRIVER_NAME);
    if (current_sda != -1) gpio_free(current_sda);
    if (current_scl != -1) gpio_free(current_scl);
}

module_init(my_init);
module_exit(my_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Jamesdo & duyhao");
MODULE_DESCRIPTION("Complete Soft I2C with Fix Timing for SSD1306");
