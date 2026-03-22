/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * rs485_modbus.h — Sensor Hub Driver: Header (Serdev + Cdev / RPi4 Edition)
 *
 * v3.0.0 adds a Character Device Layer on top of the existing Serdev driver.
 * Changes vs v2.0.0:
 *   + New constants : DEVICE_NAME, CLASS_NAME, SENSORHUB_MINOR
 *   + New includes  : <linux/cdev.h>, <linux/device.h>
 *   + New fields in sensorhub_priv:
 *       struct cdev   cdev        — embedded character device (by value)
 *       dev_t         devno       — allocated major:minor pair
 *       struct class *cdev_class  — /sys/class/sensorhub
 *       struct device*cdev_device — /dev/sensorhub node
 *
 * Hardware target : Raspberry Pi 4 (BCM2711)
 * Kernel          : Linux 5.15 (Yocto Kirkstone)
 * UART framework  : Serdev  (serdev_device_driver)
 * Transceiver     : MAX485 / MAX3485 on GPIO 18 (DE+RE tied together)
 * Protocol        : Modbus RTU over RS485 (half-duplex)
 *
 * Author: Graduation Thesis — Embedded Operating Systems
 */

#ifndef RS485_MODBUS_H
#define RS485_MODBUS_H

#include <linux/types.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/completion.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/serdev.h>

/* -----------------------------------------------------------------------
 * Driver Identity
 * --------------------------------------------------------------------- */
#define DRIVER_NAME     "rs485-sensorhub"
#define DRIVER_VERSION  "3.0.0"

/** OF compatible string — must match the Device Tree node exactly. */
#define DT_COMPATIBLE   "mycompany,rs485-sensorhub"

/* -----------------------------------------------------------------------
 * Character Device Node
 * --------------------------------------------------------------------- */

/**
 * DEVICE_NAME - basename of the node created under /dev/
 *
 * After device_create() succeeds, user space can open /dev/sensorhub.
 */
#define DEVICE_NAME     "sensorhub"

/**
 * CLASS_NAME - sysfs class name visible under /sys/class/
 *
 * Creates /sys/class/sensorhub/ which udev/mdev uses to generate the
 * /dev/sensorhub node automatically.
 */
#define CLASS_NAME      "sensorhub"

/**
 * SENSORHUB_MINOR - the one minor number this driver allocates.
 *
 * alloc_chrdev_region() is called with baseminor=0 and count=1, so the
 * driver owns exactly MAJOR(devno):0.
 */
#define SENSORHUB_MINOR  0

/* -----------------------------------------------------------------------
 * UART / Serdev defaults (overridden by DT "current-speed" property)
 * --------------------------------------------------------------------- */
#define DEFAULT_BAUD_RATE   9600U

/* -----------------------------------------------------------------------
 * Modbus RTU constants
 * --------------------------------------------------------------------- */
#define MODBUS_CRC16_POLY           0xA001U

#define MODBUS_FC_READ_HOLDING_REGS 0x03
#define MODBUS_FC_READ_INPUT_REGS   0x04
#define MODBUS_FC_WRITE_SINGLE_REG  0x06
#define MODBUS_FC_WRITE_MULTI_REGS  0x10

#define MODBUS_MIN_FRAME_LEN        4U
#define MODBUS_MAX_FRAME_LEN        256U

#define MODBUS_SLAVE_ADDR           0x01
#define MODBUS_REG_START            0x0000
#define MODBUS_REG_COUNT            4U     /* temp, humidity, pressure, lux */

/* -----------------------------------------------------------------------
 * RS485 / GPIO timing
 * --------------------------------------------------------------------- */
/** Hold time (µs) after asserting DE before UART TX begins. */
#define RS485_TX_ENABLE_DELAY_US    100UL

/** Guard time (µs) after UART drains before de-asserting DE. */
#define RS485_TX_DONE_GUARD_US      200UL

/* -----------------------------------------------------------------------
 * Polling / kthread
 * --------------------------------------------------------------------- */
#define POLL_INTERVAL_MS            1000UL
#define KTHREAD_NAME                "sensorhub_poll"

/** Timeout (ms) the kthread waits for a complete Modbus response.
 *  Must be < POLL_INTERVAL_MS to avoid overlapping transactions. */
#define RX_RESPONSE_TIMEOUT_MS      500UL

/* -----------------------------------------------------------------------
 * sensor_data — Decoded Sensor Payload
 *
 * Fixed-point, no FP in kernel:
 *   temperature : 1/10 °C   (e.g. 253  = 25.3 °C; signed → negative OK)
 *   humidity    : 1/10 %RH  (e.g. 456  = 45.6 %RH)
 *   pressure    : 1/10 hPa  (e.g. 10132 = 1013.2 hPa)
 *   lux         : raw register value (lux)
 *
 * This exact struct is transferred to user space via copy_to_user().
 * Layout: 4 × 4 bytes = 16 bytes; naturally aligned on ARM64.
 *
 * Stability contract: field order and types must never change between
 * driver versions without a corresponding user-space ABI bump.
 * --------------------------------------------------------------------- */
struct sensor_data {
    s32  temperature;   /* 1/10 °C  — signed (sub-zero supported)  */
    u32  humidity;      /* 1/10 %RH                                 */
    u32  pressure;      /* 1/10 hPa                                 */
    u32  lux;           /* lux                                      */
};

/* -----------------------------------------------------------------------
 * rx_frame — RX Accumulation State
 *
 * Holds bytes being assembled by receive_buf() for one Modbus response.
 * Protected by a spinlock because receive_buf() (workqueue) and the
 * kthread (kernel thread) both touch this structure.
 * --------------------------------------------------------------------- */
struct rx_frame {
    spinlock_t  lock;
    u8          buf[MODBUS_MAX_FRAME_LEN];
    size_t      len;        /* bytes accumulated so far    */
    size_t      expected;   /* complete frame length       */
};

/* -----------------------------------------------------------------------
 * sensorhub_priv — Driver Private Context
 *
 * Allocated in probe() with devm_kzalloc() and stored via
 * serdev_device_set_drvdata().  The devm framework keeps it alive until
 * all device-managed resources are released, which means it outlives
 * remove() — important because cdev keeps an internal kobject reference
 * that must remain valid while any file descriptor is open.
 *
 * Field layout (grouped by subsystem for readability):
 *
 *   [Serdev / hardware]
 *     serdev        — serdev_device handle (borrowed, not owned)
 *     de_gpio       — DE/RE direction GPIO (devm-managed)
 *
 *   [Sensor cache] — written by kthread, read by cdev .read fop
 *     cache         — latest valid decoded sensor readings
 *     cache_lock    — mutex protecting cache
 *
 *   [RX pipeline]
 *     rx            — per-transaction byte accumulator
 *     rx_done       — completion signalled by receive_buf
 *
 *   [kthread]
 *     poll_thread   — task_struct handle
 *     running       — atomic stop flag
 *
 *   [Character device] ← NEW in v3.0.0
 *     cdev          — embedded struct cdev (by value, NOT a pointer)
 *     devno         — allocated major:minor
 *     cdev_class    — /sys/class/sensorhub
 *     cdev_device   — /dev/sensorhub device node
 *
 *   [Diagnostics]
 *     poll_count / err_count
 *
 *   [UART config]
 *     baud_rate     — from Device Tree "current-speed" property
 * --------------------------------------------------------------------- */
struct sensorhub_priv {
    /* ------------------------------------------------------------------ */
    /* Serdev / hardware                                                   */
    /* ------------------------------------------------------------------ */
    struct serdev_device   *serdev;
    struct gpio_desc       *de_gpio;    /* HIGH = TX, LOW = RX (idle)     */

    /* ------------------------------------------------------------------ */
    /* Sensor cache                                                        */
    /* ------------------------------------------------------------------ */
    struct sensor_data      cache;
    struct mutex            cache_lock; /* kthread writes / cdev reads    */

    /* ------------------------------------------------------------------ */
    /* RX pipeline                                                         */
    /* ------------------------------------------------------------------ */
    struct rx_frame         rx;
    struct completion       rx_done;

    /* ------------------------------------------------------------------ */
    /* Background kthread                                                  */
    /* ------------------------------------------------------------------ */
    struct task_struct     *poll_thread;
    atomic_t                running;    /* set to 0 to request stop       */

    /* ------------------------------------------------------------------ */
    /* Character Device Layer  (v3.0.0)                                   */
    /*                                                                     */
    /* cdev is stored BY VALUE so that container_of(inode->i_cdev, …)    */
    /* resolves correctly.  Never embed a pointer here.                    */
    /* ------------------------------------------------------------------ */
    struct cdev             cdev;       /* embedded by value              */
    dev_t                   devno;      /* major:minor allocated pair     */
    struct class           *cdev_class; /* /sys/class/sensorhub           */
    struct device          *cdev_device;/* /dev/sensorhub node            */

    /* ------------------------------------------------------------------ */
    /* Diagnostics                                                         */
    /* ------------------------------------------------------------------ */
    unsigned long           poll_count;
    unsigned long           err_count;

    /* ------------------------------------------------------------------ */
    /* UART configuration (from Device Tree)                               */
    /* ------------------------------------------------------------------ */
    u32                     baud_rate;
};

/* -----------------------------------------------------------------------
 * dev_*() logging wrappers
 *
 * Using dev_*() instead of pr_*() or printk() ties each log line to the
 * device path in dmesg (e.g. "rs485-sensorhub serial0-0: …"), making
 * multi-device systems easy to debug.
 * --------------------------------------------------------------------- */
/* Bộ 1: Dùng cho rs485_modbus.c (Có biến dev) */
#define sh_info(dev, fmt, ...)  dev_info((dev),  fmt, ##__VA_ARGS__)
#define sh_warn(dev, fmt, ...)  dev_warn((dev),  fmt, ##__VA_ARGS__)
#define sh_err(dev,  fmt, ...)  dev_err((dev),   fmt, ##__VA_ARGS__)
#define sh_dbg(dev,  fmt, ...)  dev_dbg((dev),   fmt, ##__VA_ARGS__)

/* Bộ 2: Dùng cho rs485_rtu.c (Không cần biến dev, in trực tiếp ra log kernel) */
#define SH_INFO(fmt, ...)  pr_info(DRIVER_NAME ": " fmt, ##__VA_ARGS__)
#define SH_WARN(fmt, ...)  pr_warn(DRIVER_NAME ": " fmt, ##__VA_ARGS__)
#define SH_ERR(fmt, ...)   pr_err(DRIVER_NAME ": " fmt, ##__VA_ARGS__)
#define SH_DBG(fmt, ...)   pr_debug(DRIVER_NAME ": " fmt, ##__VA_ARGS__)

/* -----------------------------------------------------------------------
 * Compile-time assertions
 * --------------------------------------------------------------------- */
static_assert(sizeof(struct sensor_data) <= PAGE_SIZE,
              "sensor_data exceeds PAGE_SIZE — review field sizes");

static_assert(sizeof(struct sensor_data) == 16,
              "sensor_data ABI changed — update user-space consumers");

#endif /* RS485_MODBUS_H */
