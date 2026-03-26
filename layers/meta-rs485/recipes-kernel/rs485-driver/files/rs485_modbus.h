/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * rs485_modbus.h — Sensor Hub Driver: Header
 *
 * Target sensor : EPCB ES-INTEGRATE-ODR-01
 *
 * Register map:
 *   Reg 500  Humidity             ×10  %RH        (u16, ÷10)
 *   Reg 501  Temperature          ×10  °C         (s16, ÷10, 2's complement <0)
 *   Reg 502  Noise                ×10  dB         — UNUSED/IGNORED
 *   Reg 503  PM2.5                raw  µg/m³      (u16)
 *   Reg 504  PM10                 raw  µg/m³      — UNUSED/IGNORED
 *   Reg 505  Atmospheric Pressure ×10  kPa        (u16, ÷10)
 *   Reg 506  Lux HIGH 16 bits                     (u16)
 *   Reg 507  Lux LOW  16 bits                     (u16)
 *            Lux = (reg506 << 16) | reg507        (u32, raw lux)
 *
 * Hardware : Raspberry Pi 4 (BCM2711), Linux 5.15 (Yocto Kirkstone)
 * UART     : uart1 / ttyS0 via Serdev
 *
 * Author: JamedDo&HaoDuy256 
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
#define DRIVER_VERSION  "4.0.0"
#define DT_COMPATIBLE   "mycompany,rs485-sensorhub"

/* -----------------------------------------------------------------------
 * Character Device Node
 * --------------------------------------------------------------------- */
#define DEVICE_NAME      "sensorhub"
#define CLASS_NAME       "sensorhub"
#define SENSORHUB_MINOR  0

/* -----------------------------------------------------------------------
 * UART defaults
 *
 * Datasheet section 4.2: factory default baud rate = 4800, 8N1.
 * Override via DT "current-speed" if the device was reconfigured.
 * --------------------------------------------------------------------- */
#define DEFAULT_BAUD_RATE   9600U

/* -----------------------------------------------------------------------
 * Modbus RTU constants
 * --------------------------------------------------------------------- */
#define MODBUS_CRC16_POLY            0xA001U
#define MODBUS_FC_READ_HOLDING_REGS  0x03U
#define MODBUS_MIN_FRAME_LEN         4U
#define MODBUS_MAX_FRAME_LEN         256U
#define MODBUS_FC_READ_INPUT_REGS    0x04U   
#define MODBUS_FC_WRITE_SINGLE_REG   0x06U   

/* Slave address — factory default 0x15 (adjustable via register 2000) */
#define MODBUS_SLAVE_ADDR   0x15U

/* Read all 8 sensor registers in one request: 500–507 */
#define MODBUS_REG_START    500U    /* 0x01F4 */
#define MODBUS_REG_COUNT    8U

/* -----------------------------------------------------------------------
 * RS485 / GPIO timing
 * --------------------------------------------------------------------- */
#define RS485_TX_ENABLE_DELAY_US    100UL   /* DE assert → UART TX start  */
#define RS485_TX_DONE_GUARD_US      200UL   /* UART drain → DE de-assert  */

/* -----------------------------------------------------------------------
 * Polling / kthread
 * --------------------------------------------------------------------- */
#define POLL_INTERVAL_MS        1000UL
#define KTHREAD_NAME            "sensorhub_poll"
#define RX_RESPONSE_TIMEOUT_MS   1500UL

/* -----------------------------------------------------------------------
 * sensor_data — Decoded Sensor Payload
 *
 * Fixed-point representation (no FP in kernel):
 *
 *   humidity    : 1/10 %RH   e.g.  658 →  65.8 %RH
 *   temperature : 1/10 °C    e.g. -101 → -10.1 °C  (signed s16 cast)
 *   pm25        : µg/m³      raw u16 register value
 *   pressure    : 1/10 kPa   e.g. 1013 → 101.3 kPa
 *   lux         : lux        32-bit: (reg506 << 16) | reg507
 *
 * Copied verbatim to user space via copy_to_user().
 * Size: 5 × 4 = 20 bytes; ARM64-aligned.
 *
 * ABI: field order and types are stable — do not reorder without
 * bumping DRIVER_VERSION and updating user-space readers.
 * --------------------------------------------------------------------- */
struct sensor_data {
    u32  humidity;      /* 1/10 %RH                                    */
    s32  temperature;   /* 1/10 °C — signed (2's complement for <0°C) */
    u32  pm25;          /* µg/m³ — raw                                 */
    u32  pressure;      /* 1/10 kPa                                    */
    u32  lux;           /* lux — 32-bit, (reg506<<16)|reg507           */
};

/* -----------------------------------------------------------------------
 * rx_frame — RX Byte Accumulation Buffer
 * --------------------------------------------------------------------- */
struct rx_frame {
    spinlock_t  lock;
    u8          buf[MODBUS_MAX_FRAME_LEN];
    size_t      len;        /* bytes accumulated so far */
    size_t      expected;   /* total expected frame length */
};

/* -----------------------------------------------------------------------
 * sensorhub_priv — Driver Private Context
 * --------------------------------------------------------------------- */
struct sensorhub_priv {
    /* Serdev / hardware */
    struct serdev_device   *serdev;
    struct gpio_desc       *de_gpio;    /* DE/RE direction — HIGH=TX LOW=RX */

    /* Sensor cache (written by kthread, read by cdev) */
    struct sensor_data      cache;
    struct mutex            cache_lock;

    /* RX pipeline */
    struct rx_frame         rx;
    struct completion       rx_done;

    /* Background kthread */
    struct task_struct     *poll_thread;
    atomic_t                running;

    /* Character device (/dev/sensorhub) */
    struct cdev             cdev;       /* embedded by value — container_of */
    dev_t                   devno;
    struct class           *cdev_class;
    struct device          *cdev_device;

    /* Diagnostics */
    unsigned long           poll_count;
    unsigned long           err_count;

    /* UART config from Device Tree */
    u32                     baud_rate;
};

/* -----------------------------------------------------------------------
 * Logging helpers
 * --------------------------------------------------------------------- */
/* rs485_modbus.c — has struct device* */
#define sh_info(dev, fmt, ...)  dev_info((dev),  fmt, ##__VA_ARGS__)
#define sh_warn(dev, fmt, ...)  dev_warn((dev),  fmt, ##__VA_ARGS__)
#define sh_err(dev,  fmt, ...)  dev_err((dev),   fmt, ##__VA_ARGS__)
#define sh_dbg(dev,  fmt, ...)  dev_dbg((dev),   fmt, ##__VA_ARGS__)

/* rs485_rtu.c — no device context */
#define SH_INFO(fmt, ...)  pr_info(DRIVER_NAME  ": " fmt, ##__VA_ARGS__)
#define SH_WARN(fmt, ...)  pr_warn(DRIVER_NAME  ": " fmt, ##__VA_ARGS__)
#define SH_ERR(fmt, ...)   pr_err(DRIVER_NAME   ": " fmt, ##__VA_ARGS__)
#define SH_DBG(fmt, ...)   pr_debug(DRIVER_NAME ": " fmt, ##__VA_ARGS__)

/* -----------------------------------------------------------------------
 * Compile-time assertions
 * --------------------------------------------------------------------- */
static_assert(sizeof(struct sensor_data) == 20,
              "sensor_data ABI changed (now 20 bytes, 5 fields) — update user-space consumers");

static_assert(sizeof(struct sensor_data) <= PAGE_SIZE,
              "sensor_data exceeds PAGE_SIZE");

#endif /* RS485_MODBUS_H */
