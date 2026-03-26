#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/serdev.h>
#include <linux/gpio/consumer.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/kthread.h>
#include <linux/completion.h>
#include <linux/of.h>
#include <linux/property.h>

#include "rs485_modbus.h"
#include "rs485_rtu.h"
/* =========================================================================
 * Forward declarations
 * =====================================================================  */

/* cdev lifecycle helpers (defined in Section 5, used in Section 6) */
static int  sensorhub_cdev_create(struct sensorhub_priv *priv);
static void sensorhub_cdev_destroy(struct sensorhub_priv *priv);

/* =========================================================================
 * Section 1 — RS485 Half-Duplex Direction Control
 * =====================================================================  */

/**
 * rs485_set_tx_mode() - Assert DE/RE 
 *
 * Sets GPIO HIGH then waits RS485_TX_ENABLE_DELAY_US microseconds for the
 * transceiver output stage to stabilise before the UART begins shifting.
 *
 * @priv: Driver context.
 */
static void rs485_set_tx_mode(struct sensorhub_priv *priv)
{
    if (!IS_ERR_OR_NULL(priv->de_gpio))
        gpiod_set_value_cansleep(priv->de_gpio, 1);
    usleep_range(RS485_TX_ENABLE_DELAY_US,
                 RS485_TX_ENABLE_DELAY_US + 50UL);
    sh_dbg(&priv->serdev->dev, "RS485: TX mode (DE=HIGH)\n");
}

/**
 * rs485_set_rx_mode() - Drain UART then de-assert DE/RE → receive mode
 *
 * serdev_device_wait_until_sent() blocks until the UART hardware shift
 * register is fully empty (kernel equivalent of POSIX tcdrain()).
 * A guard delay is then applied before the GPIO is pulled LOW so that
 * the last stop bit is not clipped by the transceiver switching too early.
 *
 * @priv:       Driver context.
 * @timeout_ms: Upper bound for the drain wait.  Non-zero to avoid
 *              stalling the kthread forever if the UART locks up.
 */
static void rs485_set_rx_mode(struct sensorhub_priv *priv,
                               unsigned int           timeout_ms)
{
    serdev_device_wait_until_sent(priv->serdev, timeout_ms);
    usleep_range(RS485_TX_DONE_GUARD_US,
                 RS485_TX_DONE_GUARD_US + 100UL);
    if (!IS_ERR_OR_NULL(priv->de_gpio))
        gpiod_set_value_cansleep(priv->de_gpio, 0);
    sh_dbg(&priv->serdev->dev, "RS485: RX mode (DE=LOW)\n");
}

/* =========================================================================
 * Section 2 — Serdev RX Callback
 * =====================================================================  */

/**
 * sensorhub_receive_buf() - Serdev byte-stream receive callback
 *
 * The serdev core calls this from a workqueue each time the UART DMA or
 * FIFO threshold fires. A single Modbus frame may arrive across multiple
 * calls (FIFO fragmentation), so bytes are accumulated in rx.buf.
 *
 * Three-phase accumulation logic:
 * Phase 0 (Start):  Wait for the Slave ID (0x15). Discards any preceding 
 * bytes (e.g., local echo from the transmit phase).
 * Phase 1 (Header): Collect the first 3 bytes [addr][FC][byte_count].
 * Derive rx.expected = 3 + byte_count + 2 (CRC).
 * Phase 2 (Body):   Accumulate until rx.len == rx.expected.
 * Signal kthread via complete().
 *
 * Return: @count. All bytes are consumed. Returning less would cause 
 * the core to re-deliver them, breaking Modbus frame alignment.
 */
static int sensorhub_receive_buf(struct serdev_device *serdev,
                                 const u8 *data, size_t count)
{
    struct sensorhub_priv *priv = serdev_device_get_drvdata(serdev);
    struct rx_frame *rx = &priv->rx;
    unsigned long flags;
    size_t i;
    bool frame_complete = false;

    /* Debug: log every chunk received from UART */
    sh_dbg(&serdev->dev, "UART RX: received %zu bytes\n", count);

    spin_lock_irqsave(&rx->lock, flags);

    for (i = 0; i < count; i++) {
        /* * Filter Echo/Trash: 
         * If buffer is empty, first byte MUST be the Slave ID (0x15).
         * This drops the sent command bytes that often leak back to RX.
         */
        if (rx->len == 0 && data[i] != 0x15) {
            continue; 
        }

        /* Prevent buffer overflow */
        if (rx->len >= MODBUS_MAX_FRAME_LEN) {
            rx->len = 0;
            rx->expected = 0;
            continue;
        }

        rx->buf[rx->len++] = data[i];
        
        /* Phase 1: Compute expected length once header (3 bytes) is received */
        if (rx->len == 3 && rx->expected == 0) {
            u8 fc = rx->buf[1];
            u8 byte_count = rx->buf[2];

            if (fc & 0x80) {
                /* Modbus Exception Frame: [Addr][FC|0x80][Error][CRC][CRC] = 5 bytes */
                rx->expected = 5;
            } else {
                /* Normal Response: [Addr][FC][ByteCount][Data...][CRC][CRC] */
                if (byte_count > 0 && byte_count <= 250) {
                    rx->expected = 3U + (size_t)byte_count + 2U;
                } else {
                    /* Invalid byte count: reset to find next valid frame start */
                    rx->len = 0;
                    rx->expected = 0;
                }
            }
        }

        /* Phase 2: Check if frame is complete */
        if (rx->expected > 0 && rx->len >= rx->expected) {
            frame_complete = true;
            break; 
        }
    }

    spin_unlock_irqrestore(&rx->lock, flags);

    if (frame_complete) {
        sh_info(&serdev->dev, "RX: frame complete (%zu bytes), waking up kthread\n", rx->len);
        complete(&priv->rx_done);
    }

    return (int)count;
}

/**
 * sensorhub_write_wakeup() - Serdev TX-available callback
 *
 * Called when the serdev TX FIFO drains after a partial write.
 * Our frames are only 8 bytes, so the FIFO should never fill; this
 * callback is provided for protocol correctness and debug visibility.
 *
 * @serdev: Serdev device handle.
 */
static void sensorhub_write_wakeup(struct serdev_device *serdev)
{
    struct sensorhub_priv *priv = serdev_device_get_drvdata(serdev);

    sh_dbg(&serdev->dev,
           "TX wakeup: FIFO available (poll_count=%lu)\n",
           priv->poll_count);
}

/** Serdev client operations registered in probe(). */
static const struct serdev_device_ops sensorhub_serdev_ops = {
    .receive_buf  = sensorhub_receive_buf,
    .write_wakeup = sensorhub_write_wakeup,
};

/* =========================================================================
 * Section 3 — Modbus Transmit Helper
 * =====================================================================  */

/**
 * sensorhub_send_request() - Transmit one Modbus RTU request frame
 *
 * Complete half-duplex TX sequence:
 *   1. Reset the RX accumulator and re-arm the completion.
 *   2. Assert DE (TX mode) + stabilisation delay.
 *   3. Write frame bytes via serdev_device_write().
 *   4. Wait for UART to drain (serdev_device_wait_until_sent).
 *   5. Guard delay then de-assert DE (RX mode).
 *
 * @priv: Driver context.
 * @buf:  Frame bytes to transmit (must not be NULL).
 * @len:  Frame length in bytes.
 *
 * Return: 0 on success, negative errno on error.
 */
static int sensorhub_send_request(struct sensorhub_priv *priv,
                                   const u8              *buf,
                                   size_t                 len)
{
    struct device *dev = &priv->serdev->dev;
    unsigned long  flags;
    int            written;

    /* ---- 1. Reset RX state for this new transaction ------------------- */
    spin_lock_irqsave(&priv->rx.lock, flags);
    memset(priv->rx.buf, 0, sizeof(priv->rx.buf));
    priv->rx.len      = 0;
    priv->rx.expected = 0;
    reinit_completion(&priv->rx_done);
    spin_unlock_irqrestore(&priv->rx.lock, flags);

    /* ---- 2 & 3. Assert DE, then write --------------------------------- */
    rs485_set_tx_mode(priv);

    written = serdev_device_write(priv->serdev, buf, len,
                                  msecs_to_jiffies(100));
    if (written < 0) {
        sh_err(dev, "serdev_device_write failed: %d\n", written);
        rs485_set_rx_mode(priv, 50);
        return written;
    }
    if ((size_t)written != len) {
        sh_warn(dev, "partial write: %d of %zu bytes sent\n",
                written, len);
    }

    sh_dbg(dev, "TX %zu bytes: [%*ph]\n", len, (int)len, buf);

    /* ---- 4 & 5. Drain UART, then release bus -------------------------- */
    rs485_set_rx_mode(priv, 200);

    return 0;
}

/* =========================================================================
 * Section 4 — Background Polling kthread
 * =====================================================================  */

/**
 * sensorhub_do_poll() - Execute one complete Modbus RTU poll cycle
 *
 * Steps:
 *   1. Build an FC03 Read Holding Registers request.
 *   2. Transmit via sensorhub_send_request().
 *   3. Block on rx_done completion (with timeout).
 *   4. Snapshot the RX buffer under spinlock.
 *   5. Parse and CRC-verify the response frame.
 *   6. Update priv->cache under cache_lock mutex.
 *
 * @priv: Driver context.
 *
 * Return: 0 on success, negative errno on any failure.
 */
static int sensorhub_do_poll(struct sensorhub_priv *priv)
{
    struct device  *dev = &priv->serdev->dev;
    u8              tx_buf[8];
    u8              rx_snap[MODBUS_MAX_FRAME_LEN];
    u16             regs[MODBUS_REG_COUNT];  /* 8 registers: 500-507 */
    size_t          tx_len, rx_snap_len;
    unsigned long   remaining, flags;
    int             ret;

    /* ---- 1. Build request --------------------------------------------- */
    tx_len = rtu_build_read_regs_request(tx_buf,
                                          MODBUS_SLAVE_ADDR,
                                          MODBUS_FC_READ_HOLDING_REGS,
                                          MODBUS_REG_START,
                                          MODBUS_REG_COUNT);
    if (!tx_len) {
        sh_err(dev, "poll: failed to build request frame\n");
        return -EINVAL;
    }

    /* ---- 2. Transmit -------------------------------------------------- */
    ret = sensorhub_send_request(priv, tx_buf, tx_len);
    if (ret) {
        sh_err(dev, "poll: TX error: %d\n", ret);
        return ret;
    }

    serdev_device_wait_until_sent(priv->serdev, msecs_to_jiffies(50));

    spin_lock_irqsave(&priv->rx.lock, flags);
    priv->rx.len = 0;
    priv->rx.expected = 0;
    reinit_completion(&priv->rx_done); 
    spin_unlock_irqrestore(&priv->rx.lock, flags);

    /* ---- 3. Wait for complete response -------------------------------- */
    remaining = wait_for_completion_timeout(
                    &priv->rx_done,
                    msecs_to_jiffies(RX_RESPONSE_TIMEOUT_MS));
    if (!remaining) {
        sh_warn(dev, "poll: response timeout after %lu ms\n",
                RX_RESPONSE_TIMEOUT_MS);
        priv->err_count++;
        return -ETIMEDOUT;
    }

    /* ---- 4. Snapshot RX buffer ---------------------------------------- */
    spin_lock_irqsave(&priv->rx.lock, flags);
    rx_snap_len = priv->rx.len;
    memcpy(rx_snap, priv->rx.buf,
           min(rx_snap_len, sizeof(rx_snap)));
    spin_unlock_irqrestore(&priv->rx.lock, flags);

    sh_dbg(dev, "poll: RX snapshot %zu bytes [%*ph]\n",
           rx_snap_len, (int)rx_snap_len, rx_snap);

    /* ---- 5. Parse + CRC verify ---------------------------------------- */
    memset(regs, 0, sizeof(regs));
    ret = rtu_parse_read_regs_response(rx_snap, rx_snap_len,
                                        MODBUS_SLAVE_ADDR,
                                        MODBUS_FC_READ_HOLDING_REGS,
                                        regs,
                                        MODBUS_REG_COUNT);
    if (ret) {
        sh_err(dev, "poll: parse error %d — frame discarded\n", ret);
        priv->err_count++;
        return ret;
    }

    /* ---- 6. Decode registers and update cache -------------------------
     * Register map:
     *   regs[0] reg500 = humidity    ×10 unsigned
     *   regs[1] reg501 = temperature ×10 signed 
     *   regs[2] reg502 = noise       — IGNORED
     *   regs[3] reg503 = PM2.5       actual µg/m³
     *   regs[4] reg504 = PM10        — IGNORED
     *   regs[5] reg505 = pressure    ×10 kPa
     *   regs[6] reg506 = lux HIGH 16 bits
     *   regs[7] reg507 = lux LOW  16 bits
     * ----------------------------------------------------------------- */
    mutex_lock(&priv->cache_lock);

    priv->cache.humidity    = (u32)regs[0];
    priv->cache.temperature = (s32)(s16)regs[1];  
    priv->cache.pm25        = (u32)regs[3];
    priv->cache.pressure    = (u32)regs[5];
    priv->cache.lux         = ((u32)regs[6] << 16) | (u32)regs[7];

    mutex_unlock(&priv->cache_lock);

    priv->poll_count++;

    sh_info(dev,
        "Poll #%lu - [Status: OK] "
        "Hum: %u.%u%%RH, Temp: %s%d.%u degC, "
        "PM2.5: %u ug/m3, Pres: %u.%ukPa, Lux: %u Lux\n",
        priv->poll_count,
        (u32)(priv->cache.humidity / 10), (u32)(priv->cache.humidity % 10),
        priv->cache.temperature < 0 ? "-" : "+",
        (int)(abs(priv->cache.temperature) / 10),
        (u32)(abs(priv->cache.temperature) % 10),
        (u32)priv->cache.pm25,
        (u32)(priv->cache.pressure / 10), (u32)(priv->cache.pressure % 10),
        (u32)priv->cache.lux);

    return 0;
}

/**
 * sensorhub_poll_fn() - kthread main loop
 *
 * Polls the sensor at POLL_INTERVAL_MS intervals.  Uses
 * msleep_interruptible() so kthread_stop() wakes it immediately
 * instead of waiting up to one full polling period.
 *
 * @data: sensorhub_priv pointer.
 * Return: 0 (always).
 */
static int sensorhub_poll_fn(void *data)
{
    struct sensorhub_priv *priv = (struct sensorhub_priv *)data;
    struct device         *dev  = &priv->serdev->dev;

    sh_info(dev, "kthread started (interval=%lu ms)\n", POLL_INTERVAL_MS);

    while (!kthread_should_stop() && atomic_read(&priv->running)) {
        sensorhub_do_poll(priv);
        msleep_interruptible(POLL_INTERVAL_MS);
    }

    sh_info(dev, "kthread exiting (polls=%lu errors=%lu)\n",
            priv->poll_count, priv->err_count);
    return 0;
}

/* =========================================================================
 * Section 5 — Character Device Layer
 *
 * Exposes /dev/sensorhub so user processes can read struct sensor_data
 * with a simple open()+read() pair, or write raw bytes for custom Modbus
 * commands.  No ioctl, no mmap.
 *
 * Concurrency model:
 *   The kthread holds cache_lock while writing priv->cache.
 *   sensorhub_cdev_read() holds cache_lock while copying to a local
 *   stack snapshot, then releases it before copy_to_user().  The lock
 *   is therefore never held while sleeping in copy_to_user(), avoiding
 *   potential priority inversion.
 *   sensorhub_cdev_write() does not hold cache_lock (no cache access).
 *
 * Lifetime:
 *   sensorhub_cdev_create() is called at the end of probe().
 *   sensorhub_cdev_destroy() is the FIRST action in remove(), ensuring
 *   no new file descriptors can be opened on a half-torn-down device.
 *   Existing open() descriptors remain valid because the kernel cdev
 *   kobject holds a reference to the module until the last fd is closed.
 * =====================================================================  */

/* -----------------------------------------------------------------------
 * 5a. File Operations
 * --------------------------------------------------------------------- */

/**
 * sensorhub_cdev_open() - Handle open() on /dev/sensorhub
 *
 * Recovers the sensorhub_priv pointer from the embedded cdev using
 * container_of(), then stores it in filp->private_data for O(1) access
 * in subsequent read() calls.
 *
 * @inode: Kernel inode for /dev/sensorhub.
 * @filp:  Opened file instance.
 *
 * Return: 0 (always succeeds — no per-open state to allocate).
 */
static int sensorhub_cdev_open(struct inode *inode, struct file *filp)
{
    /*
     * container_of() works because cdev is embedded BY VALUE inside
     * sensorhub_priv.  inode->i_cdev points to exactly that field.
     */
    struct sensorhub_priv *priv =
        container_of(inode->i_cdev, struct sensorhub_priv, cdev);

    filp->private_data = priv;

    sh_dbg(&priv->serdev->dev,
           "cdev: opened by PID %d (comm=\"%s\")\n",
           current->pid, current->comm);

    return 0;
}

/**
 * sensorhub_cdev_release() - Handle close() on /dev/sensorhub
 *
 * No per-open resources to release; provided for completeness and for
 * future extension (e.g. per-fd subscription filters).
 *
 * @inode: Kernel inode (unused).
 * @filp:  File instance being closed.
 *
 * Return: 0 (always).
 */
static int sensorhub_cdev_release(struct inode *inode, struct file *filp)
{
    struct sensorhub_priv *priv = filp->private_data;

    sh_dbg(&priv->serdev->dev,
           "cdev: closed by PID %d\n", current->pid);

    return 0;
}

/**
 * sensorhub_cdev_read() - Non-blocking read of the latest sensor cache
 *
 * Returns one struct sensor_data to the caller.  The operation is
 * non-blocking: it always returns the most recently polled values from
 * the in-kernel cache without triggering a new Modbus transaction.
 *
 * Concurrency safety:
 *   1. cache_lock is acquired with mutex_lock_interruptible() so the
 *      process can be interrupted (e.g. by SIGINT) while waiting.
 *   2. The cache is memcpy'd into a local stack variable while the lock
 *      is held — the lock is released BEFORE copy_to_user() to keep
 *      the critical section as short as possible.
 *   3. copy_to_user() executes without any lock held, preventing the
 *      kthread from being blocked behind a page fault in user space.
 *
 * @filp:  Open file instance (private_data = sensorhub_priv *).
 * @ubuf:  User-space destination buffer.
 * @count: Bytes requested by the caller.
 * @ppos:  File position (device is not seekable; value is ignored).
 *
 * Return:
 *   sizeof(struct sensor_data)  on success
 *  -EINVAL   if @count < sizeof(struct sensor_data) (buffer too small)
 *  -ERESTARTSYS  if interrupted while waiting for cache_lock
 *  -EFAULT   if copy_to_user() fails (bad user pointer)
 */
static ssize_t sensorhub_cdev_read(struct file  *filp,
                                    char __user  *ubuf,
                                    size_t        count,
                                    loff_t       *ppos)
{
    struct sensorhub_priv *priv = filp->private_data;
    struct sensor_data     snapshot;

    /* ---- Validate buffer size ----------------------------------------- */
    if (count < sizeof(struct sensor_data)) {
        sh_warn(&priv->serdev->dev,
                "cdev: read() buffer too small: got %zu, need %zu\n",
                count, sizeof(struct sensor_data));
        return -EINVAL;
    }

    /* ---- Snapshot the cache under mutex (short critical section) ------- */
    if (mutex_lock_interruptible(&priv->cache_lock))
        return -ERESTARTSYS;

    memcpy(&snapshot, &priv->cache, sizeof(snapshot));

    mutex_unlock(&priv->cache_lock);

    sh_dbg(&priv->serdev->dev,
           "cdev: read() by PID %d — temp=%d.%u°C hum=%u.%u%%\n",
           current->pid,
           snapshot.temperature < 0 ? '-' : '+',
           abs(snapshot.temperature) / 10,
           (u32)(abs(snapshot.temperature) % 10),
           snapshot.humidity / 10,
           snapshot.humidity % 10);

    /* ---- Transfer to user space (no lock held) ------------------------ */
    if (copy_to_user(ubuf, &snapshot, sizeof(snapshot))) {
        sh_err(&priv->serdev->dev,
               "cdev: copy_to_user failed for PID %d\n",
               current->pid);
        return -EFAULT;
    }

    return (ssize_t)sizeof(struct sensor_data);
}

/**
 * sensorhub_cdev_write() - Debug helper: send raw bytes to UART
 *
 * Allows user space to push arbitrary bytes directly onto the RS485 bus
 * through the driver's GPIO direction-control sequence.  Intended for
 * hardware bring-up and protocol debugging — not for production use.
 *
 * The function performs the full half-duplex TX sequence:
 *   1. Validate buffer size (1 – MODBUS_MAX_FRAME_LEN bytes).
 *   2. Copy bytes from user space into a kernel-side buffer.
 *   3. Assert DE/RE GPIO HIGH.
 *   4. Write bytes via serdev_device_write().
 *   5. Drain UART, guard delay, de-assert DE/RE GPIO LOW (RX mode).
 *
 * @filp:  Open file instance (private_data = sensorhub_priv *).
 * @ubuf:  User-space source buffer containing bytes to transmit.
 * @count: Number of bytes to send (must be 1 – MODBUS_MAX_FRAME_LEN).
 * @ppos:  File position pointer (ignored — device is not seekable).
 *
 * Return:
 *   Number of bytes written on success.
 *  -EINVAL  if count is 0 or exceeds MODBUS_MAX_FRAME_LEN.
 *  -EFAULT  if copy_from_user() fails (bad user-space pointer).
 *  -EIO     if serdev_device_write() returns an error.
 */
static ssize_t sensorhub_cdev_write(struct file       *filp,
                                     const char __user *ubuf,
                                     size_t             count,
                                     loff_t            *ppos)
{
    struct sensorhub_priv *priv = filp->private_data;
    struct device         *dev  = &priv->serdev->dev;
    u8                     kbuf[MODBUS_MAX_FRAME_LEN];
    int                    written;

    if (count == 0 || count > MODBUS_MAX_FRAME_LEN) {
        sh_warn(dev, "cdev_write: invalid count %zu (max %u)\n",
                count, MODBUS_MAX_FRAME_LEN);
        return -EINVAL;
    }

    if (copy_from_user(kbuf, ubuf, count))
        return -EFAULT;

    sh_info(dev, "cdev_write: DEBUG TX %zu bytes: [%*ph]\n",
            count, (int)count, kbuf);

    rs485_set_tx_mode(priv);

    written = serdev_device_write(priv->serdev, kbuf, count,
                                  msecs_to_jiffies(200));

    rs485_set_rx_mode(priv, 200);

    if (written < 0) {
        sh_err(dev, "cdev_write: serdev_device_write failed: %d\n",
               written);
        return -EIO;
    }

    sh_info(dev, "cdev_write: sent %d of %zu bytes\n", written, count);
    return (ssize_t)written;
}

/**
 * sensorhub_cdev_llseek() - Reject seek attempts on /dev/sensorhub
 *
 * /dev/sensorhub is a streaming sensor interface; seeking makes no
 * semantic sense.  Returning -ESPIPE mirrors the behaviour of pipes and
 * tells callers (e.g. lseek()) the device is not seekable.
 */
static loff_t sensorhub_cdev_llseek(struct file *filp,
                                     loff_t       offset,
                                     int          whence)
{
    return -ESPIPE;
}

/** File operations registered with the kernel cdev subsystem. */
static const struct file_operations sensorhub_fops = {
    .owner   = THIS_MODULE,
    .open    = sensorhub_cdev_open,
    .release = sensorhub_cdev_release,
    .read    = sensorhub_cdev_read,
    .write   = sensorhub_cdev_write,
    .llseek  = sensorhub_cdev_llseek,
};

/* -----------------------------------------------------------------------
 * 5b. Character Device Lifecycle Helpers
 *
 * Factored out of probe()/remove() so each step has a clear name and
 * the error-unwind logic in probe() remains readable.
 * --------------------------------------------------------------------- */

/**
 * sensorhub_cdev_create() - Register cdev and create /dev/sensorhub
 *
 * Steps:
 *   1. alloc_chrdev_region() — dynamic major, one minor.
 *   2. cdev_init() + cdev_add() — register fops with the VFS.
 *   3. class_create() — /sys/class/sensorhub/ (used by udev/mdev).
 *   4. device_create() — triggers udev rule → creates /dev/sensorhub.
 *
 * On any failure the function unwinds all completed steps so that
 * sensorhub_cdev_destroy() does not need to be called on error paths.
 *
 * @priv: Driver context (serdev must already be bound).
 *
 * Return: 0 on success, negative errno on failure.
 */
static int sensorhub_cdev_create(struct sensorhub_priv *priv)
{
    struct device *serdev_dev = &priv->serdev->dev;
    int            ret;

    /* ---- Step 1: allocate device number ------------------------------- */
    ret = alloc_chrdev_region(&priv->devno,
                               SENSORHUB_MINOR, /* baseminor */
                               1,               /* count     */
                               DEVICE_NAME);
    if (ret < 0) {
        sh_err(serdev_dev,
               "cdev: alloc_chrdev_region failed: %d\n", ret);
        return ret;
    }

    sh_info(serdev_dev,
            "cdev: allocated major=%d minor=%d\n",
            MAJOR(priv->devno), MINOR(priv->devno));

    /* ---- Step 2: initialise and add the cdev -------------------------- */
    cdev_init(&priv->cdev, &sensorhub_fops);
    priv->cdev.owner = THIS_MODULE;

    ret = cdev_add(&priv->cdev, priv->devno, 1);
    if (ret) {
        sh_err(serdev_dev, "cdev: cdev_add failed: %d\n", ret);
        goto err_unregister_chrdev;
    }

    /* ---- Step 3: create sysfs class ----------------------------------- */
    priv->cdev_class = class_create(THIS_MODULE, CLASS_NAME);
    if (IS_ERR(priv->cdev_class)) {
        ret = PTR_ERR(priv->cdev_class);
        priv->cdev_class = NULL;
        sh_err(serdev_dev, "cdev: class_create failed: %d\n", ret);
        goto err_cdev_del;
    }

    /* ---- Step 4: create device node (/dev/sensorhub) ------------------ */
    priv->cdev_device = device_create(priv->cdev_class,
                                       serdev_dev,    /* parent */
                                       priv->devno,
                                       priv,          /* drvdata */
                                       DEVICE_NAME);
    if (IS_ERR(priv->cdev_device)) {
        ret = PTR_ERR(priv->cdev_device);
        priv->cdev_device = NULL;
        sh_err(serdev_dev, "cdev: device_create failed: %d\n", ret);
        goto err_class_destroy;
    }

    sh_info(serdev_dev,
            "cdev: /dev/%s created (major=%d minor=%d)\n",
            DEVICE_NAME, MAJOR(priv->devno), MINOR(priv->devno));
    return 0;

    /* ---- Error unwind (reverse of creation order) --------------------- */
err_class_destroy:
    class_destroy(priv->cdev_class);
    priv->cdev_class = NULL;
err_cdev_del:
    cdev_del(&priv->cdev);
err_unregister_chrdev:
    unregister_chrdev_region(priv->devno, 1);
    priv->devno = 0;
    return ret;
}

/**
 * sensorhub_cdev_destroy() - Remove /dev/sensorhub and unregister cdev
 *
 * Reverses sensorhub_cdev_create() in strict reverse order.
 * Safe to call even if creation was only partially completed (each step
 * checks for NULL / zero before acting).
 *
 * Called as the FIRST step in sensorhub_remove() to ensure no new
 * user-space file descriptors can be opened on a device being torn down.
 *
 * @priv: Driver context.
 */
static void sensorhub_cdev_destroy(struct sensorhub_priv *priv)
{
    struct device *serdev_dev = &priv->serdev->dev;

    /* Step 4 reverse: remove /dev/sensorhub */
    if (priv->cdev_device) {
        device_destroy(priv->cdev_class, priv->devno);
        priv->cdev_device = NULL;
        sh_dbg(serdev_dev, "cdev: device node removed\n");
    }

    /* Step 3 reverse: destroy sysfs class */
    if (priv->cdev_class) {
        class_destroy(priv->cdev_class);
        priv->cdev_class = NULL;
        sh_dbg(serdev_dev, "cdev: class destroyed\n");
    }

    /* Step 2 reverse: remove cdev from VFS */
    cdev_del(&priv->cdev);
    sh_dbg(serdev_dev, "cdev: cdev removed from VFS\n");

    /* Step 1 reverse: release the major:minor number */
    if (priv->devno) {
        unregister_chrdev_region(priv->devno, 1);
        priv->devno = 0;
        sh_dbg(serdev_dev, "cdev: device number released\n");
    }

    sh_info(serdev_dev, "cdev: /dev/%s removed\n", DEVICE_NAME);
}

/* =========================================================================
 * Section 6 — Device Tree Parsing
 * =====================================================================  */

/**
 * sensorhub_parse_dt() - Read hardware configuration from Device Tree
 *
 * Reads:
 *   "current-speed"  → priv->baud_rate  (optional; falls back to default)
 *   "rts-gpios"      → priv->de_gpio    (mandatory; probe fails if absent)
 *
 * The GPIO is requested as GPIOD_OUT_LOW (idle RX mode).
 * devm_gpiod_get() ensures the descriptor is released automatically
 * when the device is unbound, even if probe() fails afterward.
 *
 * @priv: Driver context (priv->serdev must be set before calling).
 *
 * Return: 0 on success, negative errno on failure.
 */
static int sensorhub_parse_dt(struct sensorhub_priv *priv)
{
    struct device      *dev = &priv->serdev->dev;
    struct device_node *np  = dev->of_node;

    if (!np) {
        sh_err(dev, "DT: no Device Tree node for this device\n");
        return -ENODEV;
    }

    /* Optional: baud rate */
    if (of_property_read_u32(np, "current-speed", &priv->baud_rate)) {
        sh_warn(dev, "DT: 'current-speed' missing — "
                "using default %u baud\n", DEFAULT_BAUD_RATE);
        priv->baud_rate = DEFAULT_BAUD_RATE;
    } else {
        sh_info(dev, "DT: baud rate = %u\n", priv->baud_rate);
    }

    /* Mandatory: RS485 DE/RE direction GPIO */
    
    priv->de_gpio = devm_gpiod_get(dev, "rts", GPIOD_OUT_LOW);
    if (IS_ERR(priv->de_gpio)) {
        long err = PTR_ERR(priv->de_gpio);
        priv->de_gpio = NULL;

        if (err == -ENOENT) {
            sh_info(dev, "DT: no rts-gpios — "
                    "assuming auto-direction RS485 module\n");
        } else {
            sh_err(dev, "DT: devm_gpiod_get('rts') failed: %ld\n", err);
            return (int)err;
        }
    }

    sh_info(dev, "DT: RS485 DE GPIO acquired (idle=LOW/RX)\n");
    return 0;
}

/* =========================================================================
 * Section 7 — Serdev UART Configuration
 * =====================================================================  */

/**
 * sensorhub_configure_uart() - Apply UART settings via Serdev API
 *
 * Configures the port to Modbus RTU requirements: N baud, 8 data, No
 * parity, 1 stop bit (8N1).  Must be called after serdev_device_open().
 *
 * @priv: Driver context (priv->baud_rate must be set before calling).
 *
 * Return: 0 on success, negative errno on failure.
 */
static int sensorhub_configure_uart(struct sensorhub_priv *priv)
{
    struct device *dev = &priv->serdev->dev;
    unsigned int   actual_baud;
    int            ret;

    /* Baud rate */
    actual_baud = serdev_device_set_baudrate(priv->serdev, priv->baud_rate);
    if (actual_baud != priv->baud_rate)
        sh_warn(dev, "UART: requested %u baud, got %u baud\n",
                priv->baud_rate, actual_baud);
    else
        sh_info(dev, "UART: baud rate = %u\n", actual_baud);

    /* Parity: None */
    ret = serdev_device_set_parity(priv->serdev, SERDEV_PARITY_NONE);
    if (ret) {
        sh_err(dev, "UART: set_parity(none) failed: %d\n", ret);
        return ret;
    }

    /* Disable hardware flow control (RS485 handles direction via GPIO) */
    serdev_device_set_flow_control(priv->serdev, false);

    sh_info(dev, "UART: configured %u 8N1 (no HW flow control)\n",
            actual_baud);
    return 0;
}

/* =========================================================================
 * Section 8 — Probe / Remove
 *
 * Probe initialisation order (each step is numbered in the code):
 *   1. Allocate sensorhub_priv (devm_kzalloc)
 *   2. Parse Device Tree
 *   3. Open serdev port + register client ops
 *   4. Configure UART (baud / parity / stop)
 *   5. Initialise synchronisation primitives
 *   6. Start kthread
 *   7. Register character device  ← NEW (Section 5)
 *
 * Remove teardown order (strict reverse):
 *   1. Destroy character device   ← NEW (must be FIRST)
 *   2. Stop kthread
 *   3. Idle DE GPIO
 *   4. Close serdev
 *   5. Destroy mutex
 *
 * Destroying the cdev FIRST ensures no new open() calls can arrive on a
 * device that is in the middle of tearing down its Modbus pipeline.
 * =====================================================================  */

/**
 * sensorhub_probe() - Bind driver to a matched serdev device
 *
 * @serdev: Serdev device found by the OF match table.
 *
 * Return: 0 on success, negative errno on any failure.
 *         All resources are released on failure (via goto unwind or devm).
 */
static int sensorhub_probe(struct serdev_device *serdev)
{
    struct device         *dev = &serdev->dev;
    struct sensorhub_priv *priv;
    int                    ret;

    sh_info(dev, "probe: " DRIVER_NAME " v" DRIVER_VERSION "\n");

    /* ---- 1. Allocate driver context ----------------------------------- */
    priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
    if (!priv) {
        sh_err(dev, "probe: out of memory\n");
        return -ENOMEM;
    }

    priv->serdev = serdev;
    serdev_device_set_drvdata(serdev, priv);

    /* ---- 2. Parse Device Tree ----------------------------------------- */
    ret = sensorhub_parse_dt(priv);
    if (ret) {
        sh_err(dev, "probe: Device Tree parsing failed: %d\n", ret);
        return ret;   /* devm_gpiod_get cleanup is automatic */
    }

    /* ---- 3. Open serdev port ------------------------------------------ */
    serdev_device_set_client_ops(serdev, &sensorhub_serdev_ops);

    ret = serdev_device_open(serdev);
    if (ret) {
        sh_err(dev, "probe: serdev_device_open failed: %d\n", ret);
        return ret;
    }

    /* ---- 4. Configure UART -------------------------------------------- */
    ret = sensorhub_configure_uart(priv);
    if (ret) {
        sh_err(dev, "probe: UART configuration failed: %d\n", ret);
        goto err_serdev_close;
    }

    /* ---- 5. Initialise synchronisation primitives --------------------- */
    mutex_init(&priv->cache_lock);
    spin_lock_init(&priv->rx.lock);
    init_completion(&priv->rx_done);
    atomic_set(&priv->running, 1);

    memset(&priv->cache, 0, sizeof(priv->cache));
    priv->rx.len      = 0;
    priv->rx.expected = 0;

    /* ---- 6. Register character device (/dev/sensorhub) ---------------- */
    ret = sensorhub_cdev_create(priv);
    if (ret) {
        sh_err(dev, "probe: cdev registration failed: %d\n", ret);
        goto err_kthread_stop;
    }

    /* ---- 7. Start background kthread ---------------------------------- */
    priv->poll_thread = kthread_run(sensorhub_poll_fn, priv, KTHREAD_NAME);
    if (IS_ERR(priv->poll_thread)) {
        ret = PTR_ERR(priv->poll_thread);
        priv->poll_thread = NULL;
        sh_err(dev, "probe: kthread_run failed: %d\n", ret);
        goto err_mutex_destroy;
    }

    sh_info(dev, "probe: kthread '%s' started\n", KTHREAD_NAME);

    sh_info(dev,
            "probe: driver bound — baud=%u gpio=%s cdev=/dev/%s\n",
            priv->baud_rate,
            priv->de_gpio ? "ok" : "n/a",
            DEVICE_NAME);

    return 0;

    /* ---- Error unwind (reverse of initialisation order) --------------- */
err_kthread_stop:
    atomic_set(&priv->running, 0);
    kthread_stop(priv->poll_thread);
    priv->poll_thread = NULL;
err_mutex_destroy:
    mutex_destroy(&priv->cache_lock);
err_serdev_close:
    serdev_device_close(serdev);
    return ret;
}

/**
 * sensorhub_remove() - Unbind driver from a serdev device
 *
 * Tears down all resources in strict reverse-probe order.
 * The character device is destroyed FIRST so that no new read() calls
 * can arrive while the kthread and Modbus pipeline are being shut down.
 *
 * @serdev: Serdev device being unbound.
 */
static void sensorhub_remove(struct serdev_device *serdev)
{
    struct sensorhub_priv *priv = serdev_device_get_drvdata(serdev);
    struct device         *dev  = &serdev->dev;

    sh_info(dev, "remove: shutting down "
            "(polls=%lu errors=%lu)\n",
            priv->poll_count, priv->err_count);

    /* ---- 1. Destroy cdev FIRST — no new open() after this point ------- */
    sensorhub_cdev_destroy(priv);

    /* ---- 2. Stop kthread ---------------------------------------------- */
    if (priv->poll_thread) {
        atomic_set(&priv->running, 0);
        kthread_stop(priv->poll_thread);
        priv->poll_thread = NULL;
        sh_info(dev, "remove: kthread stopped\n");
    }

    /* ---- 3. Leave DE GPIO in idle RX state (LOW) ---------------------- */
    if (!IS_ERR_OR_NULL(priv->de_gpio)) {
        gpiod_set_value_cansleep(priv->de_gpio, 0);
        sh_dbg(dev, "remove: DE GPIO set LOW (idle RX)\n");
        /* devm_gpiod_get → released automatically by device framework  */
    }

    /* ---- 4. Close the serdev UART port -------------------------------- */
    serdev_device_close(serdev);
    sh_info(dev, "remove: serdev port closed\n");

    /* ---- 5. Destroy mutex --------------------------------------------- */
    mutex_destroy(&priv->cache_lock);

    /* priv itself is freed by devm when the device reference drops       */
    sh_info(dev, "remove: " DRIVER_NAME " unbound\n");
}

/* =========================================================================
 * Section 9 — OF Match Table & Serdev Driver Registration
 * =====================================================================  */

static const struct of_device_id sensorhub_of_match[] = {
    { .compatible = DT_COMPATIBLE },
    { /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, sensorhub_of_match);

static struct serdev_device_driver sensorhub_driver = {
    .probe  = sensorhub_probe,
    .remove = sensorhub_remove,
    .driver = {
        .name           = DRIVER_NAME,
        .of_match_table = sensorhub_of_match,
    },
};

/* =========================================================================
 * Section 10 — Module Init / Exit
 * =====================================================================  */

static int __init sensorhub_init(void)
{
    int ret;

    pr_info(DRIVER_NAME ": loading v" DRIVER_VERSION
            " (serdev+cdev / RPi4 / Yocto Kirkstone)\n");

    ret = serdev_device_driver_register(&sensorhub_driver);
    if (ret)
        pr_err(DRIVER_NAME
               ": serdev_device_driver_register failed: %d\n", ret);
    else
        pr_info(DRIVER_NAME
                ": registered (compatible=%s device=/dev/%s)\n",
                DT_COMPATIBLE, DEVICE_NAME);

    return ret;
}

static void __exit sensorhub_exit(void)
{
    serdev_device_driver_unregister(&sensorhub_driver);
    pr_info(DRIVER_NAME ": unloaded\n");
}

module_init(sensorhub_init);
module_exit(sensorhub_exit); 

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Driver RS485 By JamesDo");
MODULE_DESCRIPTION("RS485/Modbus RTU Sensor Hub — Serdev + Cdev / RPi4");
MODULE_VERSION(DRIVER_VERSION);
MODULE_ALIAS("of:" DT_COMPATIBLE);
