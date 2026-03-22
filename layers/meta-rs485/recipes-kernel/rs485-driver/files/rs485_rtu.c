// SPDX-License-Identifier: GPL-2.0-only
/*
 * rs485_rtu.c - Modbus RTU Protocol Layer Implementation
 *
 * Implements:
 *   - CRC16 computation using the Modbus reflected polynomial (0xA001)
 *   - RTU request frame construction  (FC03, FC04, FC06)
 *   - RTU response frame validation and register extraction
 *
 * All functions operate on plain byte arrays and have no I/O side effects;
 * they are fully unit-testable in isolation.
 *
 * Author: Graduation Thesis — Embedded Operating Systems
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/types.h>

#include "rs485_modbus.h"
#include "rs485_rtu.h"

/* =========================================================================
 * Section 1: CRC16 — Modbus Reflected Polynomial (0xA001)
 * =====================================================================  */

/**
 * modbus_crc16() - Compute CRC16 over a byte buffer
 *
 * The Modbus CRC16 uses an initial value of 0xFFFF and the reflected
 * generator polynomial 0xA001 (bit-reversal of 0x8005).
 *
 * For each byte:
 *   1. XOR the byte into the low byte of the running CRC.
 *   2. Shift the CRC right by 1 bit, eight times.
 *   3. If the shifted-out bit was 1, XOR the CRC with 0xA001.
 *
 * @data: Input buffer (must not be NULL).
 * @len:  Number of bytes to process.
 *
 * Return: 16-bit CRC in host byte order.
 */
u16 modbus_crc16(const u8 *data, size_t len)
{
    u16 crc = 0xFFFF;
    size_t i, b;

    if (unlikely(!data || len == 0))
        return crc;

    for (i = 0; i < len; i++) {
        crc ^= (u16)data[i];          /* XOR byte into low byte of CRC   */
        for (b = 0; b < 8; b++) {     /* process each bit                */
            if (crc & 0x0001) {
                crc >>= 1;
                crc  ^= MODBUS_CRC16_POLY;
            } else {
                crc >>= 1;
            }
        }
    }

    return crc;
}
EXPORT_SYMBOL_GPL(modbus_crc16);

/**
 * modbus_crc16_append() - Append two CRC bytes (LSB first) to a buffer
 */
size_t modbus_crc16_append(u8 *buf, size_t len)
{
    u16 crc;

    if (unlikely(!buf || len == 0))
        return 0;

    crc = modbus_crc16(buf, len);
    buf[len]     = (u8)(crc & 0x00FF);   /* CRC low byte  */
    buf[len + 1] = (u8)(crc >> 8);       /* CRC high byte */

    return len + 2;
}
EXPORT_SYMBOL_GPL(modbus_crc16_append);

/**
 * modbus_crc16_verify() - Validate trailing CRC bytes of a complete frame
 */
bool modbus_crc16_verify(const u8 *buf, size_t len)
{
    u16 computed, received;

    /* Need at least 3 bytes: 1 data + 2 CRC */
    if (unlikely(!buf || len < MODBUS_MIN_FRAME_LEN))
        return false;

    computed = modbus_crc16(buf, len - 2);

    /* CRC is stored LSB-first in the frame */
    received = (u16)buf[len - 2] | ((u16)buf[len - 1] << 8);

    SH_DBG("CRC verify: computed=0x%04X received=0x%04X\n",
           computed, received);

    return (computed == received);
}
EXPORT_SYMBOL_GPL(modbus_crc16_verify);

/* =========================================================================
 * Section 2: RTU Request Frame Construction
 * =====================================================================  */

/**
 * rtu_build_read_regs_request() - Build an 8-byte Read Registers PDU
 *
 * Frame layout (before CRC):
 *   Byte 0: slave_addr
 *   Byte 1: func_code  (0x03 or 0x04)
 *   Byte 2: start_reg  high byte
 *   Byte 3: start_reg  low byte
 *   Byte 4: reg_count  high byte
 *   Byte 5: reg_count  low byte
 *   Byte 6: CRC low
 *   Byte 7: CRC high
 */
size_t rtu_build_read_regs_request(u8 *out_buf,
                                   u8   slave_addr,
                                   u8   func_code,
                                   u16  start_reg,
                                   u16  reg_count)
{
    size_t frame_len;

    if (unlikely(!out_buf)) {
        SH_ERR("%s: NULL output buffer\n", __func__);
        return 0;
    }

    if (func_code != MODBUS_FC_READ_HOLDING_REGS &&
        func_code != MODBUS_FC_READ_INPUT_REGS) {
        SH_ERR("%s: unsupported function code 0x%02X\n",
               __func__, func_code);
        return 0;
    }

    if (reg_count == 0 || reg_count > 125) {
        SH_ERR("%s: invalid register count %u (must be 1–125)\n",
               __func__, reg_count);
        return 0;
    }

    out_buf[0] = slave_addr;
    out_buf[1] = func_code;
    out_buf[2] = (u8)(start_reg >> 8);
    out_buf[3] = (u8)(start_reg & 0xFF);
    out_buf[4] = (u8)(reg_count >> 8);
    out_buf[5] = (u8)(reg_count & 0xFF);

    frame_len = modbus_crc16_append(out_buf, 6);

    SH_DBG("Built FC%02X request: slave=0x%02X reg=0x%04X count=%u "
           "frame=[%*ph]\n",
           func_code, slave_addr, start_reg, reg_count,
           (int)frame_len, out_buf);

    return frame_len;   /* always 8 */
}
EXPORT_SYMBOL_GPL(rtu_build_read_regs_request);

/**
 * rtu_build_write_single_reg_request() - Build an 8-byte FC06 request
 *
 * Frame layout (before CRC):
 *   Byte 0: slave_addr
 *   Byte 1: 0x06
 *   Byte 2: reg_addr high byte
 *   Byte 3: reg_addr low byte
 *   Byte 4: value    high byte
 *   Byte 5: value    low byte
 *   Byte 6: CRC low
 *   Byte 7: CRC high
 */
size_t rtu_build_write_single_reg_request(u8  *out_buf,
                                          u8   slave_addr,
                                          u16  reg_addr,
                                          u16  value)
{
    if (unlikely(!out_buf)) {
        SH_ERR("%s: NULL output buffer\n", __func__);
        return 0;
    }

    out_buf[0] = slave_addr;
    out_buf[1] = MODBUS_FC_WRITE_SINGLE_REG;
    out_buf[2] = (u8)(reg_addr >> 8);
    out_buf[3] = (u8)(reg_addr & 0xFF);
    out_buf[4] = (u8)(value >> 8);
    out_buf[5] = (u8)(value & 0xFF);

    return modbus_crc16_append(out_buf, 6);   /* always 8 */
}
EXPORT_SYMBOL_GPL(rtu_build_write_single_reg_request);

/* =========================================================================
 * Section 3: RTU Response Frame Decapsulation
 * =====================================================================  */

/**
 * rtu_is_exception_response() - Detect a Modbus exception frame
 *
 * An exception response sets bit 7 of the function code byte.
 * Frame: [slave][fc|0x80][exception_code][crc_lo][crc_hi]  — 5 bytes total.
 */
u8 rtu_is_exception_response(const u8 *buf, size_t buf_len)
{
    if (!buf || buf_len < 5)
        return 0;

    /* Bit 7 of function code byte set → exception */
    if (buf[1] & 0x80)
        return buf[2];   /* exception code */

    return 0;
}
EXPORT_SYMBOL_GPL(rtu_is_exception_response);

/**
 * rtu_parse_read_regs_response() - Validate and extract register values
 *
 * Expected frame structure for @expected_regs registers:
 *
 *  Offset  Field
 *  ------  -----
 *    0     Slave address
 *    1     Function code (FC03 or FC04)
 *    2     Byte count  = expected_regs * 2
 *    3..   Register data (big-endian pairs)
 *   -2     CRC low byte
 *   -1     CRC high byte
 *
 * The function first checks CRC, then validates the address / FC / byte
 * count, and finally copies the decoded u16 values into @reg_out.
 */
int rtu_parse_read_regs_response(const u8 *buf,
                                 size_t    buf_len,
                                 u8        expected_slave,
                                 u8        func_code,
                                 u16      *reg_out,
                                 u16       expected_regs)
{
    size_t expected_len;
    u8     byte_count;
    u16    i;
    u8     exc;

    /* ------------------------------------------------------------------ */
    /* 1. Basic pointer / length sanity                                    */
    /* ------------------------------------------------------------------ */
    if (unlikely(!buf || !reg_out)) {
        SH_ERR("%s: NULL pointer argument\n", __func__);
        return -EINVAL;
    }

    expected_len = rtu_expected_response_len(expected_regs);

    if (buf_len < MODBUS_MIN_FRAME_LEN) {
        SH_ERR("%s: response too short: got %zu, minimum %d\n",
               __func__, buf_len, MODBUS_MIN_FRAME_LEN);
        return -EINVAL;
    }

    /* ------------------------------------------------------------------ */
    /* 2. Check for Modbus exception before CRC (exception frames are 5 B)*/
    /* ------------------------------------------------------------------ */
    exc = rtu_is_exception_response(buf, buf_len);
    if (exc) {
        SH_WARN("%s: slave 0x%02X returned exception code 0x%02X\n",
                __func__, buf[0], exc);
        return -EPROTO;
    }

    /* ------------------------------------------------------------------ */
    /* 3. CRC verification                                                 */
    /* ------------------------------------------------------------------ */
    if (!modbus_crc16_verify(buf, buf_len)) {
        SH_ERR("%s: CRC16 mismatch on response (len=%zu)\n",
               __func__, buf_len);
        return -EBADMSG;
    }

    /* ------------------------------------------------------------------ */
    /* 4. Validate frame length matches expectation                        */
    /* ------------------------------------------------------------------ */
    if (buf_len != expected_len) {
        SH_ERR("%s: unexpected response length: got %zu, expected %zu\n",
               __func__, buf_len, expected_len);
        return -EMSGSIZE;
    }

    /* ------------------------------------------------------------------ */
    /* 5. Validate slave address                                           */
    /* ------------------------------------------------------------------ */
    if (buf[0] != expected_slave) {
        SH_ERR("%s: slave address mismatch: got 0x%02X, expected 0x%02X\n",
               __func__, buf[0], expected_slave);
        return -EPROTO;
    }

    /* ------------------------------------------------------------------ */
    /* 6. Validate function code                                           */
    /* ------------------------------------------------------------------ */
    if (buf[1] != func_code) {
        SH_ERR("%s: function code mismatch: got 0x%02X, expected 0x%02X\n",
               __func__, buf[1], func_code);
        return -EPROTO;
    }

    /* ------------------------------------------------------------------ */
    /* 7. Validate byte count field                                        */
    /* ------------------------------------------------------------------ */
    byte_count = buf[2];
    if (byte_count != (u8)(expected_regs * 2)) {
        SH_ERR("%s: byte_count=%u inconsistent with expected_regs=%u\n",
               __func__, byte_count, expected_regs);
        return -EMSGSIZE;
    }

    /* ------------------------------------------------------------------ */
    /* 8. Extract register values (big-endian → host order)               */
    /* ------------------------------------------------------------------ */
    for (i = 0; i < expected_regs; i++) {
        size_t offset = 3U + (size_t)i * 2U;
        reg_out[i] = ((u16)buf[offset] << 8) | (u16)buf[offset + 1];
        SH_DBG("%s: reg[%u] = 0x%04X (%u)\n",
               __func__, i, reg_out[i], reg_out[i]);
    }

    SH_INFO("%s: parsed %u register(s) from slave 0x%02X OK\n",
            __func__, expected_regs, expected_slave);

    return 0;
}
EXPORT_SYMBOL_GPL(rtu_parse_read_regs_response);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Embedded OS Thesis");
MODULE_DESCRIPTION("Modbus RTU Frame Encode/Decode Library");
MODULE_VERSION(DRIVER_VERSION);
