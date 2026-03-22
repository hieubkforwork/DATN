/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * rs485_rtu.h - Modbus RTU Protocol Layer: Function Prototypes
 *
 * Declares the public API for:
 *   - CRC16 calculation (Modbus polynomial)
 *   - RTU request frame construction
 *   - RTU response frame validation and decapsulation
 *
 * Author: Graduation Thesis — Embedded Operating Systems
 */

#ifndef RS485_RTU_H
#define RS485_RTU_H

#include <linux/types.h>
#include "rs485_modbus.h"

/* -----------------------------------------------------------------------
 * CRC16 (Modbus)
 * --------------------------------------------------------------------- */

/**
 * modbus_crc16() - Compute the Modbus CRC16 checksum
 *
 * Implements the standard Modbus CRC16 algorithm using the reflected
 * polynomial 0xA001.  The result is appended to RTU frames in
 * little-endian byte order (low byte first, high byte second).
 *
 * @data: Pointer to the data buffer over which to compute the CRC.
 *        Must not be NULL.
 * @len:  Number of bytes to include in the calculation.
 *
 * Return: 16-bit CRC value in host byte order.
 */
u16 modbus_crc16(const u8 *data, size_t len);

/**
 * modbus_crc16_append() - Compute and append CRC16 to a frame buffer
 *
 * Writes the two CRC bytes (low byte first) at @buf[@len] and
 * @buf[@len + 1].  The caller must ensure the buffer has at least
 * @len + 2 bytes allocated.
 *
 * @buf: Frame buffer; CRC bytes are written at the end.
 * @len: Number of data bytes already in @buf (excluding CRC).
 *
 * Return: Total frame length including the two CRC bytes (@len + 2).
 */
size_t modbus_crc16_append(u8 *buf, size_t len);

/**
 * modbus_crc16_verify() - Validate the CRC at the tail of a frame
 *
 * Recomputes the CRC over @buf[0..@len-3] and compares it against
 * the two trailing bytes in @buf[@len-2] and @buf[@len-1].
 *
 * @buf: Complete RTU frame including the trailing CRC bytes.
 * @len: Total frame length (data bytes + 2 CRC bytes).
 *
 * Return: true if the CRC matches, false otherwise.
 */
bool modbus_crc16_verify(const u8 *buf, size_t len);

/* -----------------------------------------------------------------------
 * RTU Request Frame Construction
 * --------------------------------------------------------------------- */

/**
 * rtu_build_read_regs_request() - Build a Modbus Read Registers request
 *
 * Constructs the 8-byte RTU ADU for function codes FC03 or FC04:
 *
 *   [slave_addr][func_code][reg_hi][reg_lo][count_hi][count_lo][crc_lo][crc_hi]
 *
 * @out_buf:    Output buffer; must be at least 8 bytes.
 * @slave_addr: Modbus slave address (1–247).
 * @func_code:  MODBUS_FC_READ_HOLDING_REGS (0x03) or
 *              MODBUS_FC_READ_INPUT_REGS   (0x04).
 * @start_reg:  Starting register address (0x0000–0xFFFF).
 * @reg_count:  Number of registers to read (1–125).
 *
 * Return: Length of the built frame (always 8 on success, 0 on error).
 */
size_t rtu_build_read_regs_request(u8 *out_buf,
                                   u8   slave_addr,
                                   u8   func_code,
                                   u16  start_reg,
                                   u16  reg_count);

/**
 * rtu_build_write_single_reg_request() - Build a Write Single Register request
 *
 * Constructs the 8-byte RTU ADU for FC06:
 *
 *   [slave_addr][0x06][reg_hi][reg_lo][val_hi][val_lo][crc_lo][crc_hi]
 *
 * @out_buf:    Output buffer; must be at least 8 bytes.
 * @slave_addr: Modbus slave address (1–247).
 * @reg_addr:   Register address to write.
 * @value:      16-bit value to write into the register.
 *
 * Return: Length of the built frame (always 8 on success, 0 on error).
 */
size_t rtu_build_write_single_reg_request(u8  *out_buf,
                                          u8   slave_addr,
                                          u16  reg_addr,
                                          u16  value);

/* -----------------------------------------------------------------------
 * RTU Response Frame Decapsulation
 * --------------------------------------------------------------------- */

/**
 * rtu_parse_read_regs_response() - Decode a Read Registers response frame
 *
 * Validates the frame structure, verifies the CRC, and extracts the
 * register values into @reg_out.
 *
 * Expected frame layout (for N registers):
 *
 *   [slave_addr][func_code][byte_count = N*2]
 *   [reg0_hi][reg0_lo] … [regN-1_hi][regN-1_lo]
 *   [crc_lo][crc_hi]
 *
 * @buf:          Raw response buffer received from the slave.
 * @buf_len:      Number of bytes actually received.
 * @expected_slave: Slave address that should appear in the response.
 * @func_code:    Expected function code (FC03 or FC04).
 * @reg_out:      Output array; must hold at least @expected_regs entries.
 * @expected_regs: Number of register values expected (1–125).
 *
 * Return:
 *   0           on success (CRC valid, all fields match)
 *  -EINVAL      if @buf is NULL or frame length is too short
 *  -EBADMSG     if CRC check fails
 *  -EPROTO      if slave address or function code does not match
 *  -EMSGSIZE    if byte_count in the frame is inconsistent
 */
int rtu_parse_read_regs_response(const u8 *buf,
                                 size_t    buf_len,
                                 u8        expected_slave,
                                 u8        func_code,
                                 u16      *reg_out,
                                 u16       expected_regs);

/**
 * rtu_is_exception_response() - Check whether a frame is a Modbus exception
 *
 * In Modbus RTU, an exception response sets bit 7 of the function code
 * (e.g., 0x83 for an exception to FC03).  The third byte contains the
 * exception code.
 *
 * @buf:     Response buffer (at least 5 bytes: addr+fc+exc+crc_lo+crc_hi).
 * @buf_len: Number of valid bytes in @buf.
 *
 * Return: The exception code byte (1–255) if this is an exception frame,
 *         or 0 if it is not an exception response.
 */
u8 rtu_is_exception_response(const u8 *buf, size_t buf_len);

/* -----------------------------------------------------------------------
 * Utility
 * --------------------------------------------------------------------- */

/**
 * rtu_expected_response_len() - Calculate expected response length
 *
 * For a Read Registers response the length is:
 *   1 (addr) + 1 (fc) + 1 (byte_count) + reg_count*2 (data) + 2 (CRC)
 *
 * @reg_count: Number of registers requested.
 *
 * Return: Expected total byte length of the response frame.
 */
static inline size_t rtu_expected_response_len(u16 reg_count)
{
    return 5U + (size_t)reg_count * 2U;
}

#endif /* RS485_RTU_H */
