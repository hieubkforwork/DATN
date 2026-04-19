#!/usr/bin/env python3
import fcntl
import struct
import os
import time

# --- MÃ LỆNH IOCTL TỪ KERNEL DRIVER ---
SET_SDA_PIN = 0x40046b01
SET_SCL_PIN = 0x40046b02

# --- CẤU HÌNH LCD PCF8574 ---
LCD_ADDR = 0x3c      # Dùng đúng 0x21, Kernel đã tự dịch bit rồi!
LCD_CHR = 1          # Chế độ gửi Data (Ký tự)
LCD_CMD = 0          # Chế độ gửi Lệnh (Clear, Xuống dòng...)
LCD_BACKLIGHT = 0x08 # Bật đèn nền (0x00 là tắt)
ENABLE = 0b00000100  # Chân Enable của LCD

def setup_i2c_driver(sda, scl):
    fd = os.open("/dev/my_i2c_soft", os.O_RDWR)
    fcntl.ioctl(fd, SET_SDA_PIN, struct.pack('i', sda))
    fcntl.ioctl(fd, SET_SCL_PIN, struct.pack('i', scl))
    print(f"[*] Đã cấu hình Driver I2C Mềm trên SDA={sda}, SCL={scl}")
    return fd

def write_i2c(fd, data):
    # Gửi mảng byte xuống Kernel (Byte 0: Địa chỉ, Byte 1: Dữ liệu)
    os.write(fd, bytes([LCD_ADDR, data]))

def lcd_toggle_enable(fd, bits):
    # Tạo xung HIGH -> LOW trên chân Enable
    time.sleep(0.0005)
    write_i2c(fd, (bits | ENABLE))
    time.sleep(0.0005)
    write_i2c(fd, (bits & ~ENABLE))
    time.sleep(0.0005)

def lcd_byte(fd, bits, mode):
    # Băm 1 byte (8-bit) thành 2 mảnh 4-bit (High & Low Nibble)
    bits_high = mode | (bits & 0xF0) | LCD_BACKLIGHT
    bits_low = mode | ((bits << 4) & 0xF0) | LCD_BACKLIGHT

    # Gửi nửa trên rồi tạo xung
    write_i2c(fd, bits_high)
    lcd_toggle_enable(fd, bits_high)
    
    # Gửi nửa dưới rồi tạo xung
    write_i2c(fd, bits_low)
    lcd_toggle_enable(fd, bits_low)

def lcd_init(fd):
    lcd_byte(fd, 0x33, LCD_CMD) # Lệnh khởi tạo
    lcd_byte(fd, 0x32, LCD_CMD) # Lệnh khởi tạo
    lcd_byte(fd, 0x06, LCD_CMD) # Tự động tăng con trỏ
    lcd_byte(fd, 0x0C, LCD_CMD) # Bật màn hình, tắt nhấp nháy
    lcd_byte(fd, 0x28, LCD_CMD) # Chế độ 4-bit, 2 dòng
    lcd_byte(fd, 0x01, LCD_CMD) # Xóa trắng màn hình
    time.sleep(0.005)

def lcd_string(fd, message, line):
    # Di chuyển con trỏ tới dòng mong muốn (0x80 = Dòng 1, 0xC0 = Dòng 2)
    message = message.ljust(16, " ")
    lcd_byte(fd, line, LCD_CMD)
    for i in range(16):
        lcd_byte(fd, ord(message[i]), LCD_CHR)

if __name__ == '__main__':
    try:
        # Gọi xuống Kernel để chọn chân GPIO làm I2C
        # (Duy đang cắm dây SDA, SCL vào chân nào thì thay số vào đây nhé)
        fd = setup_i2c_driver(sda=17, scl=27)
        
        print("[*] Đang khởi tạo LCD...")
        lcd_init(fd)
        
        # In chữ lên màn hình để thầy cô lác mắt
        lcd_string(fd, "HCMUT K22", 0x80)
        lcd_string(fd, "I2C Bit-Banging!", 0xC0)
        
        print("[*] Thành công! Hãy nhìn lên màn hình LCD.")
        os.close(fd)
        
    except Exception as e:
        print(f"[!] Lỗi: {e}")