#!/bin/sh

echo "--- He thong ket noi WiFi tu dong ---"

# 1. Bat card mang
ip link set wlan0 up
echo "[+] Da kich hoat wlan0"

# 2. Quet cac mang xung quanh
echo "[*] Dang tim kiem WiFi xung quanh..."
iw dev wlan0 scan | grep SSID | awk '{print $2}' | sort -u

echo "--------------------------------------"
read -p "Nhap SSID (Ten WiFi): " ssid
read -s -p "Nhap Password: " pass
echo ""

# 3. Tao file cau hinh
wpa_passphrase "$ssid" "$pass" > /etc/wpa_supplicant.conf

# 4. Ket noi (Kill cac tien trinh cu de tranh xung dot)
killall wpa_supplicant 2>/dev/null
wpa_supplicant -B -i wlan0 -c /etc/wpa_supplicant.conf

# 5. Cap IP
echo "[*] Dang xin cap dia chi IP..."
dhcpcd -n wlan0

# 6. Kiem tra ket qua
echo "--------------------------------------"
ifconfig wlan0 | grep "inet "
echo "Ket noi hoan tat!"
