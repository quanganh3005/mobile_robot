#!/bin/bash

echo "=== Công cụ hỗ trợ tạo Udev Rules cho Lidar và ESP32 ==="
echo ""
echo "Bước 1: Rút TẤT CẢ các thiết bị USB ra khỏi Raspberry Pi."
read -p "Nhấn Enter khi bạn đã rút hết các thiết bị USB..."

echo ""
echo "Bước 2: Cắm CHỈ MỘT MÌNH thiết bị LiDAR vào Raspberry Pi."
read -p "Nhấn Enter khi bạn đã cắm LiDAR..."

# Tìm ttyUSB mới nhất
LIDAR_PORT=$(ls /dev/ttyUSB* 2>/dev/null | head -n 1)

if [ -z "$LIDAR_PORT" ]; then
    echo "Lỗi: Không tìm thấy thiết bị /dev/ttyUSB* nào. Đảm bảo LiDAR đã được cắm."
    exit 1
fi

echo "Đã tìm thấy LiDAR ở cổng $LIDAR_PORT"
LIDAR_VENDOR=$(udevadm info -a -n $LIDAR_PORT | grep -m 1 'ATTRS{idVendor}' | cut -d '"' -f 2)
LIDAR_PRODUCT=$(udevadm info -a -n $LIDAR_PORT | grep -m 1 'ATTRS{idProduct}' | cut -d '"' -f 2)
LIDAR_SERIAL=$(udevadm info -a -n $LIDAR_PORT | grep -m 1 'ATTRS{serial}' | cut -d '"' -f 2)

echo "Vendor ID: $LIDAR_VENDOR"
echo "Product ID: $LIDAR_PRODUCT"
echo "Serial: $LIDAR_SERIAL"

echo ""
echo "Bước 3: Rút LiDAR ra, và cắm CHỈ MỘT MÌNH thiết bị ESP32 vào Raspberry Pi."
read -p "Nhấn Enter khi bạn đã cắm ESP32..."

# Đợi 1 chút để hệ thống nhận diện
sleep 2

ESP32_PORT=$(ls /dev/ttyUSB* 2>/dev/null | head -n 1)

if [ -z "$ESP32_PORT" ]; then
    echo "Lỗi: Không tìm thấy thiết bị /dev/ttyUSB* nào. Đảm bảo ESP32 đã được cắm."
    exit 1
fi

echo "Đã tìm thấy ESP32 ở cổng $ESP32_PORT"
ESP32_VENDOR=$(udevadm info -a -n $ESP32_PORT | grep -m 1 'ATTRS{idVendor}' | cut -d '"' -f 2)
ESP32_PRODUCT=$(udevadm info -a -n $ESP32_PORT | grep -m 1 'ATTRS{idProduct}' | cut -d '"' -f 2)
ESP32_SERIAL=$(udevadm info -a -n $ESP32_PORT | grep -m 1 'ATTRS{serial}' | cut -d '"' -f 2)

echo "Vendor ID: $ESP32_VENDOR"
echo "Product ID: $ESP32_PRODUCT"
echo "Serial: $ESP32_SERIAL"

echo ""
echo "=== Đang tạo file udev rules... ==="
RULES_FILE="99-usb-serial.rules"

echo "# Lidar Rule" > $RULES_FILE
echo "KERNEL==\"ttyUSB*\", ATTRS{idVendor}==\"$LIDAR_VENDOR\", ATTRS{idProduct}==\"$LIDAR_PRODUCT\", ATTRS{serial}==\"$LIDAR_SERIAL\", MODE:=\"0666\", SYMLINK+=\"rplidar\"" >> $RULES_FILE

echo "# ESP32 Rule" >> $RULES_FILE
echo "KERNEL==\"ttyUSB*\", ATTRS{idVendor}==\"$ESP32_VENDOR\", ATTRS{idProduct}==\"$ESP32_PRODUCT\", ATTRS{serial}==\"$ESP32_SERIAL\", MODE:=\"0666\", SYMLINK+=\"esp32\"" >> $RULES_FILE

echo ""
echo "Đã tạo xong file: $RULES_FILE trong thư mục hiện tại."
echo "Nội dung file:"
cat $RULES_FILE
echo ""
echo "Chạy các lệnh sau để copy file vào hệ thống và kích hoạt:"
echo "sudo cp $RULES_FILE /etc/udev/rules.d/"
echo "sudo udevadm control --reload-rules"
echo "sudo udevadm trigger"
echo ""
echo "Sau khi làm xong, cắm cả 2 thiết bị vào và kiểm tra bằng lệnh: ls -l /dev/rplidar /dev/esp32"
