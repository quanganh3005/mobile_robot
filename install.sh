#!/usr/bin/env bash
# ==============================================================================
# Script cài đặt tự động môi trường và gói phụ thuộc cho Linorobot2 Workspace
# Tham chiếu & nâng cấp từ: 
# https://github.com/linorobot/linorobot2/blob/humble/install_linorobot2.bash
# ==============================================================================

set -e

# Xác định thư mục workspace hiện tại (thư mục chứa file script này)
WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo "------------------------------------------------------------------"
echo " Linorobot2 Workspace Setup Script"
echo " Workspace: $WORKSPACE"
echo "------------------------------------------------------------------"

# Kiểm tra ROS_DISTRO
if [ -z "$ROS_DISTRO" ]; then
    if [ -f "/opt/ros/humble/setup.bash" ]; then
        export ROS_DISTRO="humble"
    elif [ -f "/opt/ros/jazzy/setup.bash" ]; then
        export ROS_DISTRO="jazzy"
    elif [ -f "/opt/ros/iron/setup.bash" ]; then
        export ROS_DISTRO="iron"
    else
        echo "Lỗi: Không tìm thấy cài đặt ROS 2 trong /opt/ros/."
        echo "Vui lòng cài đặt ROS 2 (khuyến nghị Humble) trước khi chạy script."
        exit 1
    fi
fi

# Source ROS 2 base
source "/opt/ros/$ROS_DISTRO/setup.bash"

# Các tham số cấu hình xe (mặc định lấy theo cấu hình hiện tại của workspace)
BASE=${1:-"2wd"}
LASER_SENSOR=${2:-"a1"}
DEPTH_SENSOR=${3:-""}
ARCH="$(uname -m)"

ROBOT_TYPE_ARRAY=(2wd 4wd mecanum)
DEPTH_SENSOR_ARRAY=(realsense zed zedm zed2 zed2i oakd oakdlite oakdpro)
LASER_SENSOR_ARRAY=(ydlidar xv11 ld06 ld19 stl27l a1 a2 a3 c1 s1 s2 s3 rplidar)
LASER_SENSOR_ARRAY+=("${DEPTH_SENSOR_ARRAY[@]}")

# Hiển thị thông tin tóm tắt
echo ""
echo "=========== THÔNG SỐ CÀI ĐẶT ============"
echo " ROS DISTRO   : $ROS_DISTRO"
echo " WORKSPACE    : $WORKSPACE"
echo " ROBOT TYPE   : $BASE"
echo " LASER SENSOR : $LASER_SENSOR"
echo " DEPTH SENSOR : ${DEPTH_SENSOR:-'(None)'}"
echo " KIẾN TRÚC CPU: $ARCH"
echo "========================================="
echo ""
echo "Script này sẽ thực hiện:"
echo " 1. Cài đặt các công cụ build (colcon, rosdep, vcstool, build-essential)."
echo " 2. Cài đặt các package ROS 2 cần thiết (Nav2, Slam Toolbox, EKF, Robot State Publisher...)."
echo " 3. Tải và cài đặt driver cảm biến LiDAR / Depth Camera (nếu chưa có trong src)."
echo " 4. Thiết lập micro-ROS setup & micro-ROS Agent (cho ESP32 / vi điều khiển)."
echo " 5. Chạy rosdep install để quét toàn bộ dependencies trong thư mục src/."
echo " 6. Cấu hình biến môi trường & source workspace vào ~/.bashrc."
echo " 7. Build toàn bộ workspace bằng 'colcon build --symlink-install'."
echo ""

read -p "Bạn có muốn tiếp tục? (y/N): " reply
if [[ "$reply" != "y" && "$reply" != "Y" ]]; then
    echo "Đã hủy tiến trình cài đặt."
    exit 0
fi

echo ""
echo ">>> BƯỚC 1: Cập nhật hệ thống & Cài đặt công cụ nền tảng..."
sudo apt-get update
sudo apt-get install -y \
    build-essential \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    git \
    wget \
    curl \
    udev

# Khởi tạo rosdep nếu chưa có
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    echo "Khởi tạo rosdep..."
    sudo rosdep init || true
fi
rosdep update

echo ""
echo ">>> BƯỚC 2: Cài đặt các gói ROS 2 cốt lõi cho Navigation, SLAM & Hardware..."
sudo apt-get install -y \
    ros-$ROS_DISTRO-navigation2 \
    ros-$ROS_DISTRO-nav2-bringup \
    ros-$ROS_DISTRO-slam-toolbox \
    ros-$ROS_DISTRO-robot-localization \
    ros-$ROS_DISTRO-robot-state-publisher \
    ros-$ROS_DISTRO-joint-state-publisher \
    ros-$ROS_DISTRO-xacro \
    ros-$ROS_DISTRO-teleop-twist-keyboard \
    ros-$ROS_DISTRO-teleop-twist-joy \
    ros-$ROS_DISTRO-joy \
    ros-$ROS_DISTRO-imu-tools \
    ros-$ROS_DISTRO-imu-filter-madgwick

# Cài thêm Gazebo bridge nếu là máy PC (x86_64) để hỗ trợ mô phỏng
if [[ "$ARCH" == "x86_64" ]]; then
    echo "Phát hiện máy tính x86_64: Cài đặt hỗ trợ mô phỏng Gazebo / Ignition..."
    sudo apt-get install -y \
        ros-$ROS_DISTRO-ros-gz \
        ros-$ROS_DISTRO-ros-gz-bridge \
        ros-$ROS_DISTRO-ros-gz-sim || true
fi

mkdir -p "$WORKSPACE/src"
cd "$WORKSPACE"

# Các hàm cài đặt driver cảm biến tương tự script gốc
function install_sllidar_ros2 {
    echo ">>> Cài đặt driver Slamtec RPLidar (sllidar_ros2)..."
    if [ ! -d "$WORKSPACE/src/sllidar_ros2" ]; then
        git clone https://github.com/Slamtec/sllidar_ros2.git "$WORKSPACE/src/sllidar_ros2"
    else
        echo "Package sllidar_ros2 đã có sẵn trong src/."
    fi

    # Cài đặt udev rule cho RPLidar
    if [ -f "$WORKSPACE/src/sllidar_ros2/scripts/rplidar.rules" ]; then
        echo "Cài đặt udev rules cho RPLidar..."
        sudo cp "$WORKSPACE/src/sllidar_ros2/scripts/rplidar.rules" /etc/udev/rules.d/
        sudo udevadm control --reload-rules || true
        sudo udevadm trigger || true
    fi
}

function install_ydlidar {
    echo ">>> Cài đặt driver YDLIDAR..."
    cd /tmp
    if [ ! -d "/tmp/YDLidar-SDK" ]; then
        git clone https://github.com/YDLIDAR/YDLidar-SDK.git
        mkdir -p /tmp/YDLidar-SDK/build
        cd /tmp/YDLidar-SDK/build
        cmake ..
        make -j$(nproc)
        sudo make install
    fi
    cd "$WORKSPACE"
    if [ ! -d "$WORKSPACE/src/ydlidar_ros2_driver" ]; then
        git clone https://github.com/YDLIDAR/ydlidar_ros2_driver "$WORKSPACE/src/ydlidar_ros2_driver"
    fi
    echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", GROUP:="dialout", SYMLINK+="ydlidar"' | sudo tee /etc/udev/rules.d/ydlidar.rules >/dev/null
    sudo udevadm control --reload-rules || true
    sudo udevadm trigger || true
}

function install_ldlidar_stl_ros2 {
    echo ">>> Cài đặt driver LDLIDAR..."
    cd "$WORKSPACE"
    if [ ! -d "$WORKSPACE/src/ldlidar_stl_ros2" ]; then
        git clone https://github.com/hippo5329/ldlidar_stl_ros2.git "$WORKSPACE/src/ldlidar_stl_ros2"
    fi
    cd /tmp
    wget -q https://raw.githubusercontent.com/linorobot/ldlidar/ros2/ldlidar.rules -O ldlidar.rules || true
    if [ -f ldlidar.rules ]; then
        sudo cp ldlidar.rules /etc/udev/rules.d/
        sudo udevadm control --reload-rules || true
        sudo udevadm trigger || true
    fi
}

function install_realsense {
    echo ">>> Cài đặt driver Intel RealSense..."
    sudo apt-get install -y ros-$ROS_DISTRO-realsense2-camera
    cd /tmp
    wget -q https://raw.githubusercontent.com/IntelRealSense/librealsense/master/config/99-realsense-libusb.rules -O 99-realsense-libusb.rules || true
    if [ -f 99-realsense-libusb.rules ]; then
        sudo cp 99-realsense-libusb.rules /etc/udev/rules.d/
        sudo udevadm control --reload-rules || true
        sudo udevadm trigger || true
    fi
}

function install_oakd {
    echo ">>> Cài đặt driver OAK-D..."
    echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules >/dev/null
    sudo udevadm control --reload-rules || true
    sudo udevadm trigger || true
    sudo apt-get install -y ros-$ROS_DISTRO-depthai-ros
}

echo ""
echo ">>> BƯỚC 3: Cài đặt Driver cảm biến đã chọn..."
case "$LASER_SENSOR" in
    a1|a2|a3|c1|s1|s2|s3|rplidar|sllidar)
        install_sllidar_ros2
        ;;
    ydlidar)
        install_ydlidar
        ;;
    ld06|ld19|stl27l|ldlidar)
        install_ldlidar_stl_ros2
        ;;
    realsense)
        install_realsense
        ;;
    oakd|oakdlite|oakdpro)
        install_oakd
        ;;
    "")
        echo "Không chọn cảm biến LiDAR."
        ;;
    *)
        echo "Cảm biến $LASER_SENSOR không thuộc danh sách tự động, bỏ qua."
        ;;
esac

case "$DEPTH_SENSOR" in
    realsense)
        install_realsense
        ;;
    oakd|oakdlite|oakdpro)
        install_oakd
        ;;
    "")
        ;;
    *)
        echo "Cảm biến chiều sâu $DEPTH_SENSOR không thuộc danh sách tự động, bỏ qua."
        ;;
esac

echo ""
echo ">>> BƯỚC 4: Kiểm tra và thiết lập micro-ROS cho ESP32..."
cd "$WORKSPACE"
if [ ! -d "$WORKSPACE/src/micro_ros_setup" ]; then
    echo "Cloning micro_ros_setup..."
    git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git "$WORKSPACE/src/micro_ros_setup"
fi

# Build micro_ros_setup trước để có lệnh tạo agent
colcon build --packages-select micro_ros_setup --symlink-install
source "$WORKSPACE/install/setup.bash"

# Tạo và build micro-ROS Agent (nếu chưa có agent_ws)
if [ ! -d "$WORKSPACE/src/uros/micro-ROS-Agent" ] && [ ! -d "$WORKSPACE/uros_ws" ]; then
    echo "Đang tạo và build micro-ROS Agent workspace..."
    ros2 run micro_ros_setup create_agent_ws.sh || true
    ros2 run micro_ros_setup build_agent.sh || true
    source "$WORKSPACE/install/setup.bash" || true
else
    echo "micro-ROS Agent đã được cài đặt trong workspace."
fi

echo ""
echo ">>> BƯỚC 5: Cài đặt toàn bộ dependencies của các package trong src/..."
cd "$WORKSPACE"
rosdep install --from-paths src --ignore-src -y -r --skip-keys microxrcedds_agent

echo ""
echo ">>> BƯỚC 6: Cấu hình biến môi trường trong ~/.bashrc..."
BASHRC="$HOME/.bashrc"

# Hàm thêm dòng vào .bashrc nếu chưa tồn tại
function add_to_bashrc {
    local line="$1"
    if ! grep -Fxq "$line" "$BASHRC"; then
        echo "$line" >> "$BASHRC"
    fi
}

add_to_bashrc "source /opt/ros/$ROS_DISTRO/setup.bash"
add_to_bashrc "export LINOROBOT2_BASE=$BASE"

if [[ -n "$LASER_SENSOR" && "$LASER_SENSOR" != "-" ]]; then
    add_to_bashrc "export LINOROBOT2_LASER_SENSOR=$LASER_SENSOR"
fi

if [[ -n "$DEPTH_SENSOR" && "$DEPTH_SENSOR" != "-" ]]; then
    add_to_bashrc "export LINOROBOT2_DEPTH_SENSOR=$DEPTH_SENSOR"
fi

# Thêm source workspace
add_to_bashrc "source $WORKSPACE/install/setup.bash"

echo "Đã cập nhật các biến môi trường vào ~/.bashrc."

echo ""
echo ">>> BƯỚC 7: Build toàn bộ workspace..."
cd "$WORKSPACE"
colcon build --symlink-install

echo ""
echo "=================================================================="
echo " CÀI ĐẶT THÀNH CÔNG LINOROBOT2 WORKSPACE!"
echo "=================================================================="
echo "Để cập nhật môi trường dòng lệnh hiện tại, hãy chạy:"
echo "    source ~/.bashrc"
echo ""
echo "Để chạy robot thực tế:"
echo "    ros2 launch linorobot2_bringup bringup.launch.py"
echo ""
echo "Để chạy mô phỏng Gazebo (nếu dùng trên máy tính):"
echo "    ros2 launch linorobot2_gazebo gazebo.launch.py world_name:=office.sdf"
echo "=================================================================="
