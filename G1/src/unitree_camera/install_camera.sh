# 1. Cài librealsense mới nhất (v2.54.2) – thay bản cũ
# Xóa bản cũ (nếu có)
sudo apt remove ros-noetic-librealsense2 librealsense2-* -y

# Cài bản mới
cd ~
rm -rf librealsense
git clone https://github.com/IntelRealSense/librealsense.git
cd librealsense
git checkout v2.54.2

mkdir build && cd build
cmake ../ -DCMAKE_BUILD_TYPE=Release \
          -DBUILD_EXAMPLES=true \
          -DBUILD_GRAPHICAL_EXAMPLES=false \
          -DFORCE_RSUSB_BACKEND=true \
          -DBUILD_WITH_CUDA=false

make -j3
sudo make install
sudo ldconfig


# 2. Tạo workspace và cài ROS2 wrapper (phiên bản hoạt động 100% với Foxy)
mkdir -p ~/realsense_ws/src
cd ~/realsense_ws/src
rm -rf realsense-ros

# Dùng đúng tag 4.51.1 từ repo mới (đã test thành công trên G1)
wget https://github.com/realsenseai/realsense-ros/archive/refs/tags/4.51.1.tar.gz
tar -xzf 4.51.1.tar.gz
mv realsense-ros-4.51.1 realsense-ros
rm 4.51.1.tar.gz


# 3. Build workspace
cd ~/realsense_ws

# Cài dependencies (nếu chưa có)
sudo apt update
sudo apt install -y python3-rosdep python3-colcon-common-extensions
sudo rosdep init    # bỏ qua nếu báo đã init
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build (sạch sẽ)
rm -rf build install log
source /opt/ros/foxy/setup.bash
colcon build --symlink-install


# new terminal

source /opt/ros/foxy/setup.bash
source ~/realsense_ws/install/setup.bash --extend
ros2 pkg list | grep realsense

ros2 launch realsense2_camera rs_launch.py \
enable_gyro:=false \
enable_accel:=false \
unite_imu_method:="none" \
depth_width:=640 depth_height:=480 depth_fps:=30 \
color_width:=640 color_height:=480 color_fps:=30

ros2 launch realsense2_camera rs_launch.py \
enable_color:=true \
enable_depth:=true \
enable_gyro:=false \
enable_accel:=false \
align_depth.enable:=false \
pointcloud.enable:=true \
color_width:=640 \
color_height:=480 \
color_fps:=15
