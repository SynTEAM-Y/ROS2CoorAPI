#!/usr/bin/env bash
set -euo pipefail

# --- Detect distro + fix ROS 2 repo key (expired key issue) ---
ROS_KEYRING="/usr/share/keyrings/ros-archive-keyring.gpg"
ROS_LIST="/etc/apt/sources.list.d/ros2.list"
ARCH="$(dpkg --print-architecture)"
UBU_CODENAME="$(. /etc/os-release; echo ${UBUNTU_CODENAME:-jammy})"

# Map Ubuntu -> ROS 2 default (override by exporting ROS_DISTRO before running)
case "${UBU_CODENAME}" in
  jammy) DEFAULT_ROS_DISTRO=humble ;;
  noble) DEFAULT_ROS_DISTRO=jazzy ;;
  focal) DEFAULT_ROS_DISTRO=foxy  ;;
  *)     DEFAULT_ROS_DISTRO=humble ;;
esac
ROS_DISTRO="${ROS_DISTRO:-${DEFAULT_ROS_DISTRO}}"

sudo apt-key del F42ED6FBAB17C654 >/dev/null 2>&1 || true
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  | sudo gpg --dearmor -o "$ROS_KEYRING"

echo "deb [arch=${ARCH} signed-by=${ROS_KEYRING}] http://packages.ros.org/ros2/ubuntu ${UBU_CODENAME} main" \
  | sudo tee "$ROS_LIST" >/dev/null

sudo rm -rf /var/lib/apt/lists/*
sudo apt-get update

# --- Base tools ---
sudo apt-get install -y git curl gnupg lsb-release \
  build-essential cmake pkg-config \
  python3-rosdep python3-vcstool python3-colcon-common-extensions

# --- MoveIt 2 ---
# I--- Core MoveIt stack, setup assistant, and Servo for jogging
sudo apt-get install -y \
  "ros-${ROS_DISTRO}-moveit" \
  "ros-${ROS_DISTRO}-moveit-ros-planning" \
  "ros-${ROS_DISTRO}-moveit-ros-move-group" \
  "ros-${ROS_DISTRO}-moveit-setup-assistant" \
  "ros-${ROS_DISTRO}-moveit-servo" || {
    echo "WARNING: Some MoveIt packages failed to install. Check ROS_DISTRO='${ROS_DISTRO}' and apt sources."
}

# --- ros2_control + controllers (needed by many demos) ---
sudo apt-get install -y \
  "ros-${ROS_DISTRO}-ros2-control" \
  "ros-${ROS_DISTRO}-ros2-controllers" \
  "ros-${ROS_DISTRO}-controller-manager" \
  "ros-${ROS_DISTRO}-joint-state-broadcaster" \
  "ros-${ROS_DISTRO}-joint-trajectory-controller" || {
    echo "WARNING: Some ros2_control packages failed to install. Check ROS_DISTRO='${ROS_DISTRO}'."
}

sudo apt-get install "ros-${ROS_DISTRO}-topic-tools"

# --- Import third party repos ---
mkdir -p src/third_party
vcs import src/third_party < x3plus.repos

# --- Init submodules if any ---
find src/third_party -maxdepth 2 -name .gitmodules -execdir git submodule update --init --recursive \; || true

# --- Ensure YDLidar-SDK is present and up to date ---
SDK_DIR="src/third_party/ydlidar_sdk"
if [[ ! -d "$SDK_DIR" ]]; then
  echo "Cloning YDLidar-SDK..."
  git clone https://github.com/YDLIDAR/YDLidar-SDK.git "$SDK_DIR"
else
  echo "Updating YDLidar-SDK..."
  pushd "$SDK_DIR" >/dev/null
  git fetch --all
  git pull --ff-only
  popd >/dev/null
fi

echo "Building and installing YDLidar-SDK..."
pushd "$SDK_DIR" >/dev/null
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j"$(nproc)"
sudo make install
sudo ldconfig
popd >/dev/null

# --- Try to install known binary deps ---
need_src_v4l2=false
need_src_backward=false

if ! apt-cache show "ros-${ROS_DISTRO}-v4l2-camera" >/dev/null 2>&1; then
  need_src_v4l2=true
else
  sudo apt-get install -y "ros-${ROS_DISTRO}-v4l2-camera" || need_src_v4l2=true
fi

if ! apt-cache show "ros-${ROS_DISTRO}-backward-ros" >/dev/null 2>&1; then
  need_src_backward=true
else
  sudo apt-get install -y "ros-${ROS_DISTRO}-backward-ros" || need_src_backward=true
fi

# --- Fallback: clone missing packages into src/third_party ---
pushd src/third_party >/dev/null

if $need_src_v4l2 && [[ ! -d v4l2_camera ]]; then
  echo "Cloning v4l2_camera from source..."
  git clone https://github.com/ros-drivers/v4l2_camera.git -b ros2
fi

if $need_src_backward && [[ ! -d backward_ros ]]; then
  echo "Cloning backward_ros from source..."
  git clone https://github.com/pal-robotics/backward_ros.git -b "${ROS_DISTRO}" || \
  git clone https://github.com/pal-robotics/backward_ros.git
fi

popd >/dev/null

# --- Rosdep ---
sudo rosdep init 2>/dev/null || true
rosdep update
rosdep install --from-paths src --ignore-src -r -y \
  --skip-keys "ament_python gazebo_ros warehouse_ros_mongo"

# Python dependencies
pip3 install -U keyboard

# --- Build ---
# colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
colcon build --symlink-install --packages-select ydlidar_sdk ydlidar_ros2_driver --cmake-args -Wno-dev -DCMAKE_C_FLAGS=-w -DCMAKE_CXX_FLAGS=-w
colcon build --symlink-install --packages-skip ydlidar_sdk ydlidar_ros2_driver

echo
echo "Done. Run:  source install/setup.bash"
