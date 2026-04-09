#!/usr/bin/env bash
set -euo pipefail

# =============================
# PC-SAFE ROS2 WORKSPACE BOOTSTRAP
# =============================
# Defaults (override by exporting before running, e.g. INSTALL_MOVEIT=1 ./bootstrap_pc.sh)
INSTALL_MOVEIT=${INSTALL_MOVEIT:-0}          # 0/1 – MoveIt is big; off by default
INSTALL_NAV2=${INSTALL_NAV2:-1}              # Nav2 is useful on a PC; on by default
INSTALL_LIDAR=${INSTALL_LIDAR:-0}            # Build YDLidar SDK/driver; off by default
INSTALL_CAMERA_EXTRAS=${INSTALL_CAMERA_EXTRAS:-0}  # v4l2/backward from source if missing
ROS_DESKTOP=${ROS_DESKTOP:-1}                # Install ros-<distro>-desktop (1) or ros-base (0)

# Workspace + locations
WS_ROOT="$(pwd)"
SRC_DIR="${WS_ROOT}/src"
THIRD="${SRC_DIR}/third_party"
mkdir -p "${THIRD}"

# Distro detection
. /etc/os-release
UBU_CODENAME="${UBUNTU_CODENAME:-jammy}"   # jammy=22.04, noble=24.04
ARCH="$(dpkg --print-architecture)"

case "${UBU_CODENAME}" in
  jammy) DEFAULT_ROS_DISTRO=humble ;;
  noble) DEFAULT_ROS_DISTRO=jazzy  ;;
  *)     DEFAULT_ROS_DISTRO=humble ;;
esac
ROS_DISTRO="${ROS_DISTRO:-${DEFAULT_ROS_DISTRO}}"

echo "==> Ubuntu: ${UBU_CODENAME}  Arch: ${ARCH}  ROS_DISTRO: ${ROS_DISTRO}"
echo "==> WS_ROOT: ${WS_ROOT}"

# ---- Functions ----
need_cmd() { command -v "$1" >/dev/null 2>&1 || { echo "Missing tool: $1"; exit 1; }; }
apt_has_repo_line() { grep -Rqs "$1" /etc/apt/sources.list /etc/apt/sources.list.d || return 1; }

ensure_ros_repo() {
  local KEYRING="/usr/share/keyrings/ros-archive-keyring.gpg"
  local LIST="/etc/apt/sources.list.d/ros2.list"
  if ! apt_has_repo_line "packages.ros.org/ros2/ubuntu"; then
    echo "==> Adding ROS 2 APT repo for ${UBU_CODENAME}"
    curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o "${KEYRING}"
    echo "deb [arch=${ARCH} signed-by=${KEYRING}] http://packages.ros.org/ros2/ubuntu ${UBU_CODENAME} main" \
      | sudo tee "${LIST}" >/dev/null
  else
    echo "==> ROS 2 APT repo already present"
  fi
}

install_base_tools() {
  echo "==> Installing base tools"
  sudo apt-get update -y
  sudo apt-get install -y --no-install-recommends \
    git curl gnupg lsb-release \
    build-essential cmake pkg-config \
    python3-pip python3-rosdep python3-vcstool \
    python3-colcon-common-extensions

  # ament lint helpers so message pkgs don't fail
  sudo apt-get install -y --no-install-recommends \
    "ros-${ROS_DISTRO}-ament-lint-auto" \
    "ros-${ROS_DISTRO}-ament-lint-common" \
    "ros-${ROS_DISTRO}-ament-cmake-auto" || true

  # Desktop or base
  if [[ "${ROS_DESKTOP}" == "1" ]]; then
    sudo apt-get install -y "ros-${ROS_DISTRO}-desktop"
  else
    sudo apt-get install -y "ros-${ROS_DISTRO}-ros-base"
  fi

  # Optional stacks (safe on PC)
  if [[ "${INSTALL_NAV2}" == "1" ]]; then
    sudo apt-get install -y "ros-${ROS_DISTRO}-navigation2" "ros-${ROS_DISTRO}-nav2-bringup"
  fi

  if [[ "${INSTALL_MOVEIT}" == "1" ]]; then
    sudo apt-get install -y \
      "ros-${ROS_DISTRO}-moveit" \
      "ros-${ROS_DISTRO}-moveit-setup-assistant" \
      "ros-${ROS_DISTRO}-moveit-servo" \
      "ros-${ROS_DISTRO}-moveit-resources-panda-moveit-config" || true
  fi

  # Robot state/joint publishers are handy
  sudo apt-get install -y --no-install-recommends \
    "ros-${ROS_DISTRO}-robot-state-publisher" \
    "ros-${ROS_DISTRO}-joint-state-publisher" \
    "ros-${ROS_DISTRO}-joint-state-publisher-gui" || true

  sudo apt-get install "ros-${ROS_DISTRO}-topic-tools"
}

pull_third_party() {
  echo "==> Importing third party repos (if x3plus.repos exists)"
  [[ -f "${WS_ROOT}/x3plus.repos" ]] && vcs import "${THIRD}" < "${WS_ROOT}/x3plus.repos" || true

  if [[ "${INSTALL_CAMERA_EXTRAS}" == "1" ]]; then
    # Try binary first; if absent, fall back to source
    if ! apt-cache show "ros-${ROS_DISTRO}-v4l2-camera" >/dev/null 2>&1; then
      [[ -d "${THIRD}/v4l2_camera" ]] || git -C "${THIRD}" clone https://github.com/ros-drivers/v4l2_camera.git -b ros2 || true
    else
      sudo apt-get install -y "ros-${ROS_DISTRO}-v4l2-camera" || \
        ([[ -d "${THIRD}/v4l2_camera" ]] || git -C "${THIRD}" clone https://github.com/ros-drivers/v4l2_camera.git -b ros2)
    fi

    if ! apt-cache show "ros-${ROS_DISTRO}-backward-ros" >/dev/null 2>&1; then
      [[ -d "${THIRD}/backward_ros" ]] || git -C "${THIRD}" clone https://github.com/pal-robotics/backward_ros.git -b "${ROS_DISTRO}" || \
      git -C "${THIRD}" clone https://github.com/pal-robotics/backward_ros.git || true
    else
      sudo apt-get install -y "ros-${ROS_DISTRO}-backward-ros" || true
    fi
  fi
}

setup_lidar_locally() {
  [[ "${INSTALL_LIDAR}" == "1" ]] || { echo "==> Skipping LiDAR (INSTALL_LIDAR=0)"; return 0; }
  echo "==> Setting up YDLidar-SDK locally (no sudo install)"
  local SDK_SRC="${THIRD}/YDLidar-SDK"
  local SDK_BUILD="${SDK_SRC}/build"
  local SDK_INSTALL="${SDK_SRC}/install"   # local prefix

  if [[ ! -d "${SDK_SRC}" ]]; then
    git -C "${THIRD}" clone https://github.com/YDLIDAR/YDLidar-SDK.git
  else
    git -C "${SDK_SRC}" fetch --all
    git -C "${SDK_SRC}" pull --ff-only
  fi

  mkdir -p "${SDK_BUILD}"
  cmake -S "${SDK_SRC}" -B "${SDK_BUILD}" -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="${SDK_INSTALL}"
  cmake --build "${SDK_BUILD}" -j"$(nproc)"
  cmake --install "${SDK_BUILD}"

  # Write an env snippet to help colcon/CMake find the local SDK
  cat > "${WS_ROOT}/env_pc.sh" <<EOF
# Source this after your ROS setup.bash
export CMAKE_PREFIX_PATH="${SDK_INSTALL}:\${CMAKE_PREFIX_PATH:-}"
export ydlidar_sdk_DIR="${SDK_INSTALL}/lib/cmake/ydlidar_sdk"
EOF
  echo "==> Wrote ${WS_ROOT}/env_pc.sh (source it before colcon build if building ydlidar_ros2_driver)"
}

rosdep_install() {
  echo "==> Running rosdep (no sudo installs beyond apt)"
  sudo rosdep init 2>/dev/null || true
  rosdep update
  rosdep install --from-paths "${SRC_DIR}" --ignore-src -r -y || true
  # User-level python deps only (avoid system pip)
  python3 -m pip install --user -U keyboard || true
}

build_ws() {
  echo "==> Building workspace"
  colcon build \
    --symlink-install --packages-skip orbbec_camera astra_camera ydlidar_ros2_driver ydlidar_sdk x3plus_lidar_bringup x3plus_mapping_bringup x3plus_vision_bringup astra_camera_msgs orbbec_camera_msgs \
    --continue-on-error \
    || true
}

# ---- Execution ----
need_cmd curl
need_cmd git
need_cmd cmake

ensure_ros_repo
install_base_tools
pull_third_party
setup_lidar_locally
rosdep_install
build_ws

echo
echo "====================================="
echo "All set!"
echo "Next steps:"
echo "  1) Open a NEW terminal and run:"
echo "       source /opt/ros/${ROS_DISTRO}/setup.bash"
echo "       source ${WS_ROOT}/install/setup.bash"
[[ "${INSTALL_LIDAR}" == "1" ]] && echo "       source ${WS_ROOT}/env_pc.sh   # so CMake sees local YDLidar-SDK"
echo "  2) Build again if you add packages:  colcon build --symlink-install"
