#!/usr/bin/env bash
set -eo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export DEBIAN_FRONTEND=noninteractive

echo "==> Configuring apt repositories (ROS 2 Jazzy + Gazebo)"
if [ ! -f /etc/apt/sources.list.d/ros2.list ]; then
  sudo apt-get update
  sudo apt-get install -y curl gnupg lsb-release software-properties-common
  sudo add-apt-repository -y universe
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "$UBUNTU_CODENAME") main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
fi

if [ ! -f /etc/apt/sources.list.d/gazebo-stable.list ]; then
  sudo curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
    -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
fi

echo "==> Installing ROS 2 Jazzy, Gazebo Harmonic and build tooling"
sudo apt-get update
sudo apt-get install -y \
  ros-jazzy-desktop \
  ros-dev-tools \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  python3-venv \
  build-essential \
  cmake \
  git \
  xvfb \
  gz-harmonic \
  ros-jazzy-ros-gz \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-ros-gz-interfaces \
  ros-jazzy-ackermann-msgs \
  ros-jazzy-tf-transformations \
  ros-jazzy-velodyne-msgs \
  python3-transforms3d \
  python3-numpy \
  python3-yaml

echo "==> Initialising rosdep"
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init
fi
rosdep update

echo "==> Setting up Python virtual environment (system-site-packages)"
VENV_DIR="$REPO_DIR/.venv"
if [ ! -d "$VENV_DIR" ]; then
  python3 -m venv --system-site-packages "$VENV_DIR"
fi
# shellcheck disable=SC1091
source "$VENV_DIR/bin/activate"
pip install --upgrade pip
pip install pyaml transforms3d

echo "==> Resolving ROS dependencies with rosdep"
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths "$REPO_DIR" --ignore-src -r -y \
  --skip-keys "zed_wrapper zed_components zed_ros2 zed_msgs velodyne" \
  || echo "rosdep reported unresolved keys (expected for proprietary ZED packages)"

echo "==> Building the colcon workspace (excluding ZED-dependent bringup)"
cd "$REPO_DIR"
export CC=gcc
export CXX=g++
colcon build --symlink-install \
  --packages-skip bringup ultralytics_ros mission_supervisor \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_C_COMPILER=gcc -DCMAKE_CXX_COMPILER=g++

echo "==> Configuring shell environment for interactive use"
BASHRC="$HOME/.bashrc"
MARKER="# >>> gra-ros2 workspace >>>"
if ! grep -qF "$MARKER" "$BASHRC" 2>/dev/null; then
  {
    echo ""
    echo "$MARKER"
    echo "source /opt/ros/jazzy/setup.bash"
    echo "[ -f \"$REPO_DIR/install/setup.bash\" ] && source \"$REPO_DIR/install/setup.bash\""
    echo "[ -f \"$REPO_DIR/.venv/bin/activate\" ] && source \"$REPO_DIR/.venv/bin/activate\""
    echo "export CC=gcc"
    echo "export CXX=g++"
    MODELS="$REPO_DIR/install/simulation/share/simulation/models"
    echo "export GZ_SIM_RESOURCE_PATH=\"$MODELS/tracks:$MODELS/vehicle:$MODELS/cones:$MODELS/world:$MODELS/sensors\""
    echo "# <<< gra-ros2 workspace <<<"
  } >> "$BASHRC"
fi

echo "==> Install complete"
