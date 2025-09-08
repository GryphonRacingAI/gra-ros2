#!/usr/bin/env bash
# this installs ROS 2 Jazzy + Gazebo Harmonic
set -e
set -u
set -o pipefail

echo "=== [SYSTEM SETUP] Installing ROS 2 Jazzy + Gazebo Harmonic ==="

if [ "$EUID" -ne 0 ]; then
  echo "Please run this script with sudo:"
  echo "  sudo ./setup_system.sh"
  exit 1
fi

apt update
apt install -y software-properties-common curl lsb-release gnupg

# Enable universe repo
add-apt-repository universe -y

# Add ROS 2 repo
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | tee /etc/apt/sources.list.d/ros2.list > /dev/null

apt update
apt upgrade -y
apt install -y ros-jazzy-desktop

# Add Gazebo repo
curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
  -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
  http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
  | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

apt update
apt install -y gz-harmonic

# ROS build tools
apt install -y python3-colcon-common-extensions python3-rosdep

# Initialize rosdep (ignore if already done)
rosdep init 2>/dev/null || true
rosdep update

echo "=== [SYSTEM SETUP COMPLETE] ==="
echo "Now run './setup_user.sh' as your normal user (no sudo)."

