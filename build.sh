#!/usr/bin/env bash
set -e
set -u
set -o pipefail

echo "=== [USER SETUP] Configuring ROS 2 Workspace ==="

WORKSPACE=$HOME/colcon_ws
if [ ! -d "$WORKSPACE/src" ]; then
  echo "Creating workspace at $WORKSPACE"
  mkdir -p "$WORKSPACE/src"
fi

cd "$WORKSPACE"

# Check for requested packages
if [ "$#" -gt 0 ]; then
  TARGET_PKGS=("$@")
  echo "Installing dependencies for selected packages: ${TARGET_PKGS[*]}"
  rosdep install -i --from-path src --rosdistro jazzy -y --skip-keys="${TARGET_PKGS[*]}" || true
  echo "Building only: ${TARGET_PKGS[*]}"
  colcon build --symlink-install --packages-select "${TARGET_PKGS[@]}"
else
  echo "No specific packages requested, building all packages."
  rosdep install -i --from-path src --rosdistro jazzy -y || true
  colcon build --symlink-install
fi

# Source setup files (temporarily for this shell)
set +u
source /opt/ros/jazzy/setup.bash
set -u
source "$WORKSPACE/install/setup.bash"

# Persist environment setup in ~/.bashrc
if ! grep -q "source /opt/ros/jazzy/setup.bash" ~/.bashrc; then
  echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
fi
if ! grep -q "source ~/colcon_ws/install/setup.bash" ~/.bashrc; then
  echo "source ~/colcon_ws/install/setup.bash" >> ~/.bashrc
fi
if ! grep -q "export GZ_SIM_RESOURCE_PATH=" ~/.bashrc; then
  echo "export GZ_SIM_RESOURCE_PATH=\$HOME/colcon_ws/install/simulation/share/" >> ~/.bashrc
fi

echo "=== [USER SETUP COMPLETE] ==="
echo "Open a new terminal or run 'source ~/.bashrc' to start using ROS 2 Jazzy + Gazebo Harmonic."

