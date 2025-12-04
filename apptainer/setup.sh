#!/bin/bash

# ---- Create ROS workspace directory -----
if [ -d "$HOME/colcon_ws/src" ]; then
    echo "Directory $HOME/colcon_ws/src already exists; exiting." >&2
    exit 1
fi

mkdir -p "$HOME/colcon_ws/src"
cd "$HOME/colcon_ws/src"

# ---- Clone gra-ros2 repo (dev branch) and pull submodules-----
git clone -b dev https://github.com/GryphonRacingAI/gra-ros2.git ~/colcon_ws/src
cd src; git submodule update --init --recursive

# ---- Apptainer setup -----
cd ~/colcon_ws/src/apptainer
chmod u+x setup_apptainer.sh
sh setup_apptainer.sh

# ---- Add convenient alias to ~/.bashrc for entering apptainer -----
echo "alias fsai='apptainer shell --nv /local/data/$USER/ros_jazzy.sif'" >> ~/.bashrc
echo "source /opt/ros/jazzy/setup.bash && source ~/colcon_ws/install/setup.bash && export GZ_SIM_RESOURCE_PATH=~/colcon_ws/install/simulation/share/" > ~/.fsairc

# ---- Enter the apptainer container (enable NVIDIA passthrough) -----
apptainer shell --nv /local/data/$USER/ros_jazzy.sif
