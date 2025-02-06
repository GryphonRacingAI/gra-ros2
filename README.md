# Prerequisites
This package is intended for ROS 2 Jazzy. Follow the [installation guide](https://docs.ros.org/en/jazzy/Installation.html)

Make sure to install `ros-jazzy-desktop`, and make sure RViz2 is installed.
```
sudo apt install ros-jazzy-desktop
```
Install [Gazebo Harmonic](https://gazebosim.org/docs/harmonic/install_ubuntu/), then proceed.

# Build Instructions

Source the ROS 2 environment
```bash
source /opt/ros/jazzy/setup.bash
```

Create a workspace
```bash
mkdir -p ~/colcon_ws/src
cd ~/colcon_ws/src
```

Clone the repository into the source directory
```bash
git clone https://github.com/GryphonRacingAI/gra-ros2.git
```

Resolve dependencies
```bash
cd ~/colcon_ws
rosdep install -i --from-path src --rosdistro jazzy -y
```

Then build and source the workspace
```bash
cd ~/colcon_ws
colcon build --symlink-install
source install/setup.bash
```

# Usage
To source the overlays automatically every time you open a new terminal, add the following lines to the .bashrc script:
```bash
source /opt/ros/jazzy/setup.bash
source ~/colcon_ws/install/setup.bash
```

or simply run the following
```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "source ~/colcon_ws/install/setup.bash" >> ~/.bashrc
```

Run the following command so Gazebo can find the vehicle mesh
```bash
echo "export GZ_SIM_RESOURCE_PATH=$HOME/colcon_ws/install/simulation/share/" >> ~/.bashrc
```
then continue in a new terminal.

To run the simulator, run the following commands

```bash
ros2 launch simulation dynamic_event.launch.py autostart:=true
```
This should launch Gazebo Sim, with the acceleration track and ADS-DV vehicle model spawned in.

The following launch arguments are provided for this launch file
  | Argument |Description| Options | Default
--|--|--|--|
event | specifies which track to spawn in based on the dynamic event |`acceleration`, `skidpad`, `autocross`, `trackdrive` |`acceleration`
autostart | starts the simulation automatically |`true`, `false`|`true`
model_file | path to the vehicle model sdf file | Any valid path to vehicle sdf model |hard-coded path
name | sets the vehicle name in Gazebo | Any valid string|`ads_dv`
