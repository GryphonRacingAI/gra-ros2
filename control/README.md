Path following / vehicle control nodes.

## Installation

### Python dependencies
If you use a Python virtual environment in this workspace (as done in [`slam`](../slam/README.md) and [`path_planning`](../path_planning/README.md)), activate it before building/running:

Install Python packages used by the ROS 2 Python nodes in this package:

```bash
pip install numpy
```

Optional (only required for the MPPI scripts in `control/scripts/`):

```bash
pip install tf-transformations
```

### Build
Build just this package:

```bash
cd ~/colcon_ws
colcon build --packages-select control
source install/setup.bash
```

# Usage

## Pure Pursuit
Runs a pure pursuit controller that consumes a planned path and odometry and publishes an Ackermann command.

```bash
ros2 run control pure_pursuit.py
```

Parameters:

```bash
ros2 run control pure_pursuit.py --ros-args \
  -p lookahead_distance:=3.0 \
  -p target_speed:=4.0 \
  -p wheelbase:=1.534 \
  -p max_steering_angle:=0.366
```

## AckermannDrive to Twist
Converts `ackermann_msgs/msg/AckermannDrive` into `geometry_msgs/msg/Twist` using a bicycle-model approximation for angular velocity.

```bash
ros2 run control ackermann_to_cmdvel_node.py
```

Topic parameters:

```bash
ros2 run control ackermann_to_cmdvel_node.py --ros-args \
  -p input_topic:=/ackermann_cmd \
  -p output_topic:=/speed_cmd
```

# Interface

| Node | Inputs | Outputs | Description |
|------|--------|---------|-------------|
| `pure_pursuit.py` | `/path` (`nav_msgs/Path`)<br>`/odom` (`nav_msgs/Odometry`) | `/ackermann_cmd` (`ackermann_msgs/AckermannDrive`) | Path-following controller |
| `ackermann_to_cmdvel_node.py` | `/ackermann_cmd` (`ackermann_msgs/AckermannDrive`) | `/speed_cmd` (`geometry_msgs/Twist`) | Converts Ackermann commands to a Twist topic (topic names are parameters) |
