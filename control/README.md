Path following / vehicle control nodes.

## Installation

### Python dependencies
If you use a Python virtual environment in this workspace (as done in [`slam`](../slam/README.md) and [`path_planning`](../path_planning/README.md)), activate it before building/running:

```bash
pip install numpy tf-transformations
```

### Build
Build just this package:

```bash
cd ~/colcon_ws
colcon build --packages-select control
source install/setup.bash
```

# Usage

## MPPI Controller (`mppi_ros_modified.py`)

Model Predictive Path Integral controller that consumes a planned path, odometry, and cone obstacles to publish Ackermann commands.

```bash
ros2 run control mppi_ros_modified.py
```

### ROS Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `path_topic` | string | `/perfect_path` | Topic for the reference path (`nav_msgs/Path`) |
| `ask_path_topic` | bool | `false` | If true, prompt at startup to choose between `/path` or `/perfect_path` |

Example:

```bash
ros2 run control mppi_ros_modified.py --ros-args \
  -p path_topic:=/path \
  -p ask_path_topic:=false
```

### Script Constants (edit `mppi_ros_modified.py` to change)

| Constant | Value | Description |
|----------|-------|-------------|
| `DT` | 0.05 | Control loop period (s) |
| `H` | 16 | Planning horizon (steps) |
| `K` | 200 | Number of rollouts |
| `LAMBDA` | 2.0 | MPPI temperature |
| `SIGMA_U_BASE` | [0.6, 0.15] | Base noise std for [accel, steer] |
| `SIGMA_U_MIN` | [0.2, 0.05] | Min noise std |
| `W_PATH` | 20.0 | Path deviation weight |
| `W_HEADING` | 3.0 | Heading error weight |
| `W_SPEED` | 10.0 | Overspeed penalty weight |
| `W_CONTROL` | 1.5 | Control effort weight |
| `W_TERMINAL` | 5.0 | Terminal cost weight |
| `W_OBSTACLE` | 15.0 | Obstacle avoidance weight |
| `MAX_A` | 3.0 | Max acceleration (m/s²) |
| `MIN_V` | -1.0 | Min velocity (m/s) |
| `MAX_V` | 8.0 | Max velocity (m/s) |
| `MAX_STEER` | π/6 | Max steering angle (rad) |
| `L_BASE` | 1.6 | Wheelbase (m) |

# Interface

| Node | Inputs | Outputs | Description |
|------|--------|---------|-------------|
| `mppi_ros_modified.py` | `/odom` (`nav_msgs/Odometry`)<br>`path_topic` (`nav_msgs/Path`, default `/perfect_path`)<br>`/slam/cones` (`sensor_msgs/PointCloud2`) | `/drive` (`ackermann_msgs/AckermannDriveStamped`)<br>`/ackermann_cmd` (`ackermann_msgs/AckermannDrive`)<br>`/viz/mppi_path` (`nav_msgs/Path`, optional) | MPPI path-following controller with obstacle avoidance |
