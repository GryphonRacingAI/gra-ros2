# simulation
Leeds Gryphon Racing AI Gazebo simulation environment with an ADS-DV model, sensors and tracks.

## Installation

1. **Install Gazebo Harmonic:**

   ```bash
   # Install necessary tools
   sudo apt-get update
   sudo apt-get install curl lsb-release gnupg

   # Add Gazebo repository
   sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

   # Install Gazebo Harmonic
   sudo apt-get update
   sudo apt-get install gz-harmonic
   ```

2. **Install dependencies:**
   ```bash
   cd ~/colcon_ws
   rosdep install --from-paths src -r -y
   ```

3. **Build the workspace:**
   ```bash
   colcon build --packages-select simulation
   source install/setup.bash
   ```

4. **Setup relevant environment variables**
   ```bash
   export GZ_SIM_RESOURCE_PATH=$HOME/colcon_ws/install/simulation/share/
   export GZ_SIM_RESOURCE_PATH="$(ros2 pkg prefix --share simulation)/models/tracks:$(ros2 pkg prefix --share simulation)/models/vehicle:$(ros2 pkg prefix --share simulation)/models/cones:$(ros2 pkg prefix --share simulation)/models/world:$(ros2 pkg prefix --share simulation)/models/sensors:${GZ_SIM_RESOURCE_PATH:-}"
   ```


## Usage

### Launch the Simulator

```bash
ros2 launch simulation dynamic_event.launch.py autostart:=true
pkill gz # This must be ran, to prevent Gazebo from glitching and actually stop all launched gz and ros2 nodes 
```


This should launch Gazebo Sim, with the acceleration track and ADS-DV vehicle model spawned in.
![Example](https://github.com/user-attachments/assets/99254e31-ed0a-49ae-9bee-ec22e6a2810f)

The following launch arguments are provided for `dynamic_event.launch.py`
  | Argument |Description| Options | Default
--|--|--|--|
event | specifies which track to spawn in based on the dynamic event |`acceleration`, `skidpad`, `autocross`, `trackdrive`, `mppi_track` |`acceleration`
autostart | starts the simulation automatically |`true`, `false`|`true`
model_file | path to the vehicle model sdf file | Any valid path to vehicle sdf model |hard-coded path
name | sets the vehicle name in Gazebo | Any valid string|`ads_dv`
verbosity | sets the Gazebo console output verbosity | 0 - 4| `1`

### Vehicle Control Example

Publish commands to /ackermann_cmd (ackermann_msgs/msg/AckermannDrive):
- steering_angle (rad)
- speed (m/s)

The ackermann_to_speed_steer node will handle conversion to separate speed and steer commands, which will be bridged to Gazebo and applied by the JointController and AckermannSteering plugins, respectively.

The same node also computes and publishes the (approximate) bicycle-model steering angle on the ROS-side from the joint angles.

Quick examples:
```bash
# one-off ackermann command
ros2 topic pub /ackermann_cmd ackermann_msgs/msg/AckermannDrive "{steering_angle: 0.0, speed: 1.0}" --once
```

```bash
# view current measured steering angle
ros2 topic echo /steer_angle
```

### Run Perfect Perception
```bash
ros2 run simulation perfect_perception --ros-args -p use_sim_time:=true
```

<img width="1523" height="342" alt="perfect_perception" src="https://github.com/user-attachments/assets/61144fb0-c627-45b5-877f-c11864accfd0" />

### Run Perfect SLAM
```bash
ros2 run simulation perfect_SLAM --ros-args -p use_sim_time:=true
```

<img width="1525" height="396" alt="perfect_SLAM" src="https://github.com/user-attachments/assets/560ce03d-fa46-4be7-990e-6f47c514d203" />

### Run Perfect Path
```bash
ros2 run simulation perfect_path --ros-args -p use_sim_time:=true
```

To select the track for the perfect path (must match the spawned world):

| Parameter | Description | Options | Default |
|-----------|-------------|---------|---------|
| track | Track whose cone-pairs config is used | `acceleration`, `mppi_track` | `acceleration` |
| config_file | Override with a custom cone-pairs file path | Any valid path | Derived from `track` |

Example for MPPI track (loads cone CSVs and publishes the full midline path on `/path`):

```bash
ros2 run simulation perfect_path --ros-args -p use_sim_time:=true -p track:=mppi_track
```

| Parameter | Description | Default |
|-----------|-------------|---------|
| `inner_cones_csv` | Inner cone CSV for `mppi_track` | `share/simulation/tracks/mppi_track/inner_cones.csv` |
| `outer_cones_csv` | Outer cone CSV for `mppi_track` | `share/simulation/tracks/mppi_track/outer_cones.csv` |

### Run Teleop 

Install the Slider Publisher package
```bash
sudo apt install ros-jazzy-slider-publisher
```

Launch:
```bash
ros2 launch simulation teleop.launch
```

There are two configs:

- Default (`teleop.yaml`): publishes `speed_cmd` / `steer_angle_cmd` (Float64) — used with the Gazebo simulation.
- VCU test config (`teleop_vcu.yaml`): publishes `/ackermann_cmd` (ackermann_msgs/msg/AckermannDrive) + `/brake`, `/emergency_brake`, `/chequered_flag` (Bool). This is what `ackermann_can` expects.

Example — launch the VCU teleop config directly:
```bash
ros2 launch simulation teleop.launch config:=$(ros2 pkg prefix simulation)/share/simulation/config/teleop_vcu.yaml
```

This is used by [`tmux/vcu_sim.sh`](../tmux/README.md) (together with `vcu_sim.py` on vcan0 + `ackermann_can vcan0`) to verify that steering commands flow through to the VCU simulator (watch the `steer=...deg` line in the vcu_sim pane). Full Gazebo + planner + CAN is `./tmux/startup.sh` (`event:=mppi_track` by default).

## Interface

| Node | Inputs | Outputs | Description |
|------|--------|---------|-------------|
| `perfect_SLAM` | `/logical_camera` (ros_gz_interfaces/msg/LogicalCameraImage) | `/perfect_cone_map` (common_msgs/msg/ConeArray)<br>`/perfect_cone_map_markers` (visualization_msgs/msg/MarkerArray)<br>`/perfect_odom` (nav_msgs/msg/Odometry) | Perfect SLAM simulation node |
| `perfect_perception` | `/logical_camera` (ros_gz_interfaces/msg/LogicalCameraImage) | `/perfect_cone_array` (common_msgs/msg/ConeArray)<br>`/perfect_cone_array_markers` (visualization_msgs/msg/MarkerArray) | Perfect perception simulation node |
| `perfect_path` | `/odom` (nav_msgs/msg/Odometry) when `track!=mppi_track` | `/path` (nav_msgs/msg/Path) when `track=mppi_track`<br>`/perfect_path` (nav_msgs/msg/Path) otherwise<br>`/perfect_path_markers` (visualization_msgs/msg/MarkerArray) | For `mppi_track`, loads inner/outer cone CSVs and publishes the full midline path on `/path`. Otherwise publishes a lookahead path from a cone-pairs config file. |
| `ackermann_to_speed_steer` | `/ackermann_cmd` (ackermann_msgs/msg/AckermannDrive)<br>`/joint_states` (sensor_msgs/msg/JointState) | `/speed_cmd` (std_msgs/msg/Float64)<br>`/steer_angle_cmd` (std_msgs/msg/Float64)<br>`/steer_angle` (std_msgs/msg/Float64) | Converts Ackermann commands to speed and steering commands, and publishes current steering angle

**Note:** both perfect_perception and perfect_SLAM rely on TF to get some ground truth data (vehicle pose, cone poses, etc.), as this is how it is currently bridged from Gazebo to ROS.

## ROS-Gazebo Bridge

**Transports and converts topics to/from ROS/GZ**

`ros_gz_bridge.launch.py` is a thin wrapper with config parameters pre-set. It is automatically called in `dynamic_event.launch.py`, and it can also be called directly to bridge the topics as specified.

**Configuration:** `config/ros_gz_bridge.yaml`

Example topic config:
```yaml
- ros_topic_name: "/camera/image"
  gz_topic_name: "/camera/image"
  ros_type_name: "sensor_msgs/msg/Image"
  gz_type_name: "gz.msgs.Image"
  direction: GZ_TO_ROS
```

**Getting topic message types**

For Gazebo topics:
   ```bash
   gz topic -i -t <gz_topic_name>
   ```

For ROS topics:
   ```bash
   ros2 topic info <ros_topic_name>
   ```

## Track Development

### File Structure

The track files are located in `models/tracks/`:
- **`.xacro` files:** XML templates for generating SDF files
- **`.sdf` files:** Gazebo model files (generated from .xacro)
- **`<track_name>.xacro`:** Defines cone locations and track layout
- **Dependencies:** `cone.xacro`, `physics.xacro`

### Creating a New Track
1. **Create** `<track_name>.xacro`
2. **Add GZ plugins** to track world file (typically the same for all tracks)
3. **Add world settings** (lighting, physics, etc.)
4. **Use cone macros** from `cone.xacro`, create a top-level "cones" model and specify cone locations
5. **Add pose publisher** to the "cones" model for ground truth poses (bridged to ROS via `/model/cones/pose` in GZ)
6. **Export to SDF:** `xacro <track_name>.xacro > <track_name>.sdf`


### Editing Tracks
- **Recommended:** Edit `.xacro` files, then export to `.sdf`
- **Quick changes:** Edit `.sdf` directly for testing (changes to the .sdf **will** be **overwritten** when exporting from .xacro)
- **Export command:** `xacro <track_name>.xacro > <track_name>.sdf`

## Vehicle Model Development

### ADS-DV Model
- **Main file:** `ads_dv.xacro`
- **Export:** `xacro ads_dv.xacro > ads_dv.sdf`
- **Dependencies:** `wheel.xacro`, `velodyne.xacro`, `sim_camera.xacro`, `steering.xacro`

### Adding New Sensors
1. **Create sensor .xacro file** (if complex)
   - Otherwise, add the sensor to main model (simple sensors like IMU can be added directly)
2. **Specify sensor-specific link(s) and joint(s)** (base_to sensor_link joint is usually defined outside this file)
3. **Check available sensor types** defined in SDFormat spec
4. **Add sensor plugin** to the file
5. **Include new sensor file** in `ads_dv.xacro`
6. **Export to SDF:** `xacro ads_dv.xacro > ads_dv.sdf`

## Simulated Sensors

| Sensor | Model | Gazebo Plugin | GZ Topic | ROS Topic |
|--------|-------|---------------|----------|-----------|
| Velodyne VLP-16 | VLP-16 | gpu_lidar | `/velodyne/points` | `/velodyne_points` |
| ZED-2i Depth Camera | zed2i_depth_camera | rgbd_camera | `/zed2i/depth_camera/image`<br>`/zed2i/depth_camera/points`<br>`/zed2i/depth_camera/camera_info` | `/zed2i/depth_camera/image`<br>`/zed2i/depth_camera/points`<br>`/zed2i/depth_camera/camera_info` |
| Logical Camera | logical_camera | logical_camera | `/logical_camera` | `/logical_camera` |
| IMU | imu_sensor | imu | `/imu` | `/imu/data` |

## Adding New Plugins

1. **Find the plugin's API reference page** and find the system parameters
   - [AckermannSteering plugin example](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1AckermannSteering.html)
2. **Add plugin configuration** to the target `.xacro` file
   - For example, adding a new JointController to `ads_dv.xacro`
3. **Export** to the target `.sdf` file
   - Following the same example, it would be `ads_dv.sdf`


## Development Notes

### Xacro, URDF and SDF Workflow

In this codebase, `xacro` (an XML templating tool typically used with URDF) is used to generate SDF files directly, since SDF is Gazebo Sim's native format. The `.xacro` files are templates that are exported to `.sdf` files, which are what Gazebo actually uses.

**Important:** Always remember to export `.xacro` files to `.sdf` after editing.

While you can edit `.sdf` files directly for quick testing, **any changes will be overwritten** when you export from `.xacro` again.

The model files use SDF format (not URDF). While ROS typically uses URDF, Gazebo Sim works natively with SDF. `ros_gz_bridge` or ROS 2's `robot_state_publisher` (which [supports SDF](https://gazebosim.org/docs/latest/ros2_interop)) handle the conversion seamlessly. This is done in the background by ROS using sdformat_urdf, which is not perfect (see disclaimer below).

### Important Disclaimers
- Gazebo Sim's default plugins are simplified and may be **unrealistic**. It is recommended to develop custom plugins or use alternative simulators with better models.
- The sdformat_urdf converter has some [limitations](https://github.com/ros/sdformat_urdf/tree/jazzy/sdformat_urdf) that may silently drop tags or require special flags to preserve them.





