# Running the simulated pipeline on Windows (WSL2)

Tested on WSL2 Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic, RTX 5060 laptop GPU.
The pipeline is the same as on native Ubuntu:

```
Gazebo (ads_dv + track) -> ads_dv_vcu_sim (VCU + drivetrain) / vcu_odometry (wheels + IMU)
  -> perception (YOLO + lidar, camera fallback) -> /cone_array
  -> lap_counter (/laps) -> FastSLAM (/slam/pose, /slam/cone_map, /slam/cones)
  -> pathfinder (/path) -> MPPI (/ackermann_cmd) -> ads_dv_vcu_sim -> wheel torques
  mission_control ends the event: /chequered_flag -> brakes
```

### How realistic the simulated car is

| | sim | source |
|---|---|---|
| drivetrain | 4WD, one motor per axle 3.5:1, 195 Nm/axle with the VCU's speed-dependent caps, speed loop per axle | fsai_api (fs-ai_api.c, ackermann_can.cpp) |
| brakes | 50 % on /brake or chequered flag (0.6 g measured), 100 % on emergency brake or lost commands | ackermann_can; brake torque is an estimate |
| tyres | mu 1.15 -> ~1.37 g sustained lateral (measured) | estimate for slicks, no aero |
| steering | +-24.3 deg, 1 rad/s actuator | limit from the sdf; rate is an estimate |
| resistance | rolling 0.015, CdA 1.0 | estimates |
| lidar | VLP-16: 16 rings 2 deg apart, 0.2 deg horizontal, 1.5 cm range noise | VLP-16 data sheet |
| IMU | MEMS noise and bias | ZED 2i class |
| odometry | wheel speeds + IMU yaw rate, gyro bias calibrated at standstill | as on the car |

`drive:=ideal` and `odom:=ground_truth` (launch args, or `FSAI_DRIVE` / `FSAI_ODOM` for the
launcher) give the old behaviour: wheel speed set directly and Gazebo's exact pose as /odom.

## 1. Setup

1. Install ROS 2 Jazzy (`ros-jazzy-desktop`) and Gazebo Harmonic as in the main README.
   WSLg provides the display: run `wsl --update` in PowerShell if Gazebo can't open a window.
2. Workspace: clone this repo as `~/colcon_ws/src`.
3. Python venv for YOLO / SLAM (system ROS Python + venv packages):
   ```bash
   cd ~/colcon_ws
   python3 -m venv ros_venv
   touch ros_venv/COLCON_IGNORE        # stop colcon scanning the venv
   source ros_venv/bin/activate
   pip install -r src/perception/src/ultralytics_ros/requirements.txt
   pip install pycuda git+https://github.com/GryphonRacingAI/fastslam64.git
   pip install "numpy<2"
   deactivate
   ```
   The venv's packages reach ROS nodes through `PYTHONPATH` (section 2).
   **`numpy<2` is required.** ROS Jazzy's `cv_bridge` is built against NumPy 1.x; with
   NumPy 2 in the venv the perception node prints "compiled using NumPy 1.x" and segfaults.
   pycuda needs the CUDA toolkit (`nvcc`), e.g. from NVIDIA's WSL-Ubuntu CUDA repo.
4. Build (the ZED packages and `bringup` need the ZED SDK and aren't used in sim, so skip them):
   ```bash
   source /opt/ros/jazzy/setup.bash
   cd ~/colcon_ws
   colcon build --symlink-install --packages-skip bringup --packages-skip-regex '^zed'
   ```

## 2. Environment variables (WSL specifics)

```bash
set +u   # ROS setup scripts reference unset variables
source ~/colcon_ws/ros_venv/bin/activate
source /opt/ros/jazzy/setup.bash
source ~/colcon_ws/install/setup.bash
export PYTHONPATH=~/colcon_ws/ros_venv/lib/python3.12/site-packages:$PYTHONPATH
export GZ_IP=127.0.0.1                            # Gazebo transport on localhost (WSL multicast is unreliable)
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST    # same for ROS 2 discovery
export GALLIUM_DRIVER=d3d12 MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA   # render Gazebo on the NVIDIA GPU
export PATH=$PATH:/usr/lib/wsl/lib:/usr/local/cuda/bin              # nvidia-smi, libcuda, nvcc
# If the Gazebo window is black or crashes: export LIBGL_ALWAYS_SOFTWARE=1 instead of the GALLIUM lines
```

## 3. Run

One terminal per line, in this order (after the environment above):

```bash
ros2 launch simulation dynamic_event.launch.py event:=trackdrive        # gui:=false: no window
ros2 launch ultralytics_ros predict_with_cloud.launch.xml use_sim_time:=true yolo_model:=conev11n.pt input_topic:=/zed2i/depth_camera/image verbose:=false
ros2 run mission_supervisor lap_counter --ros-args -p use_sim_time:=true
ros2 run slam fastslam_node.py --ros-args -p use_sim_time:=true
ros2 run path_planning pathfinder.py --ros-args -p use_sim_time:=true -p event:=trackdrive
ros2 run control mppi_ros_modified.py --ros-args -p use_sim_time:=true --params-file src/control/config/mppi_params.yaml
ros2 run mission_supervisor mission_control --ros-args -p use_sim_time:=true -p event:=trackdrive
```

Events: `trackdrive` (track_small, 10 laps), `mppi_track`, `autocross` (drive1, FSG 2019, 1 lap),
`acceleration`, `skidpad`. Use the same `event` for the simulation, pathfinder and mission_control.

`predict_with_cloud_node` reads its parameters (topics, cluster sizes) from
`perception/src/ultralytics_ros/config/predict_with_cloud_node.yaml`, which `main()` loads with
higher priority than launch-file parameters. Edit the YAML, not the launch arguments.

## 4. Checking it works

```bash
ros2 topic echo /cone_array --once     # no cones at (0,0,0); several yellow AND blue
ros2 topic hz /path                     # ~10 Hz once SLAM has mapped cones on both sides
ros2 topic echo /ground_truth/odom --field pose.pose.position   # where the car really is
ros2 topic echo /laps                   # increments each time the car crosses the start line
ros2 topic echo /mission_state          # DRIVING -> FINISHING (braking) -> FINISHED
```

`ros2 topic hz` under-reports high-rate topics (e.g. `/odom`) when the CPU is busy.

## 5. Status per event

Full stack in Gazebo with the realistic car (torque drivetrain, tyre grip, wheel/IMU odometry,
VLP-16 at real resolution), headless, WSL VM with 4 cores. Judged against ground truth.

| event | result |
|---|---|
| trackdrive (track_small, 98 m) | 10 laps then stop; 13.6-13.9 s laps (6.6 m/s mean), never more than 1.1 m from the centreline (several consecutive missions) |
| autocross (drive1, 242 m) | 1 lap then stop; never more than 1.05 m from the centreline, 5.4 m/s mean |
| acceleration (75 m) | 7.3 s to the finish line (13.4 m/s), within 0.2 m of the lane centre, stops ~20 m after the line |
| skidpad | right circle x2, left circle x2 at 9.2 m radius (rules: 9.125 m), exit, stop |
| mppi_track (372 m, 3 m wide) | 10 laps then stop, 43-46 s laps, but over the cone line ~17 % of the time (see below) |

## 6. Known limits

- **mppi_track localisation:** after the map freeze the car localises by matching cones to the
  frozen map. On long straights with evenly spaced cones only the lateral position can be
  corrected; along-track drift (wheel slip under hard acceleration, ~1.5 %) builds up to more
  than the 1 m match gate, and the car then drives laps offset by a few metres until a corner
  realigns it. A wider search made it worse (ambiguous). Needs a better localiser.
- **FastSLAM process noise:** with realistic odometry noise (e.g. real_config.yaml's) this
  FastSLAM builds poor maps in sim even with perfect odometry (100-130 landmarks for 72 cones);
  the sim config uses near-zero noise while mapping. Persistent weights (fixed) helped but did not
  solve it. Worth checking on the car.
- **fastslam64 (external library):** heading mean is linear (worked around in the node with a
  circular mean); data association ignores cone colour (the frozen-map localisation in the node
  is colour-aware).
- **Estimated vehicle numbers** (check against the car): speed loop gains, brake torque at 100 %,
  steering rate (1 rad/s), rolling resistance, CdA, regenerative braking.
- **VLP-16 blind band:** no ring hits a 0.325 m cone between ~10.5 and ~16 m (0.6 m mount, 2 deg
  rings); perception falls back to the camera up to 13 m.
- WSL gives the VM a share of the laptop's cores/RAM (`.wslconfig`). With 4 cores the Gazebo GUI
  costs ~half a core and control quality drops when the real-time factor falls below ~0.5
  (e.g. while recording bags): use `gui:=false` for long tests.
