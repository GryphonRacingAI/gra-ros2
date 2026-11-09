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
| cones | free rigid bodies (foot box + 2 cylinders), 0.65 kg small / 1.5 kg large, tip and slide when hit; static until the car is within 9 m | FS handbook sizes; masses assumed (see `models/cones/cone.xacro`) |

This is `drive:=torque`, which is **opt-in** (launch arg, or `FSAI_DRIVE=torque`, the launcher's default). The launch
file's default is `drive:=ideal` (wheel speed set directly by `ackermann_to_speed_steer`, Gazebo's exact pose as
/odom), which is what `tmux/startup.sh` and the CAN path (`vcu_sim.py`, `ackermann_can`, mission supervisor)
expect. `drive:=torque` starts `ads_dv_vcu_sim`, which publishes `/vcu2ai` itself and always reports
`AS_DRIVING`, so never combine it with `vcu_sim.py`. `odom:=ground_truth` gives Gazebo's pose instead of wheel/IMU odometry.

**Cones.** A dynamic cone resting on the ground costs a persistent contact per physics step; 292
dynamic cones cut the real-time factor of the 4-core VM from 0.92 to 0.57 (static cones with the same
collision shapes cost nothing: 0.90). So the track .sdf files hold static cones, and
`simulation/scripts/cone_activator.py` (started by `dynamic_event.launch.py`) swaps the cones within
9 m of the car for dynamic twins (same name and pose) and back to static, at their current pose, once the
car is 16 m away and they have rested for 1.5 s. Only top-level models can be removed and re-created in
Gazebo, so the cones are top-level models (no `cones` wrapper, hence no `/model/cones/pose`). Without the
activator the cones are immovable. Measured RTF with the activator: 0.90-0.96 (before: 0.92).

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
   pip install -r src/perception/ultralytics_ros/requirements.txt
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
ros2 launch simulation dynamic_event.launch.py event:=trackdrive drive:=torque   # gui:=false: no window
ros2 launch ultralytics_ros predict_with_cloud.launch.xml use_sim_time:=true yolo_model:=conev11n.pt input_topic:=/zed2i/depth_camera/image verbose:=false
ros2 run mission_supervisor lap_counter --ros-args -p use_sim_time:=true
ros2 run slam fastslam_node.py --ros-args -p use_sim_time:=true
ros2 run path_planning pathfinder_slam.py --ros-args -p use_sim_time:=true -p event:=trackdrive
ros2 run control mppi_ros_modified.py --ros-args -p use_sim_time:=true --params-file src/control/config/mppip.yaml
ros2 run mission_supervisor mission_control --ros-args -p use_sim_time:=true -p event:=trackdrive
```

`pathfinder_slam.py` is the planner used with this SLAM pipeline (plans in base_link from `/slam/*`, publishes an
odom-frame `/path`; event-aware, skidpad and recovery logic). `pathfinder.py` is the local-frame `track_pathfinder`
used by `tmux/startup.sh`. `mppi_ros_modified.py` follows the `frame_id` of `/path`, so it works with either.

**Under the team launcher.** `SLAM=1 CAN=0 CONTROLLER=mppi tmux/startup.sh` (env `EVENT=...`) starts this pipeline from dev's
`tmux/startup.sh`: FastSLAM + `pathfinder_slam.py` instead of `track_pathfinder`, `lap_counter` and `mission_control` (with
`CAN=1` the mission supervisor ends the event instead). Without `SLAM=1` the script is unchanged. On WSL export
`GALLIUM_DRIVER=d3d12 MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA GZ_IP=127.0.0.1 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` first.
For `EVENT=acceleration` it also passes `control/config/mppip_acceleration.yaml` (the track's cones end before the finish line).

Events: `trackdrive` (track_small, 10 laps), `mppi_track`, `autocross` (drive1, FSG 2019, 1 lap),
`acceleration`, `skidpad`. Use the same `event` for the simulation, pathfinder and mission_control.

`predict_with_cloud_node` reads its parameters (topics, cluster sizes) from
`perception/ultralytics_ros/config/predict_with_cloud_node.yaml`, which `main()` loads with
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
VLP-16 at real resolution) and dynamic cones, headless, WSL VM with 4 cores. Judged against
`/ground_truth/odom` (2026-10-29, vendored fastslam64, freeze-time landmark merge).

| event | result |
|---|---|
| trackdrive (track_small, 98 m) | 10 laps then stop, ~14 s laps (6.8 m/s mean). 4 of 5 runs clean; 1 run froze a map with extra landmarks, the planner then found no path and the car left the track (see limits) |
| autocross (drive1, 242 m) | 1 lap then stop in 46 s, never more than 1.4 m from the centreline (half width 1.9 m) |
| acceleration (75 m) | 6.8 s from launch to the line (13.2 m/s there), within 0.26 m of the lane centre, stops 27.6 m after the line |
| skidpad | finishes and stops; right and left circles ~8.8 m path radius (finite-difference estimate; rules 9.125 m) |
| mppi_track (372 m, 3 m wide) | 10 laps then stop, 45-46 s laps (8.1 m/s). Clean runs: 0-0.6 % of the time > 1.5 m off the centreline, max 1.3-2.1 m. One run froze a 331-landmark map (292 real cones): 3.9 %, max 3.4 m |

The car's centre passes within 0.5 m of the original position of 13-54 cones per 10-lap mppi_track
run (the car is ~1.1 m wide with the wheels): it clips cones on tight corners at 8-9 m/s.

## 6. Known limits

- **Frozen-map quality (FastSLAM):** the map frozen after lap 2 is sometimes warped or has extra
  landmarks (74 for 72 cones in the bad trackdrive run; 331 for 292 on mppi_track). Same-colour
  landmarks within 0.7 m are merged at the freeze (`endurance.MERGE_RADIUS`), which removes the
  plain duplicates (2-4 per run), but most remaining "ghosts" are real cones displaced > 0.6 m by
  loop-closure drift and have a high existence count (48-52), so filtering on it does not help.
  Roughly 1 in 5 runs freezes a map bad enough to hurt. Ideas: freeze only after a map-stability
  check, or un-freeze and re-map when the planner finds no path for several seconds after the freeze.
  `FSAI_MAP_DUMP=/path/map.csv` makes the node write the frozen map (x, y, type, existence count).
- **Cone physics:** a dynamic cone can wedge under the low tub (~0.07 m ground clearance): one
  mppi_track run stopped dead on yellow cone (44.91, 24.01) at lap 1 with the wheels still driving.
  Not reproduced in 3 further runs (about 1 in 9 runs).
- **FastSLAM process noise:** `sim_config.yaml` uses the measured sim odometry noise (it maps fine). The
  car's `real_config.yaml` ([0.02, 0.01], a 0.1 rad/s yaw-rate error) still maps badly with 512
  particles (per the notes in `sim_config.yaml`; not re-tested after vendoring). That points at the real
  odometry noise, to be checked on the car.
- **fastslam64** is vendored in `slam/slam/fastslam64` (see its README): the library bugs found in
  the 2026-10-27 report (EKF bearing residual, colour-blind association, linear heading mean) are
  fixed in its source, not patched at run time.
- **Estimated vehicle numbers** (check against the car): speed loop gains, brake torque at 100 %,
  steering rate (1 rad/s), rolling resistance, CdA, regenerative braking.
- **VLP-16 blind band:** no ring hits a 0.325 m cone between ~10.5 and ~16 m (0.6 m mount, 2 deg
  rings); perception falls back to the camera up to 13 m.
- WSL gives the VM a share of the laptop's cores/RAM (`.wslconfig`). With 4 cores the Gazebo GUI
  costs ~half a core and control quality drops when the real-time factor falls below ~0.5
  (e.g. while recording bags): use `gui:=false` for long tests.
