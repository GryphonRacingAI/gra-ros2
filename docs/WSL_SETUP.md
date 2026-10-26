# Running the simulated pipeline on Windows (WSL2)

Tested on WSL2 Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic, RTX 5060 laptop GPU.
The pipeline is the same as on native Ubuntu:

```
Gazebo (ads_dv + track) -> perception (YOLO + lidar clustering) -> /cone_array
  -> lap_counter (/laps) -> FastSLAM (/slam/pose, /slam/cone_map, /slam/cones)
  -> pathfinder (/path) -> MPPI (/ackermann_cmd) -> ackermann_to_speed_steer -> Gazebo
```

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
ros2 launch simulation dynamic_event.launch.py event:=trackdrive        # add gui:=false for no window
ros2 launch ultralytics_ros predict_with_cloud.launch.xml use_sim_time:=true yolo_model:=conev11n.pt input_topic:=/zed2i/depth_camera/image
ros2 run mission_supervisor lap_counter --ros-args -p use_sim_time:=true
ros2 run slam fastslam_node.py --ros-args -p use_sim_time:=true
ros2 run path_planning pathfinder.py --ros-args -p use_sim_time:=true
ros2 run control mppi_ros_modified.py --ros-args -p use_sim_time:=true --params-file src/control/config/mppi_params.yaml
```

Events: `trackdrive` (track_small), `mppi_track`, `autocross` (drive1, FSG 2019), `acceleration`, `skidpad`.

`predict_with_cloud_node` reads its parameters (topics, cluster sizes) from
`perception/src/ultralytics_ros/config/predict_with_cloud_node.yaml`, which `main()` loads with
higher priority than launch-file parameters. Edit the YAML, not the launch arguments.

## 4. Checking it works

```bash
ros2 topic echo /cone_array --once     # no cones at (0,0,0); several yellow AND blue
ros2 topic hz /path                     # ~10 Hz once SLAM has mapped cones on both sides
ros2 topic echo /odom --field pose.pose.position   # the car moves
ros2 topic echo /laps                   # increments each time the car crosses the start line
```

`ros2 topic hz` under-reports high-rate topics (e.g. `/odom`) when the CPU is busy.

## 5. Known limits

Status per event with the full pipeline (WSL, Gazebo, YOLO + lidar, FastSLAM, pathfinder, MPPI):

| event | result |
|---|---|
| trackdrive (track_small) | laps continuously (5 laps in 7 min, ~70 s/lap); map freezes after lap 2 |
| mppi_track | laps (2 laps in 6 min, ~160 s/lap) |
| acceleration | reaches the finish (76 m in 23.5 s), but nothing stops the car afterwards |
| skidpad | drives to the centre, then stops: the centreline planner has no figure-8 logic |
| autocross (drive1) | not run in Gazebo; offline planner replay on the true cones stops at the top hairpin, where drive1 has no yellow cones along ~15 m of the corner's inside |

- Laps are slow (mean ~1.4 m/s). MPPI only penalises overspeed; nothing in its cost rewards progress.

- WSL gives the VM a share of the laptop's cores/RAM (`.wslconfig`). With 4 cores, the Gazebo
  GUI alone costs ~half a core; use `gui:=false` when you don't need to watch.
- The simulated VLP-16 has 2 degrees between rings, so cones beyond ~10-12 m are often hit by
  no ring and get no 3D position; SLAM builds the map as the car drives.
