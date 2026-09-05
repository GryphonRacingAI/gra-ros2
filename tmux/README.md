# Sim / VCU tmux launchers

These scripts start the ADS-DV stack in a tmux session (one ROS/Gazebo process per window) so you can watch logs while the car, CAN, and planner run.

```bash
cd ~/colcon_ws
./tmux/startup.sh   # usual sim + CAN + YOLO + pathfinder + PP + supervisor
# or, same tree:
./src/tmux/startup.sh
```

This replaces the old gist `run_nodes.sh`. Do not use that gist.

Related overlay helper (not in this directory): `~/colcon_ws/clean_gz.sh` kills leftover Gazebo / stack processes. After it, `ros2 topic list` should show only `/parameter_events` and `/rosout`.

---



## Scripts


| Script | Session | What it starts |
|--------|---------|----------------|
| [startup.sh](startup.sh) | `fsai` (or `$SESSION`) | Full stack: Gazebo, virtual CAN, supervisor, YOLO or `perfect_path`, pathfinder, one controller |
| [log.sh](log.sh) | — | Sourced. `log_setup` mkdir's `~/colcon_ws/logs/<stamp>/`. `run_logged name cmd` is the pane one-liner |
| [gpu.sh](gpu.sh) / [reload_nvidia_uvm.sh](reload_nvidia_uvm.sh) | — | CUDA preflight; `gpu.log` recover recipe. Helper does **not** run sudo |
| [vcu_sim.sh](vcu_sim.sh) | `fsai` | Optional CAN bench (no Gazebo). Not used by `startup.sh` |


Unused stubs that may still sit in an overlay `tmux/` copy: `control_mock.sh`, `e2e_full.sh` (empty). Ignore them.

---



## Quick start (mppi_track + simulated CAN)

**Once per machine**

```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan 2>/dev/null || true
sudo ip link set vcan0 up
```

`ackermann_can` and `vcu_sim.py` bind to that interface.

**Each run**

```bash
cd ~/colcon_ws
source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash
source install/setup.bash
# optional: source ros_venv/bin/activate  (the tmux panes do this themselves)

./tmux/startup.sh
# detach: Ctrl-b d
# reattach:
tmux attach -t fsai
```

Headless / agent:

```bash
DETACH=1 ./tmux/startup.sh
```

**Arm the VCU** (it starts in `AS_OFF`; nothing is driven until `AS_DRIVING`):

In the `vcu` pane, keys (see `[fsai_api](../fsai_api/README.md)`):


| Key      | Meaning                                                 |
| -------- | ------------------------------------------------------- |
| `a`      | ASMS                                                    |
| `t`      | TSMS                                                    |
| `1`–`7`  | AMI mission (`4` = trackdrive, closest to `mppi_track`) |
| wait 5 s | `AS_READY` hold                                         |
| `g`      | RES go → `AS_DRIVING`                                   |
| `e`      | SDC open (E-stop)                                       |
| `r`      | power cycle                                             |
| `q`      | quit `vcu_sim`                                          |


`startup.sh` does **not** pass `--auto-drive`. `vcu_sim.sh` still does, because that bench is for CAN without a mission stack.

---



## Environment (`startup.sh`)


| Variable     | Values                                                             | Default                                      | Meaning                                                                                                                     |
| ------------ | ------------------------------------------------------------------ | -------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------- |
| `EVENT`      | `acceleration`, `skidpad`, `autocross`, `trackdrive`, `mppi_track` | `mppi_track`                                 | Gazebo world (`dynamic_event.launch.py event:=`)                                                                            |
| `CONES`      | `yolo`, `perfect`                                                  | `yolo`                                       | `yolo`: `/cone_array` from perception → pathfinder. `perfect`: `perfect_path` midline, no YOLO                              |
| `CONTROLLER` | `pp`, `mppi`                                                       | `pp`                                         | Pure pursuit or MPPI. Node names: `pure_pursuit_controller` / `mppi_controller`                                             |
| `CAN`        | `1`, `0`                                                           | `1`                                          | `1`: vcan0 + VCU sim + `ackermann_can` + wheels + supervisor. `0`: controller publishes `/ackermann_cmd` straight to Gazebo |
| `CAN_IFACE`  | SocketCAN name                                                     | `vcan0`                                      |                                                                                                                             |
| `SESSION`    | tmux name                                                          | `fsai`                                       |                                                                                                                             |
| `DETACH`     | `1`                                                                | unset (attach if tty)                        |                                                                                                                             |
| `VIZ`        | `1`                                                                | unset                                        | extra `rviz2` window                                                                                                        |
| `WS`         | path                                                               | colcon overlay                               |                                                                                                                             |
| `LOGFILE`    | path                                                               | `$LOG_DIR/startup.log`                       | orchestrator log                                                                                                            |


Examples:

```bash
CONTROLLER=mppi ./tmux/startup.sh
CONES=perfect CONTROLLER=mppi ./tmux/startup.sh
EVENT=trackdrive CONES=yolo ./tmux/startup.sh
CAN=0 CONTROLLER=pp ./tmux/startup.sh
```

`CONES=perfect` with `CONTROLLER=pp` is unsupported (PP is local-frame only; `perfect_path` on `mppi_track` is odom-frame). The script warns.

YOLO always uses `device:=cuda:0` (`predict_with_cloud.launch.xml sim:=true`). There is no CPU fallback.

If `nvidia-smi` works but `torch.cuda.is_available()` is False (`CUDA unknown error`), `nvidia_uvm` is wedged. The launcher writes `$LOG_DIR/gpu.log` and stops before YOLO starts. Recipe:

```bash
./src/tmux/reload_nvidia_uvm.sh    # diagnose, does not run sudo
sudo rmmod nvidia_uvm
sudo modprobe nvidia_uvm
```

`predict_node` is unchanged. Launch respawns it every 5 s so a UVM reload can bring YOLO back without a full stack restart.

---



## Windows (`startup.sh`)


| Window       | Process                                                                                       |
| ------------ | --------------------------------------------------------------------------------------------- |
| `monitor`    | overlay sourced; prints env and watch commands (not teed)                                     |
| `sim`        | `ros2 launch simulation dynamic_event.launch.py autostart:=true event:=$EVENT`                |
| `vcu`        | `vcu_sim.py $CAN_IFACE` (manual keys)                                                         |
| `can`        | `ros2 run fsai_api ackermann_can $CAN_IFACE`                                                  |
| `wheels`     | `wheel_speed_controller.py` (`/ackermann_cmd_controller` → `/ackermann_cmd`)                  |
| `candump`    | `candump -td $CAN_IFACE`                                                                      |
| `supervisor` | `ros2 run mission_supervisor mission_supervisor`                                              |
| `laps`       | `ros2 run mission_supervisor lap_counter`                                                     |
| `vcu2ai`     | `ros2 topic echo --no-arr /vcu2ai`                                                            |
| `yolo`       | `predict_with_cloud.launch.xml use_sim_time:=true sim:=true device:=cuda:0` (if `CONES=yolo`) |
| `path`       | `pathfinder.py` or `perfect_path`                                                             |
| `control`    | PP or MPPI, remapped to `/ackermann_cmd_planner` when `CAN=1`                                 |


Startup waits for `/clock` (default 45 s) after Gazebo before the rest.

---



## Command graph (`CAN=1`)

```
YOLO /cone_array  →  track_pathfinder  →  /path (frame_id=velodyne)
                                            ↓
                              PP or MPPI  →  /ackermann_cmd_planner
                                            ↓
                         mission_supervisor  (forwards only in AS_DRIVING)
                         scripted AMI 5/6/7: supervisor publishes drive itself
                                            ↓
                         /ackermann_cmd_controller
                                            ↓
                         wheel_speed_controller  →  /ackermann_cmd
                                            ↓
                         Gazebo ackermann_to_speed_steer
                         ackermann_can  →  vcan0  →  vcu_sim.py
                                            ↓
                         /vcu2ai  →  supervisor, wheels, vcu2ai echo pane, health
```

Until `AS_DRIVING`, `/ackermann_cmd` stays 0 even if the planner is already commanding `/ackermann_cmd_planner`.

Supervisor parameters the tmux script sets:

- `controller_node:=pure_pursuit_controller` or `mppi_controller`
- `path_node:=track_pathfinder` or `perfect_path`
- `require_perception:=true` unless `CONES=perfect`

Health (`/mission_supervisor/status`) checks those node names plus `lap_counter` for trackdrive/skidpad/autocross.

Details: `[mission_supervisor/README.md](../mission_supervisor/README.md)`, `[control/README.md](../control/README.md)`.

---



## Logging

[`log.sh`](log.sh) is sourced by `startup.sh` (which calls `log_setup`) and again in each pane.

```bash
source tmux/log.sh
log_setup                          # ~/colcon_ws/logs/<stamp>/ + logs/latest
run_logged sim ros2 launch ...     # pane command
```

`run_logged` is one pipeline (extension of `cmd &> file`):

```text
stdbuf -oL -eL cmd 2>&1 | python3 -u -c 'stamp epoch + #n' | tee -a $LOG_DIR/<name>.log
```

Line format (join key is the leading epoch):

```text
1788643378.647200 #85 [INFO] [1788643378.647128135] [track_pathfinder]: ...
```

`/vcu2ai` is `ros2 topic echo --no-arr /vcu2ai` through the same `run_logged` (no extra Python node).

`ROS_LOG_DIR=$LOG_DIR/ros`. `wheels.log` stays empty until AS_DRIVING. `candump.log` grows with the bus. `monitor` is not teed.

---



## Perception launch (`sim:=true`)

`CONES=yolo` runs:

```bash
ros2 launch ultralytics_ros predict_with_cloud.launch.xml \
  use_sim_time:=true sim:=true device:=cuda:0 yolo_model:=conev11n.pt
```

`sim:=true` selects Gazebo topics (not the launch-file defaults `/image_raw`, `/camera_info`, `/points_raw`):

- image `/zed2i/depth_camera/image`
- camera_info `/zed2i/depth_camera/camera_info`
- lidar `/velodyne_points`
- cluster `0.15 / 0.01 / 5 / 70`
- `gz_camera_convention:=true`

On this Humble box, keep `cv_bridge/cv_bridge.h` (not `.hpp`). Jazzy uses `.hpp`.

---



## Cleanup

```bash
cd ~/colcon_ws
./clean_gz.sh
ros2 topic list    # expect /parameter_events and /rosout
tmux ls            # no fsai
```

`clean_gz.sh` skips its own process group so a diagnostic whose cmdline contains `gz sim` is not SIGKILL’d.

---



## Humble vs Jazzy

Team README / Bragg is Jazzy. This overlay sources Jazzy if present, else Humble. `tmux` scripts do the same in every pane.

---



## Jetson vs this overlay


|                | Jetson `startup_tmux.sh`           | These scripts                                            |
| -------------- | ---------------------------------- | -------------------------------------------------------- |
| CAN            | `can0` (real)                      | `vcan0` + `vcu_sim.py`                                   |
| Layout         | split-panes, one window            | one process per **window**                               |
| Node logs      | `&> $LOG_DIR/foo.log` (pane blank) | `tee` (pane + file)                                      |
| `/vcu2ai` file | `ros2 topic echo` + awk/python     | `run_logged vcu2ai ros2 topic echo --no-arr /vcu2ai`     |
| Drive gate     | mission launch on hardware         | `mission_supervisor` relays planner only in `AS_DRIVING` |


Hardware boot script: `[bringup/startup_tmux.sh](../bringup/startup_tmux.sh)` (`can0`, no Gazebo).