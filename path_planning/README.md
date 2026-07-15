# path_planning
Generates a body-relative track centreline from cone detections using [`fsd_path_planning`](https://github.com/GryphonRacingAI/ft-fsd-path-planning).

## Installation
1. Create and/or source your virtual environment if you haven't already:

```bash
cd ~/colcon_ws
python3 -m venv ros_venv
source ros_venv/bin/activate
```

2. Install the forked path planner:

```bash
pip install git+https://github.com/GryphonRacingAI/ft-fsd-path-planning.git
```

3. Build the package:

```bash
cd ~/colcon_ws
colcon build --packages-select path_planning
source install/setup.bash
```

## Usage

```bash
ros2 run path_planning pathfinder.py --ros-args \
  --params-file $(ros2 pkg prefix path_planning)/share/path_planning/config/pathfinder_params.yaml
```

Override a single parameter:

```bash
ros2 run path_planning pathfinder.py --ros-args \
  --params-file $(ros2 pkg prefix path_planning)/share/path_planning/config/pathfinder_params.yaml \
  -p event:=trackdrive
```

In simulation, also set `use_sim_time:=true`.

### Node Parameters

ROS defaults are in `config/pathfinder_params.yaml` (node name `track_pathfinder`). Those values match the library defaults from [`fsd_path_planning/config.py`](https://github.com/GryphonRacingAI/ft-fsd-path-planning/blob/main/fsd_path_planning/config.py) (`get_cone_sorting_config`, `get_default_matching_kwargs`, `get_path_calculation_config`, `get_cone_fitting_config`). Angle parameters are exposed in degrees here and converted to radians before being passed into `PathPlanner`.

#### PathPlanner

| Parameter | Description | Default |
|-----------|-------------|---------|
| `event` | Mission type: `acceleration`, `skidpad`, `autocross`, `trackdrive` | `trackdrive` |
| `experimental_performance_improvements` | Faster cone sorting (experimental); also passed into `cone_sorting` | `false` |

#### cone_sorting (`ConeSorting`)

| Parameter | Description | Default |
|-----------|-------------|---------|
| `max_n_neighbors` | Max neighbors per cone during sorting | `5` |
| `max_dist` | Max neighbor distance (m) | `6.5` |
| `max_dist_to_first` | Max distance to first cone (m) | `6.0` |
| `max_length` | Max sorted trace length | `12` |
| `threshold_directional_angle_deg` | Directional angle threshold (deg) | `40.0` |
| `threshold_absolute_angle_deg` | Absolute angle threshold (deg) | `65.0` |
| `use_unknown_cones` | Include unknown-color cones in sorting | `true` |

#### cone_matching (`ConeMatching`)

| Parameter | Description | Default |
|-----------|-------------|---------|
| `min_track_width` | Min track width for matching (m) | `3.0` |
| `max_search_range` | Max match search range (m) | `5.0` |
| `max_search_angle_deg` | Max match search angle (deg) | `50.0` |
| `matches_should_be_monotonic` | Require monotonic left/right matches | `false` |

#### pathing (`CalculatePath`)

| Parameter | Description | Default |
|-----------|-------------|---------|
| `maximal_distance_for_valid_path` | Max car-to-path distance for a valid update (m) | `5.0` |
| `mpc_path_length` | Desired path length (m) | `20.0` |
| `mpc_prediction_horizon` | Number of path waypoints | `40` |
| `smoothing` | Cone-boundary spline smoothing | `0.2` |
| `predict_every` | Spline sample spacing (m) | `0.1` |
| `max_deg` | Max spline degree | `3` |

## Interface

| Node | Inputs | Outputs | Description |
|------|--------|---------|-------------|
| `track_pathfinder` (`pathfinder.py`) | `/cone_array` (`common_msgs/ConeArray`)<br>`/odom` (`nav_msgs/Odometry`) | `/path` (`nav_msgs/Path`, `frame_id=velodyne`) | Centreline in body frame (car at origin, +x forward) |
