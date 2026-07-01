# path_planning
Generates the centreline of the track based on the selected event

## Installation
1. Create and/or source your virtual environment if you haven't already

```bash
cd ~/colcon_ws
python3 -m venv ros_venv
source ros_venv/bin/activate
```

## Usage
1. Source your virtual environment
2. Run the path planning node:
    ```bash
    ros2 run path_planning pathfinder.py --ros-args -p event:=trackdrive
    ```

### Node Parameters
The following parameters are provided for `pathfinder.py`:

| Parameter | Description | Default |
|----------|-------------|---------|
| `output_frame` | Frame for published `/path` | `odom` |
| `cone_map_topic` | SLAM cone map input topic | `/slam/cone_map` |
| `lookahead_distance` | Max forward cone distance (m) | `20.0` |
| `test_mode` | Publish `/viz/path` and `/viz/path_markers` for RViz | `true` |
| `viz_frame` | Frame for test-mode visualization topics | `map` |
| `marker_scale` | Waypoint sphere size in test mode | `0.25` |
| `line_width` | Path line width in test mode | `0.15` |

In simulation, run with `use_sim_time:=true` and set RViz **Fixed Frame** to `map`. Test-mode viz is published in `map` so it aligns with `/slam/cone_map_markers` without needing an `odom`→`map` lookup.

```bash
ros2 run path_planning pathfinder.py --ros-args -p use_sim_time:=true
rviz2 --ros-args -p use_sim_time:=true
```

## Interface

| Node | Inputs | Outputs | Description |
|------|-------------|---------|---------|
| `pathfinder.py` | `/slam/cone_map` (`common_msgs/ConeArray`)<br>`/slam/pose` (`geometry_msgs/PoseStamped`)<br>`/odom` (`nav_msgs/Odometry`) | `/path` (`nav_msgs/Path`)<br>`/viz/path` (`nav_msgs/Path`, test mode)<br>`/viz/path_markers` (`visualization_msgs/MarkerArray`, test mode) | Centreline from SLAM cone map; test-mode viz mirrors `perfect_path` markers |
