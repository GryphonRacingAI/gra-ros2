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

| Parameter | Description | Options | Default |
|----------|-------------|---------|---------|
| event | specifies which dynamic event to plan for | `acceleration`, `skidpad`, `autocross`, `trackdrive` | `trackdrive` |

## Interface

| Node | Inputs | Outputs | Description |
|------|-------------|---------|---------|
| `pathfinder.py` | `/track_map` (`common_msgs/ConeArray`)<br>`/odom` (`nav_msgs/Odometry`) | `/path` (`nav_msgs/Path`) | Processes the track cone map and the vehicle pose to generate the centreline for different dynamic events |
