# Headless MPPI Simulation System

This system enables automated testing of MPPI control algorithms in headless Gazebo simulations with configurable speed, automatic failure detection, and comprehensive statistical analysis.

## Features

- **Headless Simulation**: Run Gazebo without GUI for faster execution
- **Configurable Simulation Speed**: Run at 2x, 5x, or any desired speed multiplier
- **Automatic Failure Detection**: Stops when vehicle velocity = 0 for 5+ seconds
- **Statistical Tracking**: 
  - Collision count with cones
  - Laps completed around track
  - Average/max speed
  - Total distance traveled
  - Run duration
- **Results Output**:
  - JSON file with detailed statistics and MPPI parameters
  - PNG trajectory visualization (top-down track view)
- **Batch Execution**: Run multiple sequential tests automatically

## Quick Start

### Single Headless Run

```bash
# Source your workspace
source /home/prabo/colcon_ws/install/setup.bash

# Launch headless simulation (default 2x speed)
ros2 launch simulation headless_mppi_test.launch.py

# With custom parameters
ros2 launch simulation headless_mppi_test.launch.py \
  sim_speed:=5.0 \
  results_dir:=/tmp/my_mppi_results \
  velocity_timeout:=10.0
```

### Batch Runs

```bash
# Run 5 sequential tests at 2x speed
python3 /home/prabo/colcon_ws/src/simulation/scripts/run_headless_batch.py \
  --num-runs 5 \
  --sim-speed 2.0 \
  --results-dir /tmp/mppi_batch_results

# Full options
python3 /home/prabo/colcon_ws/src/simulation/scripts/run_headless_batch.py \
  --num-runs 10 \
  --sim-speed 5.0 \
  --results-dir /tmp/mppi_results \
  --track mppi_track \
  --delay 3.0
```

## Launch File Parameters

### `headless_mppi_test.launch.py`

| Parameter | Default | Description |
|-----------|---------|-------------|
| `sim_speed` | `2.0` | Simulation real-time factor (2.0 = 2x speed) |
| `results_dir` | `/tmp/mppi_results` | Directory for results and visualizations |
| `track_name` | `mppi_track` | Track identifier for results |
| `velocity_timeout` | `5.0` | Seconds of zero velocity before ending run |
| `headless` | `true` | Run Gazebo without GUI |
| `autostart` | `true` | Auto-start simulation |
| `inner_cones_csv` | (auto) | Path to inner cones CSV |
| `outer_cones_csv` | (auto) | Path to outer cones CSV |

## Output Files

### JSON Results

Location: `{results_dir}/mppi_test_YYYYMMDD_HHMMSS.json`

```json
{
  "run_id": "mppi_test_20260316_201530",
  "timestamp": "2026-03-16T20:15:30Z",
  "track": "mppi_track",
  "mppi_params": {
    "H": 12,
    "K": 500,
    "LAMBDA": 2.0,
    "W_PATH": 40.0,
    "W_HEADING": 5.0,
    "W_SPEED": 2.0,
    "W_CONTROL": 4.5,
    "W_TERMINAL": 1.0,
    "W_OBSTACLE": 150.0,
    "MAX_A": 3.0,
    "MIN_V": -1.0,
    "MAX_V": 8.0,
    "MAX_STEER": 0.5236
  },
  "results": {
    "collisions": 2,
    "laps_completed": 1.5,
    "avg_speed_mps": 4.2,
    "max_speed_mps": 7.1,
    "total_distance_m": 125.3,
    "duration_s": 45.2,
    "failure_reason": "velocity_zero_timeout",
    "final_position": [12.5, 38.2]
  },
  "trajectory_file": "mppi_test_20260316_201530_trajectory.png"
}
```

### Trajectory Visualization

Location: `{results_dir}/mppi_test_YYYYMMDD_HHMMSS_trajectory.png`

Top-down view showing:
- Track outline (inner/outer cones)
- Vehicle trajectory (colored by speed)
- Start position (green circle)
- Final position (red square)
- Collision points (red X markers)

### Batch Summary

Location: `{results_dir}/batch_summary_YYYYMMDD_HHMMSS.json`

Aggregated statistics across all runs including:
- Average and standard deviation for laps, collisions, distance
- Maximum speeds achieved
- Failure reason distribution

## Monitoring Topics

While simulation is running, you can monitor:

```bash
# Simulation status updates
ros2 topic echo /sim/status

# MPPI parameters
ros2 topic echo /mppi/parameters

# Vehicle odometry
ros2 topic echo /odom
```

## Comparison with Manual Testing

### Before (Manual)
```bash
# Terminal 1
ros2 launch simulation dynamic_event.launch.py autostart:=true event:=mppi_track

# Terminal 2
ros2 run control mppi_ros_modified.py --ros-args \
  -p inner_cones_csv:=/home/prabo/colcon_ws/src/simulation/tracks/mppi_track/inner_cones.csv \
  -p outer_cones_csv:=/home/prabo/colcon_ws/src/simulation/tracks/mppi_track/outer_cones.csv \
  -p test_mode:=static_test

# Wait for collision, manually observe results
```

### Now (Automated)
```bash
# Single command for batch testing
python3 /home/prabo/colcon_ws/src/simulation/scripts/run_headless_batch.py \
  --num-runs 10 --sim-speed 5.0

# Automatic results in JSON + visualizations
# 10 runs complete in fraction of the time
```

## Troubleshooting

### Simulation doesn't start
- Ensure Gazebo is not already running: `killall gz`
- Check that workspace is sourced: `source install/setup.bash`

### No results generated
- Check results directory permissions
- Verify monitor node is running: `ros2 node list | grep headless_sim_monitor`

### Simulation runs forever
- Adjust `velocity_timeout` parameter
- Check that vehicle is actually moving (view `/odom` topic)

### Python import errors
- Install matplotlib: `pip3 install matplotlib`
- Install numpy: `pip3 install numpy`

## Advanced Usage

### Custom Track Testing

```bash
ros2 launch simulation headless_mppi_test.launch.py \
  inner_cones_csv:=/path/to/custom_inner.csv \
  outer_cones_csv:=/path/to/custom_outer.csv \
  track_name:=custom_track
```

### Parameter Tuning Workflow

1. Modify MPPI parameters in `mppi_ros_modified.py`
2. Run batch test: `python3 run_headless_batch.py --num-runs 20 --sim-speed 5.0`
3. Analyze results JSON files
4. Iterate on parameters

### Viewing Results

```python
import json
from pathlib import Path

results_dir = Path('/tmp/mppi_results')
for result_file in results_dir.glob('mppi_test_*.json'):
    with open(result_file) as f:
        data = json.load(f)
        print(f"{data['run_id']}: {data['results']['laps_completed']:.1f} laps, "
              f"{data['results']['collisions']} collisions")
```

## Architecture

### Components

1. **headless_sim_monitor.py**: Monitors vehicle state, detects failures, saves results
2. **results_manager.py**: Utility for JSON output and matplotlib visualizations
3. **run_headless_batch.py**: Orchestrates multiple sequential runs
4. **headless_mppi_test.launch.py**: Launch file with all necessary nodes

### Data Flow

```
Gazebo (headless) → /odom → headless_sim_monitor
                            ↓
MPPI Controller → /mppi/parameters → headless_sim_monitor
                            ↓
                    Results JSON + Visualization
                            ↓
                    Batch Summary (if batch mode)
```

## Performance Notes

- **2x speed**: Stable, recommended for accurate results
- **5x speed**: Fast testing, may have physics instabilities
- **10x+ speed**: Very fast but physics may be unreliable

Test at different speeds to find optimal balance for your use case.
