# Pre-requisites
### Python virtual environment setup if you haven't already

```bash
cd ~/colcon_ws
python3 -m venv ros_venv
source ros_venv/bin/activate
```

### Python Dependency installations
> Source your `ros_venv` before running
```
pip install -r "requirements.txt"
```

### Gazebo (sim)

The tmux stack (`CONES=yolo`) launches:

```bash
ros2 launch ultralytics_ros predict_with_cloud.launch.xml \
  use_sim_time:=true sim:=true device:=cuda:0 yolo_model:=conev11n.pt
```

`sim:=true` sets ZED image + Velodyne cloud + Gazebo camera convention. Do not rely on the launch-file defaults (`/image_raw`, `/points_raw`) in simulation. See [`tmux/README.md`](../../../tmux/README.md).
