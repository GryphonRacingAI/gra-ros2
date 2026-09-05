# bringup
Handles sensor start-up and configuration

## Installation
1. **Install ZED 2i packages**
   - Install [zed-ros2-wrapper](https://github.com/stereolabs/zed-ros2-wrapper)

    Notes: 
    - [An NVIDIA® GPU with Compute Capabilities >= 5.3 based on Pascal or more recent micro-architecture is required.](https://www.stereolabs.com/docs/development/zed-sdk/specifications) 
    - Select the ZED SDK with CUDA and TensorRT versions compatible with your specific GPU.

2. **Install VLP-16 packages**
   ```bash
   sudo apt install ros-${ROS_DISTRO}-velodyne
   ```

## Configuration

### ZED 2i Configuration
**IMPORTANT:**
If you are running any of the NEURAL depth modes on a new device, optimization will take time so **Do not use NEURAL depth modes at dynamic events unless it is already optimized for the compute platform** due to the limited time per run.
- **Config files location:** `zed_wrapper/config/`
  - `common_stereo.yaml` - General stereo camera settings
  - `zed2i.yaml` - ZED 2i specific parameters

### VLP-16 Configuration
**IMPORTANT:** Use `VLP16_hires_db.yaml` instead of `VLP16db.yaml` in the calibration params in `velodyne/launch/velodyne-all-nodes-VLP16-launch.py` since we are using the VLP-16 Hi-Res, which has a narrower VFOV. Otherwise, the LiDAR point cloud would appear angularly stretched vertically compared to the ZED 2i's point cloud, even when perfectly aligned.

- **Config files locations:**
  - `$(ros2 pkg prefix velodyne_driver --share)/config/VLP16-velodyne_driver_node-params.yaml` - Driver parameters
  - `$(ros2 pkg prefix velodyne_pointcloud --share)/config/VLP16-velodyne_transform_node-params.yaml` - Transform parameters
     - `view_width` can be set to any angle (0.0 to 2*pi), and all points outside the specified range will be clipped. 
  - `$(ros2 pkg prefix velodyne_laserscan --share)/config/default-velodyne_laserscan_node-params.yaml` - Laser scan parameters
- **Launch file:** `$(ros2 pkg prefix velodyne --share)/launch/velodyne-all-nodes-VLP16-launch.py`

    **Note:** If this is the first time setting up the VLP-16 LiDAR on a new device, follow the [ROS Velodyne tutorial](https://wiki.ros.org/velodyne/Tutorials/Getting%20Started%20with%20the%20Velodyne%20VLP16)

Refer to the individual config files for detailed parameter descriptions and default values.

## Usage
1. **Source your virtual environment (for the ZED python dependencies)**
2. **Launch sensor bringup:**
   ```bash
   ros2 launch bringup sensor_bringup.launch.py
   ```

## Startup script setup

This file is **Jetson / car boot** (`can0`, no Gazebo). Gazebo + virtual CAN on a desktop is [`tmux/startup.sh`](../tmux/README.md), not `startup_tmux.sh`.

Steps to run startup_tmux.sh on boot:

1. Make the script executable
```bash
chmod +x ~/colcon_ws/src/bringup/startup_tmux.sh
```

2. Create systemd service `/etc/systemd/system/startup_tmux.service` (replace USERNAME with your username)
```ini
[Unit]
Description=Run tmux startup script at boot
After=graphical.target network.target
Wants=graphical.target

[Service]
Type=simple
User=USERNAME
ExecStart=/home/USERNAME/colcon_ws/src/bringup/startup_tmux.sh
Environment=DISPLAY=:0
Environment=XAUTHORITY=/home/user/.Xauthority
RemainAfterExit=yes

[Install]
WantedBy=graphical.target
```

3. Enable and start
```bash
sudo systemctl daemon-reload
sudo systemctl enable startup_tmux.service
sudo systemctl start startup_tmux.service
```

4. Verify (now and after rebooting)
```bash
tmux attach -t ros_startup
```

## Sensors

| Sensor | Model | Datasheet |
|--------|-------|-----------|
| Stereo Camera | ZED 2i 4mm (with polarizer) | [ZED 2i Datasheet](https://cdn.sanity.io/files/s18ewfw4/staging/c059860f8fe49f3856f6b8da770eb13cc543ac2c.pdf/ZED%202i%20Datasheet%20v1.2.pdf) |
| LiDAR | VLP-16 Hi-Res | [VLP-16 Hi-Res Datasheet](https://www.mapix.com/wp-content/uploads/2018/07/63-9318_Rev-E_Puck-Hi-Res_Datasheet_Web.pdf) |






