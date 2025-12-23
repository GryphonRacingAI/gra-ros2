# Launches ZED 2i through zed_wrapper and velodyne through velodyne-all...
# run with the zed_env virtual environment sourced

import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import TextSubstitution

def generate_launch_description():

    # if you want to change zed camera configs,
    # common_stereo.yaml or zed2i.yaml in zed_wrapper/config

    # NOTE: if running on a new device and you are using any of the NERUAL depth modes in common_stereo.yaml, it will take a while to optimize
    # do not use any NEURAL depth mode at the dynamic events because we have only a limited time per run. 
    # that is unless the model is already optimized for the compute platform on the vehicle so that the camera can start immediately.

    # if you want to change velodyne configs,
    # VLP16-velodyne_driver_node-params.yaml in velodyne_driver/config
    # or 
    # VLP16-velodyne_transform_node-params.yaml in velodyne_pointcloud/config
    # or 
    # default-velodyne_laserscan_node-params.yaml in velodyne_laserscan/config
    # or
    # velodyne-all-nodes-VLP16-launch.py in velodyne/launch (IMPORTANT: make sure to use VLP_hires_db.yaml instead of VLP16_db.yaml in calibration params)

    # NOTE: If this is your first time using the VLP16 on this device, follow the instructions in the link below.
    # https://wiki.ros.org/velodyne/Tutorials/Getting%20Started%20with%20the%20Velodyne%20VLP16

    
    zed_wrapper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('zed-ros2-wrapper'), 'launch'),
                '/zed_camera.launch.py']),
        launch_arguments={'camera_model': 'zed2i', 'serial_number': '36485776'}.items(),
    )

    velodyne_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('velodyne'), 'launch'),
                '/velodyne-all-nodes-VLP16-launch.py'])
    )

    return LaunchDescription([
        velodyne_launch,
        zed_wrapper_launch
    ])
