# Launches the ros_gz_bridge (parameter_bridge) directly as a Node.
# This is more robust on Humble setups where the installed ros_gz_bridge
# package may not provide the full Python launch action (ros_gz_bridge.py).

import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    config_file = os.path.join(get_package_share_directory('simulation'),
                               'config/ros_gz_bridge.yaml')

    bridge_name_launch_arg = DeclareLaunchArgument(
        "bridge_name", default_value=TextSubstitution(text="ros_gz_bridge")
    )
    config_file_launch_arg = DeclareLaunchArgument(
        "config_file", default_value=TextSubstitution(text=config_file)
    )
    log_level_launch_arg = DeclareLaunchArgument(
        "log_level", default_value=TextSubstitution(text="info")
    )

    # Directly launch the C++ parameter_bridge executable.
    # This avoids any dependency on the system's ros_gz_bridge Python launch actions.
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name=LaunchConfiguration('bridge_name'),
        output='screen',
        parameters=[{'config_file': LaunchConfiguration('config_file')}],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
    )

    return LaunchDescription([
        bridge_name_launch_arg,
        config_file_launch_arg,
        log_level_launch_arg,
        ros_gz_bridge
    ])
