#!/usr/bin/env python3
import os
from pathlib import Path

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.actions import SetLaunchConfiguration, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import IfElseSubstitution, LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    simulation_share = get_package_share_directory('simulation')

    inner_cones_csv_default = os.path.join(simulation_share, 'tracks', 'mppi_track', 'inner_cones.csv')
    outer_cones_csv_default = os.path.join(simulation_share, 'tracks', 'mppi_track', 'outer_cones.csv')

    model_file = os.path.join(simulation_share, 'models', 'vehicle', 'ads_dv.sdf')
    with open(model_file, 'r') as infp:
        robot_desc = infp.read()

    model_file_launch_arg = DeclareLaunchArgument(
        'model_file', default_value=TextSubstitution(text=model_file)
    )
    name_launch_arg = DeclareLaunchArgument(
        'name', default_value=TextSubstitution(text='ads_dv')
    )
    autostart_launch_arg = DeclareLaunchArgument(
        'autostart', default_value=TextSubstitution(text='true')
    )
    verbosity_launch_arg = DeclareLaunchArgument(
        'verbosity', default_value=TextSubstitution(text='1')
    )
    headless_launch_arg = DeclareLaunchArgument(
        'headless', default_value=TextSubstitution(text='true')
    )
    world_launch_arg = DeclareLaunchArgument(
        'world', default_value=TextSubstitution(text='map')
    )
    map_file_launch_arg = DeclareLaunchArgument(
        'map_file', default_value=TextSubstitution(text='mppi_track.sdf')
    )
    inner_cones_csv_launch_arg = DeclareLaunchArgument(
        'inner_cones_csv', default_value=TextSubstitution(text=inner_cones_csv_default)
    )
    outer_cones_csv_launch_arg = DeclareLaunchArgument(
        'outer_cones_csv', default_value=TextSubstitution(text=outer_cones_csv_default)
    )
    sim_speed_launch_arg = DeclareLaunchArgument(
        'sim_speed', default_value=TextSubstitution(text='2.0'),
        description='Simulation real-time factor (e.g., 2.0 for 2x speed)'
    )
    results_dir_launch_arg = DeclareLaunchArgument(
        'results_dir', default_value=TextSubstitution(text='/tmp/mppi_results'),
        description='Directory to save results and visualizations'
    )
    track_name_launch_arg = DeclareLaunchArgument(
        'track_name', default_value=TextSubstitution(text='mppi_track')
    )
    velocity_timeout_launch_arg = DeclareLaunchArgument(
        'velocity_timeout', default_value=TextSubstitution(text='5.0'),
        description='Seconds of zero velocity before ending simulation'
    )

    set_autostart = SetLaunchConfiguration(
        name='autostart_flag',
        value=IfElseSubstitution(
            LaunchConfiguration('autostart'),
            if_value='-r',
            else_value='',
        ),
    )
    set_headless = SetLaunchConfiguration(
        name='headless_flag',
        value=IfElseSubstitution(
            LaunchConfiguration('headless'),
            if_value='-s',
            else_value='',
        ),
    )

    set_spawn_pose = GroupAction(
        [
            SetLaunchConfiguration(name='x', value='7.5'),
            SetLaunchConfiguration(name='y', value='44.0'),
            SetLaunchConfiguration(name='z', value='0.0'),
            SetLaunchConfiguration(name='R', value='0.0'),
            SetLaunchConfiguration(name='P', value='0.0'),
            SetLaunchConfiguration(name='Y', value='0.0'),
        ],
        scoped=False,
    )

    set_gz_args = SetLaunchConfiguration(
        name='gz_args',
        value=[
            LaunchConfiguration('map_file'),
            ' ',
            LaunchConfiguration('headless_flag'),
            ' ',
            LaunchConfiguration('autostart_flag'),
            ' ',
            '-v ',
            LaunchConfiguration('verbosity'),
        ],
    )

    ros_gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory('ros_gz_sim'), 'launch'),
                '/gz_sim.launch.py',
            ]
        ),
        launch_arguments={'gz_args': LaunchConfiguration('gz_args')}.items(),
    )

    spawn_vehicle = Node(
        package='ros_gz_sim',
        executable='create',
        name='create_node',
        output='both',
        parameters=[
            {'world': LaunchConfiguration('world')},
            {'file': 'ads_dv.sdf'},
            {'name': LaunchConfiguration('name')},
            {'x': LaunchConfiguration('x')},
            {'y': LaunchConfiguration('y')},
            {'z': LaunchConfiguration('z')},
            {'R': LaunchConfiguration('R')},
            {'P': LaunchConfiguration('P')},
            {'Y': LaunchConfiguration('Y')},
        ],
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_desc},
        ],
    )

    initial_map_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='initial_map_odom_tf',
        arguments=[
            '--x', '0',
            '--y', '0',
            '--z', '0',
            '--roll', '0',
            '--pitch', '0',
            '--yaw', '0',
            '--frame-id', LaunchConfiguration('world'),
            '--child-frame-id', 'odom',
        ],
    )

    ros_gz_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory('simulation'), 'launch'),
                '/ros_gz_bridge.launch.py',
            ]
        )
    )

    ackermann_to_speed_steer_node = Node(
        package='simulation',
        executable='ackermann_to_speed_steer',
        name='ackermann_to_speed_steer_node',
        output='screen',
        parameters=[
            {'speed_cmd_topic': '/speed_cmd'},
            {'steer_cmd_topic': '/steer_angle_cmd'},
            {'steer_angle_topic': '/steer_angle'},
            {'ackermann_cmd_topic': '/ackermann_cmd'},
            {'joint_states_topic': '/joint_states'},
        ],
    )

    mppi_controller = Node(
        package='control',
        executable='mppi_ros_modified.py',
        name='mppi_controller',
        output='screen',
        parameters=[
            {
                'use_sim_time': True,
                'test_mode': 'static_test',
                'inner_cones_csv': LaunchConfiguration('inner_cones_csv'),
                'outer_cones_csv': LaunchConfiguration('outer_cones_csv'),
                'path_topic': '/path',
            }
        ],
    )

    headless_sim_monitor = Node(
        package='simulation',
        executable='headless_sim_monitor.py',
        name='headless_sim_monitor',
        output='screen',
        parameters=[
            {
                'use_sim_time': True,
                'results_dir': LaunchConfiguration('results_dir'),
                'inner_cones_csv': LaunchConfiguration('inner_cones_csv'),
                'outer_cones_csv': LaunchConfiguration('outer_cones_csv'),
                'track_name': LaunchConfiguration('track_name'),
                'velocity_timeout': LaunchConfiguration('velocity_timeout'),
                'min_velocity_threshold': 0.1,
            }
        ],
    )

    set_sim_speed = ExecuteProcess(
        cmd=[
            'gz', 'physics',
            '-u',
            '--real-time-factor', LaunchConfiguration('sim_speed')
        ],
        output='screen',
        shell=False,
    )

    return LaunchDescription(
        [
            model_file_launch_arg,
            name_launch_arg,
            autostart_launch_arg,
            verbosity_launch_arg,
            headless_launch_arg,
            world_launch_arg,
            map_file_launch_arg,
            inner_cones_csv_launch_arg,
            outer_cones_csv_launch_arg,
            sim_speed_launch_arg,
            results_dir_launch_arg,
            track_name_launch_arg,
            velocity_timeout_launch_arg,
            set_autostart,
            set_headless,
            set_spawn_pose,
            set_gz_args,
            ros_gz_sim,
            spawn_vehicle,
            robot_state_publisher,
            initial_map_odom_tf,
            ros_gz_bridge,
            ackermann_to_speed_steer_node,
            mppi_controller,
            headless_sim_monitor,
            set_sim_speed,
        ]
    )
