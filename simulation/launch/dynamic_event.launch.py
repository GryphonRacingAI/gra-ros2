# Launches Gazebo with specified dynamic event

# Each dynamic event is associated with an vehicle spawn origin and a map

import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import SetLaunchConfiguration, GroupAction, SetEnvironmentVariable, TimerAction
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    TextSubstitution,
    IfElseSubstitution,
    EqualsSubstitution,
    PathJoinSubstitution,
)
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    # vehicle model file (hard-coded, but can be extended to get path from user)
    model_file = os.path.join(get_package_share_directory('simulation'),
                               'models/vehicle/ads_dv.sdf')

    # copy file contents
    with open(model_file, 'r') as infp:
        robot_desc = infp.read()

    # args that can be set from the command line or a default will be used
    event_launch_arg = DeclareLaunchArgument(
        "event", default_value=TextSubstitution(text="acceleration")
    )
    model_file_launch_arg = DeclareLaunchArgument(
        "model_file", default_value=TextSubstitution(text=model_file)
    )
    name_launch_arg = DeclareLaunchArgument(
        "name", default_value=TextSubstitution(text="ads_dv")
    )
    autostart_launch_arg = DeclareLaunchArgument(
        "autostart", default_value=TextSubstitution(text="true")
    )
    verbosity_launch_arg = DeclareLaunchArgument(
        "verbosity", default_value=TextSubstitution(text="1")
    )
    
    set_autostart = SetLaunchConfiguration(
        name='autostart_flag',
        value=IfElseSubstitution(
            LaunchConfiguration("autostart"),
            if_value="-r",
            else_value=""),
    )

    set_acceleration = GroupAction([
        SetLaunchConfiguration(
             name='x',
             value='-51.0'
        ),
        SetLaunchConfiguration(
            name='y',
            value='0.0'
        ),
        SetLaunchConfiguration(
             name='z',
             value='0.0'
        ),
        SetLaunchConfiguration(
            name='R',
            value='0.0'
        ),
        SetLaunchConfiguration(
             name='P',
             value='0.0'
        ),
        SetLaunchConfiguration(
            name='Y',
            value='0.0'
        ),
        SetLaunchConfiguration(
            name='world',
            value='map'
        ),
        SetLaunchConfiguration(
            name='map_file',
            value='acceleration.sdf'
        )
        ], 
        scoped=False,
        condition=IfCondition(
            EqualsSubstitution(LaunchConfiguration('event'), "acceleration")
            )
    )

    set_skidpad = GroupAction([
        SetLaunchConfiguration(
             name='x',
             value='0.0'
        ),
        SetLaunchConfiguration(
            name='y',
            value='-12.0'
        ),
        SetLaunchConfiguration(
             name='z',
             value='0.0'
        ),
        SetLaunchConfiguration(
            name='R',
            value='0.0'
        ),
        SetLaunchConfiguration(
             name='P',
             value='0.0'
        ),
        SetLaunchConfiguration(
            name='Y',
            value='1.57079632679'
        ),
        SetLaunchConfiguration(
            name='world',
            value='map'
        ),
        SetLaunchConfiguration(
            name='map_file',
            value='skidpad.sdf'
        )
        ], 
        scoped=False,
        condition=IfCondition(
            EqualsSubstitution(LaunchConfiguration('event'), "skidpad")
            )
    )

    set_autocross = GroupAction([
        SetLaunchConfiguration(
             name='x',
             value='0.0'
        ),
        SetLaunchConfiguration(
            name='y',
            value='0.0'
        ),
        SetLaunchConfiguration(
             name='z',
             value='0.0'
        ),
        SetLaunchConfiguration(
            name='R',
            value='0.0'
        ),
        SetLaunchConfiguration(
             name='P',
             value='0.0'
        ),
        SetLaunchConfiguration(
            name='Y',
            value='0.0'
        ),
        SetLaunchConfiguration(
            name='world',
            value='map'
        ),
        SetLaunchConfiguration(
            name='map_file',
            value='autocross.sdf'
        )
        ], 
        scoped=False,
        condition=IfCondition(
            EqualsSubstitution(LaunchConfiguration('event'), "autocross")
            )
    )

    set_trackdrive = GroupAction([
        SetLaunchConfiguration(
             name='x',
             value='0.0'
        ),
        SetLaunchConfiguration(
            name='y',
            value='0.0'
        ),
        SetLaunchConfiguration(
             name='z',
             value='0.0'
        ),
        SetLaunchConfiguration(
            name='R',
            value='0.0'
        ),
        SetLaunchConfiguration(
             name='P',
             value='0.0'
        ),
        SetLaunchConfiguration(
            name='Y',
            value='0.0'
        ),
        SetLaunchConfiguration(
            name='world',
            value='map'
        ),
        SetLaunchConfiguration(
            name='map_file',
            value='track_small.sdf'
        )
        ], 
        scoped=False,
        condition=IfCondition(
            EqualsSubstitution(LaunchConfiguration('event'), "trackdrive")
            )
    )

    set_mppi_track = GroupAction([
        SetLaunchConfiguration(
             name='x',
             value='7.5'
        ),
        SetLaunchConfiguration(
            name='y',
            value='46.0'
        ),
        SetLaunchConfiguration(
             name='z',
             value='0.0'
        ),
        SetLaunchConfiguration(
            name='R',
            value='0.0'
        ),
        SetLaunchConfiguration(
             name='P',
             value='0.0'
        ),
        SetLaunchConfiguration(
            name='Y',
            value='0.0'
        ),
        SetLaunchConfiguration(
            name='world',
            value='map'
        ),
        SetLaunchConfiguration(
            name='map_file',
            value='mppi_track.sdf'
        )
        ], 
        scoped=False,
        condition=IfCondition(
            EqualsSubstitution(LaunchConfiguration('event'), "mppi_track")
            )
    )

    # model://simulation/... URIs resolve against the parent of the package share dir
    sim_resource_root = os.path.dirname(get_package_share_directory('simulation'))

    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            sim_resource_root, ':',
            EnvironmentVariable('GZ_SIM_RESOURCE_PATH', default_value='')
        ]
    )

    tracks_dir = os.path.join(
        get_package_share_directory('simulation'), 'models', 'tracks')

    set_gz_args = SetLaunchConfiguration(
        name='gz_args',
        value=[tracks_dir, '/', LaunchConfiguration('map_file'), ' ',
               LaunchConfiguration('autostart_flag'), ' ',
               '-v ', LaunchConfiguration('verbosity')]
    )

    ros_gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('ros_gz_sim'), 'launch'),
                '/gz_sim.launch.py']),
        launch_arguments={'gz_args': LaunchConfiguration('gz_args')}.items(),
        )
    
    spawn_vehicle = Node(
            package="ros_gz_sim",
            executable="create",
            name="spawn_vehicle_node",
            output="both",
            arguments=[
                '-world', LaunchConfiguration('world'),
                '-file',  model_file,
                '-name',  LaunchConfiguration('name'),
                '-x',     LaunchConfiguration('x'),
                '-y',     LaunchConfiguration('y'),
                '-z',     LaunchConfiguration('z'),
                '-R',     LaunchConfiguration('R'),
                '-P',     LaunchConfiguration('P'),
                '-Y',     LaunchConfiguration('Y'),
            ]
    )
    # Other events still spawn via /world/map/create. Delay so the world
    # service exists (immediate create times out on heavy maps).
    # mppi_track embeds ads_dv in the SDF instead — see mppi_track.xacro.
    delayed_spawn = GroupAction(
        [TimerAction(period=12.0, actions=[spawn_vehicle])],
        condition=UnlessCondition(
            EqualsSubstitution(LaunchConfiguration('event'), 'mppi_track')
        ),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_desc},
        ]
    )

    initial_sim_world_odom_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="initial_sim_world_odom_tf",
        arguments=[
            "--x", "0",
            "--y", "0",
            "--z", "0",
            "--roll", "0",
            "--pitch", "0",
            "--yaw", "0",
            "--frame-id", LaunchConfiguration('world'),
            "--child-frame-id", "odom"
        ]
    )

    ros_gz_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('simulation'), 'launch'),
                '/ros_gz_bridge.launch.py']),
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
            {'joint_states_topic': '/joint_states'}
        ]
    )

    return LaunchDescription([
        event_launch_arg,
        model_file_launch_arg,
        name_launch_arg,
        autostart_launch_arg,
        verbosity_launch_arg,

        set_autostart,
        set_acceleration,
        set_skidpad,
        set_autocross,
        set_trackdrive,
        set_mppi_track,

        set_gz_resource_path,
        set_gz_args,
        
        ros_gz_sim,
        delayed_spawn,
        robot_state_publisher,
        initial_sim_world_odom_tf,
        ros_gz_bridge,
        ackermann_to_speed_steer_node
    ])
