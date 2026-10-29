# Launches Gazebo with specified dynamic event

# Each dynamic event is associated with an vehicle spawn origin and a map

import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import SetLaunchConfiguration, GroupAction, SetEnvironmentVariable, OpaqueFunction
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    TextSubstitution,
    IfElseSubstitution,
    EqualsSubstitution,
    PathJoinSubstitution,
)
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro

# AMI (mission) ids as on the ADS-DV, reported by the VCU emulator on /vcu2ai
AMI_STATE = {'acceleration': 1, 'skidpad': 2, 'autocross': 3, 'trackdrive': 4, 'mppi_track': 4}


def vehicle_nodes(context):
    """Vehicle model (xacro, processed with drive:=...), spawner, robot_state_publisher and the
    node that turns /ackermann_cmd into Gazebo commands."""
    drive = LaunchConfiguration('drive').perform(context)
    model_file = LaunchConfiguration('model_file').perform(context)
    if model_file.endswith('.xacro'):
        robot_desc = xacro.process_file(model_file, mappings={'drive': drive}).toxml()
    else:  # a ready-made .sdf
        with open(model_file, 'r') as infp:
            robot_desc = infp.read()

    spawn_vehicle = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_vehicle_node",
        output="both",
        arguments=[
            '-world', LaunchConfiguration('world'),
            '-string', robot_desc,
            '-name',  LaunchConfiguration('name'),
            '-x',     LaunchConfiguration('x'),
            '-y',     LaunchConfiguration('y'),
            '-z',     LaunchConfiguration('z'),
            '-R',     LaunchConfiguration('R'),
            '-P',     LaunchConfiguration('P'),
            '-Y',     LaunchConfiguration('Y'),
        ]
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

    if drive == 'ideal':
        command_node = Node(
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
    else:
        command_node = Node(
            package='simulation',
            executable='ads_dv_vcu_sim',
            name='ads_dv_vcu_sim',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'ami_state': AMI_STATE.get(LaunchConfiguration('event').perform(context), 0),
            }]
        )
    # /odom + odom -> base_link: wheel speeds + IMU like the real car, or Gazebo's exact pose.
    # The ideal drive publishes no /vcu2ai, so it always uses ground truth.
    odom_source = LaunchConfiguration('odom').perform(context)
    if drive == 'ideal':
        odom_source = 'ground_truth'
    odometry = Node(
        package='simulation',
        executable='vcu_odometry.py',
        name='vcu_odometry',
        output='screen',
        parameters=[{'use_sim_time': True, 'source': odom_source}],
    )
    return [spawn_vehicle, robot_state_publisher, command_node, odometry]


def generate_launch_description():

    # vehicle model: the xacro, so drive:=torque|ideal can be chosen (ads_dv.sdf = drive:=torque)
    model_file = os.path.join(get_package_share_directory('simulation'),
                               'models/vehicle/ads_dv.xacro')

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
    drive_launch_arg = DeclareLaunchArgument(
        "drive", default_value=TextSubstitution(text="torque"),
        description="torque: realistic ADS-DV drivetrain (VCU emulator, tyre grip, brakes); "
                    "ideal: wheel speed set directly (old behaviour)"
    )
    odom_launch_arg = DeclareLaunchArgument(
        "odom", default_value=TextSubstitution(text="wheel"),
        description="wheel: /odom from wheel speeds + IMU (drifts, like the real car); "
                    "ground_truth: Gazebo's exact pose"
    )
    gui_launch_arg = DeclareLaunchArgument(
        "gui", default_value=TextSubstitution(text="true"),
        description="false runs the Gazebo server only (-s), which saves a lot of CPU"
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
            value='sim_world'
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
            value='sim_world'
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
            value='1.57079632679'
        ),
        SetLaunchConfiguration(
            name='world',
            value='sim_world'
        ),
        SetLaunchConfiguration(
            name='map_file',
            value='drive1.sdf'
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
             value='-2.0'
        ),
        SetLaunchConfiguration(
            name='y',
            value='10.4'
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
            value='sim_world'
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
            value='46.35'
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
            value='sim_world'
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

    set_gz_resource_path = SetEnvironmentVariable(
    name='GZ_SIM_RESOURCE_PATH',
    value=[
        os.path.join(get_package_share_directory('simulation'), 'models', 'tracks'),
        ':',
        os.path.join(get_package_share_directory('simulation'), 'models', 'vehicle'),
        ':',
        os.path.join(get_package_share_directory('simulation'), 'models', 'cones'),
        ':',
        os.path.join(get_package_share_directory('simulation'), 'models', 'world'),
        ':',
        os.path.join(get_package_share_directory('simulation'), 'models', 'sensors'),
        ':',
        # parent of the package share dir, so "model://simulation/models/..." URIs resolve
        os.path.dirname(get_package_share_directory('simulation')),
        ':',
        EnvironmentVariable('GZ_SIM_RESOURCE_PATH', default_value='')
    ]
)

    set_gz_args = SetLaunchConfiguration(
        name='gz_args',
        value=[LaunchConfiguration('map_file'), ' ',
               LaunchConfiguration('autostart_flag'), ' ',
               IfElseSubstitution(LaunchConfiguration('gui'), if_value='', else_value='-s '),
               '-v ', LaunchConfiguration('verbosity')]
    )

    ros_gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('ros_gz_sim'), 'launch'),
                '/gz_sim.launch.py']),
        launch_arguments={'gz_args': LaunchConfiguration('gz_args')}.items(),
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

    # Cones are static in the world file (cheap); this node makes the ones near the car dynamic so the
    # car can knock them over (see scripts/cone_activator.py).
    cone_activator = Node(
        package='simulation',
        executable='cone_activator.py',
        name='cone_activator',
        output='screen',
        arguments=[[os.path.join(get_package_share_directory('simulation'), 'models', 'tracks', ''),
                    LaunchConfiguration('map_file')]],
    )

    return LaunchDescription([
        event_launch_arg,
        model_file_launch_arg,
        name_launch_arg,
        autostart_launch_arg,
        verbosity_launch_arg,
        gui_launch_arg,

        set_autostart,
        set_acceleration,
        set_skidpad,
        set_autocross,
        set_trackdrive,
        set_mppi_track,

        set_gz_resource_path,
        set_gz_args,
        
        ros_gz_sim,
        drive_launch_arg,
        odom_launch_arg,
        OpaqueFunction(function=vehicle_nodes),
        initial_sim_world_odom_tf,
        ros_gz_bridge,
        cone_activator,
    ])
