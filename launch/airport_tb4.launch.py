import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # -----------------------------
    # Launch arguments
    # -----------------------------
    use_cropped = LaunchConfiguration('use_cropped')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')
    namespace = LaunchConfiguration('namespace')

    declare_use_cropped = DeclareLaunchArgument(
        'use_cropped',
        default_value='true',
        description='Use cropped airport model instead of full RMF terminal'
    )

    # For cropped world, default spawn is center concourse (0.0, 0.0)
    # For full RMF airport, user can pass x:=136.5 y:=-41.7
    declare_x = DeclareLaunchArgument(
        'x', default_value='0.0'
    )
    declare_y = DeclareLaunchArgument(
        'y', default_value='0.0'
    )
    declare_z = DeclareLaunchArgument(
        'z', default_value='0.1'
    )
    declare_yaw = DeclareLaunchArgument(
        'yaw', default_value='0.0'
    )
    declare_namespace = DeclareLaunchArgument(
        'namespace',
        default_value=''
    )

    # -----------------------------
    # Package paths
    # -----------------------------
    this_pkg = get_package_share_directory(
        'sicnav_airport_bringup'
    )
    rmf_demos_gz = get_package_share_directory(
        'rmf_demos_gz'
    )
    turtlebot4_gz = get_package_share_directory(
        'turtlebot4_gz_bringup'
    )

    models_path = os.path.join(this_pkg, 'models')
    cropped_world_path = os.path.join(
        this_pkg, 'worlds', 'airport_cropped.sdf'
    )

    # -----------------------------
    # Gazebo Environment Variables
    # -----------------------------
    gz_resource_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        models_path + ':/opt/ros/jazzy/share'
    )
    gz_plugin_path = AppendEnvironmentVariable(
        'GZ_SIM_SYSTEM_PLUGIN_PATH',
        '/opt/ros/jazzy/lib'
    )

    # -----------------------------
    # 1. Option A: Cropped Airport (gz sim direct)
    # -----------------------------
    gz_sim_cropped = ExecuteProcess(
        condition=IfCondition(use_cropped),
        cmd=['gz', 'sim', '-r', cropped_world_path],
        output='screen'
    )

    # -----------------------------
    # 1. Option B: Full RMF Airport + Crowd
    # -----------------------------
    airport_launch = os.path.join(
        rmf_demos_gz,
        'airport_terminal.launch.xml'
    )

    airport_full = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(airport_launch),
        condition=UnlessCondition(use_cropped),
        launch_arguments={
            'use_crowdsim': '1'
        }.items()
    )

    # -----------------------------
    # 2. Manual /clock bridge
    # -----------------------------
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='manual_clock_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ]
    )

    # -----------------------------
    # 3. TurtleBot4 spawn
    # -----------------------------
    turtlebot_spawn_launch = os.path.join(
        turtlebot4_gz,
        'launch',
        'turtlebot4_spawn.launch.py'
    )

    turtlebot_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            turtlebot_spawn_launch
        ),
        launch_arguments={
            'model': 'standard',
            'namespace': namespace,
            'x': x,
            'y': y,
            'z': z,
            'yaw': yaw,
            'rviz': 'false',
            'nav2': 'false',
            'slam': 'false',
            'localization': 'false',
            'use_sim_time': 'true',
        }.items()
    )

    delayed_turtlebot_spawn = TimerAction(
        period=6.0,
        actions=[turtlebot_spawn]
    )

    # -----------------------------
    # 4. diffdrive controller bootstrap
    # -----------------------------
    controller_script = os.path.join(
        this_pkg,
        'scripts',
        'controller_bootstrap.sh'
    )

    controller_bootstrap = ExecuteProcess(
        cmd=[
            '/bin/bash',
            controller_script
        ],
        output='screen'
    )

    delayed_controller_bootstrap = TimerAction(
        period=10.0,
        actions=[controller_bootstrap]
    )

    return LaunchDescription([
        declare_use_cropped,
        declare_x,
        declare_y,
        declare_z,
        declare_yaw,
        declare_namespace,

        gz_resource_path,
        gz_plugin_path,

        gz_sim_cropped,
        airport_full,
        clock_bridge,
        delayed_turtlebot_spawn,
        delayed_controller_bootstrap,
    ])
