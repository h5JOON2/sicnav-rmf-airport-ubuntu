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
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # -----------------------------
    # 1. Launch arguments
    # -----------------------------
    headless = LaunchConfiguration('headless')
    use_tcp_bridge = LaunchConfiguration('use_tcp_bridge')
    use_local_frame = LaunchConfiguration('use_local_frame')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')
    namespace = LaunchConfiguration('namespace')

    declare_headless = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run Gazebo in headless mode (-s) without GUI'
    )

    declare_use_tcp_bridge = DeclareLaunchArgument(
        'use_tcp_bridge',
        default_value='true',
        description='Launch TCP bridge node to communicate with macOS JMID/SICNav'
    )

    declare_use_local_frame = DeclareLaunchArgument(
        'use_local_frame',
        default_value='false',
        description='If true, use (0,0) centered local frame instead of global airport frame (136.5, -41.7)'
    )

    # Defaults: Global frame matches original RMF Airport Terminal Concourse
    declare_x = DeclareLaunchArgument(
        'x', default_value='136.5',
        description='Initial robot X position (Use 0.0 if use_local_frame:=true)'
    )
    declare_y = DeclareLaunchArgument(
        'y', default_value='-41.7',
        description='Initial robot Y position (Use 0.0 if use_local_frame:=true)'
    )
    declare_z = DeclareLaunchArgument(
        'z', default_value='0.1',
        description='Initial robot Z position'
    )
    declare_yaw = DeclareLaunchArgument(
        'yaw', default_value='0.0',
        description='Initial robot Yaw angle'
    )
    declare_namespace = DeclareLaunchArgument(
        'namespace', default_value='',
        description='Robot namespace'
    )

    # -----------------------------
    # 2. Package paths
    # -----------------------------
    current_dir = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
    try:
        this_pkg = get_package_share_directory('sicnav_airport_bringup')
        pkg_root = this_pkg if os.path.exists(os.path.join(this_pkg, 'worlds')) else current_dir
    except Exception:
        pkg_root = current_dir

    models_path = os.path.join(pkg_root, 'models')
    maps_path = os.path.join(pkg_root, 'maps')
    world_global = os.path.join(pkg_root, 'worlds', 'airport_cropped_world.sdf')
    world_local = os.path.join(pkg_root, 'worlds', 'airport_cropped_world_local.sdf')

    # -----------------------------
    # 3. Environment Variables
    # -----------------------------
    gz_resource_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        models_path + ':/opt/ros/jazzy/share'
    )

    menge_resource_path = AppendEnvironmentVariable(
        'MENGE_RESOURCE_PATH',
        maps_path
    )

    gz_plugin_path = AppendEnvironmentVariable(
        'GZ_SIM_SYSTEM_PLUGIN_PATH',
        '/home/leeojoon/sicnav_airport_ws/install/rmf_building_sim_gz_plugins/lib/rmf_building_sim_gz_plugins:/opt/ros/jazzy/lib/rmf_building_sim_gz_plugins:/opt/ros/jazzy/lib'
    )



    # -----------------------------
    # 4. Gazebo Harmonic Simulator
    # -----------------------------
    # Global Frame - GUI Mode (using ogre 1 for VM compatibility)
    gz_sim_global_gui = ExecuteProcess(
        condition=UnlessCondition(headless),
        cmd=['gz', 'sim', '--render-engine-gui', 'ogre', '-r', world_global],
        output='screen'
    )

    # Global Frame - Headless Mode
    gz_sim_global_headless = ExecuteProcess(
        condition=IfCondition(headless),
        cmd=['gz', 'sim', '-s', '-r', world_global],
        output='screen'
    )

    # -----------------------------
    # 5. ROS 2 <-> Gazebo Bridge
    # -----------------------------
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        ]
    )

    # -----------------------------
    # 6. TurtleBot4 spawn (Optional - already in world or spawned via launch)
    # -----------------------------
    # In airport_cropped_world.sdf, TurtleBot4 is already included natively.
    # We bootstrap controller below to ensure diffdrive is ready.

    # -----------------------------
    # 7. diffdrive controller bootstrap
    # -----------------------------
    controller_script = os.path.join(
        pkg_root,
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

    # -----------------------------
    # 8. SICNav TCP Bridge Node
    # -----------------------------
    tcp_bridge_script = os.path.join(
        pkg_root,
        'scripts',
        'sicnav_tcp_bridge.py'
    )

    tcp_bridge_node = ExecuteProcess(
        condition=IfCondition(use_tcp_bridge),
        cmd=['python3', tcp_bridge_script],
        output='screen'
    )

    return LaunchDescription([
        declare_headless,
        declare_use_tcp_bridge,
        declare_use_local_frame,
        declare_x,
        declare_y,
        declare_z,
        declare_yaw,
        declare_namespace,

        gz_resource_path,
        menge_resource_path,
        gz_plugin_path,

        gz_sim_global_gui,
        gz_sim_global_headless,
        bridge,
        delayed_controller_bootstrap,
        tcp_bridge_node,
    ])
