import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    package_name = 'trace_description'

    # ── Declare arguments FIRST so they exist before anything consumes them ──

    use_ros2_control_arg = DeclareLaunchArgument(
        'use_ros2_control',
        default_value='true',
        description='Use ros2_control (true) or Gazebo integrated diff-drive plugin (false)'
    )

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(
            get_package_share_directory(package_name), 'worlds', 'test_world.sdf'
        ),
        description='World SDF file to load into Gazebo'
    )

    # ── LaunchConfigurations (read the declared args) ────────────────────────

    use_ros2_control = LaunchConfiguration('use_ros2_control')
    world = LaunchConfiguration('world')

    # ── Robot State Publisher ─────────────────────────────────────────────────
    # Pass use_ros2_control through so the xacro arg selects the right plugin

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'rsp.launch.py'
        )]),
        launch_arguments={
            'use_sim_time': 'true',
            'use_ros2_control': use_ros2_control   # ← was hardcoded 'false'
        }.items()
    )

    # ── Gazebo ────────────────────────────────────────────────────────────────

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
        )]),
        launch_arguments={
            'gz_args': ['-r -v4 ', world],
            'on_exit_shutdown': 'true'
        }.items()
    )

    # ── Spawn robot ───────────────────────────────────────────────────────────

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'my_robot', '-z', '0.1'],
        output='screen'
    )

    # ── ros2_control spawners (only when use_ros2_control:=true) ─────────────

    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_cont',
            '--controller-ros-args',
            '-r /diff_cont/cmd_vel:=/cmd_vel'
        ],
        condition=IfCondition(use_ros2_control)   # ← only spawned when true
    )

    joint_broad_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_broad'],
        condition=IfCondition(use_ros2_control)   # ← only spawned when true
    )

    # ── ROS-Gazebo bridge ─────────────────────────────────────────────────────

    bridge_params = os.path.join(
        get_package_share_directory(package_name), 'config', 'gz_bridge.yaml'
    )
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={bridge_params}']
    )

    # ── RViz ──────────────────────────────────────────────────────────────────

    rviz_config_file = os.path.join(
        get_package_share_directory(package_name), 'rviz', 'view_robot.rviz'
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}]
    )

    # ── Launch everything ─────────────────────────────────────────────────────

    return LaunchDescription([
        use_ros2_control_arg,   # ← declare args before anything else
        world_arg,
        rsp,
        gazebo,
        spawn_entity,
        diff_drive_spawner,
        joint_broad_spawner,
        ros_gz_bridge,
        rviz,
    ])