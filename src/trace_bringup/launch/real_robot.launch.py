import os
from launch import LaunchDescription
from launch.actions import TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg = get_package_share_directory('trace_description')

    robot_description = ParameterValue(
        Command(['xacro ', os.path.join(pkg, 'description', 'robot.urdf.xacro'),
                 ' use_ros2_control:=true']),
        value_type=str
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False
        }]
    )

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'use_sim_time': False},
            os.path.join(pkg, 'config', 'my_controllers.yaml')
        ]
    )

    joint_broad_spawner = TimerAction(
        period=3.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_broad'],
        )]
    )

    diff_drive_spawner = TimerAction(
        period=3.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'diff_cont',
                '--controller-ros-args',
                '-r /diff_cont/cmd_vel:=/cmd_vel'
            ],
        )]
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        parameters=[os.path.join(pkg, 'config', 'joy_config.yaml')]
    )

    teleop_node = Node(
        package='joy_teleop',
        executable='joy_teleop',
        parameters=[os.path.join(pkg, 'config', 'joy_teleop.yaml')]
    )

    bldc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('trace_firmware'), 'launch', 'bldc_control.launch.py')
        ])
    )

    ball_orient_node = Node(
    package='trace_firmware',
    executable='ball_orient_node.py',
    name='ball_orient_node',
    output='screen',
    parameters=[{
        'kp': 0.002,
        'dead_zone': 20.0,
        'max_angular_z': 0.5,
        'watchdog_timeout': 0.5,
        'toggle_button': 0,
    }]
    )

    return LaunchDescription([
        robot_state_publisher,
        controller_manager,
        joint_broad_spawner,
        diff_drive_spawner,
        joy_node,
        teleop_node,
        bldc_launch,
        ball_orient_node,
    ])