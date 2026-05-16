import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Start the Kinect driver + ball tracker
    ball_tracker_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('kinect_ball_tracker'),
                'launch',
                'light_ball_tracker.launch.py'
            )
        ])
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        parameters=[os.path.join(
            get_package_share_directory('trace_description'),
            'config',
            'joy_config.yaml'
        )]
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
        ball_tracker_launch,
        joy_node,
        ball_orient_node,
    ])