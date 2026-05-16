import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg = get_package_share_directory('trace_description')

    joy_node = Node(
        package='joy',
        executable='joy_node',
        parameters=[os.path.join(pkg, 'config', 'joy_config.yaml')]
    )

    bldc_node = Node(
        package='trace_firmware',
        executable='bldc_node.py',
        parameters=[{
            'use_sim_time': False,
            'port': '/dev/ttyBLDC',
            'button': 4,        # R2 — change to whichever button you want
        }]
    )

    return LaunchDescription([
        joy_node,
        bldc_node,
    ])