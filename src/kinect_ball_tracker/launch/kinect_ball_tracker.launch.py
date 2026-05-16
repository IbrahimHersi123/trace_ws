from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([

        # Start Kinect driver + RGB showimage + Depth showimage
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("kinect_ros2"),
                    "launch",
                    "showimage.launch.py"
                ])
            )
        ),

        # Start the full Kinect ball tracker node
        Node(
            package="kinect_ball_tracker",
            executable="kinect_ball_tracker_node",
            name="kinect_ball_tracker_node",
            output="screen",
            parameters=[{
                "image_topic": "/kinect/image_raw",
                "depth_topic": "/kinect/depth/image_raw",
                "show_window": True,
            }]
        )
    ])
