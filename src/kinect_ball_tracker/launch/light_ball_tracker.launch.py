from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([

        # Start the Kinect driver + RGB showimage + Depth showimage
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("kinect_ros2"),
                    "launch",
                    "showimage.launch.py"
                ])
            )
        ),

        # Start the lightweight ball tracker
        Node(
            package="kinect_ball_tracker",
            executable="light_ball_tracker_node",
            name="light_kinect_ball_tracker",
            output="screen",
            parameters=[{
                "image_topic": "/kinect/image_raw",
                "depth_topic": "/kinect/depth/image_raw",

                # Values copied from your tuned slider screenshot
                "h_low": 24,
                "h_high": 50,
                "s_low": 50,
                "s_high": 255,
                "v_low": 50,
                "v_high": 255,

                "morph_kernel": 6,
                "min_area_x100": 7,
                "min_circularity_x100": 25,
                "min_radius_px": 7,

                "depth_sample_percent": 80,
                "min_valid_depth_px": 10,
                "min_depth_cm": 50,
                "max_depth_cm": 450,

                # Set to 2 or 3 on Raspberry Pi if CPU usage is too high
                "process_every_n_frames": 1,
            }]
        )
    ])