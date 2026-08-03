from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("linker_manipulation")
    camera_launch = PathJoinSubstitution(
        [package_share, "launch", "iphone_camera.launch.py"]
    )
    default_output = str(
        Path.home() / ".ros" / "camera_info" / "iphone_15_pro.yaml"
    )
    arguments = {
        "rtsp_url": "rtsp://172.20.10.1:554/stream",
        "rotation_degrees": "0",
        "resize_width": "0",
        "resize_height": "0",
        "publish_rate_hz": "15.0",
        "output_path": default_output,
        "square_length_m": "0.030",
        "marker_length_m": "0.022",
    }
    return LaunchDescription(
        [
            *[
                DeclareLaunchArgument(name, default_value=value)
                for name, value in arguments.items()
            ],
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(camera_launch),
                launch_arguments={
                    "rtsp_url": LaunchConfiguration("rtsp_url"),
                    "rotation_degrees": LaunchConfiguration(
                        "rotation_degrees"
                    ),
                    "resize_width": LaunchConfiguration("resize_width"),
                    "resize_height": LaunchConfiguration("resize_height"),
                    "publish_rate_hz": LaunchConfiguration(
                        "publish_rate_hz"
                    ),
                    "camera_info_url": "",
                }.items(),
            ),
            Node(
                package="linker_manipulation",
                executable="intrinsic_calibrator_node",
                name="iphone_intrinsic_calibrator",
                output="screen",
                parameters=[
                    {
                        "output_path": LaunchConfiguration("output_path"),
                        "square_length_m": ParameterValue(
                            LaunchConfiguration("square_length_m"),
                            value_type=float,
                        ),
                        "marker_length_m": ParameterValue(
                            LaunchConfiguration("marker_length_m"),
                            value_type=float,
                        ),
                    }
                ],
            ),
        ]
    )
