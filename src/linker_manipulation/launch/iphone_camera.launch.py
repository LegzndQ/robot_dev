from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rtsp_url",
                default_value="rtsp://172.20.10.1:554/stream",
                description="OctoStream RTSP URL.",
            ),
            DeclareLaunchArgument(
                "frame_id",
                default_value="iphone_camera_optical_frame",
            ),
            DeclareLaunchArgument("publish_rate_hz", default_value="30.0"),
            DeclareLaunchArgument("rtsp_transport", default_value="tcp"),
            DeclareLaunchArgument("reconnect_delay_sec", default_value="2.0"),
            DeclareLaunchArgument("open_timeout_ms", default_value="5000"),
            DeclareLaunchArgument("read_timeout_ms", default_value="5000"),
            DeclareLaunchArgument(
                "rotation_degrees",
                default_value="0",
                description="Clockwise image rotation: 0, 90, 180, or 270.",
            ),
            DeclareLaunchArgument("resize_width", default_value="0"),
            DeclareLaunchArgument("resize_height", default_value="0"),
            DeclareLaunchArgument(
                "camera_info_url",
                default_value="",
                description=(
                    "Optional ROS camera calibration YAML path or file:// URL."
                ),
            ),
            Node(
                package="linker_manipulation",
                executable="rtsp_camera_node",
                name="iphone_camera",
                output="screen",
                parameters=[
                    {
                        "rtsp_url": LaunchConfiguration("rtsp_url"),
                        "frame_id": LaunchConfiguration("frame_id"),
                        "publish_rate_hz": ParameterValue(
                            LaunchConfiguration("publish_rate_hz"),
                            value_type=float,
                        ),
                        "rtsp_transport": LaunchConfiguration(
                            "rtsp_transport"
                        ),
                        "reconnect_delay_sec": ParameterValue(
                            LaunchConfiguration("reconnect_delay_sec"),
                            value_type=float,
                        ),
                        "open_timeout_ms": ParameterValue(
                            LaunchConfiguration("open_timeout_ms"),
                            value_type=int,
                        ),
                        "read_timeout_ms": ParameterValue(
                            LaunchConfiguration("read_timeout_ms"),
                            value_type=int,
                        ),
                        "rotation_degrees": ParameterValue(
                            LaunchConfiguration("rotation_degrees"),
                            value_type=int,
                        ),
                        "resize_width": ParameterValue(
                            LaunchConfiguration("resize_width"),
                            value_type=int,
                        ),
                        "resize_height": ParameterValue(
                            LaunchConfiguration("resize_height"),
                            value_type=int,
                        ),
                        "camera_info_url": LaunchConfiguration(
                            "camera_info_url"
                        ),
                    }
                ],
            ),
        ]
    )
