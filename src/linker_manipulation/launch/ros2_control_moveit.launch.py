from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("linker_manipulation")
    use_rviz = LaunchConfiguration("use_rviz")
    allow_execution = LaunchConfiguration("allow_execution")

    ros2_control_launch = PathJoinSubstitution(
        [package_share, "launch", "ros2_control_bringup.launch.py"]
    )
    moveit_launch = PathJoinSubstitution(
        [package_share, "launch", "moveit_ros2_control.launch.py"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Start RViz2 with MoveIt planning scene.",
            ),
            DeclareLaunchArgument(
                "allow_execution",
                default_value="false",
                description="Allow MoveIt to execute through ros2_control.",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(ros2_control_launch),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(moveit_launch),
                launch_arguments={
                    "use_rviz": use_rviz,
                    "allow_execution": allow_execution,
                }.items(),
            ),
        ]
    )
