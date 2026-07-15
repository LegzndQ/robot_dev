from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("linker_manipulation")
    ros2_control_launch = PathJoinSubstitution(
        [package_share, "launch", "ros2_control_bringup.launch.py"]
    )
    moveit_launch = PathJoinSubstitution(
        [package_share, "launch", "moveit_ros2_control.launch.py"]
    )

    argument_defaults = {
        "use_rviz": "true",
        "allow_execution": "false",
        "use_mock_hardware": "true",
        "can_interface": "can0",
        "arm_side": "right",
        "motor_ids": "",
        "velocity_limit": "5.0",
        "acceleration_limit": "20.0",
        "state_timeout_ms": "250",
        "command_epsilon": "0.00001",
        "command_keepalive_ms": "100",
        "position_signs": "1,1,1,1,1,1,1",
        "position_offsets": "0,0,0,0,0,0,0",
    }
    arguments = {
        name: LaunchConfiguration(name) for name in argument_defaults
    }
    hardware_arguments = {
        name: value
        for name, value in arguments.items()
        if name not in {"use_rviz", "allow_execution"}
    }

    return LaunchDescription(
        [
            *[
                DeclareLaunchArgument(name, default_value=default)
                for name, default in argument_defaults.items()
            ],
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(ros2_control_launch),
                launch_arguments=hardware_arguments.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(moveit_launch),
                launch_arguments=arguments.items(),
            ),
        ]
    )
