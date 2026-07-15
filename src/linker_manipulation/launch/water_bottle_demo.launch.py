from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("linker_manipulation")
    moveit_launch = PathJoinSubstitution(
        [package_share, "launch", "ros2_control_moveit.launch.py"]
    )
    default_config = PathJoinSubstitution(
        [package_share, "config", "robot.yaml"]
    )

    config_path = LaunchConfiguration("config_path")
    start_hand = LaunchConfiguration("start_hand")
    hardware_arguments = {
        "use_rviz": LaunchConfiguration("use_rviz"),
        "allow_execution": "true",
        "use_mock_hardware": LaunchConfiguration("use_mock_hardware"),
        "can_interface": LaunchConfiguration("can_interface"),
        "arm_side": LaunchConfiguration("arm_side"),
        "motor_ids": LaunchConfiguration("motor_ids"),
        "velocity_limit": LaunchConfiguration("velocity_limit"),
        "acceleration_limit": LaunchConfiguration("acceleration_limit"),
        "state_timeout_ms": LaunchConfiguration("state_timeout_ms"),
        "command_epsilon": LaunchConfiguration("command_epsilon"),
        "command_keepalive_ms": LaunchConfiguration("command_keepalive_ms"),
        "position_signs": LaunchConfiguration("position_signs"),
        "position_offsets": LaunchConfiguration("position_offsets"),
    }

    common_parameters = [{"config_path": config_path}]
    return LaunchDescription(
        [
            DeclareLaunchArgument("config_path", default_value=default_config),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument(
                "use_mock_hardware",
                default_value="true",
                description="Set false only when the real A7 Lite workspace is clear.",
            ),
            DeclareLaunchArgument("start_hand", default_value="true"),
            DeclareLaunchArgument("can_interface", default_value="can0"),
            DeclareLaunchArgument("arm_side", default_value="right"),
            DeclareLaunchArgument("motor_ids", default_value=""),
            DeclareLaunchArgument("velocity_limit", default_value="5.0"),
            DeclareLaunchArgument("acceleration_limit", default_value="20.0"),
            DeclareLaunchArgument("state_timeout_ms", default_value="250"),
            DeclareLaunchArgument("command_epsilon", default_value="0.00001"),
            DeclareLaunchArgument("command_keepalive_ms", default_value="100"),
            DeclareLaunchArgument(
                "position_signs", default_value="1,1,1,1,1,1,1"
            ),
            DeclareLaunchArgument(
                "position_offsets", default_value="0,0,0,0,0,0,0"
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(moveit_launch),
                launch_arguments=hardware_arguments.items(),
            ),
            Node(
                package="linker_manipulation",
                executable="hand_driver_node",
                name="hand_driver",
                output="screen",
                parameters=common_parameters,
                condition=IfCondition(start_hand),
            ),
            Node(
                package="linker_manipulation",
                executable="grasp_controller_node",
                name="grasp_controller",
                output="screen",
                parameters=[
                    {"config_path": config_path, "arm_backend": "moveit"}
                ],
            ),
            Node(
                package="linker_manipulation",
                executable="demo_task_node",
                name="demo_task",
                output="screen",
                parameters=[
                    {
                        "config_path": config_path,
                        "enable_and_home": False,
                        "target_profile": "water_bottle",
                    }
                ],
            ),
        ]
    )
