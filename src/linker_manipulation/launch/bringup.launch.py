from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_config = PathJoinSubstitution(
        [FindPackageShare("linker_manipulation"), "config", "robot.yaml"]
    )
    config_path = LaunchConfiguration("config_path")

    common_params = [{"config_path": config_path}]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_path",
                default_value=default_config,
                description="Path to the A7/LinkerHand robot configuration YAML.",
            ),
            Node(
                package="linker_manipulation",
                executable="a7_driver_node",
                name="a7_driver",
                output="screen",
                parameters=common_params,
            ),
            Node(
                package="linker_manipulation",
                executable="hand_driver_node",
                name="hand_driver",
                output="screen",
                parameters=common_params,
            ),
            Node(
                package="linker_manipulation",
                executable="grasp_controller_node",
                name="grasp_controller",
                output="screen",
                parameters=common_params,
            ),
            Node(
                package="linker_manipulation",
                executable="demo_task_node",
                name="demo_task",
                output="screen",
                parameters=common_params,
            ),
        ]
    )
