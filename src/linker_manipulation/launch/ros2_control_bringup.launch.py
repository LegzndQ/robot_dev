from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = "linker_manipulation"
    package_share = FindPackageShare(package_name)

    default_urdf = PathJoinSubstitution(
        [package_share, "urdf", "a7_lite_right_mesh.urdf"]
    )
    default_controllers = PathJoinSubstitution(
        [package_share, "config", "ros2_control", "a7_lite_controllers.yaml"]
    )

    urdf_path = LaunchConfiguration("urdf_path")
    controllers_path = LaunchConfiguration("controllers_path")
    robot_description = {
        "robot_description": ParameterValue(Command(["cat ", urdf_path]), value_type=str)
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "urdf_path",
                default_value=default_urdf,
                description="URDF containing ros2_control hardware declarations.",
            ),
            DeclareLaunchArgument(
                "controllers_path",
                default_value=default_controllers,
                description="ros2_control controller configuration YAML.",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="linker_ros2_control_robot_state_publisher",
                output="screen",
                parameters=[robot_description],
            ),
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                name="controller_manager",
                output="screen",
                parameters=[robot_description, controllers_path],
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "joint_state_broadcaster",
                    "--controller-manager",
                    "/controller_manager",
                    "--param-file",
                    controllers_path,
                ],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "a7_arm_controller",
                    "--controller-manager",
                    "/controller_manager",
                    "--param-file",
                    controllers_path,
                ],
                output="screen",
            ),
        ]
    )
