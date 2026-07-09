from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_urdf = PathJoinSubstitution(
        [
            FindPackageShare("linker_manipulation"),
            "urdf",
            "a7_lite_right_mesh.urdf",
        ]
    )
    default_rviz = PathJoinSubstitution(
        [
            FindPackageShare("linker_manipulation"),
            "rviz",
            "a7_lite_visualization.rviz",
        ]
    )

    urdf_path = LaunchConfiguration("urdf_path")
    rviz_config = LaunchConfiguration("rviz_config")
    use_rviz = LaunchConfiguration("use_rviz")

    robot_description = ParameterValue(
        Command(["cat ", urdf_path]),
        value_type=str,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "urdf_path",
                default_value=default_urdf,
                description="URDF to visualize.",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=default_rviz,
                description="RViz2 config file.",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Start RViz2. Set false on headless/SSH sessions.",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="linker_robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description}],
                remappings=[("joint_states", "/linker/arm/state")],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="linker_rviz2",
                output="screen",
                arguments=["-d", rviz_config],
                condition=IfCondition(use_rviz),
            ),
        ]
    )
