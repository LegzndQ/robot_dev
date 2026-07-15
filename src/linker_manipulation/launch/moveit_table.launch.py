from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _load_yaml(package_name: str, relative_path: str):
    package_path = Path(get_package_share_directory(package_name))
    with (package_path / relative_path).open("r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def generate_launch_description():
    package_name = "linker_manipulation"
    package_share = FindPackageShare(package_name)

    default_urdf = PathJoinSubstitution(
        [package_share, "urdf", "a7_lite_right_mesh.urdf"]
    )
    default_srdf = PathJoinSubstitution(
        [package_share, "config", "moveit", "a7_lite_right.srdf"]
    )
    default_rviz = PathJoinSubstitution(
        [package_share, "rviz", "moveit_table.rviz"]
    )

    urdf_path = LaunchConfiguration("urdf_path")
    srdf_path = LaunchConfiguration("srdf_path")
    rviz_config = LaunchConfiguration("rviz_config")
    use_rviz = LaunchConfiguration("use_rviz")
    allow_execution = LaunchConfiguration("allow_execution")

    robot_description = {
        "robot_description": ParameterValue(Command(["cat ", urdf_path]), value_type=str)
    }
    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(
            Command(["cat ", srdf_path]), value_type=str
        )
    }
    robot_description_kinematics = {
        "robot_description_kinematics": _load_yaml(
            package_name,
            "config/moveit/kinematics.yaml",
        )
    }
    joint_limits = _load_yaml(package_name, "config/moveit/joint_limits.yaml")
    ompl_planning = _load_yaml(package_name, "config/moveit/ompl_planning.yaml")
    moveit_controllers = _load_yaml(package_name, "config/moveit/moveit_controllers.yaml")
    planning_scene_monitor = _load_yaml(
        package_name,
        "config/moveit/planning_scene_monitor.yaml",
    )

    moveit_common = {
        "allow_trajectory_execution": ParameterValue(
            allow_execution,
            value_type=bool,
        ),
        "capabilities": "",
        "disable_capabilities": "",
        "planning_scene_monitor_options": {
            "name": "planning_scene_monitor",
            "robot_description": "robot_description",
            "joint_state_topic": "/linker/arm/state",
            "attached_collision_object_topic": "/moveit_cpp/planning_scene_monitor",
            "publish_planning_scene_topic": "/monitored_planning_scene",
            "monitored_planning_scene_topic": "/monitored_planning_scene",
            "wait_for_initial_state_timeout": 10.0,
        },
    }

    remappings = [("joint_states", "/linker/arm/state")]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "urdf_path",
                default_value=default_urdf,
                description="URDF containing A7 Lite and the fixed table collision box.",
            ),
            DeclareLaunchArgument(
                "srdf_path",
                default_value=default_srdf,
                description="MoveIt SRDF for A7 Lite.",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=default_rviz,
                description="RViz2 config file.",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Start RViz2 with MoveIt planning scene.",
            ),
            DeclareLaunchArgument(
                "allow_execution",
                default_value="false",
                description=(
                    "Allow MoveIt to send planned trajectories to the A7 SDK bridge."
                ),
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="linker_moveit_robot_state_publisher",
                output="screen",
                parameters=[robot_description],
                remappings=remappings,
            ),
            Node(
                package="moveit_ros_move_group",
                executable="move_group",
                name="move_group",
                output="screen",
                parameters=[
                    robot_description,
                    robot_description_semantic,
                    robot_description_kinematics,
                    joint_limits,
                    ompl_planning,
                    moveit_controllers,
                    planning_scene_monitor,
                    moveit_common,
                ],
                remappings=remappings,
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="linker_moveit_rviz2",
                output="screen",
                arguments=["-d", rviz_config],
                parameters=[
                    robot_description,
                    robot_description_semantic,
                    robot_description_kinematics,
                    ompl_planning,
                ],
                remappings=remappings,
                condition=IfCondition(use_rviz),
            ),
        ]
    )
