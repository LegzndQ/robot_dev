from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from linker_manipulation.ros2_control_description import build_robot_description


def _load_yaml(package_name: str, relative_path: str):
    package_path = Path(get_package_share_directory(package_name))
    with (package_path / relative_path).open("r", encoding="utf-8") as stream:
        return yaml.safe_load(stream) or {}


def _value(context, name: str) -> str:
    return LaunchConfiguration(name).perform(context)


def _as_bool(value: str) -> bool:
    return value.strip().lower() in {"1", "true", "yes", "on"}


def _launch_setup(context):
    package_name = "linker_manipulation"
    use_mock_hardware = _as_bool(_value(context, "use_mock_hardware"))
    hardware_parameters = {
        "can_interface": _value(context, "can_interface"),
        "side": _value(context, "arm_side"),
        "motor_ids": _value(context, "motor_ids"),
        "velocity_limit": _value(context, "velocity_limit"),
        "acceleration_limit": _value(context, "acceleration_limit"),
        "state_timeout_ms": _value(context, "state_timeout_ms"),
        "command_epsilon": _value(context, "command_epsilon"),
        "command_keepalive_ms": _value(context, "command_keepalive_ms"),
        "position_signs": _value(context, "position_signs"),
        "position_offsets": _value(context, "position_offsets"),
    }
    robot_description = {
        "robot_description": build_robot_description(
            _value(context, "urdf_path"),
            use_mock_hardware=use_mock_hardware,
            hardware_parameters=hardware_parameters,
        )
    }
    robot_description_semantic = {
        "robot_description_semantic": Path(
            _value(context, "srdf_path")
        ).read_text(encoding="utf-8")
    }
    robot_description_kinematics = {
        "robot_description_kinematics": _load_yaml(
            package_name, "config/moveit/kinematics.yaml"
        )
    }
    joint_limits = {
        "robot_description_planning": _load_yaml(
            package_name, "config/moveit/joint_limits.yaml"
        )
    }
    ompl_planning = _load_yaml(package_name, "config/moveit/ompl_planning.yaml")
    moveit_controllers = _load_yaml(
        package_name, "config/moveit/moveit_controllers_ros2_control.yaml"
    )
    planning_scene_monitor = _load_yaml(
        package_name, "config/moveit/planning_scene_monitor.yaml"
    )
    moveit_common = {
        "allow_trajectory_execution": _as_bool(_value(context, "allow_execution")),
        "capabilities": "",
        "disable_capabilities": "",
        "planning_scene_monitor_options": {
            "name": "planning_scene_monitor",
            "robot_description": "robot_description",
            "joint_state_topic": "/joint_states",
            "attached_collision_object_topic": "/moveit_cpp/planning_scene_monitor",
            "publish_planning_scene_topic": "/monitored_planning_scene",
            "monitored_planning_scene_topic": "/monitored_planning_scene",
            "wait_for_initial_state_timeout": 10.0,
        },
    }

    nodes = [
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
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
        )
    ]
    if _as_bool(_value(context, "use_rviz")):
        nodes.append(
            Node(
                package="rviz2",
                executable="rviz2",
                output="screen",
                arguments=["-d", _value(context, "rviz_config")],
                parameters=[
                    robot_description,
                    robot_description_semantic,
                    robot_description_kinematics,
                    ompl_planning,
                ],
            )
        )
    return nodes


def generate_launch_description():
    package_share = FindPackageShare("linker_manipulation")
    default_urdf = PathJoinSubstitution(
        [package_share, "urdf", "a7_lite_right_mesh.urdf"]
    )
    default_srdf = PathJoinSubstitution(
        [package_share, "config", "moveit", "a7_lite_right.srdf"]
    )
    default_rviz = PathJoinSubstitution(
        [package_share, "rviz", "moveit_table.rviz"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("urdf_path", default_value=default_urdf),
            DeclareLaunchArgument("srdf_path", default_value=default_srdf),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("allow_execution", default_value="false"),
            DeclareLaunchArgument("use_mock_hardware", default_value="true"),
            DeclareLaunchArgument("can_interface", default_value="can0"),
            DeclareLaunchArgument("arm_side", default_value="right"),
            DeclareLaunchArgument("motor_ids", default_value=""),
            DeclareLaunchArgument("velocity_limit", default_value="5.0"),
            DeclareLaunchArgument("acceleration_limit", default_value="20.0"),
            DeclareLaunchArgument("state_timeout_ms", default_value="250"),
            DeclareLaunchArgument("command_epsilon", default_value="0.00001"),
            DeclareLaunchArgument("command_keepalive_ms", default_value="100"),
            DeclareLaunchArgument("position_signs", default_value="1,1,1,1,1,1,1"),
            DeclareLaunchArgument("position_offsets", default_value="0,0,0,0,0,0,0"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
