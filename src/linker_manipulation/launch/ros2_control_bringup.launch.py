from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from linker_manipulation.ros2_control_description import build_robot_description


def _value(context, name: str) -> str:
    return LaunchConfiguration(name).perform(context)


def _as_bool(value: str) -> bool:
    return value.strip().lower() in {"1", "true", "yes", "on"}


def _launch_setup(context):
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
    controllers_path = _value(context, "controllers_path")

    nodes = [
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

    if not use_mock_hardware:
        safety_parameters = {
            "can_interface": hardware_parameters["can_interface"],
            "side": hardware_parameters["side"],
        }
        if hardware_parameters["motor_ids"]:
            safety_parameters["motor_ids"] = [
                int(value.strip())
                for value in hardware_parameters["motor_ids"].split(",")
            ]
        nodes.append(
            Node(
                package="linker_a7_ros2_control",
                executable="a7_lite_safety_node",
                name="a7_lite_safety",
                output="screen",
                parameters=[safety_parameters],
            )
        )

    mode = "mock" if use_mock_hardware else "real A7 Lite"
    print(f"[linker_manipulation] ros2_control hardware mode: {mode}")
    return nodes


def generate_launch_description():
    package_share = FindPackageShare("linker_manipulation")
    default_urdf = PathJoinSubstitution(
        [package_share, "urdf", "a7_lite_right_mesh.urdf"]
    )
    default_controllers = PathJoinSubstitution(
        [package_share, "config", "ros2_control", "a7_lite_controllers.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("urdf_path", default_value=default_urdf),
            DeclareLaunchArgument("controllers_path", default_value=default_controllers),
            DeclareLaunchArgument(
                "use_mock_hardware",
                default_value="true",
                description="Use GenericSystem; false connects and enables the real A7 Lite.",
            ),
            DeclareLaunchArgument("can_interface", default_value="can0"),
            DeclareLaunchArgument("arm_side", default_value="right"),
            DeclareLaunchArgument(
                "motor_ids",
                default_value="",
                description="Optional comma-separated IDs; right defaults to 51..57.",
            ),
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
            OpaqueFunction(function=_launch_setup),
        ]
    )
