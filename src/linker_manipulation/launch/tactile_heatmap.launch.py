from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    topic = LaunchConfiguration("topic")
    vmin = LaunchConfiguration("vmin")
    vmax = LaunchConfiguration("vmax")
    update_hz = LaunchConfiguration("update_hz")
    cmap = LaunchConfiguration("cmap")
    auto_scale = LaunchConfiguration("auto_scale")
    delta_mode = LaunchConfiguration("delta_mode")
    baseline_samples = LaunchConfiguration("baseline_samples")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "topic",
                default_value="/linker/hand/tactile",
                description="Hand tactile topic to visualize.",
            ),
            DeclareLaunchArgument(
                "vmin",
                default_value="0.0",
                description="Heatmap lower color scale.",
            ),
            DeclareLaunchArgument(
                "vmax",
                default_value="80.0",
                description="Heatmap upper color scale.",
            ),
            DeclareLaunchArgument(
                "update_hz",
                default_value="20.0",
                description="GUI refresh rate.",
            ),
            DeclareLaunchArgument(
                "cmap",
                default_value="inferno",
                description="Heatmap palette: inferno, viridis, turbo, or gray.",
            ),
            DeclareLaunchArgument(
                "auto_scale",
                default_value="true",
                description="Automatically scale colors to the current tactile range.",
            ),
            DeclareLaunchArgument(
                "delta_mode",
                default_value="true",
                description="Show tactile values after subtracting a no-touch baseline.",
            ),
            DeclareLaunchArgument(
                "baseline_samples",
                default_value="10",
                description="Number of initial frames used for the tactile baseline.",
            ),
            Node(
                package="linker_manipulation",
                executable="tactile_heatmap_node",
                name="tactile_heatmap",
                output="screen",
                parameters=[
                    {
                        "topic": topic,
                        "vmin": ParameterValue(vmin, value_type=float),
                        "vmax": ParameterValue(vmax, value_type=float),
                        "update_hz": ParameterValue(update_hz, value_type=float),
                        "cmap": cmap,
                        "auto_scale": ParameterValue(auto_scale, value_type=bool),
                        "delta_mode": ParameterValue(delta_mode, value_type=bool),
                        "baseline_samples": ParameterValue(baseline_samples, value_type=int),
                    }
                ],
            ),
        ]
    )
