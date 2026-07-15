from pathlib import Path
from xml.etree import ElementTree


REAL_HARDWARE_PLUGIN = "linker_a7_ros2_control/A7LiteSystem"
MOCK_HARDWARE_PLUGIN = "mock_components/GenericSystem"


def build_robot_description(
    urdf_path: str,
    *,
    use_mock_hardware: bool,
    hardware_parameters: dict[str, str] | None = None,
) -> str:
    root = ElementTree.parse(Path(urdf_path)).getroot()
    hardware = root.find("./ros2_control/hardware")
    if hardware is None:
        raise RuntimeError(f"No ros2_control/hardware element in {urdf_path}")

    for child in list(hardware):
        hardware.remove(child)

    plugin = ElementTree.SubElement(hardware, "plugin")
    plugin.text = MOCK_HARDWARE_PLUGIN if use_mock_hardware else REAL_HARDWARE_PLUGIN

    if not use_mock_hardware:
        for name, value in (hardware_parameters or {}).items():
            if value == "":
                continue
            parameter = ElementTree.SubElement(hardware, "param", {"name": name})
            parameter.text = str(value)

    return ElementTree.tostring(root, encoding="unicode")
