from __future__ import annotations

from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster


class CameraExtrinsicsNode(Node):
    def __init__(self) -> None:
        super().__init__("iphone_camera_extrinsics")
        self.declare_parameter(
            "extrinsics_path", "~/.ros/camera_info/iphone_to_base.yaml"
        )
        path = Path(
            str(self.get_parameter("extrinsics_path").value)
        ).expanduser()
        document = yaml.safe_load(path.read_text(encoding="utf-8"))
        message = TransformStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = str(document["parent_frame"])
        message.child_frame_id = str(document["child_frame"])
        translation = document["translation"]
        rotation = document["rotation"]
        message.transform.translation.x = float(translation["x"])
        message.transform.translation.y = float(translation["y"])
        message.transform.translation.z = float(translation["z"])
        message.transform.rotation.x = float(rotation["x"])
        message.transform.rotation.y = float(rotation["y"])
        message.transform.rotation.z = float(rotation["z"])
        message.transform.rotation.w = float(rotation["w"])
        self._broadcaster = StaticTransformBroadcaster(self)
        self._broadcaster.sendTransform(message)
        self.get_logger().info(
            f"Published static TF {message.header.frame_id} -> "
            f"{message.child_frame_id} from {path}"
        )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = CameraExtrinsicsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
