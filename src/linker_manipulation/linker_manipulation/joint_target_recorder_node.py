from __future__ import annotations

import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from .config import load_robot_config


class JointTargetRecorderNode(Node):
    def __init__(self) -> None:
        super().__init__("joint_target_recorder")
        self.declare_parameter("config_path", "")
        self.declare_parameter("target_name", "pregrasp")
        self.declare_parameter("joint_state_topic", "/linker/arm/state")
        self.declare_parameter("timeout_sec", 5.0)

        config_path = str(self.get_parameter("config_path").value or "")
        self._cfg = load_robot_config(config_path or None)
        self._done = threading.Event()
        self._result: list[float] | None = None
        self._error: str | None = None

        topic = str(self.get_parameter("joint_state_topic").value)
        self.create_subscription(JointState, topic, self._handle_joint_state, 10)

    def _handle_joint_state(self, msg: JointState) -> None:
        if self._done.is_set():
            return

        positions = dict(zip(msg.name, msg.position))
        missing = [
            joint_name
            for joint_name in self._cfg.moveit.joint_names
            if joint_name not in positions
        ]
        if missing:
            self._error = f"JointState is missing joints: {missing}"
            self._done.set()
            return

        self._result = [
            float(positions[joint_name])
            for joint_name in self._cfg.moveit.joint_names
        ]
        self._done.set()

    def run(self) -> int:
        timeout_sec = float(self.get_parameter("timeout_sec").value)
        deadline = self.get_clock().now().nanoseconds / 1e9 + timeout_sec
        while rclpy.ok() and not self._done.is_set():
            rclpy.spin_once(self, timeout_sec=0.1)
            now_sec = self.get_clock().now().nanoseconds / 1e9
            if now_sec > deadline:
                self._error = "Timed out waiting for /linker/arm/state"
                break

        if self._result is None:
            self.get_logger().error(self._error or "No joint target captured")
            return 1

        target_name = str(self.get_parameter("target_name").value)
        values = ", ".join(f"{value:.6f}" for value in self._result)
        print("")
        print("Paste this under moveit.joint_targets in robot.yaml:")
        print("")
        print(f"    {target_name}: [{values}]")
        print("")
        return 0


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JointTargetRecorderNode()
    try:
        raise SystemExit(node.run())
    finally:
        node.destroy_node()
        rclpy.shutdown()
