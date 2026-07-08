from __future__ import annotations

import threading
from typing import Any

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger

from linker_manip_interfaces.msg import HandState, HandTactile
from linker_manip_interfaces.srv import SetHandAngles

from .config import HandConfig, load_robot_config
from .ros_utils import FINGER_NAMES, fixed_float_list
from .sdk_loader import ensure_linkerbot_sdk
from .tactile import finger_msg_from_matrix


class L20LiteDriverNode(Node):
    def __init__(self) -> None:
        super().__init__("l20lite_driver")
        self.declare_parameter("config_path", "")
        config_path = self.get_parameter("config_path").get_parameter_value().string_value
        self._cfg: HandConfig = load_robot_config(config_path or None).hand

        self._lock = threading.RLock()
        self._hand: Any | None = None
        self._sensor_source: Any | None = None

        self._state_pub = self.create_publisher(HandState, "/linker/hand/state", 10)
        self._tactile_pub = self.create_publisher(HandTactile, "/linker/hand/tactile", 10)

        self.create_service(Trigger, "/linker/hand/open", self._handle_open)
        self.create_service(SetHandAngles, "/linker/hand/set_angles", self._handle_set_angles)

        self.create_timer(1.0 / max(1.0, self._cfg.state_rate_hz), self._publish_state)
        self.create_timer(1.0 / max(1.0, self._cfg.tactile_rate_hz), self._publish_tactile)

        if self._cfg.connect_on_start:
            self._connect()

    def _connect(self) -> bool:
        with self._lock:
            if self._hand is not None:
                return True
            try:
                ensure_linkerbot_sdk()
                from linkerbot import L20lite
                from linkerbot.hand.l20lite import SensorSource

                if self._cfg.type != "L20lite":
                    raise ValueError(f"Unsupported hand type: {self._cfg.type}")

                self._hand = L20lite(
                    side=self._cfg.side,
                    interface_name=self._cfg.can,
                    interface_type=self._cfg.interface_type,
                )
                self._sensor_source = SensorSource
                self._hand.speed.set_speeds(self._cfg.default_speeds)
                self._hand.torque.set_torques(self._cfg.default_torques)
                self._start_configured_polling()
                self.get_logger().info(
                    f"Connected L20 Lite {self._cfg.side} hand on {self._cfg.can}."
                )
                return True
            except Exception as exc:
                self._hand = None
                self._sensor_source = None
                self.get_logger().error(f"Failed to connect L20 Lite hand: {exc}")
                return False

    def _start_configured_polling(self) -> None:
        if self._hand is None or self._sensor_source is None:
            return
        source = self._sensor_source
        mapping = {
            "angle": source.ANGLE,
            "force_sensor": source.FORCE_SENSOR,
            "speed": source.SPEED,
            "torque": source.TORQUE,
            "temperature": source.TEMPERATURE,
        }
        intervals = {
            mapping[name]: interval
            for name, interval in self._cfg.polling.items()
            if name in mapping and interval > 0
        }
        if intervals:
            self._hand.start_polling(intervals)

    def _require_hand(self) -> Any:
        if not self._connect() or self._hand is None:
            raise RuntimeError("L20 Lite hand is not connected")
        return self._hand

    def _trigger_response(self, success: bool, message: str) -> Trigger.Response:
        response = Trigger.Response()
        response.success = success
        response.message = message
        return response

    def _handle_open(self, _request, _response) -> Trigger.Response:
        try:
            with self._lock:
                hand = self._require_hand()
                hand.speed.set_speeds(self._cfg.default_speeds)
                hand.torque.set_torques(self._cfg.default_torques)
                hand.angle.set_angles(self._cfg.open_angles)
            return self._trigger_response(True, "L20 Lite hand opened")
        except Exception as exc:
            return self._trigger_response(False, str(exc))

    def _handle_set_angles(self, request, _response) -> SetHandAngles.Response:
        response = SetHandAngles.Response()
        try:
            angles = fixed_float_list(request.angles, 10)
            with self._lock:
                self._require_hand().angle.set_angles(angles)
            response.success = True
            response.message = "L20 Lite angles sent"
        except Exception as exc:
            response.success = False
            response.message = str(exc)
        return response

    def _snapshot_values(self, snapshot: Any | None, field_name: str) -> list[float]:
        if snapshot is None:
            return [0.0] * 10
        values = getattr(snapshot, field_name, None)
        if values is None:
            return [0.0] * 10
        if hasattr(values, "to_list"):
            return fixed_float_list(values.to_list(), 10)
        return fixed_float_list(values, 10)

    def _publish_state(self) -> None:
        with self._lock:
            if self._hand is None:
                return
            try:
                msg = HandState()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "l20lite_hand"
                msg.angles = self._snapshot_values(self._hand.angle.get_snapshot(), "angles")
                msg.speeds = self._snapshot_values(self._hand.speed.get_snapshot(), "speeds")
                msg.torques = self._snapshot_values(self._hand.torque.get_snapshot(), "torques")
                msg.temperatures = self._snapshot_values(
                    self._hand.temperature.get_snapshot(), "temperatures"
                )
                self._state_pub.publish(msg)
            except Exception as exc:
                self.get_logger().warn(f"Failed to publish L20 Lite state: {exc}")

    def _publish_tactile(self) -> None:
        with self._lock:
            if self._hand is None:
                return
            try:
                data = self._hand.force_sensor.get_snapshot()
                if data is None:
                    return
                msg = HandTactile()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "l20lite_hand"
                for name in FINGER_NAMES:
                    finger = getattr(data, name)
                    msg.fingers.append(finger_msg_from_matrix(name, finger.values))
                self._tactile_pub.publish(msg)
            except Exception as exc:
                self.get_logger().warn(f"Failed to publish L20 Lite tactile data: {exc}")

    def destroy_node(self) -> bool:
        with self._lock:
            if self._hand is not None:
                try:
                    self._hand.close()
                except Exception:
                    pass
                self._hand = None
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = L20LiteDriverNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
