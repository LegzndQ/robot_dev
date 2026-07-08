from __future__ import annotations

import threading
from typing import Any

import rclpy
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from linker_manip_interfaces.action import MoveArm
from linker_manip_interfaces.msg import TcpPose

from .config import ArmConfig, load_robot_config
from .ros_utils import pose_msg_from_sdk_pose, sdk_pose_from_msg
from .sdk_loader import ensure_linkerbot_sdk


class A7DriverNode(Node):
    def __init__(self) -> None:
        super().__init__("a7_driver")
        self.declare_parameter("config_path", "")
        config_path = self.get_parameter("config_path").get_parameter_value().string_value
        self._cfg: ArmConfig = load_robot_config(config_path or None).arm

        self._lock = threading.RLock()
        self._arm: Any | None = None
        self._last_pose = TcpPose()

        self._joint_pub = self.create_publisher(JointState, "/linker/arm/state", 10)
        self._pose_pub = self.create_publisher(TcpPose, "/linker/arm/tcp_pose", 10)

        self.create_service(Trigger, "/linker/arm/enable", self._handle_enable)
        self.create_service(Trigger, "/linker/arm/disable", self._handle_disable)
        self.create_service(Trigger, "/linker/arm/home", self._handle_home)
        self.create_service(Trigger, "/linker/arm/emergency_stop", self._handle_emergency_stop)

        self._move_server = ActionServer(
            self,
            MoveArm,
            "/linker/arm/move_arm",
            execute_callback=self._execute_move_arm,
        )

        period = 1.0 / max(1.0, self._cfg.state_rate_hz)
        self.create_timer(period, self._publish_state)

        if self._cfg.connect_on_start:
            self._connect()

    def _connect(self) -> bool:
        with self._lock:
            if self._arm is not None:
                return True
            try:
                ensure_linkerbot_sdk()
                from linkerbot import A7, A7lite

                arm_type = self._cfg.type.lower().replace("_", "")
                if arm_type == "a7":
                    arm_class = A7
                    display_type = "A7"
                elif arm_type == "a7lite":
                    arm_class = A7lite
                    display_type = "A7lite"
                else:
                    raise ValueError(f"Unsupported arm type: {self._cfg.type}")

                self._arm = arm_class(
                    side=self._cfg.side,
                    interface_name=self._cfg.can,
                    interface_type=self._cfg.interface_type,
                    tcp_offset=self._cfg.tcp_offset,
                    world_frame=self._cfg.world_frame,
                )
                self._arm.set_velocities(self._cfg.default_joint_velocities)
                self._arm.set_accelerations(self._cfg.default_joint_accelerations)
                self.get_logger().info(
                    f"Connected {display_type} {self._cfg.side} arm on {self._cfg.can} "
                    f"({self._cfg.world_frame} frame)."
                )
                return True
            except Exception as exc:
                self._arm = None
                self.get_logger().error(f"Failed to connect {self._cfg.type} arm: {exc}")
                return False

    def _require_arm(self) -> Any:
        if not self._connect() or self._arm is None:
            raise RuntimeError(f"{self._cfg.type} arm is not connected")
        return self._arm

    def _trigger_response(self, success: bool, message: str) -> Trigger.Response:
        response = Trigger.Response()
        response.success = success
        response.message = message
        return response

    def _handle_enable(self, _request, _response) -> Trigger.Response:
        try:
            with self._lock:
                self._require_arm().enable()
            return self._trigger_response(True, f"{self._cfg.type} arm enabled")
        except Exception as exc:
            return self._trigger_response(False, str(exc))

    def _handle_disable(self, _request, _response) -> Trigger.Response:
        try:
            with self._lock:
                self._require_arm().disable()
            return self._trigger_response(True, f"{self._cfg.type} arm disabled")
        except Exception as exc:
            return self._trigger_response(False, str(exc))

    def _handle_home(self, _request, _response) -> Trigger.Response:
        try:
            with self._lock:
                self._require_arm().home(blocking=True)
            return self._trigger_response(True, f"{self._cfg.type} arm homed")
        except Exception as exc:
            return self._trigger_response(False, str(exc))

    def _handle_emergency_stop(self, _request, _response) -> Trigger.Response:
        try:
            with self._lock:
                self._require_arm().emergency_stop()
            return self._trigger_response(True, f"{self._cfg.type} emergency stop sent")
        except Exception as exc:
            return self._trigger_response(False, str(exc))

    def _publish_move_feedback(self, goal_handle, phase: str, progress: float) -> None:
        feedback = MoveArm.Feedback()
        feedback.phase = phase
        feedback.progress = float(progress)
        feedback.current_pose = self._read_pose_msg()
        goal_handle.publish_feedback(feedback)

    def _read_pose_msg(self) -> TcpPose:
        try:
            if self._arm is None:
                return self._last_pose
            pose = self._arm.get_pose()
            self._last_pose = pose_msg_from_sdk_pose(pose)
            return self._last_pose
        except Exception:
            return self._last_pose

    def _execute_move_arm(self, goal_handle):
        goal = goal_handle.request
        result = MoveArm.Result()
        try:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.error = "Goal canceled before execution"
                return result

            with self._lock:
                arm = self._require_arm()
                if len(goal.joint_velocities) == 7:
                    arm.set_velocities([float(v) for v in goal.joint_velocities])
                if len(goal.joint_accelerations) == 7:
                    arm.set_accelerations([float(v) for v in goal.joint_accelerations])

                self._publish_move_feedback(goal_handle, "executing", 0.0)
                if goal.mode == MoveArm.Goal.MODE_JOINT:
                    if len(goal.target_joints) != 7:
                        raise ValueError("JOINT mode requires exactly 7 target_joints")
                    arm.move_j([float(v) for v in goal.target_joints], blocking=True)
                elif goal.mode == MoveArm.Goal.MODE_POSE:
                    arm.move_p(sdk_pose_from_msg(goal.target_pose), blocking=True)
                elif goal.mode == MoveArm.Goal.MODE_LINEAR:
                    kwargs: dict[str, float] = {}
                    if goal.max_velocity > 0:
                        kwargs["max_velocity"] = float(goal.max_velocity)
                    if goal.max_angular_velocity > 0:
                        kwargs["max_angular_velocity"] = float(goal.max_angular_velocity)
                    if goal.acceleration > 0:
                        kwargs["acceleration"] = float(goal.acceleration)
                    if goal.angular_acceleration > 0:
                        kwargs["angular_acceleration"] = float(goal.angular_acceleration)
                    arm.move_l(sdk_pose_from_msg(goal.target_pose), **kwargs)
                else:
                    raise ValueError(f"Unsupported MoveArm mode: {goal.mode}")

            if goal_handle.is_cancel_requested:
                with self._lock:
                    self._require_arm().emergency_stop()
                goal_handle.canceled()
                result.success = False
                result.error = "Goal canceled after motion"
                return result

            self._publish_move_feedback(goal_handle, "done", 1.0)
            goal_handle.succeed()
            result.success = True
            result.error = ""
            return result
        except Exception as exc:
            self.get_logger().error(f"MoveArm failed: {exc}")
            goal_handle.abort()
            result.success = False
            result.error = str(exc)
            return result

    def _publish_state(self) -> None:
        with self._lock:
            arm = self._arm
            if arm is None:
                return
            try:
                state = arm.get_state()
                stamp = self.get_clock().now().to_msg()

                joint_state = JointState()
                joint_state.header.stamp = stamp
                joint_state.name = [f"a7_joint_{i + 1}" for i in range(7)]
                joint_state.position = [float(v.angle) for v in state.joint_angles]
                joint_state.velocity = [float(v.velocity) for v in state.joint_velocities]
                joint_state.effort = [float(v.torque) for v in state.joint_torques]
                self._joint_pub.publish(joint_state)

                pose_msg = pose_msg_from_sdk_pose(state.pose)
                self._last_pose = pose_msg
                self._pose_pub.publish(pose_msg)
            except Exception as exc:
                self.get_logger().warn(f"Failed to publish {self._cfg.type} state: {exc}")

    def destroy_node(self) -> bool:
        with self._lock:
            if self._arm is not None:
                try:
                    self._arm.close()
                except Exception:
                    pass
                self._arm = None
        self._move_server.destroy()
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = A7DriverNode()
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
