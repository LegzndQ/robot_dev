from __future__ import annotations

import threading
from typing import Any

from control_msgs.action import FollowJointTrajectory
import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectoryPoint

from linker_manip_interfaces.action import MoveArm
from linker_manip_interfaces.msg import TcpPose

from .config import ArmConfig, load_robot_config
from .ros_utils import pose_msg_from_sdk_pose, sdk_pose_from_msg
from .sdk_loader import ensure_linkerbot_sdk


class A7DriverNode(Node):
    def __init__(self) -> None:
        super().__init__("a7_driver")
        self.declare_parameter("config_path", "")
        self.declare_parameter("trajectory_execution_mode", "sparse")
        self.declare_parameter("trajectory_min_waypoint_delta", 0.06)
        self.declare_parameter("trajectory_max_waypoints", 25)
        self.declare_parameter("trajectory_joint_velocity", 0.20)
        self.declare_parameter("trajectory_joint_acceleration", 1.0)
        config_path = self.get_parameter("config_path").get_parameter_value().string_value
        self._cfg: ArmConfig = load_robot_config(config_path or None).arm

        self._lock = threading.RLock()
        self._arm: Any | None = None
        self._last_pose = TcpPose()
        self._motion_callback_group = MutuallyExclusiveCallbackGroup()
        self._state_stop_event = threading.Event()

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
            callback_group=self._motion_callback_group,
        )
        self._trajectory_server = ActionServer(
            self,
            FollowJointTrajectory,
            "/linker_arm_controller/follow_joint_trajectory",
            execute_callback=self._execute_follow_joint_trajectory,
            callback_group=self._motion_callback_group,
        )

        period = 1.0 / max(1.0, self._cfg.state_rate_hz)
        self._state_thread = threading.Thread(
            target=self._state_publish_loop,
            args=(period,),
            daemon=True,
        )
        self._state_thread.start()

        if self._cfg.connect_on_start:
            self._connect()

    def _joint_names(self) -> list[str]:
        arm_type = self._cfg.type.lower().replace("_", "")
        if arm_type == "a7lite":
            prefix = "L" if self._cfg.side == "left" else "R"
            return [f"{prefix}{i}_JOINT" for i in range(1, 8)]
        if self._cfg.side == "left":
            prefix = "Left"
        else:
            prefix = "Right"
        return [
            f"{prefix}_Shoulder_Pitch_Joint",
            f"{prefix}_Shoulder_Roll_Joint",
            f"{prefix}_Shoulder_Yaw_Joint",
            f"{prefix}_Elbow_Pitch_Joint",
            f"{prefix}_Wrist_Yaw_Joint",
            f"{prefix}_Wrist_Pitch_Joint",
            f"{prefix}_Wrist_Roll_Joint",
        ]

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

    def _ordered_values(
        self,
        values: list[float] | tuple[float, ...],
        order: list[int],
    ) -> list[float]:
        return [float(values[index]) for index in order]

    def _current_joint_positions(self) -> list[float]:
        arm = self._arm
        if arm is None:
            return []
        state = arm.get_state()
        return [float(v.angle) for v in state.joint_angles]

    def _joint_distance(self, first: list[float], second: list[float]) -> float:
        return max(abs(a - b) for a, b in zip(first, second))

    def _downsample_targets(
        self,
        targets: list[list[float]],
        max_waypoints: int,
    ) -> list[list[float]]:
        if max_waypoints <= 0 or len(targets) <= max_waypoints:
            return targets
        if max_waypoints == 1:
            return [targets[-1]]

        last_index = len(targets) - 1
        selected: list[list[float]] = []
        for output_index in range(max_waypoints):
            source_index = round(output_index * last_index / (max_waypoints - 1))
            selected.append(targets[source_index])
        return selected

    def _select_trajectory_targets(
        self,
        points: list[JointTrajectoryPoint],
        order: list[int],
        joint_count: int,
    ) -> list[list[float]]:
        all_targets: list[list[float]] = []
        for point_index, point in enumerate(points):
            if len(point.positions) != joint_count:
                raise ValueError(
                    f"Point {point_index} has {len(point.positions)} positions, "
                    f"expected {joint_count}"
                )
            all_targets.append(self._ordered_values(point.positions, order))

        mode = str(self.get_parameter("trajectory_execution_mode").value).lower()
        if mode in ("final", "final_only"):
            return [all_targets[-1]]
        if mode == "all":
            return all_targets
        if mode != "sparse":
            raise ValueError(
                "trajectory_execution_mode must be one of: sparse, all, final"
            )

        min_delta = float(self.get_parameter("trajectory_min_waypoint_delta").value)
        max_waypoints = int(self.get_parameter("trajectory_max_waypoints").value)
        selected: list[list[float]] = []
        last_selected: list[float] | None = None

        # MoveIt trajectories usually include the current state as the first point.
        # Skip tiny consecutive changes so the SDK does not stop/start at every sample.
        for target in all_targets[:-1]:
            if last_selected is None:
                last_selected = target
                continue
            if self._joint_distance(last_selected, target) >= min_delta:
                selected.append(target)
                last_selected = target

        final_target = all_targets[-1]
        if not selected or self._joint_distance(selected[-1], final_target) > 1e-6:
            selected.append(final_target)

        return self._downsample_targets(selected, max_waypoints)

    def _trajectory_limits(self) -> tuple[list[float], list[float]]:
        velocity = float(self.get_parameter("trajectory_joint_velocity").value)
        acceleration = float(self.get_parameter("trajectory_joint_acceleration").value)
        velocities = (
            [velocity] * 7
            if velocity > 0.0
            else [float(v) for v in self._cfg.default_joint_velocities]
        )
        accelerations = (
            [acceleration] * 7
            if acceleration > 0.0
            else [float(v) for v in self._cfg.default_joint_accelerations]
        )
        return velocities, accelerations

    def _publish_trajectory_feedback(
        self,
        goal_handle,
        joint_names: list[str],
        desired_positions: list[float],
    ) -> None:
        feedback = FollowJointTrajectory.Feedback()
        feedback.header.stamp = self.get_clock().now().to_msg()
        feedback.joint_names = joint_names

        desired = JointTrajectoryPoint()
        desired.positions = desired_positions
        feedback.desired = desired

        try:
            current_positions = self._current_joint_positions()
        except Exception:
            current_positions = []
        if len(current_positions) == len(desired_positions):
            actual = JointTrajectoryPoint()
            actual.positions = current_positions
            feedback.actual = actual

            error = JointTrajectoryPoint()
            error.positions = [
                float(actual_pos - desired_pos)
                for actual_pos, desired_pos in zip(current_positions, desired_positions)
            ]
            feedback.error = error

        goal_handle.publish_feedback(feedback)

    def _execute_follow_joint_trajectory(self, goal_handle):
        result = FollowJointTrajectory.Result()
        goal = goal_handle.request
        trajectory = goal.trajectory
        expected_joint_names = self._joint_names()

        try:
            if not trajectory.points:
                raise ValueError("Trajectory has no points")

            missing = [
                joint_name
                for joint_name in expected_joint_names
                if joint_name not in trajectory.joint_names
            ]
            if missing:
                result.error_code = FollowJointTrajectory.Result.INVALID_JOINTS
                result.error_string = f"Trajectory missing joints: {missing}"
                goal_handle.abort()
                return result

            order = [
                trajectory.joint_names.index(joint_name)
                for joint_name in expected_joint_names
            ]

            with self._lock:
                arm = self._require_arm()
                velocities, accelerations = self._trajectory_limits()
                arm.set_velocities(velocities)
                arm.set_accelerations(accelerations)

                targets = self._select_trajectory_targets(
                    trajectory.points,
                    order,
                    len(trajectory.joint_names),
                )
                self.get_logger().info(
                    "Executing MoveIt trajectory with "
                    f"{len(targets)}/{len(trajectory.points)} waypoints "
                    f"(mode={self.get_parameter('trajectory_execution_mode').value}, "
                    f"velocity={velocities[0]:.3f}, acceleration={accelerations[0]:.3f})"
                )

                for target in targets:
                    if goal_handle.is_cancel_requested:
                        arm.emergency_stop()
                        result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                        result.error_string = "Trajectory execution canceled; emergency stop sent"
                        goal_handle.canceled()
                        return result

                    self._publish_trajectory_feedback(
                        goal_handle,
                        expected_joint_names,
                        target,
                    )
                    arm.move_j(target, blocking=True)

            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            result.error_string = ""
            goal_handle.succeed()
            return result
        except Exception as exc:
            self.get_logger().error(f"FollowJointTrajectory failed: {exc}")
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = str(exc)
            goal_handle.abort()
            return result

    def _publish_state(self) -> None:
        arm = self._arm
        if arm is None:
            return
        try:
            state = arm.get_state()
            stamp = self.get_clock().now().to_msg()

            joint_state = JointState()
            joint_state.header.stamp = stamp
            joint_state.name = self._joint_names()
            joint_state.position = [float(v.angle) for v in state.joint_angles]
            joint_state.velocity = [float(v.velocity) for v in state.joint_velocities]
            joint_state.effort = [float(v.torque) for v in state.joint_torques]
            self._joint_pub.publish(joint_state)

            pose_msg = pose_msg_from_sdk_pose(state.pose)
            self._last_pose = pose_msg
            self._pose_pub.publish(pose_msg)
        except Exception as exc:
            self.get_logger().warn(f"Failed to publish {self._cfg.type} state: {exc}")

    def _state_publish_loop(self, period: float) -> None:
        while not self._state_stop_event.wait(period):
            self._publish_state()

    def destroy_node(self) -> bool:
        self._state_stop_event.set()
        if self._state_thread.is_alive():
            self._state_thread.join(timeout=2.0)
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
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
