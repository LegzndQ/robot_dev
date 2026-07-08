from __future__ import annotations

import threading
import time

import numpy as np
import rclpy
from rclpy.action import ActionClient, ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger

from linker_manip_interfaces.action import Grasp, MoveArm
from linker_manip_interfaces.msg import HandState, HandTactile
from linker_manip_interfaces.srv import SetHandAngles

from .config import GraspConfig, HandConfig, load_robot_config
from .ros_utils import clamp, fixed_float_list, pose_msg_from_list
from .tactile import TactileSnapshot, hand_tactile_to_snapshot


class GraspControllerNode(Node):
    def __init__(self) -> None:
        super().__init__("grasp_controller")
        self.declare_parameter("config_path", "")
        config_path = self.get_parameter("config_path").get_parameter_value().string_value
        robot_cfg = load_robot_config(config_path or None)
        self._grasp_cfg: GraspConfig = robot_cfg.grasp
        self._hand_cfg: HandConfig = robot_cfg.hand

        self._cb_group = ReentrantCallbackGroup()
        self._data_lock = threading.RLock()
        self._latest_hand_state: HandState | None = None
        self._latest_tactile: HandTactile | None = None
        self._baseline: dict[str, np.ndarray] = {}

        self.create_subscription(
            HandState,
            "/linker/hand/state",
            self._on_hand_state,
            10,
            callback_group=self._cb_group,
        )
        self.create_subscription(
            HandTactile,
            "/linker/hand/tactile",
            self._on_tactile,
            10,
            callback_group=self._cb_group,
        )

        self._set_angles_client = self.create_client(
            SetHandAngles,
            "/linker/hand/set_angles",
            callback_group=self._cb_group,
        )
        self._open_client = self.create_client(
            Trigger,
            "/linker/hand/open",
            callback_group=self._cb_group,
        )
        self._arm_estop_client = self.create_client(
            Trigger,
            "/linker/arm/emergency_stop",
            callback_group=self._cb_group,
        )
        self._move_arm_client = ActionClient(
            self,
            MoveArm,
            "/linker/arm/move_arm",
            callback_group=self._cb_group,
        )

        self.create_service(
            Trigger,
            "/linker/grasp/calibrate_baseline",
            self._handle_calibrate_baseline,
            callback_group=self._cb_group,
        )
        self._grasp_server = ActionServer(
            self,
            Grasp,
            "/linker/grasp",
            execute_callback=self._execute_grasp,
            callback_group=self._cb_group,
        )

    def _on_hand_state(self, msg: HandState) -> None:
        with self._data_lock:
            self._latest_hand_state = msg

    def _on_tactile(self, msg: HandTactile) -> None:
        with self._data_lock:
            self._latest_tactile = msg

    def _wait_future(self, future, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and not future.done():
            if time.monotonic() >= deadline:
                return False
            time.sleep(0.01)
        return future.done()

    def _call_trigger(self, client, name: str, timeout_sec: float = 5.0) -> Trigger.Response:
        if not client.wait_for_service(timeout_sec=timeout_sec):
            raise RuntimeError(f"Service unavailable: {name}")
        future = client.call_async(Trigger.Request())
        if not self._wait_future(future, timeout_sec):
            raise TimeoutError(f"Timed out waiting for {name}")
        response = future.result()
        if not response.success:
            raise RuntimeError(f"{name} failed: {response.message}")
        return response

    def _set_angles(self, angles: list[float], timeout_sec: float = 5.0) -> None:
        if not self._set_angles_client.wait_for_service(timeout_sec=timeout_sec):
            raise RuntimeError("Service unavailable: /linker/hand/set_angles")
        request = SetHandAngles.Request()
        request.angles = fixed_float_list(angles, 10)
        future = self._set_angles_client.call_async(request)
        if not self._wait_future(future, timeout_sec):
            raise TimeoutError("Timed out waiting for /linker/hand/set_angles")
        response = future.result()
        if not response.success:
            raise RuntimeError(f"/linker/hand/set_angles failed: {response.message}")

    def _handle_calibrate_baseline(self, _request, _response) -> Trigger.Response:
        response = Trigger.Response()
        try:
            self._calibrate_baseline(self._grasp_cfg.baseline_samples)
            response.success = True
            response.message = "Tactile baseline calibrated"
        except Exception as exc:
            response.success = False
            response.message = str(exc)
        return response

    def _calibrate_baseline(self, samples: int) -> None:
        collected: dict[str, list[np.ndarray]] = {
            finger: [] for finger in self._grasp_cfg.active_fingers
        }
        deadline = time.monotonic() + max(3.0, samples * 0.2)

        while time.monotonic() < deadline:
            with self._data_lock:
                tactile_msg = self._latest_tactile
            if tactile_msg is not None:
                snapshot = hand_tactile_to_snapshot(tactile_msg)
                for finger in self._grasp_cfg.active_fingers:
                    collected[finger].append(snapshot.values[finger].astype(np.float32))
            if all(len(items) >= samples for items in collected.values()):
                break
            time.sleep(self._grasp_cfg.regulate_period_sec)

        if not any(collected.values()):
            raise RuntimeError("No tactile data available for baseline calibration")

        baseline: dict[str, np.ndarray] = {}
        for finger, matrices in collected.items():
            if matrices:
                baseline[finger] = np.mean(matrices[:samples], axis=0)
            else:
                baseline[finger] = np.zeros((12, 6), dtype=np.float32)
        with self._data_lock:
            self._baseline = baseline

    def _current_tactile(self) -> TactileSnapshot | None:
        with self._data_lock:
            msg = self._latest_tactile
            baseline = dict(self._baseline)
        if msg is None:
            return None
        return hand_tactile_to_snapshot(msg, baseline)

    def _current_angles(self) -> list[float]:
        with self._data_lock:
            msg = self._latest_hand_state
        if msg is None:
            return list(self._hand_cfg.open_angles)
        return fixed_float_list(msg.angles, 10)

    def _active_scores(self) -> list[float]:
        snapshot = self._current_tactile()
        if snapshot is None:
            return []
        return [float(snapshot.scores.get(finger, 0.0)) for finger in self._grasp_cfg.active_fingers]

    def _aggregate_force(self) -> float:
        scores = self._active_scores()
        return float(np.mean(scores)) if scores else 0.0

    def _step_towards(self, current: list[float], target: list[float], step: float) -> list[float]:
        next_angles: list[float] = []
        for cur, tgt in zip(current, target):
            diff = tgt - cur
            if abs(diff) <= step:
                next_angles.append(tgt)
            else:
                next_angles.append(cur + (step if diff > 0 else -step))
        return [clamp(v, 0.0, 100.0) for v in next_angles]

    def _can_step_towards(self, current: list[float], target: list[float]) -> bool:
        return any(abs(cur - tgt) > 0.5 for cur, tgt in zip(current, target))

    def _publish_feedback(self, goal_handle, phase: str, progress: float) -> None:
        feedback = Grasp.Feedback()
        feedback.phase = phase
        feedback.finger_scores = self._active_scores()
        feedback.progress = float(progress)
        goal_handle.publish_feedback(feedback)

    def _check_cancel(self, goal_handle) -> None:
        if goal_handle.is_cancel_requested:
            try:
                self._call_trigger(self._arm_estop_client, "/linker/arm/emergency_stop", timeout_sec=1.0)
            except Exception:
                pass
            raise InterruptedError("Grasp goal canceled")

    def _close_until_contact(self, goal_handle, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            self._check_cancel(goal_handle)
            scores = self._active_scores()
            if scores and max(scores) >= self._grasp_cfg.contact_threshold:
                return True
            current = self._current_angles()
            if not self._can_step_towards(current, self._grasp_cfg.max_closed_angles):
                return False
            self._set_angles(
                self._step_towards(
                    current, self._grasp_cfg.max_closed_angles, self._grasp_cfg.close_step
                )
            )
            self._publish_feedback(goal_handle, "close_until_contact", 0.35)
            time.sleep(self._grasp_cfg.regulate_period_sec)
        return False

    def _regulate_force(
        self,
        goal_handle,
        force_low: float,
        force_high: float,
        timeout_sec: float,
    ) -> bool:
        stable = 0
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            self._check_cancel(goal_handle)
            aggregate = self._aggregate_force()
            current = self._current_angles()
            if aggregate < force_low:
                stable = 0
                if not self._can_step_towards(current, self._grasp_cfg.max_closed_angles):
                    return False
                target = self._step_towards(
                    current, self._grasp_cfg.max_closed_angles, self._grasp_cfg.close_step
                )
                self._set_angles(target)
            elif aggregate > force_high:
                stable = 0
                target = self._step_towards(
                    current, self._hand_cfg.open_angles, self._grasp_cfg.open_step
                )
                self._set_angles(target)
            else:
                stable += 1
                if stable >= self._grasp_cfg.stable_cycles:
                    return True
            self._publish_feedback(goal_handle, "force_regulate", 0.50)
            time.sleep(self._grasp_cfg.regulate_period_sec)
        return False

    def _monitor_slip(self, reference_force: float, force_low: float, low_since: float | None) -> float | None:
        aggregate = self._aggregate_force()
        low_limit = max(force_low, reference_force * (1.0 - self._grasp_cfg.slip_drop_ratio))
        now = time.monotonic()
        if aggregate >= low_limit:
            return None
        if low_since is None:
            return now
        if now - low_since >= self._grasp_cfg.slip_window_sec:
            current = self._current_angles()
            if not self._can_step_towards(current, self._grasp_cfg.max_closed_angles):
                raise RuntimeError("Slip detected and fingers are already at closure limit")
            self._set_angles(
                self._step_towards(
                    current, self._grasp_cfg.max_closed_angles, self._grasp_cfg.close_step
                )
            )
            return now
        return low_since

    def _make_pose_goal(self, mode: int, pose_name: str) -> MoveArm.Goal:
        if pose_name not in self._grasp_cfg.poses:
            raise KeyError(f"Missing pose in config: grasp.poses.{pose_name}")
        goal = MoveArm.Goal()
        goal.mode = mode
        goal.target_pose = pose_msg_from_list(self._grasp_cfg.poses[pose_name])
        return goal

    def _move_arm(
        self,
        goal: MoveArm.Goal,
        timeout_sec: float,
        phase: str,
        grasp_goal_handle=None,
        monitor_slip: bool = False,
        reference_force: float = 0.0,
        force_low: float = 0.0,
    ) -> MoveArm.Result:
        if not self._move_arm_client.wait_for_server(timeout_sec=timeout_sec):
            raise RuntimeError("Action server unavailable: /linker/arm/move_arm")
        goal_future = self._move_arm_client.send_goal_async(goal)
        if not self._wait_future(goal_future, timeout_sec):
            raise TimeoutError("Timed out sending MoveArm goal")
        arm_goal_handle = goal_future.result()
        if not arm_goal_handle.accepted:
            raise RuntimeError("MoveArm goal rejected")

        result_future = arm_goal_handle.get_result_async()
        low_since: float | None = None
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and not result_future.done():
            if time.monotonic() > deadline:
                raise TimeoutError(f"MoveArm phase timed out: {phase}")
            if grasp_goal_handle is not None:
                self._check_cancel(grasp_goal_handle)
                self._publish_feedback(grasp_goal_handle, phase, 0.70 if monitor_slip else 0.25)
            if monitor_slip:
                low_since = self._monitor_slip(reference_force, force_low, low_since)
            time.sleep(self._grasp_cfg.regulate_period_sec)

        wrapped = result_future.result()
        result = wrapped.result
        if not result.success:
            raise RuntimeError(f"MoveArm failed during {phase}: {result.error}")
        return result

    def _hold_with_monitor(
        self,
        goal_handle,
        reference_force: float,
        force_low: float,
        hold_sec: float,
    ) -> None:
        deadline = time.monotonic() + hold_sec
        low_since: float | None = None
        while time.monotonic() < deadline:
            self._check_cancel(goal_handle)
            low_since = self._monitor_slip(reference_force, force_low, low_since)
            self._publish_feedback(goal_handle, "hold", 0.80)
            time.sleep(self._grasp_cfg.regulate_period_sec)

    def _execute_grasp(self, goal_handle):
        request = goal_handle.request
        result = Grasp.Result()
        force_low = float(request.force_low) if request.force_low > 0 else self._grasp_cfg.force_low
        force_high = float(request.force_high) if request.force_high > 0 else self._grasp_cfg.force_high
        timeout_sec = (
            float(request.timeout_sec)
            if request.timeout_sec > 0
            else self._grasp_cfg.action_timeout_sec
        )

        try:
            self._check_cancel(goal_handle)
            self._publish_feedback(goal_handle, "open", 0.05)
            self._call_trigger(self._open_client, "/linker/hand/open")
            time.sleep(0.5)

            self._calibrate_baseline(self._grasp_cfg.baseline_samples)

            self._publish_feedback(goal_handle, "approach", 0.15)
            self._move_arm(
                self._make_pose_goal(MoveArm.Goal.MODE_POSE, "pregrasp_pose"),
                timeout_sec,
                "approach",
                goal_handle,
            )

            self._publish_feedback(goal_handle, "pregrasp", 0.25)
            self._set_angles(self._hand_cfg.pregrasp_angles)
            self._move_arm(
                self._make_pose_goal(MoveArm.Goal.MODE_LINEAR, "grasp_pose"),
                timeout_sec,
                "pregrasp",
                goal_handle,
            )

            if not self._close_until_contact(goal_handle, timeout_sec):
                raise RuntimeError("No tactile contact before closure limit")

            if not self._regulate_force(goal_handle, force_low, force_high, timeout_sec):
                raise RuntimeError("Unable to regulate grasp force into target band")

            reference_force = max(force_low, self._aggregate_force())

            self._publish_feedback(goal_handle, "lift", 0.65)
            self._move_arm(
                self._make_pose_goal(MoveArm.Goal.MODE_LINEAR, "lift_pose"),
                timeout_sec,
                "lift",
                goal_handle,
                monitor_slip=True,
                reference_force=reference_force,
                force_low=force_low,
            )

            self._hold_with_monitor(goal_handle, reference_force, force_low, self._grasp_cfg.hold_sec)

            self._publish_feedback(goal_handle, "place", 0.90)
            self._move_arm(
                self._make_pose_goal(MoveArm.Goal.MODE_LINEAR, "place_pose"),
                timeout_sec,
                "place",
                goal_handle,
                monitor_slip=True,
                reference_force=reference_force,
                force_low=force_low,
            )

            self._publish_feedback(goal_handle, "release", 0.98)
            self._call_trigger(self._open_client, "/linker/hand/open")

            goal_handle.succeed()
            result.success = True
            result.result_code = "ok"
            result.error = ""
            return result
        except InterruptedError as exc:
            goal_handle.canceled()
            result.success = False
            result.result_code = "canceled"
            result.error = str(exc)
            return result
        except Exception as exc:
            self.get_logger().error(f"Grasp failed: {exc}")
            try:
                self._call_trigger(self._open_client, "/linker/hand/open", timeout_sec=1.0)
            except Exception:
                pass
            goal_handle.abort()
            result.success = False
            result.result_code = "failed"
            result.error = str(exc)
            return result

    def destroy_node(self) -> bool:
        self._grasp_server.destroy()
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = GraspControllerNode()
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
