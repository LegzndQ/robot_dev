from __future__ import annotations

import subprocess
import threading
import time
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger

from linker_manip_interfaces.action import Grasp

from .config import DemoConfig, load_robot_config


class DemoTaskNode(Node):
    def __init__(self) -> None:
        super().__init__("demo_task")
        self.declare_parameter("config_path", "")
        config_path = self.get_parameter("config_path").get_parameter_value().string_value
        self._cfg: DemoConfig = load_robot_config(config_path or None).demo
        self.declare_parameter("enable_and_home", self._cfg.enable_and_home)
        self.declare_parameter("target_profile", "water_bottle")
        self._enable_and_home = bool(
            self.get_parameter("enable_and_home").value
        )
        self._target_profile = str(self.get_parameter("target_profile").value)

        self._cb_group = ReentrantCallbackGroup()
        self._run_lock = threading.Lock()
        self._goal_lock = threading.RLock()
        self._active_goal_handle = None
        self._last_feedback_phase = ""
        self._arm_enable_client = self.create_client(
            Trigger, "/linker/arm/enable", callback_group=self._cb_group
        )
        self._arm_home_client = self.create_client(
            Trigger, "/linker/arm/home", callback_group=self._cb_group
        )
        self._grasp_client = ActionClient(
            self, Grasp, "/linker/grasp", callback_group=self._cb_group
        )
        self.create_service(
            Trigger,
            "/linker/demo/run_once",
            self._handle_run_once,
            callback_group=self._cb_group,
        )
        self.create_service(
            Trigger,
            "/linker/demo/cancel",
            self._handle_cancel,
            callback_group=self._cb_group,
        )

        self._auto_started = False
        if self._cfg.run_once_on_start:
            self.create_timer(1.0, self._auto_start_once, callback_group=self._cb_group)

    def _auto_start_once(self) -> None:
        if self._auto_started:
            return
        self._auto_started = True
        threading.Thread(target=self._run_demo_logged, daemon=True).start()

    def _wait_future(self, future, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and not future.done():
            if time.monotonic() >= deadline:
                return False
            time.sleep(0.01)
        return future.done()

    def _call_trigger(self, client, name: str, timeout_sec: float = 10.0) -> Trigger.Response:
        if not client.wait_for_service(timeout_sec=timeout_sec):
            raise RuntimeError(f"Service unavailable: {name}")
        future = client.call_async(Trigger.Request())
        if not self._wait_future(future, timeout_sec):
            raise TimeoutError(f"Timed out waiting for {name}")
        response = future.result()
        if not response.success:
            raise RuntimeError(f"{name} failed: {response.message}")
        return response

    def _start_bag_recording(self) -> subprocess.Popen | None:
        if not self._cfg.record_bag:
            return None
        if not self._cfg.record_topics:
            self.get_logger().warn("record_bag is true but no record_topics are configured")
            return None
        bag_prefix = Path(self._cfg.bag_dir).expanduser()
        bag_prefix.parent.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        bag_dir = bag_prefix.with_name(f"{bag_prefix.name}_{timestamp}")
        cmd = ["ros2", "bag", "record", "-o", str(bag_dir), *self._cfg.record_topics]
        self.get_logger().info(f"Starting rosbag recording: {' '.join(cmd)}")
        return subprocess.Popen(cmd)

    def _stop_bag_recording(self, proc: subprocess.Popen | None) -> None:
        if proc is None:
            return
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()

    def _send_grasp_goal(self, timeout_sec: float = 300.0) -> Grasp.Result:
        if not self._grasp_client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("Action server unavailable: /linker/grasp")
        goal = Grasp.Goal()
        goal.target_profile = self._target_profile
        goal.force_low = 0.0
        goal.force_high = 0.0
        goal.timeout_sec = 0.0
        self._last_feedback_phase = ""
        send_future = self._grasp_client.send_goal_async(
            goal, feedback_callback=self._on_grasp_feedback
        )
        if not self._wait_future(send_future, 10.0):
            raise TimeoutError("Timed out sending Grasp goal")
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            raise RuntimeError("Grasp goal rejected")
        with self._goal_lock:
            self._active_goal_handle = goal_handle
        try:
            result_future = goal_handle.get_result_async()
            if not self._wait_future(result_future, timeout_sec):
                goal_handle.cancel_goal_async()
                raise TimeoutError("Timed out waiting for Grasp result")
            return result_future.result().result
        finally:
            with self._goal_lock:
                self._active_goal_handle = None

    def _on_grasp_feedback(self, feedback_msg) -> None:
        feedback = feedback_msg.feedback
        if feedback.phase == self._last_feedback_phase:
            return
        self._last_feedback_phase = feedback.phase
        scores = ", ".join(f"{score:.1f}" for score in feedback.finger_scores)
        self.get_logger().info(
            f"Demo phase: {feedback.phase}; tactile scores: [{scores}]"
        )

    def _run_demo(self) -> str:
        bag_proc = self._start_bag_recording()
        try:
            if self._enable_and_home:
                self._call_trigger(self._arm_enable_client, "/linker/arm/enable")
                self._call_trigger(self._arm_home_client, "/linker/arm/home", timeout_sec=30.0)
            result = self._send_grasp_goal()
            if not result.success:
                raise RuntimeError(f"Grasp failed: {result.result_code} {result.error}")
            return "Demo completed successfully"
        finally:
            self._stop_bag_recording(bag_proc)

    def _run_demo_logged(self) -> None:
        if not self._run_lock.acquire(blocking=False):
            self.get_logger().warn("Demo is already running")
            return
        try:
            message = self._run_demo()
            self.get_logger().info(message)
        except Exception as exc:
            self.get_logger().error(f"Demo failed: {exc}")
        finally:
            self._run_lock.release()

    def _handle_run_once(self, _request, _response) -> Trigger.Response:
        response = Trigger.Response()
        if not self._run_lock.acquire(blocking=False):
            response.success = False
            response.message = "Demo is already running"
            return response
        try:
            response.message = self._run_demo()
            response.success = True
        except Exception as exc:
            response.message = str(exc)
            response.success = False
        finally:
            self._run_lock.release()
        return response

    def _handle_cancel(self, _request, _response) -> Trigger.Response:
        response = Trigger.Response()
        with self._goal_lock:
            goal_handle = self._active_goal_handle
        if goal_handle is None:
            response.success = False
            response.message = "No demo is currently running"
            return response

        cancel_future = goal_handle.cancel_goal_async()
        if not self._wait_future(cancel_future, 3.0):
            response.success = False
            response.message = "Timed out canceling demo"
            return response
        response.success = True
        response.message = "Demo cancellation requested"
        return response


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = DemoTaskNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
