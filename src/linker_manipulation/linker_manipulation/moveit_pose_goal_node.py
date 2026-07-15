from __future__ import annotations

import math
from typing import Sequence

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, OrientationConstraint, PositionConstraint
from rclpy.action import ActionClient
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive

from .config import load_robot_config


_MOVEIT_ERROR_NAMES = {
    1: "SUCCESS",
    99999: "FAILURE",
    -1: "PLANNING_FAILED",
    -2: "INVALID_MOTION_PLAN",
    -3: "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE",
    -4: "CONTROL_FAILED",
    -5: "UNABLE_TO_AQUIRE_SENSOR_DATA",
    -6: "TIMED_OUT",
    -7: "PREEMPTED",
    -10: "START_STATE_IN_COLLISION",
    -11: "START_STATE_VIOLATES_PATH_CONSTRAINTS",
    -12: "GOAL_IN_COLLISION",
    -13: "GOAL_VIOLATES_PATH_CONSTRAINTS",
    -14: "GOAL_CONSTRAINTS_VIOLATED",
    -15: "INVALID_GROUP_NAME",
    -16: "INVALID_GOAL_CONSTRAINTS",
    -17: "INVALID_ROBOT_STATE",
    -18: "INVALID_LINK_NAME",
    -19: "INVALID_OBJECT_NAME",
    -21: "FRAME_TRANSFORM_FAILURE",
    -22: "COLLISION_CHECKING_UNAVAILABLE",
    -23: "ROBOT_STATE_STALE",
    -24: "SENSOR_INFO_STALE",
    -25: "COMMUNICATION_FAILURE",
    -26: "START_STATE_INVALID",
    -27: "GOAL_STATE_INVALID",
    -28: "UNRECOGNIZED_GOAL_TYPE",
    -29: "CRASH",
    -30: "ABORT",
    -31: "NO_IK_SOLUTION",
}


def _rpy_to_quaternion(rx: float, ry: float, rz: float):
    cy = math.cos(rz * 0.5)
    sy = math.sin(rz * 0.5)
    cp = math.cos(ry * 0.5)
    sp = math.sin(ry * 0.5)
    cr = math.cos(rx * 0.5)
    sr = math.sin(rx * 0.5)

    q = Pose().orientation
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


def _pose_from_xyz_rpy(values: Sequence[float]) -> Pose:
    if len(values) != 6:
        raise ValueError(f"Expected [x, y, z, rx, ry, rz], got {len(values)} values")

    pose = Pose()
    pose.position.x = float(values[0])
    pose.position.y = float(values[1])
    pose.position.z = float(values[2])
    pose.orientation = _rpy_to_quaternion(
        float(values[3]),
        float(values[4]),
        float(values[5]),
    )
    return pose


class MoveItPoseGoalNode(Node):
    def __init__(self) -> None:
        super().__init__("moveit_pose_goal")
        self.declare_parameter("config_path", "")
        self.declare_parameter("goal_type", "pose")
        self.declare_parameter("pose_name", "pregrasp_pose")
        self.declare_parameter("joint_target_name", "")
        self.declare_parameter("group_name", "a7_lite_arm")
        self.declare_parameter("target_link", "tcp_link")
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("action_name", "/move_action")
        self.declare_parameter("execute", False)
        self.declare_parameter("position_tolerance", 0.03)
        self.declare_parameter("orientation_tolerance", 0.6)
        self.declare_parameter("joint_tolerance", 0.0)
        self.declare_parameter("allowed_planning_time", 8.0)
        self.declare_parameter("planning_attempts", 10)
        self.declare_parameter("velocity_scaling", 0.15)
        self.declare_parameter("acceleration_scaling", 0.15)
        self.declare_parameter("wait_timeout_sec", 15.0)

        action_name = self.get_parameter("action_name").value
        self._client = ActionClient(self, MoveGroup, str(action_name))

    def _fill_common_request(self, goal: MoveGroup.Goal, group_name: str, frame_id: str) -> None:
        request = goal.request
        request.group_name = group_name
        request.num_planning_attempts = int(self.get_parameter("planning_attempts").value)
        request.allowed_planning_time = float(
            self.get_parameter("allowed_planning_time").value
        )
        request.max_velocity_scaling_factor = float(
            self.get_parameter("velocity_scaling").value
        )
        request.max_acceleration_scaling_factor = float(
            self.get_parameter("acceleration_scaling").value
        )
        request.start_state.is_diff = True
        request.workspace_parameters.header.frame_id = frame_id
        request.workspace_parameters.min_corner.x = -1.5
        request.workspace_parameters.min_corner.y = -1.5
        request.workspace_parameters.min_corner.z = -1.0
        request.workspace_parameters.max_corner.x = 1.5
        request.workspace_parameters.max_corner.y = 1.5
        request.workspace_parameters.max_corner.z = 1.0

    def _make_pose_constraints(
        self,
        pose_name: str,
        target_link: str,
        frame_id: str,
    ) -> Constraints:
        position_tolerance = float(self.get_parameter("position_tolerance").value)
        orientation_tolerance = float(self.get_parameter("orientation_tolerance").value)

        cfg = self._cfg
        if pose_name not in cfg.grasp.poses:
            available = ", ".join(sorted(cfg.grasp.poses))
            raise ValueError(f"Unknown pose_name '{pose_name}'. Available poses: {available}")

        target_pose = _pose_from_xyz_rpy(cfg.grasp.poses[pose_name])

        position_region = SolidPrimitive()
        position_region.type = SolidPrimitive.BOX
        position_region.dimensions = [
            position_tolerance * 2.0,
            position_tolerance * 2.0,
            position_tolerance * 2.0,
        ]

        position = PositionConstraint()
        position.header.frame_id = frame_id
        position.link_name = target_link
        position.constraint_region.primitives = [position_region]
        position.constraint_region.primitive_poses = [target_pose]
        position.weight = 1.0

        orientation = OrientationConstraint()
        orientation.header.frame_id = frame_id
        orientation.link_name = target_link
        orientation.orientation = target_pose.orientation
        orientation.absolute_x_axis_tolerance = orientation_tolerance
        orientation.absolute_y_axis_tolerance = orientation_tolerance
        orientation.absolute_z_axis_tolerance = orientation_tolerance
        orientation.parameterization = OrientationConstraint.ROTATION_VECTOR
        orientation.weight = 1.0

        constraints = Constraints()
        constraints.name = pose_name
        constraints.position_constraints = [position]
        constraints.orientation_constraints = [orientation]

        self.get_logger().info(
            f"Pose goal '{pose_name}' for {target_link} in {frame_id}: "
            f"x={target_pose.position.x:.3f}, y={target_pose.position.y:.3f}, "
            f"z={target_pose.position.z:.3f}, "
            f"position_tol={position_tolerance:.3f}, "
            f"orientation_tol={orientation_tolerance:.3f}"
        )
        return constraints

    def _make_joint_constraints(self, joint_target_name: str) -> Constraints:
        cfg = self._cfg
        if joint_target_name not in cfg.moveit.joint_targets:
            available = ", ".join(sorted(cfg.moveit.joint_targets))
            raise ValueError(
                f"Unknown joint_target_name '{joint_target_name}'. "
                f"Available joint targets: {available}"
            )

        target = cfg.moveit.joint_targets[joint_target_name]
        tolerance = float(self.get_parameter("joint_tolerance").value)
        if tolerance <= 0:
            tolerance = cfg.moveit.joint_tolerance

        constraints = Constraints()
        constraints.name = joint_target_name
        for joint_name, position in zip(cfg.moveit.joint_names, target):
            joint = JointConstraint()
            joint.joint_name = joint_name
            joint.position = float(position)
            joint.tolerance_above = tolerance
            joint.tolerance_below = tolerance
            joint.weight = 1.0
            constraints.joint_constraints.append(joint)

        self.get_logger().info(
            f"Joint goal '{joint_target_name}': "
            + ", ".join(
                f"{name}={value:.3f}"
                for name, value in zip(cfg.moveit.joint_names, target)
            )
            + f", tolerance={tolerance:.3f}"
        )
        return constraints

    def _make_goal(self) -> MoveGroup.Goal:
        config_path = str(self.get_parameter("config_path").value or "")
        self._cfg = load_robot_config(config_path or None)

        goal_type = str(self.get_parameter("goal_type").value).lower()
        pose_name = str(self.get_parameter("pose_name").value)
        joint_target_name = str(self.get_parameter("joint_target_name").value or "")
        group_name = str(self.get_parameter("group_name").value)
        target_link = str(self.get_parameter("target_link").value)
        frame_id = str(self.get_parameter("frame_id").value)
        execute = bool(self.get_parameter("execute").value)

        goal = MoveGroup.Goal()
        self._fill_common_request(goal, group_name, frame_id)
        if goal_type == "pose":
            goal.request.goal_constraints = [
                self._make_pose_constraints(pose_name, target_link, frame_id)
            ]
            goal_label = pose_name
        elif goal_type in ("joint", "joints"):
            target_name = joint_target_name or pose_name
            goal.request.goal_constraints = [self._make_joint_constraints(target_name)]
            goal_label = target_name
        else:
            raise ValueError("goal_type must be 'pose' or 'joint'")

        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.plan_only = not execute
        goal.planning_options.replan = execute
        goal.planning_options.replan_attempts = 2 if execute else 0
        goal.planning_options.replan_delay = 0.2

        mode = "execute" if execute else "plan_only"
        self.get_logger().info(
            f"Sending MoveIt {mode} {goal_type} goal '{goal_label}'"
        )
        return goal

    def _feedback_callback(self, feedback_msg) -> None:
        state = getattr(feedback_msg.feedback, "state", "")
        if state:
            self.get_logger().info(f"MoveIt state: {state}")

    def run(self) -> int:
        wait_timeout = float(self.get_parameter("wait_timeout_sec").value)
        if not self._client.wait_for_server(timeout_sec=wait_timeout):
            self.get_logger().error("MoveIt action server unavailable: /move_action")
            return 1

        try:
            goal = self._make_goal()
        except Exception as exc:
            self.get_logger().error(str(exc))
            return 1

        goal_future = self._client.send_goal_async(
            goal,
            feedback_callback=self._feedback_callback,
        )
        rclpy.spin_until_future_complete(self, goal_future)
        goal_handle = goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("MoveIt goal rejected")
            return 1

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        wrapped = result_future.result()
        if wrapped is None:
            self.get_logger().error("MoveIt goal finished without a result")
            return 1

        result = wrapped.result
        error_code = int(result.error_code.val)
        if error_code != 1:
            error_name = _MOVEIT_ERROR_NAMES.get(error_code, "UNKNOWN")
            self.get_logger().error(
                f"MoveIt goal failed: error_code={error_code} ({error_name}), "
                f"planning_time={result.planning_time:.3f}s"
            )
            if error_code == 99999 and result.planning_time <= 0.001:
                self.get_logger().error(
                    "MoveIt failed before producing a plan. Check that /linker/arm/state "
                    "is publishing, move_group was restarted after config changes, and "
                    "try plan-only first without -p execute:=true."
                )
            return 1

        points = len(result.planned_trajectory.joint_trajectory.points)
        self.get_logger().info(
            f"MoveIt goal succeeded: points={points}, "
            f"planning_time={result.planning_time:.3f}s"
        )
        return 0


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MoveItPoseGoalNode()
    try:
        raise SystemExit(node.run())
    finally:
        node.destroy_node()
        rclpy.shutdown()
