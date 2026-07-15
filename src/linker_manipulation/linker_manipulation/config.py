from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml
from ament_index_python.packages import get_package_share_directory


def _as_float_list(value: Any, length: int | None = None) -> list[float]:
    result = [float(v) for v in (value or [])]
    if length is not None and len(result) != length:
        raise ValueError(f"Expected {length} values, got {len(result)}")
    return result


def _as_pose(value: Any) -> list[float]:
    return _as_float_list(value, 6)


def _default_arm_joint_names(arm_type: str, side: str) -> list[str]:
    normalized = arm_type.lower().replace("_", "")
    if normalized == "a7lite":
        prefix = "L" if side == "left" else "R"
        return [f"{prefix}{index}_JOINT" for index in range(1, 8)]
    prefix = "Left" if side == "left" else "Right"
    return [
        f"{prefix}_Shoulder_Pitch_Joint",
        f"{prefix}_Shoulder_Roll_Joint",
        f"{prefix}_Shoulder_Yaw_Joint",
        f"{prefix}_Elbow_Pitch_Joint",
        f"{prefix}_Wrist_Yaw_Joint",
        f"{prefix}_Wrist_Pitch_Joint",
        f"{prefix}_Wrist_Roll_Joint",
    ]


def _hand_model_defaults(hand_type: str) -> dict[str, Any]:
    normalized = hand_type.lower()
    if normalized == "o6":
        return {
            "type": "O6",
            "joint_count": 6,
            "tactile_shape": [10, 4],
            "open_angles": [100.0, 50.0, 100.0, 100.0, 100.0, 100.0],
            "pregrasp_angles": [85.0, 45.0, 70.0, 70.0, 75.0, 80.0],
            "default_speeds": [50.0] * 6,
            "default_torques": [45.0] * 6,
            "max_closed_angles": [30.0, 35.0, 0.0, 0.0, 5.0, 20.0],
        }
    if normalized in ("l20lite", "l20_lite"):
        return {
            "type": "L20lite",
            "joint_count": 10,
            "tactile_shape": [12, 6],
            "open_angles": [100.0] * 10,
            "pregrasp_angles": [85.0] * 10,
            "default_speeds": [50.0] * 10,
            "default_torques": [45.0] * 10,
            "max_closed_angles": [
                28.0,
                100.0,
                0.0,
                0.0,
                5.0,
                20.0,
                100.0,
                100.0,
                100.0,
                70.0,
            ],
        }
    raise ValueError(f"Unsupported hand type: {hand_type}")


@dataclass(frozen=True)
class ArmConfig:
    type: str = "A7"
    side: str = "left"
    can: str = "can0"
    interface_type: str = "socketcan"
    world_frame: str = "maestro"
    tcp_offset: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    connect_on_start: bool = True
    state_rate_hz: float = 20.0
    default_joint_velocities: list[float] = field(default_factory=lambda: [0.3] * 7)
    default_joint_accelerations: list[float] = field(default_factory=lambda: [5.0] * 7)


@dataclass(frozen=True)
class HandConfig:
    type: str = "O6"
    side: str = "left"
    can: str = "can1"
    interface_type: str = "socketcan"
    joint_count: int = 6
    tactile_shape: list[int] = field(default_factory=lambda: [10, 4])
    connect_on_start: bool = True
    state_rate_hz: float = 30.0
    tactile_rate_hz: float = 30.0
    open_angles: list[float] = field(default_factory=lambda: [100.0, 50.0, 100.0, 100.0, 100.0, 100.0])
    pregrasp_angles: list[float] = field(default_factory=lambda: [85.0, 45.0, 70.0, 70.0, 75.0, 80.0])
    default_speeds: list[float] = field(default_factory=lambda: [50.0] * 6)
    default_torques: list[float] = field(default_factory=lambda: [45.0] * 6)
    polling: dict[str, float] = field(default_factory=dict)


@dataclass(frozen=True)
class GraspConfig:
    active_fingers: list[str] = field(default_factory=lambda: ["thumb", "index", "middle"])
    baseline_samples: int = 10
    contact_threshold: float = 12.0
    force_low: float = 20.0
    force_high: float = 45.0
    close_step: float = 2.0
    open_step: float = 1.0
    regulate_period_sec: float = 0.05
    stable_cycles: int = 8
    hold_sec: float = 5.0
    slip_drop_ratio: float = 0.30
    slip_window_sec: float = 0.10
    action_timeout_sec: float = 20.0
    approach_mode: str = "linear"
    approach_waypoints: list[str] = field(default_factory=list)
    max_closed_angles: list[float] = field(
        default_factory=lambda: [28.0, 100.0, 0.0, 0.0, 5.0, 20.0, 100.0, 100.0, 100.0, 70.0]
    )
    poses: dict[str, list[float]] = field(default_factory=dict)


@dataclass(frozen=True)
class DemoConfig:
    run_once_on_start: bool = False
    enable_and_home: bool = True
    record_bag: bool = False
    bag_dir: str = "bags/grasp_demo"
    record_topics: list[str] = field(default_factory=list)


@dataclass(frozen=True)
class MoveItConfig:
    joint_names: list[str] = field(default_factory=list)
    joint_tolerance: float = 0.03
    joint_targets: dict[str, list[float]] = field(default_factory=dict)
    planning_time: float = 8.0
    planning_attempts: int = 10
    velocity_scaling: float = 0.15
    acceleration_scaling: float = 0.15
    execution_timeout_sec: float = 90.0


@dataclass(frozen=True)
class RobotConfig:
    arm: ArmConfig
    hand: HandConfig
    grasp: GraspConfig
    demo: DemoConfig
    moveit: MoveItConfig


def default_config_path() -> Path:
    return Path(get_package_share_directory("linker_manipulation")) / "config" / "robot.yaml"


def load_robot_config(path: str | Path | None = None) -> RobotConfig:
    config_path = Path(path).expanduser() if path else default_config_path()
    with config_path.open("r", encoding="utf-8") as f:
        raw = yaml.safe_load(f) or {}

    arm_raw = raw.get("arm", {})
    hand_raw = raw.get("hand", {})
    grasp_raw = raw.get("grasp", {})
    demo_raw = raw.get("demo", {})
    moveit_raw = raw.get("moveit", {})

    arm = ArmConfig(
        type=str(arm_raw.get("type", "A7")),
        side=str(arm_raw.get("side", "left")),
        can=str(arm_raw.get("can", "can0")),
        interface_type=str(arm_raw.get("interface_type", "socketcan")),
        world_frame=str(arm_raw.get("world_frame", "maestro")),
        tcp_offset=_as_float_list(arm_raw.get("tcp_offset", [0.0, 0.0, 0.0]), 3),
        connect_on_start=bool(arm_raw.get("connect_on_start", True)),
        state_rate_hz=float(arm_raw.get("state_rate_hz", 20.0)),
        default_joint_velocities=_as_float_list(
            arm_raw.get("default_joint_velocities", [0.3] * 7), 7
        ),
        default_joint_accelerations=_as_float_list(
            arm_raw.get("default_joint_accelerations", [5.0] * 7), 7
        ),
    )

    requested_hand_type = str(hand_raw.get("type", "O6"))
    hand_defaults = _hand_model_defaults(requested_hand_type)
    joint_count = int(hand_raw.get("joint_count", hand_defaults["joint_count"]))
    tactile_shape = [int(v) for v in hand_raw.get("tactile_shape", hand_defaults["tactile_shape"])]
    if len(tactile_shape) != 2:
        raise ValueError("hand.tactile_shape must be [rows, cols]")

    hand = HandConfig(
        type=str(hand_raw.get("type", hand_defaults["type"])),
        side=str(hand_raw.get("side", "left")),
        can=str(hand_raw.get("can", "can1")),
        interface_type=str(hand_raw.get("interface_type", "socketcan")),
        joint_count=joint_count,
        tactile_shape=tactile_shape,
        connect_on_start=bool(hand_raw.get("connect_on_start", True)),
        state_rate_hz=float(hand_raw.get("state_rate_hz", 30.0)),
        tactile_rate_hz=float(hand_raw.get("tactile_rate_hz", 30.0)),
        open_angles=_as_float_list(hand_raw.get("open_angles", hand_defaults["open_angles"]), joint_count),
        pregrasp_angles=_as_float_list(
            hand_raw.get("pregrasp_angles", hand_defaults["pregrasp_angles"]),
            joint_count,
        ),
        default_speeds=_as_float_list(
            hand_raw.get("default_speeds", hand_defaults["default_speeds"]),
            joint_count,
        ),
        default_torques=_as_float_list(
            hand_raw.get("default_torques", hand_defaults["default_torques"]),
            joint_count,
        ),
        polling={str(k): float(v) for k, v in (hand_raw.get("polling", {}) or {}).items()},
    )

    poses = {
        str(name): _as_pose(values)
        for name, values in (grasp_raw.get("poses", {}) or {}).items()
    }
    grasp = GraspConfig(
        active_fingers=[str(v) for v in grasp_raw.get("active_fingers", ["thumb", "index", "middle"])],
        baseline_samples=int(grasp_raw.get("baseline_samples", 10)),
        contact_threshold=float(grasp_raw.get("contact_threshold", 12.0)),
        force_low=float(grasp_raw.get("force_low", 20.0)),
        force_high=float(grasp_raw.get("force_high", 45.0)),
        close_step=float(grasp_raw.get("close_step", 2.0)),
        open_step=float(grasp_raw.get("open_step", 1.0)),
        regulate_period_sec=float(grasp_raw.get("regulate_period_sec", 0.05)),
        stable_cycles=int(grasp_raw.get("stable_cycles", 8)),
        hold_sec=float(grasp_raw.get("hold_sec", 5.0)),
        slip_drop_ratio=float(grasp_raw.get("slip_drop_ratio", 0.30)),
        slip_window_sec=float(grasp_raw.get("slip_window_sec", 0.10)),
        action_timeout_sec=float(grasp_raw.get("action_timeout_sec", 20.0)),
        approach_mode=str(grasp_raw.get("approach_mode", "linear")).lower(),
        approach_waypoints=[
            str(name) for name in (grasp_raw.get("approach_waypoints", []) or [])
        ],
        max_closed_angles=_as_float_list(
            grasp_raw.get("max_closed_angles", hand_defaults["max_closed_angles"]),
            hand.joint_count,
        ),
        poses=poses,
    )

    demo = DemoConfig(
        run_once_on_start=bool(demo_raw.get("run_once_on_start", False)),
        enable_and_home=bool(demo_raw.get("enable_and_home", True)),
        record_bag=bool(demo_raw.get("record_bag", False)),
        bag_dir=str(demo_raw.get("bag_dir", "bags/grasp_demo")),
        record_topics=[str(topic) for topic in demo_raw.get("record_topics", [])],
    )

    default_joint_names = _default_arm_joint_names(arm.type, arm.side)
    joint_names = [
        str(name) for name in moveit_raw.get("joint_names", default_joint_names)
    ]
    if len(joint_names) != 7:
        raise ValueError(f"moveit.joint_names must contain 7 joints, got {len(joint_names)}")
    joint_targets = {
        str(name): _as_float_list(values, len(joint_names))
        for name, values in (moveit_raw.get("joint_targets", {}) or {}).items()
    }
    joint_targets.setdefault("home", [0.0] * len(joint_names))
    moveit = MoveItConfig(
        joint_names=joint_names,
        joint_tolerance=float(moveit_raw.get("joint_tolerance", 0.03)),
        joint_targets=joint_targets,
        planning_time=float(moveit_raw.get("planning_time", 8.0)),
        planning_attempts=int(moveit_raw.get("planning_attempts", 10)),
        velocity_scaling=float(
            moveit_raw.get("velocity_scaling", 0.15)
        ),
        acceleration_scaling=float(
            moveit_raw.get("acceleration_scaling", 0.15)
        ),
        execution_timeout_sec=float(
            moveit_raw.get("execution_timeout_sec", 90.0)
        ),
    )

    return RobotConfig(arm=arm, hand=hand, grasp=grasp, demo=demo, moveit=moveit)
