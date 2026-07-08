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
    type: str = "L20lite"
    side: str = "left"
    can: str = "can1"
    interface_type: str = "socketcan"
    connect_on_start: bool = True
    state_rate_hz: float = 30.0
    tactile_rate_hz: float = 30.0
    open_angles: list[float] = field(default_factory=lambda: [100.0] * 10)
    pregrasp_angles: list[float] = field(default_factory=lambda: [85.0] * 10)
    default_speeds: list[float] = field(default_factory=lambda: [50.0] * 10)
    default_torques: list[float] = field(default_factory=lambda: [45.0] * 10)
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
class RobotConfig:
    arm: ArmConfig
    hand: HandConfig
    grasp: GraspConfig
    demo: DemoConfig


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

    hand = HandConfig(
        type=str(hand_raw.get("type", "L20lite")),
        side=str(hand_raw.get("side", "left")),
        can=str(hand_raw.get("can", "can1")),
        interface_type=str(hand_raw.get("interface_type", "socketcan")),
        connect_on_start=bool(hand_raw.get("connect_on_start", True)),
        state_rate_hz=float(hand_raw.get("state_rate_hz", 30.0)),
        tactile_rate_hz=float(hand_raw.get("tactile_rate_hz", 30.0)),
        open_angles=_as_float_list(hand_raw.get("open_angles", [100.0] * 10), 10),
        pregrasp_angles=_as_float_list(hand_raw.get("pregrasp_angles", [85.0] * 10), 10),
        default_speeds=_as_float_list(hand_raw.get("default_speeds", [50.0] * 10), 10),
        default_torques=_as_float_list(hand_raw.get("default_torques", [45.0] * 10), 10),
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
        max_closed_angles=_as_float_list(grasp_raw.get("max_closed_angles", [0.0] * 10), 10),
        poses=poses,
    )

    demo = DemoConfig(
        run_once_on_start=bool(demo_raw.get("run_once_on_start", False)),
        enable_and_home=bool(demo_raw.get("enable_and_home", True)),
        record_bag=bool(demo_raw.get("record_bag", False)),
        bag_dir=str(demo_raw.get("bag_dir", "bags/grasp_demo")),
        record_topics=[str(topic) for topic in demo_raw.get("record_topics", [])],
    )

    return RobotConfig(arm=arm, hand=hand, grasp=grasp, demo=demo)
