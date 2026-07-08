from __future__ import annotations

from typing import Iterable

from linker_manip_interfaces.msg import TcpPose


FINGER_NAMES = ["thumb", "index", "middle", "ring", "pinky"]


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def fixed_float_list(values: Iterable[float] | None, length: int, default: float = 0.0) -> list[float]:
    source = [] if values is None else values
    result = [float(v) for v in source][:length]
    if len(result) < length:
        result.extend([float(default)] * (length - len(result)))
    return result


def pose_msg_from_list(values: Iterable[float]) -> TcpPose:
    x, y, z, rx, ry, rz = [float(v) for v in values]
    msg = TcpPose()
    msg.x = x
    msg.y = y
    msg.z = z
    msg.rx = rx
    msg.ry = ry
    msg.rz = rz
    return msg


def pose_list_from_msg(msg: TcpPose) -> list[float]:
    return [float(msg.x), float(msg.y), float(msg.z), float(msg.rx), float(msg.ry), float(msg.rz)]


def pose_msg_from_sdk_pose(pose) -> TcpPose:
    return pose_msg_from_list([pose.x, pose.y, pose.z, pose.rx, pose.ry, pose.rz])


def sdk_pose_from_msg(msg: TcpPose):
    from linkerbot import Pose

    return Pose(
        x=float(msg.x),
        y=float(msg.y),
        z=float(msg.z),
        rx=float(msg.rx),
        ry=float(msg.ry),
        rz=float(msg.rz),
    )
