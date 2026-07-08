from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from linker_manip_interfaces.msg import FingerTactile, HandTactile

from .ros_utils import FINGER_NAMES


@dataclass(frozen=True)
class TactileSnapshot:
    values: dict[str, np.ndarray]
    scores: dict[str, float]


def matrix_score(values: np.ndarray, baseline: np.ndarray | None = None) -> float:
    arr = values.astype(np.float32)
    if baseline is not None:
        arr = arr - baseline.astype(np.float32)
    arr = np.clip(arr, 0.0, 255.0)
    return float(np.percentile(arr, 90))


def finger_msg_from_matrix(finger: str, values: np.ndarray, score: float | None = None) -> FingerTactile:
    matrix = values.astype(np.uint8).reshape(12, 6)
    msg = FingerTactile()
    msg.finger = finger
    msg.rows = 12
    msg.cols = 6
    msg.values = [int(v) for v in matrix.reshape(-1)]
    msg.score = float(matrix_score(matrix) if score is None else score)
    return msg


def hand_tactile_to_snapshot(msg: HandTactile, baseline: dict[str, np.ndarray] | None = None) -> TactileSnapshot:
    values: dict[str, np.ndarray] = {}
    scores: dict[str, float] = {}
    baseline = baseline or {}
    for finger in msg.fingers:
        if len(finger.values) != 72:
            continue
        matrix = np.array(finger.values, dtype=np.uint8).reshape(12, 6)
        values[finger.finger] = matrix
        scores[finger.finger] = matrix_score(matrix, baseline.get(finger.finger))
    for name in FINGER_NAMES:
        values.setdefault(name, np.zeros((12, 6), dtype=np.uint8))
        scores.setdefault(name, 0.0)
    return TactileSnapshot(values=values, scores=scores)
