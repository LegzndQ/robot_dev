from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import cv2
import numpy as np
import yaml


ARUCO_DICTIONARIES = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_APRILTAG_36H11": cv2.aruco.DICT_APRILTAG_36h11,
}


def make_charuco_board(
    squares_x: int,
    squares_y: int,
    square_length_m: float,
    marker_length_m: float,
    dictionary_name: str,
):
    if squares_x < 3 or squares_y < 3:
        raise ValueError("A ChArUco board needs at least 3x3 squares")
    if not 0.0 < marker_length_m < square_length_m:
        raise ValueError(
            "marker_length_m must be smaller than square_length_m"
        )
    try:
        dictionary_id = ARUCO_DICTIONARIES[dictionary_name]
    except KeyError as exc:
        names = ", ".join(sorted(ARUCO_DICTIONARIES))
        raise ValueError(
            f"Unknown ArUco dictionary {dictionary_name!r}; "
            f"choose from {names}"
        ) from exc
    dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
    board = cv2.aruco.CharucoBoard_create(
        squares_x,
        squares_y,
        square_length_m,
        marker_length_m,
        dictionary,
    )
    return board, dictionary


def detect_charuco(image, board, dictionary):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    parameters = cv2.aruco.DetectorParameters_create()
    marker_corners, marker_ids, _ = cv2.aruco.detectMarkers(
        gray,
        dictionary,
        parameters=parameters,
    )
    if marker_ids is None or len(marker_ids) == 0:
        return marker_corners, marker_ids, None, None
    count, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
        marker_corners,
        marker_ids,
        gray,
        board,
    )
    if count is None or count < 1:
        return marker_corners, marker_ids, None, None
    return marker_corners, marker_ids, charuco_corners, charuco_ids


def draw_charuco_detection(
    image,
    marker_corners,
    marker_ids,
    charuco_corners,
    charuco_ids,
    label: str,
):
    debug = image.copy()
    if marker_ids is not None:
        cv2.aruco.drawDetectedMarkers(debug, marker_corners, marker_ids)
    if charuco_ids is not None:
        cv2.aruco.drawDetectedCornersCharuco(
            debug,
            charuco_corners,
            charuco_ids,
            (20, 220, 20),
        )
    cv2.rectangle(debug, (0, 0), (debug.shape[1], 42), (20, 20, 20), -1)
    cv2.putText(
        debug,
        label,
        (12, 29),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.72,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )
    return debug


def transform_matrix(rotation, translation) -> np.ndarray:
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = np.asarray(rotation, dtype=np.float64).reshape(3, 3)
    result[:3, 3] = np.asarray(translation, dtype=np.float64).reshape(3)
    return result


def invert_transform(transform: np.ndarray) -> np.ndarray:
    rotation = transform[:3, :3]
    translation = transform[:3, 3]
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = rotation.T
    result[:3, 3] = -rotation.T @ translation
    return result


def quaternion_from_rotation(rotation: np.ndarray) -> np.ndarray:
    matrix = np.asarray(rotation, dtype=np.float64).reshape(3, 3)
    quaternion = np.empty(4, dtype=np.float64)
    trace = np.trace(matrix)
    if trace > 0.0:
        scale = np.sqrt(trace + 1.0) * 2.0
        quaternion[3] = 0.25 * scale
        quaternion[0] = (matrix[2, 1] - matrix[1, 2]) / scale
        quaternion[1] = (matrix[0, 2] - matrix[2, 0]) / scale
        quaternion[2] = (matrix[1, 0] - matrix[0, 1]) / scale
    else:
        index = int(np.argmax(np.diag(matrix)))
        if index == 0:
            scale = np.sqrt(
                1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]
            ) * 2.0
            quaternion[3] = (matrix[2, 1] - matrix[1, 2]) / scale
            quaternion[0] = 0.25 * scale
            quaternion[1] = (matrix[0, 1] + matrix[1, 0]) / scale
            quaternion[2] = (matrix[0, 2] + matrix[2, 0]) / scale
        elif index == 1:
            scale = np.sqrt(
                1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]
            ) * 2.0
            quaternion[3] = (matrix[0, 2] - matrix[2, 0]) / scale
            quaternion[0] = (matrix[0, 1] + matrix[1, 0]) / scale
            quaternion[1] = 0.25 * scale
            quaternion[2] = (matrix[1, 2] + matrix[2, 1]) / scale
        else:
            scale = np.sqrt(
                1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]
            ) * 2.0
            quaternion[3] = (matrix[1, 0] - matrix[0, 1]) / scale
            quaternion[0] = (matrix[0, 2] + matrix[2, 0]) / scale
            quaternion[1] = (matrix[1, 2] + matrix[2, 1]) / scale
            quaternion[2] = 0.25 * scale
    quaternion /= np.linalg.norm(quaternion)
    return quaternion


def rotation_from_quaternion(quaternion: Iterable[float]) -> np.ndarray:
    x, y, z, w = np.asarray(quaternion, dtype=np.float64).reshape(4)
    norm = np.linalg.norm([x, y, z, w])
    if norm == 0.0:
        raise ValueError("Quaternion must not be zero")
    x, y, z, w = np.array([x, y, z, w]) / norm
    return np.array(
        [
            [
                1 - 2 * (y * y + z * z),
                2 * (x * y - z * w),
                2 * (x * z + y * w),
            ],
            [
                2 * (x * y + z * w),
                1 - 2 * (x * x + z * z),
                2 * (y * z - x * w),
            ],
            [
                2 * (x * z - y * w),
                2 * (y * z + x * w),
                1 - 2 * (x * x + y * y),
            ],
        ],
        dtype=np.float64,
    )


def transform_from_ros(transform) -> np.ndarray:
    translation = transform.translation
    rotation = transform.rotation
    matrix = transform_matrix(
        rotation_from_quaternion(
            [rotation.x, rotation.y, rotation.z, rotation.w]
        ),
        [translation.x, translation.y, translation.z],
    )
    return matrix


def rotation_angle(rotation: np.ndarray) -> float:
    cosine = np.clip((np.trace(rotation) - 1.0) * 0.5, -1.0, 1.0)
    return float(np.arccos(cosine))


def average_transforms(transforms: list[np.ndarray]) -> np.ndarray:
    if not transforms:
        raise ValueError("At least one transform is required")
    translations = np.array([item[:3, 3] for item in transforms])
    accumulator = np.zeros((4, 4), dtype=np.float64)
    for item in transforms:
        quaternion = quaternion_from_rotation(item[:3, :3])
        accumulator += np.outer(quaternion, quaternion)
    _, eigenvectors = np.linalg.eigh(accumulator)
    quaternion = eigenvectors[:, -1]
    if quaternion[3] < 0.0:
        quaternion = -quaternion
    return transform_matrix(
        rotation_from_quaternion(quaternion),
        np.mean(translations, axis=0),
    )


@dataclass
class EyeToHandResult:
    base_to_camera: np.ndarray
    gripper_to_target: np.ndarray
    method: str
    translation_rms_m: float
    rotation_rms_deg: float
    translation_max_m: float
    rotation_max_deg: float


def solve_eye_to_hand(
    base_to_gripper: list[np.ndarray],
    camera_to_target: list[np.ndarray],
) -> EyeToHandResult:
    if len(base_to_gripper) != len(camera_to_target):
        raise ValueError("Robot and camera sample counts do not match")
    if len(base_to_gripper) < 3:
        raise ValueError("At least three samples are required")

    gripper_rotations = [item[:3, :3] for item in base_to_gripper]
    gripper_translations = [item[:3, 3] for item in base_to_gripper]
    target_to_camera = [invert_transform(item) for item in camera_to_target]
    target_rotations = [item[:3, :3] for item in target_to_camera]
    target_translations = [item[:3, 3] for item in target_to_camera]
    methods = {
        "TSAI": cv2.CALIB_HAND_EYE_TSAI,
        "PARK": cv2.CALIB_HAND_EYE_PARK,
        "HORAUD": cv2.CALIB_HAND_EYE_HORAUD,
        "ANDREFF": cv2.CALIB_HAND_EYE_ANDREFF,
        "DANIILIDIS": cv2.CALIB_HAND_EYE_DANIILIDIS,
    }

    candidates: list[tuple[float, EyeToHandResult]] = []
    for name, method in methods.items():
        try:
            rotation, translation = cv2.calibrateHandEye(
                gripper_rotations,
                gripper_translations,
                target_rotations,
                target_translations,
                method=method,
            )
        except cv2.error:
            continue
        gripper_to_target = transform_matrix(rotation, translation)
        if not np.isfinite(gripper_to_target).all():
            continue
        estimates = [
            robot
            @ gripper_to_target
            @ invert_transform(observation)
            for robot, observation in zip(base_to_gripper, camera_to_target)
        ]
        base_to_camera = average_transforms(estimates)
        translation_errors = []
        rotation_errors = []
        for estimate in estimates:
            delta = invert_transform(base_to_camera) @ estimate
            translation_errors.append(float(np.linalg.norm(delta[:3, 3])))
            rotation_errors.append(rotation_angle(delta[:3, :3]))
        translation_rms = float(
            np.sqrt(np.mean(np.square(translation_errors)))
        )
        rotation_rms = float(
            np.degrees(np.sqrt(np.mean(np.square(rotation_errors))))
        )
        result = EyeToHandResult(
            base_to_camera=base_to_camera,
            gripper_to_target=gripper_to_target,
            method=name,
            translation_rms_m=translation_rms,
            rotation_rms_deg=rotation_rms,
            translation_max_m=max(translation_errors),
            rotation_max_deg=float(np.degrees(max(rotation_errors))),
        )
        score = translation_rms + np.radians(rotation_rms) * 0.10
        candidates.append((score, result))

    if not candidates:
        raise RuntimeError("All OpenCV hand-eye solvers failed")
    return min(candidates, key=lambda item: item[0])[1]


def save_camera_info(
    output_path: str,
    camera_name: str,
    image_width: int,
    image_height: int,
    camera_matrix: np.ndarray,
    distortion: np.ndarray,
) -> Path:
    path = Path(output_path).expanduser()
    path.parent.mkdir(parents=True, exist_ok=True)
    projection = np.zeros((3, 4), dtype=np.float64)
    projection[:3, :3] = camera_matrix
    document = {
        "image_width": int(image_width),
        "image_height": int(image_height),
        "camera_name": camera_name,
        "camera_matrix": {
            "rows": 3,
            "cols": 3,
            "data": camera_matrix.reshape(-1).astype(float).tolist(),
        },
        "distortion_model": "plumb_bob",
        "distortion_coefficients": {
            "rows": 1,
            "cols": int(distortion.size),
            "data": distortion.reshape(-1).astype(float).tolist(),
        },
        "rectification_matrix": {
            "rows": 3,
            "cols": 3,
            "data": np.eye(3).reshape(-1).tolist(),
        },
        "projection_matrix": {
            "rows": 3,
            "cols": 4,
            "data": projection.reshape(-1).astype(float).tolist(),
        },
    }
    path.write_text(
        yaml.safe_dump(document, sort_keys=False), encoding="utf-8"
    )
    return path


def save_extrinsics(
    output_path: str,
    parent_frame: str,
    child_frame: str,
    base_to_camera: np.ndarray,
    metadata: dict | None = None,
) -> Path:
    path = Path(output_path).expanduser()
    path.parent.mkdir(parents=True, exist_ok=True)
    quaternion = quaternion_from_rotation(base_to_camera[:3, :3])
    translation = base_to_camera[:3, 3]
    document = {
        "parent_frame": parent_frame,
        "child_frame": child_frame,
        "translation": {
            "x": float(translation[0]),
            "y": float(translation[1]),
            "z": float(translation[2]),
        },
        "rotation": {
            "x": float(quaternion[0]),
            "y": float(quaternion[1]),
            "z": float(quaternion[2]),
            "w": float(quaternion[3]),
        },
    }
    if metadata:
        document["calibration"] = metadata
    path.write_text(
        yaml.safe_dump(document, sort_keys=False), encoding="utf-8"
    )
    return path
