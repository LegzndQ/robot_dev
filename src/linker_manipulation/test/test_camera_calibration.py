import cv2
import numpy as np

from linker_manipulation.camera_calibration import (
    detect_charuco,
    invert_transform,
    make_charuco_board,
    rotation_from_quaternion,
    solve_eye_to_hand,
    transform_matrix,
)


def _rotation(vector):
    return cv2.Rodrigues(np.asarray(vector, dtype=np.float64))[0]


def test_charuco_board_can_be_detected():
    board, dictionary = make_charuco_board(
        7, 5, 0.030, 0.022, "DICT_4X4_50"
    )
    board_image = board.draw((1400, 1000))
    board_image = cv2.copyMakeBorder(
        board_image, 100, 100, 100, 100, cv2.BORDER_CONSTANT, value=255
    )
    image = cv2.cvtColor(board_image, cv2.COLOR_GRAY2BGR)
    _, marker_ids, corners, corner_ids = detect_charuco(
        image, board, dictionary
    )
    assert marker_ids is not None
    assert corners is not None
    assert len(corner_ids) == 24


def test_eye_to_hand_solver_recovers_synthetic_transform():
    random = np.random.default_rng(7)
    base_to_camera = transform_matrix(
        _rotation([0.2, -0.1, 0.15]), [0.70, 0.10, 0.30]
    )
    gripper_to_target = transform_matrix(
        _rotation([-0.15, 0.2, 0.1]), [0.03, -0.02, 0.08]
    )
    robot_samples = []
    camera_samples = []
    for _ in range(20):
        base_to_gripper = transform_matrix(
            _rotation(random.uniform(-1.0, 1.0, 3)),
            random.uniform([0.2, -0.3, -0.4], [0.7, 0.3, 0.3]),
        )
        camera_to_target = (
            invert_transform(base_to_camera)
            @ base_to_gripper
            @ gripper_to_target
        )
        robot_samples.append(base_to_gripper)
        camera_samples.append(camera_to_target)

    result = solve_eye_to_hand(robot_samples, camera_samples)
    error = invert_transform(base_to_camera) @ result.base_to_camera
    assert np.linalg.norm(error[:3, 3]) < 1e-8
    assert np.allclose(error[:3, :3], np.eye(3), atol=1e-8)
    assert result.translation_rms_m < 1e-8
    assert result.rotation_rms_deg < 1e-6


def test_quaternion_rotation_is_normalized():
    rotation = rotation_from_quaternion([0.0, 0.0, 0.0, 2.0])
    assert np.allclose(rotation, np.eye(3))
