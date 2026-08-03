from __future__ import annotations

import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformListener

from .camera_calibration import (
    detect_charuco,
    draw_charuco_detection,
    invert_transform,
    make_charuco_board,
    quaternion_from_rotation,
    rotation_angle,
    save_extrinsics,
    solve_eye_to_hand,
    transform_from_ros,
    transform_matrix,
)


class EyeToHandCalibratorNode(Node):
    def __init__(self) -> None:
        super().__init__("iphone_eye_to_hand_calibrator")
        self.declare_parameter("image_topic", "/iphone_camera/image_raw")
        self.declare_parameter(
            "camera_info_topic", "/iphone_camera/camera_info"
        )
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("end_effector_frame", "tcp_link")
        self.declare_parameter("camera_frame", "iphone_camera_optical_frame")
        self.declare_parameter(
            "output_path", "~/.ros/camera_info/iphone_to_base.yaml"
        )
        self.declare_parameter("squares_x", 7)
        self.declare_parameter("squares_y", 5)
        self.declare_parameter("square_length_m", 0.030)
        self.declare_parameter("marker_length_m", 0.022)
        self.declare_parameter("dictionary", "DICT_4X4_50")
        self.declare_parameter("min_corners", 12)
        self.declare_parameter("min_samples", 12)
        self.declare_parameter("max_sample_reprojection_error_px", 2.0)
        self.declare_parameter("max_translation_rms_m", 0.030)
        self.declare_parameter("max_rotation_rms_deg", 3.0)

        self._base_frame = str(self.get_parameter("base_frame").value)
        self._end_effector_frame = str(
            self.get_parameter("end_effector_frame").value
        )
        self._camera_frame = str(self.get_parameter("camera_frame").value)
        self._output_path = str(self.get_parameter("output_path").value)
        self._min_corners = int(self.get_parameter("min_corners").value)
        self._min_samples = int(self.get_parameter("min_samples").value)
        self._max_reprojection_error = float(
            self.get_parameter("max_sample_reprojection_error_px").value
        )
        self._max_translation_rms = float(
            self.get_parameter("max_translation_rms_m").value
        )
        self._max_rotation_rms = float(
            self.get_parameter("max_rotation_rms_deg").value
        )
        self._square_length_m = float(
            self.get_parameter("square_length_m").value
        )
        self._marker_length_m = float(
            self.get_parameter("marker_length_m").value
        )
        self._dictionary_name = str(self.get_parameter("dictionary").value)
        self._squares_x = int(self.get_parameter("squares_x").value)
        self._squares_y = int(self.get_parameter("squares_y").value)
        self._board, self._dictionary = make_charuco_board(
            self._squares_x,
            self._squares_y,
            self._square_length_m,
            self._marker_length_m,
            self._dictionary_name,
        )

        self._bridge = CvBridge()
        self._camera_matrix = None
        self._distortion = None
        self._latest_corners = None
        self._latest_ids = None
        self._latest_detection_time = 0.0
        self._robot_samples = []
        self._camera_samples = []
        self._reprojection_errors = []

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._static_broadcaster = StaticTransformBroadcaster(self)

        self.create_subscription(
            CameraInfo,
            str(self.get_parameter("camera_info_topic").value),
            self._camera_info_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("image_topic").value),
            self._image_callback,
            qos_profile_sensor_data,
        )
        self._debug_pub = self.create_publisher(
            Image,
            "/iphone_calibration/hand_eye/debug_image",
            qos_profile_sensor_data,
        )
        self._status_pub = self.create_publisher(
            String,
            "/iphone_calibration/hand_eye/status",
            10,
        )
        self.create_service(
            Trigger,
            "/iphone_calibration/hand_eye/capture",
            self._capture_sample,
        )
        self.create_service(
            Trigger,
            "/iphone_calibration/hand_eye/solve",
            self._solve,
        )
        self.create_service(
            Trigger,
            "/iphone_calibration/hand_eye/reset",
            self._reset,
        )
        self.get_logger().info(
            f"Eye-to-hand calibrator: {self._base_frame} -> "
            f"{self._camera_frame}, tracking {self._end_effector_frame}"
        )

    def _publish_status(self, text: str) -> None:
        message = String()
        message.data = text
        self._status_pub.publish(message)

    def _camera_info_callback(self, message: CameraInfo) -> None:
        matrix = np.asarray(message.k, dtype=np.float64).reshape(3, 3)
        if matrix[0, 0] <= 0.0 or matrix[1, 1] <= 0.0:
            return
        self._camera_matrix = matrix
        self._distortion = np.asarray(message.d, dtype=np.float64)

    def _image_callback(self, message: Image) -> None:
        try:
            image = self._bridge.imgmsg_to_cv2(
                message, desired_encoding="bgr8"
            )
        except Exception as exc:
            self.get_logger().error(f"Image conversion failed: {exc}")
            return
        marker_corners, marker_ids, corners, ids = detect_charuco(
            image,
            self._board,
            self._dictionary,
        )
        self._latest_corners = None if corners is None else corners.copy()
        self._latest_ids = None if ids is None else ids.copy()
        self._latest_detection_time = time.monotonic()
        count = 0 if ids is None else len(ids)
        label = (
            f"corners {count} | captured {len(self._robot_samples)} / "
            f"{self._min_samples} | arm must be still"
        )
        debug = draw_charuco_detection(
            image,
            marker_corners,
            marker_ids,
            corners,
            ids,
            label,
        )
        debug_message = self._bridge.cv2_to_imgmsg(debug, encoding="bgr8")
        debug_message.header = message.header
        self._debug_pub.publish(debug_message)

    def _estimate_camera_to_target(self):
        ids = self._latest_ids.reshape(-1)
        object_points = np.asarray(
            self._board.chessboardCorners[ids], dtype=np.float64
        )
        image_points = np.asarray(
            self._latest_corners.reshape(-1, 2), dtype=np.float64
        )
        success, rotation_vector, translation = cv2.solvePnP(
            object_points,
            image_points,
            self._camera_matrix,
            self._distortion,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
        if not success:
            raise RuntimeError("solvePnP failed")
        rotation, _ = cv2.Rodrigues(rotation_vector)
        camera_to_target = transform_matrix(rotation, translation)
        if camera_to_target[2, 3] <= 0.0:
            raise RuntimeError("Board pose is behind the camera")
        projected, _ = cv2.projectPoints(
            object_points,
            rotation_vector,
            translation,
            self._camera_matrix,
            self._distortion,
        )
        errors = np.linalg.norm(
            projected.reshape(-1, 2) - image_points, axis=1
        )
        reprojection_error = float(np.sqrt(np.mean(np.square(errors))))
        return camera_to_target, reprojection_error

    def _capture_sample(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        if self._camera_matrix is None:
            response.success = False
            response.message = "No calibrated CameraInfo received"
            return response
        if time.monotonic() - self._latest_detection_time > 1.5:
            response.success = False
            response.message = "No recent camera frame"
            return response
        if (
            self._latest_ids is None
            or len(self._latest_ids) < self._min_corners
        ):
            count = 0 if self._latest_ids is None else len(self._latest_ids)
            response.success = False
            response.message = (
                f"Only {count} ChArUco corners detected; need "
                f"{self._min_corners}"
            )
            return response
        try:
            camera_to_target, reprojection_error = (
                self._estimate_camera_to_target()
            )
        except (cv2.error, RuntimeError) as exc:
            response.success = False
            response.message = f"Target pose estimation failed: {exc}"
            return response
        if reprojection_error > self._max_reprojection_error:
            response.success = False
            response.message = (
                f"Reprojection error {reprojection_error:.3f}px exceeds "
                f"{self._max_reprojection_error:.3f}px"
            )
            return response
        try:
            transform = self._tf_buffer.lookup_transform(
                self._base_frame,
                self._end_effector_frame,
                Time(),
                timeout=Duration(seconds=1.0),
            )
        except Exception as exc:
            response.success = False
            response.message = (
                f"Cannot read TF {self._base_frame} <- "
                f"{self._end_effector_frame}: {exc}"
            )
            return response
        base_to_gripper = transform_from_ros(transform.transform)
        for previous in self._robot_samples:
            delta = invert_transform(previous) @ base_to_gripper
            if (
                np.linalg.norm(delta[:3, 3]) < 0.010
                and np.degrees(rotation_angle(delta[:3, :3])) < 5.0
            ):
                response.success = False
                response.message = (
                    "Robot pose is too similar to an existing sample; "
                    "change position and orientation"
                )
                return response
        self._robot_samples.append(base_to_gripper)
        self._camera_samples.append(camera_to_target)
        self._reprojection_errors.append(reprojection_error)
        count = len(self._robot_samples)
        response.success = True
        response.message = (
            f"Captured hand-eye sample {count}/{self._min_samples}; "
            f"reprojection={reprojection_error:.3f}px"
        )
        self._publish_status(response.message)
        return response

    def _sample_motion_span(self) -> tuple[float, float]:
        max_translation = 0.0
        max_rotation = 0.0
        for first in self._robot_samples:
            for second in self._robot_samples:
                delta = invert_transform(first) @ second
                max_translation = max(
                    max_translation, float(np.linalg.norm(delta[:3, 3]))
                )
                max_rotation = max(
                    max_rotation,
                    float(np.degrees(rotation_angle(delta[:3, :3]))),
                )
        return max_translation, max_rotation

    def _broadcast(self, transform_matrix_value: np.ndarray) -> None:
        message = TransformStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._base_frame
        message.child_frame_id = self._camera_frame
        translation = transform_matrix_value[:3, 3]
        quaternion = quaternion_from_rotation(transform_matrix_value[:3, :3])
        message.transform.translation.x = float(translation[0])
        message.transform.translation.y = float(translation[1])
        message.transform.translation.z = float(translation[2])
        message.transform.rotation.x = float(quaternion[0])
        message.transform.rotation.y = float(quaternion[1])
        message.transform.rotation.z = float(quaternion[2])
        message.transform.rotation.w = float(quaternion[3])
        self._static_broadcaster.sendTransform(message)

    def _solve(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        if len(self._robot_samples) < self._min_samples:
            response.success = False
            response.message = (
                f"Need {self._min_samples} samples; currently have "
                f"{len(self._robot_samples)}"
            )
            return response
        translation_span, rotation_span = self._sample_motion_span()
        if rotation_span < 20.0:
            response.success = False
            response.message = (
                f"Orientation span is only {rotation_span:.1f}deg; "
                "collect poses tilted around multiple axes"
            )
            return response
        try:
            result = solve_eye_to_hand(
                self._robot_samples,
                self._camera_samples,
            )
        except (cv2.error, RuntimeError, ValueError) as exc:
            response.success = False
            response.message = f"Hand-eye solve failed: {exc}"
            return response
        if result.translation_rms_m > self._max_translation_rms:
            response.success = False
            response.message = (
                f"Translation RMS {result.translation_rms_m:.4f}m exceeds "
                f"{self._max_translation_rms:.4f}m; reset and recollect"
            )
            return response
        if result.rotation_rms_deg > self._max_rotation_rms:
            response.success = False
            response.message = (
                f"Rotation RMS {result.rotation_rms_deg:.2f}deg exceeds "
                f"{self._max_rotation_rms:.2f}deg; reset and recollect"
            )
            return response

        metadata = {
            "type": "eye_to_hand",
            "method": result.method,
            "samples": len(self._robot_samples),
            "end_effector_frame": self._end_effector_frame,
            "translation_rms_m": result.translation_rms_m,
            "rotation_rms_deg": result.rotation_rms_deg,
            "translation_max_m": result.translation_max_m,
            "rotation_max_deg": result.rotation_max_deg,
            "mean_reprojection_error_px": float(
                np.mean(self._reprojection_errors)
            ),
            "robot_translation_span_m": translation_span,
            "robot_rotation_span_deg": rotation_span,
            "board": {
                "squares_x": self._squares_x,
                "squares_y": self._squares_y,
                "square_length_m": self._square_length_m,
                "marker_length_m": self._marker_length_m,
                "dictionary": self._dictionary_name,
            },
        }
        path = save_extrinsics(
            self._output_path,
            self._base_frame,
            self._camera_frame,
            result.base_to_camera,
            metadata,
        )
        self._broadcast(result.base_to_camera)
        response.success = True
        response.message = (
            f"Saved {path}; method={result.method}, "
            f"translation_rms={result.translation_rms_m:.4f}m, "
            f"rotation_rms={result.rotation_rms_deg:.2f}deg"
        )
        self.get_logger().info(response.message)
        self._publish_status(response.message)
        return response

    def _reset(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        self._robot_samples.clear()
        self._camera_samples.clear()
        self._reprojection_errors.clear()
        response.success = True
        response.message = "Hand-eye samples cleared"
        self._publish_status(response.message)
        return response


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = EyeToHandCalibratorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
