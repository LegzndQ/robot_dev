from __future__ import annotations

import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .camera_calibration import (
    detect_charuco,
    draw_charuco_detection,
    make_charuco_board,
    save_camera_info,
)


class IntrinsicCalibratorNode(Node):
    def __init__(self) -> None:
        super().__init__("iphone_intrinsic_calibrator")
        self.declare_parameter("image_topic", "/iphone_camera/image_raw")
        self.declare_parameter(
            "output_path", "~/.ros/camera_info/iphone_15_pro.yaml"
        )
        self.declare_parameter("camera_name", "iphone_camera")
        self.declare_parameter("squares_x", 7)
        self.declare_parameter("squares_y", 5)
        self.declare_parameter("square_length_m", 0.030)
        self.declare_parameter("marker_length_m", 0.022)
        self.declare_parameter("dictionary", "DICT_4X4_50")
        self.declare_parameter("min_corners", 12)
        self.declare_parameter("min_samples", 15)
        self.declare_parameter("max_rms_px", 1.5)

        self._output_path = str(self.get_parameter("output_path").value)
        self._camera_name = str(self.get_parameter("camera_name").value)
        self._min_corners = int(self.get_parameter("min_corners").value)
        self._min_samples = int(self.get_parameter("min_samples").value)
        self._max_rms_px = float(self.get_parameter("max_rms_px").value)
        self._board, self._dictionary = make_charuco_board(
            int(self.get_parameter("squares_x").value),
            int(self.get_parameter("squares_y").value),
            float(self.get_parameter("square_length_m").value),
            float(self.get_parameter("marker_length_m").value),
            str(self.get_parameter("dictionary").value),
        )

        self._bridge = CvBridge()
        self._latest_corners = None
        self._latest_ids = None
        self._latest_image_size = None
        self._latest_detection_time = 0.0
        self._sample_corners = []
        self._sample_ids = []
        self._image_size = None

        image_topic = str(self.get_parameter("image_topic").value)
        self.create_subscription(
            Image,
            image_topic,
            self._image_callback,
            qos_profile_sensor_data,
        )
        self._debug_pub = self.create_publisher(
            Image,
            "/iphone_calibration/intrinsics/debug_image",
            qos_profile_sensor_data,
        )
        self._status_pub = self.create_publisher(
            String,
            "/iphone_calibration/intrinsics/status",
            10,
        )
        self.create_service(
            Trigger,
            "/iphone_calibration/intrinsics/capture",
            self._capture_sample,
        )
        self.create_service(
            Trigger,
            "/iphone_calibration/intrinsics/solve",
            self._solve,
        )
        self.create_service(
            Trigger,
            "/iphone_calibration/intrinsics/reset",
            self._reset,
        )
        self.get_logger().info(
            f"Intrinsic calibrator listening on {image_topic}; "
            f"need at least {self._min_samples} samples"
        )

    def _publish_status(self, text: str) -> None:
        message = String()
        message.data = text
        self._status_pub.publish(message)

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
        count = 0 if ids is None else len(ids)
        self._latest_corners = None if corners is None else corners.copy()
        self._latest_ids = None if ids is None else ids.copy()
        self._latest_image_size = (image.shape[1], image.shape[0])
        self._latest_detection_time = time.monotonic()
        label = (
            f"corners {count} | captured {len(self._sample_ids)} / "
            f"{self._min_samples}"
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

    def _capture_sample(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
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
        if self._image_size is None:
            self._image_size = self._latest_image_size
        elif self._image_size != self._latest_image_size:
            response.success = False
            response.message = (
                f"Image resolution changed from {self._image_size} to "
                f"{self._latest_image_size}; reset calibration"
            )
            return response
        self._sample_corners.append(self._latest_corners.copy())
        self._sample_ids.append(self._latest_ids.copy())
        count = len(self._sample_ids)
        response.success = True
        response.message = (
            f"Captured intrinsic sample {count}/{self._min_samples}"
        )
        self._publish_status(response.message)
        return response

    def _solve(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        if len(self._sample_ids) < self._min_samples:
            response.success = False
            response.message = (
                f"Need {self._min_samples} samples; currently have "
                f"{len(self._sample_ids)}"
            )
            return response
        try:
            (
                rms,
                camera_matrix,
                distortion,
                rotation_vectors,
                translation_vectors,
            ) = cv2.aruco.calibrateCameraCharuco(
                self._sample_corners,
                self._sample_ids,
                self._board,
                self._image_size,
                None,
                None,
            )
        except cv2.error as exc:
            response.success = False
            response.message = f"OpenCV intrinsic calibration failed: {exc}"
            return response

        per_view_errors = []
        object_corners = np.asarray(self._board.chessboardCorners)
        for corners, ids, rvec, tvec in zip(
            self._sample_corners,
            self._sample_ids,
            rotation_vectors,
            translation_vectors,
        ):
            object_points = object_corners[ids.reshape(-1)]
            projected, _ = cv2.projectPoints(
                object_points,
                rvec,
                tvec,
                camera_matrix,
                distortion,
            )
            error = np.linalg.norm(
                projected.reshape(-1, 2) - corners.reshape(-1, 2), axis=1
            )
            per_view_errors.append(float(np.sqrt(np.mean(np.square(error)))))

        if not np.isfinite(camera_matrix).all() or not np.isfinite(rms):
            response.success = False
            response.message = "Calibration produced non-finite values"
            return response
        if float(rms) > self._max_rms_px:
            response.success = False
            response.message = (
                f"RMS {rms:.3f}px exceeds {self._max_rms_px:.3f}px; "
                "reset and capture more varied, sharper views"
            )
            return response

        path = save_camera_info(
            self._output_path,
            self._camera_name,
            self._image_size[0],
            self._image_size[1],
            camera_matrix,
            distortion,
        )
        response.success = True
        response.message = (
            f"Saved {path}; RMS={rms:.3f}px, "
            f"worst_view={max(per_view_errors):.3f}px"
        )
        self.get_logger().info(response.message)
        self._publish_status(response.message)
        return response

    def _reset(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        self._sample_corners.clear()
        self._sample_ids.clear()
        self._image_size = None
        response.success = True
        response.message = "Intrinsic samples cleared"
        self._publish_status(response.message)
        return response


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = IntrinsicCalibratorNode()
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
