from __future__ import annotations

import copy
import os
import threading
from pathlib import Path
from urllib.parse import urlparse

import cv2
import rclpy
import yaml
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


def _safe_rtsp_url(url: str) -> str:
    parsed = urlparse(url)
    if parsed.username is None:
        return url
    host = parsed.hostname or ""
    if parsed.port is not None:
        host = f"{host}:{parsed.port}"
    return parsed._replace(netloc=f"***:***@{host}").geturl()


def _rotate_frame(frame, degrees: int):
    if degrees == 0:
        return frame
    if degrees == 90:
        return cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
    if degrees == 180:
        return cv2.rotate(frame, cv2.ROTATE_180)
    if degrees == 270:
        return cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
    raise ValueError("rotation_degrees must be one of 0, 90, 180, or 270")


def _calibration_values(
    document: dict, key: str, expected: int
) -> list[float]:
    section = document.get(key, {})
    values = [float(value) for value in section.get("data", [])]
    if len(values) != expected:
        raise ValueError(f"{key}.data must contain {expected} values")
    return values


def _distortion_values(document: dict) -> list[float]:
    section = document.get("distortion_coefficients", {})
    values = [float(value) for value in section.get("data", [])]
    if not values:
        raise ValueError("distortion_coefficients.data must not be empty")
    return values


def _load_camera_info(path_or_url: str) -> CameraInfo:
    path_text = path_or_url.removeprefix("file://")
    calibration_path = Path(path_text).expanduser()
    document = yaml.safe_load(calibration_path.read_text(encoding="utf-8"))
    if not isinstance(document, dict):
        raise ValueError("camera calibration YAML must contain a mapping")

    info = CameraInfo()
    info.width = int(document["image_width"])
    info.height = int(document["image_height"])
    info.distortion_model = str(document.get("distortion_model", "plumb_bob"))
    info.d = _distortion_values(document)
    info.k = _calibration_values(document, "camera_matrix", 9)
    info.r = _calibration_values(document, "rectification_matrix", 9)
    info.p = _calibration_values(document, "projection_matrix", 12)
    return info


class RtspCameraNode(Node):
    def __init__(self) -> None:
        super().__init__("iphone_camera")
        self.declare_parameter("rtsp_url", "rtsp://172.20.10.1:554/stream")
        self.declare_parameter("frame_id", "iphone_camera_optical_frame")
        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("rtsp_transport", "tcp")
        self.declare_parameter("reconnect_delay_sec", 2.0)
        self.declare_parameter("open_timeout_ms", 5000)
        self.declare_parameter("read_timeout_ms", 5000)
        self.declare_parameter("rotation_degrees", 0)
        self.declare_parameter("resize_width", 0)
        self.declare_parameter("resize_height", 0)
        self.declare_parameter("camera_info_url", "")

        self._url = self.get_parameter("rtsp_url").value
        self._frame_id = self.get_parameter("frame_id").value
        self._publish_rate_hz = float(
            self.get_parameter("publish_rate_hz").value
        )
        self._transport = str(
            self.get_parameter("rtsp_transport").value
        ).lower()
        self._reconnect_delay_sec = float(
            self.get_parameter("reconnect_delay_sec").value
        )
        self._open_timeout_ms = int(
            self.get_parameter("open_timeout_ms").value
        )
        self._read_timeout_ms = int(
            self.get_parameter("read_timeout_ms").value
        )
        self._rotation_degrees = int(
            self.get_parameter("rotation_degrees").value
        )
        self._resize_width = int(self.get_parameter("resize_width").value)
        self._resize_height = int(self.get_parameter("resize_height").value)
        camera_info_url = str(self.get_parameter("camera_info_url").value)

        if not self._url.startswith("rtsp://"):
            raise ValueError("rtsp_url must start with rtsp://")
        if self._transport not in {"tcp", "udp"}:
            raise ValueError("rtsp_transport must be tcp or udp")
        if self._publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be greater than zero")
        if self._rotation_degrees not in {0, 90, 180, 270}:
            raise ValueError(
                "rotation_degrees must be one of 0, 90, 180, or 270"
            )

        self._camera_info_template = CameraInfo()
        if camera_info_url:
            self._camera_info_template = _load_camera_info(camera_info_url)
            self.get_logger().info(
                f"Loaded camera calibration: {camera_info_url}"
            )
        else:
            self.get_logger().warn(
                "No camera_info_url configured; publishing uncalibrated "
                "CameraInfo"
            )

        self._bridge = CvBridge()
        self._image_pub = self.create_publisher(
            Image, "~/image_raw", qos_profile_sensor_data
        )
        self._camera_info_pub = self.create_publisher(
            CameraInfo, "~/camera_info", qos_profile_sensor_data
        )

        self._frame_lock = threading.Lock()
        self._latest_frame = None
        self._latest_stamp = None
        self._latest_sequence = 0
        self._published_sequence = 0
        self._stop_event = threading.Event()
        self._calibration_warning_sent = False

        self._configure_ffmpeg()
        self._capture_thread = threading.Thread(
            target=self._capture_loop,
            name="iphone_rtsp_capture",
            daemon=True,
        )
        self._capture_thread.start()
        self.create_timer(
            1.0 / self._publish_rate_hz, self._publish_latest_frame
        )

    def _configure_ffmpeg(self) -> None:
        if "OPENCV_FFMPEG_CAPTURE_OPTIONS" not in os.environ:
            os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = (
                f"rtsp_transport;{self._transport}"
            )

    def _open_capture(self):
        parameters: list[int] = []
        if hasattr(cv2, "CAP_PROP_OPEN_TIMEOUT_MSEC"):
            parameters.extend(
                [cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, self._open_timeout_ms]
            )
        if hasattr(cv2, "CAP_PROP_READ_TIMEOUT_MSEC"):
            parameters.extend(
                [cv2.CAP_PROP_READ_TIMEOUT_MSEC, self._read_timeout_ms]
            )

        if parameters:
            capture = cv2.VideoCapture(self._url, cv2.CAP_FFMPEG, parameters)
        else:
            capture = cv2.VideoCapture(self._url, cv2.CAP_FFMPEG)
        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        return capture

    def _resize_frame(self, frame):
        if self._resize_width <= 0 and self._resize_height <= 0:
            return frame
        height, width = frame.shape[:2]
        target_width = self._resize_width
        target_height = self._resize_height
        if target_width <= 0:
            target_width = max(1, round(width * target_height / height))
        if target_height <= 0:
            target_height = max(1, round(height * target_width / width))
        return cv2.resize(
            frame,
            (target_width, target_height),
            interpolation=cv2.INTER_AREA,
        )

    def _capture_loop(self) -> None:
        safe_url = _safe_rtsp_url(self._url)
        while rclpy.ok() and not self._stop_event.is_set():
            self.get_logger().info(
                f"Connecting to iPhone RTSP stream: {safe_url}"
            )
            capture = self._open_capture()

            if not capture.isOpened():
                self.get_logger().error(
                    f"Failed to open RTSP stream; retrying in "
                    f"{self._reconnect_delay_sec:.1f}s"
                )
                capture.release()
                self._stop_event.wait(self._reconnect_delay_sec)
                continue

            first_frame = True
            while rclpy.ok() and not self._stop_event.is_set():
                success, frame = capture.read()
                if not success or frame is None:
                    self.get_logger().warn(
                        "RTSP frame read failed; reconnecting"
                    )
                    break
                frame = _rotate_frame(frame, self._rotation_degrees)
                frame = self._resize_frame(frame)
                stamp = self.get_clock().now().to_msg()
                with self._frame_lock:
                    self._latest_frame = frame
                    self._latest_stamp = stamp
                    self._latest_sequence += 1
                if first_frame:
                    height, width = frame.shape[:2]
                    source_fps = capture.get(cv2.CAP_PROP_FPS)
                    self.get_logger().info(
                        f"RTSP connected: {width}x{height}, "
                        f"source {source_fps:.1f} FPS"
                    )
                    first_frame = False

            capture.release()
            self._stop_event.wait(self._reconnect_delay_sec)

    def _publish_latest_frame(self) -> None:
        with self._frame_lock:
            if (
                self._latest_frame is None
                or self._latest_sequence == self._published_sequence
            ):
                return
            frame = self._latest_frame
            stamp = self._latest_stamp
            sequence = self._latest_sequence

        image = self._bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        image.header.stamp = stamp
        image.header.frame_id = self._frame_id

        height, width = frame.shape[:2]
        camera_info = copy.deepcopy(self._camera_info_template)
        camera_info.header = image.header
        if camera_info.width == 0 or camera_info.height == 0:
            camera_info.width = width
            camera_info.height = height
        elif (
            (camera_info.width != width or camera_info.height != height)
            and not self._calibration_warning_sent
        ):
            self.get_logger().warn(
                "Calibration resolution does not match the published image: "
                f"calibration={camera_info.width}x{camera_info.height}, "
                f"image={width}x{height}"
            )
            self._calibration_warning_sent = True

        self._image_pub.publish(image)
        self._camera_info_pub.publish(camera_info)
        self._published_sequence = sequence

    def destroy_node(self) -> bool:
        self._stop_event.set()
        join_timeout = max(3.0, self._read_timeout_ms / 1000.0 + 1.0)
        self._capture_thread.join(timeout=join_timeout)
        if self._capture_thread.is_alive():
            self.get_logger().warn(
                "RTSP capture thread did not stop before the shutdown timeout"
            )
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = RtspCameraNode()
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
