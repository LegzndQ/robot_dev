from __future__ import annotations

import os
import threading
import time
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node

from linker_manip_interfaces.msg import HandTactile

from .ros_utils import FINGER_NAMES


FINGER_LABELS = {
    "thumb": "Thumb",
    "index": "Index",
    "middle": "Middle",
    "ring": "Ring",
    "pinky": "Pinky",
}


PALETTES = {
    "inferno": [
        (0.00, (18, 10, 48)),
        (0.22, (86, 15, 109)),
        (0.45, (187, 55, 84)),
        (0.72, (249, 142, 8)),
        (1.00, (252, 255, 164)),
    ],
    "viridis": [
        (0.00, (68, 1, 84)),
        (0.25, (59, 82, 139)),
        (0.50, (33, 145, 140)),
        (0.75, (94, 201, 98)),
        (1.00, (253, 231, 37)),
    ],
    "turbo": [
        (0.00, (48, 18, 59)),
        (0.18, (37, 117, 238)),
        (0.42, (33, 211, 159)),
        (0.68, (240, 218, 70)),
        (0.86, (232, 95, 31)),
        (1.00, (122, 4, 3)),
    ],
    "gray": [
        (0.00, (24, 24, 27)),
        (1.00, (245, 245, 245)),
    ],
}


def _interpolate_color(value: float, palette_name: str) -> tuple[int, int, int]:
    palette = PALETTES.get(palette_name, PALETTES["inferno"])
    value = float(np.clip(value, 0.0, 1.0))
    for idx in range(1, len(palette)):
        left_pos, left_rgb = palette[idx - 1]
        right_pos, right_rgb = palette[idx]
        if value <= right_pos:
            span = max(1e-6, right_pos - left_pos)
            t = (value - left_pos) / span
            return tuple(
                int(round(left_rgb[channel] + (right_rgb[channel] - left_rgb[channel]) * t))
                for channel in range(3)
            )
    return palette[-1][1]


def _matrix_max(matrix: np.ndarray) -> float:
    return float(matrix.max()) if matrix.size else 0.0


def _matrix_mean(matrix: np.ndarray) -> float:
    return float(matrix.mean()) if matrix.size else 0.0


def _matrix_percentile(matrix: np.ndarray, percentile: float) -> float:
    return float(np.percentile(matrix, percentile)) if matrix.size else 0.0


class TactileHeatmapNode(Node):
    def __init__(self) -> None:
        super().__init__("tactile_heatmap")
        self.declare_parameter("topic", "/linker/hand/tactile")
        self.declare_parameter("vmin", 0.0)
        self.declare_parameter("vmax", 80.0)
        self.declare_parameter("update_hz", 30.0)
        self.declare_parameter("cmap", "inferno")
        self.declare_parameter("auto_scale", True)
        self.declare_parameter("delta_mode", True)
        self.declare_parameter("baseline_samples", 10)

        self.topic = self.get_parameter("topic").get_parameter_value().string_value
        self.vmin = self.get_parameter("vmin").get_parameter_value().double_value
        self.vmax = self.get_parameter("vmax").get_parameter_value().double_value
        self.update_hz = self.get_parameter("update_hz").get_parameter_value().double_value
        self.cmap = self.get_parameter("cmap").get_parameter_value().string_value
        self.auto_scale = self.get_parameter("auto_scale").get_parameter_value().bool_value
        self.delta_mode = self.get_parameter("delta_mode").get_parameter_value().bool_value
        self.baseline_samples = max(
            1,
            self.get_parameter("baseline_samples").get_parameter_value().integer_value,
        )

        self._lock = threading.Lock()
        self._matrices = {
            finger: np.zeros((0, 0), dtype=np.uint8) for finger in FINGER_NAMES
        }
        self._scores = {finger: 0.0 for finger in FINGER_NAMES}
        self._last_msg_time: float | None = None
        self._message_count = 0

        self.create_subscription(HandTactile, self.topic, self._on_tactile, 10)

    def _on_tactile(self, msg: HandTactile) -> None:
        matrices: dict[str, np.ndarray] = {}
        scores: dict[str, float] = {}
        for finger in msg.fingers:
            rows = int(finger.rows)
            cols = int(finger.cols)
            if (
                finger.finger not in FINGER_NAMES
                or rows <= 0
                or cols <= 0
                or len(finger.values) != rows * cols
            ):
                continue
            matrices[finger.finger] = np.array(
                finger.values, dtype=np.uint8
            ).reshape(rows, cols)
            scores[finger.finger] = float(finger.score)

        if not matrices:
            return

        with self._lock:
            self._matrices.update(matrices)
            self._scores.update(scores)
            self._last_msg_time = time.monotonic()
            self._message_count += 1

    def snapshot(self) -> tuple[dict[str, np.ndarray], dict[str, float], float | None, int]:
        with self._lock:
            matrices = {name: value.copy() for name, value in self._matrices.items()}
            scores = dict(self._scores)
            last_msg_time = self._last_msg_time
            message_count = self._message_count
        age = None if last_msg_time is None else time.monotonic() - last_msg_time
        return matrices, scores, age, message_count


class HeatmapWidget:
    def __init__(self, finger: str, palette: str, vmin: float, vmax: float):
        from PyQt5.QtCore import QSize, Qt
        from PyQt5.QtGui import QColor, QFont, QPainter, QPen
        from PyQt5.QtWidgets import QSizePolicy, QWidget

        class _Widget(QWidget):
            def __init__(self) -> None:
                super().__init__()
                self.matrix = np.zeros((0, 0), dtype=np.float32)
                self.score = 0.0
                self.raw_max = 0.0
                self.raw_mean = 0.0
                self.palette = palette
                self.vmin = vmin
                self.vmax = vmax
                self.setMinimumSize(150, 260)
                self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

            def sizeHint(self):
                return QSize(180, 290)

            def set_data(
                self,
                matrix: np.ndarray,
                score: float,
                raw_max: float,
                raw_mean: float,
                vmin: float,
                vmax: float,
            ) -> None:
                self.matrix = matrix
                self.score = score
                self.raw_max = raw_max
                self.raw_mean = raw_mean
                self.vmin = vmin
                self.vmax = vmax
                self.update()

            def paintEvent(self, _event) -> None:
                painter = QPainter(self)
                painter.setRenderHint(QPainter.Antialiasing)
                rect = self.rect()

                painter.fillRect(rect, QColor(18, 20, 24))
                painter.setPen(Qt.NoPen)
                painter.setBrush(QColor(31, 35, 42))
                painter.drawRoundedRect(rect.adjusted(1, 1, -1, -1), 8, 8)

                painter.setPen(QColor(235, 238, 244))
                title_font = QFont()
                title_font.setPointSize(11)
                title_font.setBold(True)
                painter.setFont(title_font)
                painter.drawText(12, 24, FINGER_LABELS[finger])

                painter.setPen(QColor(160, 169, 184))
                score_font = QFont()
                score_font.setPointSize(9)
                painter.setFont(score_font)
                painter.drawText(12, 46, f"view p90 {self.score:.1f}")
                painter.drawText(12, 64, f"raw max {self.raw_max:.0f}  mean {self.raw_mean:.1f}")

                margin_x = 14
                top = 82
                bottom = 16
                rows, cols = self.matrix.shape
                if rows <= 0 or cols <= 0:
                    painter.drawText(12, 92, "waiting for data")
                    return
                gap = 3
                available_w = rect.width() - margin_x * 2
                available_h = rect.height() - top - bottom
                cell = min(
                    (available_w - gap * (cols - 1)) / cols,
                    (available_h - gap * (rows - 1)) / rows,
                )
                grid_w = cell * cols + gap * (cols - 1)
                grid_h = cell * rows + gap * (rows - 1)
                start_x = (rect.width() - grid_w) / 2
                start_y = top + max(0.0, (available_h - grid_h) / 2)

                span = max(1e-6, self.vmax - self.vmin)
                for row in range(rows):
                    for col in range(cols):
                        value = float(self.matrix[row, col])
                        norm = (value - self.vmin) / span
                        r, g, b = _interpolate_color(norm, self.palette)
                        x = int(round(start_x + col * (cell + gap)))
                        y = int(round(start_y + row * (cell + gap)))
                        painter.setBrush(QColor(r, g, b))
                        painter.setPen(QPen(QColor(42, 47, 56), 1))
                        painter.drawRoundedRect(x, y, int(cell), int(cell), 3, 3)

        self.widget = _Widget()


class TactileHeatmapWindow:
    def __init__(self, node: TactileHeatmapNode) -> None:
        from PyQt5.QtCore import Qt, QTimer
        from PyQt5.QtWidgets import (
            QCheckBox,
            QGridLayout,
            QHBoxLayout,
            QLabel,
            QMainWindow,
            QPushButton,
            QSizePolicy,
            QSlider,
            QVBoxLayout,
            QWidget,
        )

        self._node = node
        self._paused = False
        self._vmin = node.vmin
        self._vmax = node.vmax
        self._auto_scale = node.auto_scale
        self._delta_mode = node.delta_mode
        self._baseline: dict[str, np.ndarray] | None = None
        self._baseline_frames: deque[dict[str, np.ndarray]] = deque()

        self.window = QMainWindow()
        self.window.setWindowTitle("LinkerHand Tactile Heatmap")
        self.window.resize(1120, 720)

        root = QWidget()
        root.setObjectName("root")
        self.window.setCentralWidget(root)

        main_layout = QVBoxLayout(root)
        main_layout.setContentsMargins(18, 18, 18, 18)
        main_layout.setSpacing(14)

        header = QHBoxLayout()
        title = QLabel("LinkerHand Tactile Heatmap")
        title.setObjectName("title")
        title.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self._status = QLabel(f"Waiting for {node.topic}")
        self._status.setObjectName("status")
        header.addWidget(title)
        header.addWidget(self._status)
        main_layout.addLayout(header)

        grid = QGridLayout()
        grid.setSpacing(14)
        self._heatmaps = {}
        positions = [(0, 0), (0, 1), (0, 2), (1, 0), (1, 1)]
        for finger, (row, col) in zip(FINGER_NAMES, positions):
            heatmap = HeatmapWidget(finger, node.cmap, self._vmin, self._vmax)
            self._heatmaps[finger] = heatmap.widget
            grid.addWidget(heatmap.widget, row, col)
        main_layout.addLayout(grid, stretch=1)

        controls = QHBoxLayout()
        self._pause_checkbox = QCheckBox("Pause")
        self._pause_checkbox.stateChanged.connect(
            lambda state: setattr(self, "_paused", state == Qt.Checked)
        )
        self._delta_checkbox = QCheckBox("Delta")
        self._delta_checkbox.setChecked(self._delta_mode)
        self._delta_checkbox.stateChanged.connect(self._set_delta_mode)
        self._baseline_button = QPushButton("Set baseline")
        self._baseline_button.clicked.connect(self._request_baseline)
        self._auto_scale_checkbox = QCheckBox("Auto scale")
        self._auto_scale_checkbox.setChecked(self._auto_scale)
        self._auto_scale_checkbox.stateChanged.connect(
            lambda state: setattr(self, "_auto_scale", state == Qt.Checked)
        )
        self._vmax_label = QLabel(f"vmax {self._vmax:.0f}")
        self._vmax_slider = QSlider(Qt.Horizontal)
        self._vmax_slider.setRange(10, 255)
        self._vmax_slider.setValue(int(np.clip(self._vmax, 10, 255)))
        self._vmax_slider.valueChanged.connect(self._set_vmax)
        controls.addWidget(self._pause_checkbox)
        controls.addWidget(self._delta_checkbox)
        controls.addWidget(self._baseline_button)
        controls.addWidget(self._auto_scale_checkbox)
        controls.addWidget(self._vmax_label)
        controls.addWidget(self._vmax_slider, stretch=1)
        main_layout.addLayout(controls)

        self.window.setStyleSheet(
            """
            QWidget#root {
                background: #111318;
                color: #eef2f8;
                font-family: Sans Serif;
            }
            QLabel#title {
                color: #f8fafc;
                font-size: 20px;
                font-weight: 700;
            }
            QLabel#status {
                color: #9aa4b5;
                font-size: 12px;
            }
            QCheckBox {
                color: #d6deea;
                spacing: 8px;
            }
            QPushButton {
                color: #eef2f8;
                background: #273142;
                border: 1px solid #3b4658;
                border-radius: 6px;
                padding: 6px 12px;
            }
            QPushButton:hover {
                background: #334155;
            }
            QSlider::groove:horizontal {
                height: 7px;
                border-radius: 3px;
                background: #293241;
            }
            QSlider::handle:horizontal {
                width: 18px;
                margin: -6px 0;
                border-radius: 9px;
                background: #38bdf8;
            }
            """
        )

        interval_ms = int(1000.0 / max(1.0, node.update_hz))
        self._timer = QTimer(self.window)
        self._timer.timeout.connect(self._update)
        self._timer.start(interval_ms)

        self._ros_timer = QTimer(self.window)
        self._ros_timer.timeout.connect(self._spin_ros_once)
        self._ros_timer.start(5)

    def _set_vmax(self, value: int) -> None:
        self._vmax = float(value)
        self._auto_scale = False
        self._auto_scale_checkbox.setChecked(False)
        self._vmax_label.setText(f"vmax {value}")

    def _set_delta_mode(self, state: int) -> None:
        self._delta_mode = state != 0
        if self._delta_mode and self._baseline is None:
            self._request_baseline()

    def _request_baseline(self) -> None:
        self._baseline = None
        self._baseline_frames.clear()

    def _spin_ros_once(self) -> None:
        if not rclpy.ok():
            return
        try:
            rclpy.spin_once(self._node, timeout_sec=0.0)
        except Exception as exc:
            self._status.setText(f"ROS spin error: {exc}")

    def _update(self) -> None:
        if self._paused:
            return
        matrices, _scores, age, message_count = self._node.snapshot()
        raw_matrices = {
            finger: matrices[finger].astype(np.float32) for finger in FINGER_NAMES
        }

        baseline_status = "raw"
        if self._delta_mode:
            baseline_status = "waiting baseline"
            if self._baseline is None and age is not None and message_count > 0:
                baseline_status = "collecting baseline"
                self._baseline_frames.append(
                    {finger: value.copy() for finger, value in raw_matrices.items()}
                )
                if len(self._baseline_frames) >= self._node.baseline_samples:
                    self._baseline = {}
                    for finger in FINGER_NAMES:
                        frames = [
                            frame[finger]
                            for frame in self._baseline_frames
                            if frame[finger].size > 0
                        ]
                        if frames:
                            shape = frames[-1].shape
                            frames = [frame for frame in frames if frame.shape == shape]
                            stack = np.stack(frames)
                            self._baseline[finger] = np.mean(stack, axis=0)
                        else:
                            self._baseline[finger] = np.zeros((0, 0), dtype=np.float32)
                    self._baseline_frames.clear()
            if self._baseline is not None:
                baseline_status = "delta"

        display_matrices: dict[str, np.ndarray] = {}
        for finger, raw_matrix in raw_matrices.items():
            baseline = None if self._baseline is None else self._baseline.get(finger)
            if (
                self._delta_mode
                and baseline is not None
                and baseline.shape == raw_matrix.shape
            ):
                display_matrices[finger] = np.clip(
                    raw_matrix - baseline,
                    0.0,
                    255.0,
                )
            else:
                display_matrices[finger] = raw_matrix

        global_raw_max = max(_matrix_max(matrix) for matrix in raw_matrices.values())
        global_view_max = max(_matrix_max(matrix) for matrix in display_matrices.values())
        display_vmax = self._vmax
        if self._auto_scale:
            display_vmax = float(np.clip(max(10.0, global_view_max * 1.2), 10.0, 255.0))
            self._vmax_label.setText(f"auto vmax {display_vmax:.0f}")
        else:
            self._vmax_label.setText(f"vmax {self._vmax:.0f}")

        for finger, widget in self._heatmaps.items():
            raw_matrix = raw_matrices[finger]
            display_matrix = display_matrices[finger]
            display_score = _matrix_percentile(display_matrix, 90)
            widget.set_data(
                display_matrix,
                display_score,
                _matrix_max(raw_matrix),
                _matrix_mean(raw_matrix),
                self._vmin,
                display_vmax,
            )

        if age is None:
            self._status.setText(f"Waiting for {self._node.topic}")
        else:
            self._status.setText(
                f"msg #{message_count} | age {age * 1000.0:.0f} ms | "
                f"raw max {global_raw_max:.0f} | view max {global_view_max:.1f} | "
                f"{baseline_status}"
            )

    def show(self) -> None:
        self.window.show()


def main(args: list[str] | None = None) -> None:
    try:
        from PyQt5.QtWidgets import QApplication
    except ImportError as exc:
        raise SystemExit(
            "PyQt5 is required for tactile_heatmap_node. "
            "Install it with: sudo apt-get install python3-pyqt5"
        ) from exc

    if not os.environ.get("DISPLAY") and not os.environ.get("WAYLAND_DISPLAY"):
        raise SystemExit(
            "No graphical display found. Run this from a desktop terminal, "
            "or enable X11 forwarding/VNC before starting tactile_heatmap_node."
        )

    rclpy.init(args=args)
    node = TactileHeatmapNode()

    app = QApplication.instance() or QApplication([])
    window = TactileHeatmapWindow(node)
    window.show()

    try:
        app.exec_()
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
