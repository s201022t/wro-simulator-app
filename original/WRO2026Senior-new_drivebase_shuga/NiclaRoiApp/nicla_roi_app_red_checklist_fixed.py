#!/usr/bin/env python3
from __future__ import annotations

import sys
import threading
import time
from pathlib import Path
from typing import Callable, Optional

import numpy as np
import serial
from PIL import Image
from serial.tools import list_ports

from PySide6.QtCore import QPointF, QRectF, Qt, QThread, Signal
from PySide6.QtGui import QBrush, QColor, QImage, QPainter, QPen
from PySide6.QtWidgets import (
    QApplication,
    QDialog,
    QComboBox,
    QCheckBox,
    QFileDialog,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QSpinBox,
    QSplitter,
    QStatusBar,
    QTabWidget,
    QTableWidget,
    QTableWidgetItem,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)


APP_NAME = "Nicla ROI App"
BAUD_RATE = 115200
IMAGE_WIDTH = 320
IMAGE_HEIGHT = 240
ROI_COUNT = 12

DATA_DIR = Path.home() / "Documents" / "NiclaRoiApp"
DATA_DIR.mkdir(parents=True, exist_ok=True)
LATEST_IMAGE_PATH = DATA_DIR / "nicla_latest.png"


class SerialProtocolError(RuntimeError):
    pass


class NiclaSerialClient:
    def __init__(self) -> None:
        self.ser: Optional[serial.Serial] = None
        self.lock = threading.Lock()

    @property
    def connected(self) -> bool:
        return self.ser is not None and self.ser.is_open

    def connect(self, port: str, log: Callable[[str], None]) -> str:
        with self.lock:
            self.disconnect_silent()

            log(f"{port} を開いています...")
            self.ser = serial.Serial(
                port=port,
                baudrate=BAUD_RATE,
                timeout=0.15,
                write_timeout=5,
            )

            # USBシリアル接続でNiclaが再起動する場合に備える
            time.sleep(1.5)
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()

            log(f"接続完了: {port} @ {BAUD_RATE}")
            return port

    def disconnect(self, log: Callable[[str], None]) -> None:
        with self.lock:
            if self.ser is not None:
                port = self.ser.port
                self.disconnect_silent()
                log(f"切断しました: {port}")

    def disconnect_silent(self) -> None:
        if self.ser is not None:
            try:
                if self.ser.is_open:
                    self.ser.close()
            finally:
                self.ser = None

    def _require_serial(self) -> serial.Serial:
        if not self.connected:
            raise SerialProtocolError("Niclaへ接続されていません。")
        assert self.ser is not None
        return self.ser

    def _prepare_command(self) -> serial.Serial:
        ser = self._require_serial()
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        return ser

    @staticmethod
    def _send_line(
        ser: serial.Serial,
        command: str,
        log: Callable[[str], None],
    ) -> None:
        ser.write((command + "\n").encode("ascii"))
        ser.flush()
        log(f"PC TX: {command}")

    @staticmethod
    def _read_line(ser: serial.Serial) -> str:
        raw = ser.readline()
        if not raw:
            return ""
        return raw.decode("ascii", errors="replace").strip()

    def _wait_for_prefix(
        self,
        ser: serial.Serial,
        prefix: str,
        log: Callable[[str], None],
        timeout_seconds: float = 20.0,
    ) -> str:
        deadline = time.monotonic() + timeout_seconds

        while time.monotonic() < deadline:
            line = self._read_line(ser)
            if not line:
                continue

            log(f"Nicla RX: {line}")

            if line.startswith("ERROR") or line.startswith("ERR:"):
                raise SerialProtocolError(line)

            if line.startswith(prefix):
                return line

        raise TimeoutError(
            f"{prefix!r}で始まる応答を受信できませんでした。"
        )

    @staticmethod
    def _read_exactly(ser: serial.Serial, size: int) -> bytes:
        data = bytearray()
        deadline = time.monotonic() + 10.0

        while len(data) < size:
            chunk = ser.read(size - len(data))

            if chunk:
                data.extend(chunk)
                deadline = time.monotonic() + 10.0
                continue

            if time.monotonic() > deadline:
                raise TimeoutError(
                    f"画像受信タイムアウト: {len(data)} / {size} bytes"
                )

        return bytes(data)

    @staticmethod
    def _rgb565_be_to_rgb888(
        raw_data: bytes,
        width: int,
        height: int,
    ) -> np.ndarray:
        pixels = np.frombuffer(raw_data, dtype=">u2").reshape(
            (height, width)
        )

        red5 = (pixels >> 11) & 0x1F
        green6 = (pixels >> 5) & 0x3F
        blue5 = pixels & 0x1F

        red8 = ((red5 << 3) | (red5 >> 2)).astype(np.uint8)
        green8 = ((green6 << 2) | (green6 >> 4)).astype(np.uint8)
        blue8 = ((blue5 << 3) | (blue5 >> 2)).astype(np.uint8)

        return np.stack((red8, green8, blue8), axis=-1)

    def receive_image(
        self,
        command: str,
        log: Callable[[str], None],
    ) -> np.ndarray:
        with self.lock:
            ser = self._prepare_command()
            self._send_line(ser, command, log)

            header = self._wait_for_prefix(ser, "IMAGE ", log)
            parts = header.split()

            if len(parts) != 4:
                raise SerialProtocolError(
                    f"IMAGEヘッダーが不正です: {header}"
                )

            width = int(parts[1])
            height = int(parts[2])
            image_size = int(parts[3])
            row_size = width * 2

            if image_size != width * height * 2:
                raise SerialProtocolError(
                    "画像サイズがヘッダーと一致しません。"
                )

            self._send_line(ser, "START", log)
            image_data = bytearray()

            for expected_row in range(height):
                row_header = self._wait_for_prefix(ser, "ROW ", log)
                row_parts = row_header.split()

                if len(row_parts) != 3:
                    raise SerialProtocolError(
                        f"ROWヘッダーが不正です: {row_header}"
                    )

                row_number = int(row_parts[1])
                received_size = int(row_parts[2])

                if row_number != expected_row:
                    raise SerialProtocolError(
                        "行番号不一致: "
                        f"expected={expected_row}, actual={row_number}"
                    )

                if received_size != row_size:
                    raise SerialProtocolError(
                        "行サイズ不一致: "
                        f"expected={row_size}, actual={received_size}"
                    )

                image_data.extend(
                    self._read_exactly(ser, received_size)
                )
                self._send_line(ser, f"ACK {expected_row}", log)

            self._wait_for_prefix(ser, "DONE", log)

        rgb = self._rgb565_be_to_rgb888(
            bytes(image_data),
            width,
            height,
        )

        # Niclaの生画像をPC表示座標へ合わせる
        rgb = np.flipud(rgb).copy()

        Image.fromarray(rgb, mode="RGB").save(LATEST_IMAGE_PATH)
        log(f"画像を保存しました: {LATEST_IMAGE_PATH}")

        return rgb

    def get_roi_config(
        self,
        log: Callable[[str], None],
    ) -> list[dict[str, int]]:
        with self.lock:
            ser = self._prepare_command()
            self._send_line(ser, "GET_ROI_CONFIG", log)
            self._wait_for_prefix(ser, "ROI_CONFIG_BEGIN", log)

            rois: list[dict[str, int]] = []

            while True:
                line = self._read_line(ser)
                if not line:
                    continue

                log(f"Nicla RX: {line}")

                if line == "ROI_CONFIG_END":
                    break

                if line.startswith("ERROR") or line.startswith("ERR:"):
                    raise SerialProtocolError(line)

                if not line.startswith("ROI_CONFIG,"):
                    continue

                parts = line.split(",")
                if len(parts) != 6:
                    continue

                rois.append(
                    {
                        "x": int(parts[2]),
                        "y": int(parts[3]),
                        "width": int(parts[4]),
                        "height": int(parts[5]),
                    }
                )

            if len(rois) != ROI_COUNT:
                raise SerialProtocolError(
                    f"ROI設定数が不正です: {len(rois)}"
                )

            return rois

    def save_roi_config(
        self,
        rois: list[dict[str, int]],
        log: Callable[[str], None],
    ) -> bool:
        if len(rois) != ROI_COUNT:
            raise ValueError("ROIは12個必要です。")

        with self.lock:
            ser = self._prepare_command()

            for index, roi in enumerate(rois, start=1):
                command = (
                    f"SET_ROI,{index},"
                    f"{int(roi['x'])},{int(roi['y'])},"
                    f"{int(roi['width'])},{int(roi['height'])}"
                )

                self._send_line(ser, command, log)
                self._wait_for_prefix(
                    ser,
                    f"ROI_SAVED,{index}",
                    log,
                )

        return True

    def run_read_test(
        self,
        log: Callable[[str], None],
    ) -> list[str]:
        with self.lock:
            ser = self._prepare_command()
            self._send_line(ser, "READ_TEST", log)
            self._wait_for_prefix(ser, "RESULT_BEGIN,", log)

            lines: list[str] = []

            while True:
                line = self._read_line(ser)
                if not line:
                    continue

                log(f"Nicla RX: {line}")

                if line.startswith("ERROR") or line.startswith("ERR:"):
                    raise SerialProtocolError(line)

                lines.append(line)

                if line.startswith("RESULT_END,"):
                    break

            return lines

    def get_hsv_config(
        self,
        log: Callable[[str], None],
    ) -> list[dict[str, int | str]]:
        with self.lock:
            ser = self._prepare_command()
            self._send_line(ser, "GET_HSV_CONFIG", log)
            self._wait_for_prefix(ser, "HSV_CONFIG_BEGIN", log)

            rows: list[dict[str, int | str]] = []

            while True:
                line = self._read_line(ser)
                if not line:
                    continue

                log(f"Nicla RX: {line}")

                if line == "HSV_CONFIG_END":
                    break

                if line.startswith("ERROR") or line.startswith("ERR:"):
                    raise SerialProtocolError(line)

                if not line.startswith("HSV_CONFIG,"):
                    continue

                parts = line.split(",")
                if len(parts) != 8:
                    continue

                rows.append(
                    {
                        "name": parts[1],
                        "h_min": int(parts[2]),
                        "h_max": int(parts[3]),
                        "s_min": int(parts[4]),
                        "s_max": int(parts[5]),
                        "v_min": int(parts[6]),
                        "v_max": int(parts[7]),
                    }
                )

            return rows

    def save_hsv_config(
        self,
        rows: list[dict[str, int | str]],
        log: Callable[[str], None],
    ) -> bool:
        with self.lock:
            ser = self._prepare_command()

            for row in rows:
                name = str(row["name"]).upper()
                command = (
                    f"SET_HSV,{name},"
                    f"{int(row['h_min'])},{int(row['h_max'])},"
                    f"{int(row['s_min'])},{int(row['s_max'])},"
                    f"{int(row['v_min'])},{int(row['v_max'])}"
                )

                self._send_line(ser, command, log)
                self._wait_for_prefix(
                    ser,
                    f"HSV_SAVED,{name}",
                    log,
                )

        return True

    def get_color_adjust(
        self,
        log: Callable[[str], None],
    ) -> tuple[int, int]:
        with self.lock:
            ser = self._prepare_command()
            self._send_line(ser, "GET_COLOR_ADJUST", log)

            line = self._wait_for_prefix(
                ser,
                "COLOR_ADJUST,",
                log,
            )

            parts = line.split(",")
            if len(parts) != 3:
                raise SerialProtocolError(
                    f"COLOR_ADJUST応答が不正です: {line}"
                )

            brightness_percent = int(parts[1])
            saturation_percent = int(parts[2])

            if not 0 <= brightness_percent <= 400:
                raise SerialProtocolError(
                    "Niclaから受信した明るさ補正値が範囲外です。"
                )

            if not 0 <= saturation_percent <= 400:
                raise SerialProtocolError(
                    "Niclaから受信した彩度補正値が範囲外です。"
                )

            return brightness_percent, saturation_percent

    def save_color_adjust(
        self,
        brightness_percent: int,
        saturation_percent: int,
        log: Callable[[str], None],
    ) -> bool:
        brightness_percent = int(brightness_percent)
        saturation_percent = int(saturation_percent)

        if not 0 <= brightness_percent <= 400:
            raise ValueError("明るさ補正は0～400%で指定してください。")

        if not 0 <= saturation_percent <= 400:
            raise ValueError("彩度補正は0～400%で指定してください。")

        with self.lock:
            ser = self._prepare_command()
            command = (
                "SET_COLOR_ADJUST,"
                f"{brightness_percent},"
                f"{saturation_percent}"
            )

            self._send_line(ser, command, log)
            response = self._wait_for_prefix(
                ser,
                "COLOR_ADJUST_SAVED,",
                log,
            )

            expected = (
                "COLOR_ADJUST_SAVED,"
                f"{brightness_percent},"
                f"{saturation_percent}"
            )

            if response != expected:
                raise SerialProtocolError(
                    "補正設定の保存応答が一致しません: "
                    f"expected={expected}, actual={response}"
                )

        return True


class OperationThread(QThread):
    result = Signal(object)
    error = Signal(str)
    log = Signal(str)

    def __init__(
        self,
        task: Callable[[Callable[[str], None]], object],
    ) -> None:
        super().__init__()
        self.task = task

    def run(self) -> None:
        try:
            result = self.task(self.log.emit)
            self.result.emit(result)
        except Exception as error:
            self.error.emit(str(error))


class ImageCanvas(QWidget):
    roi_changed = Signal()
    selection_changed = Signal(int)

    def __init__(self) -> None:
        super().__init__()

        self.setMinimumSize(640, 480)
        self.setMouseTracking(True)

        self.image: Optional[QImage] = None
        self.rois: list[dict[str, int]] = []
        self.selected_index = -1

        self.dragging = False
        self.drag_offset_x = 0
        self.drag_offset_y = 0

    @staticmethod
    def default_size_for_index(index: int) -> tuple[int, int]:
        return 5, 25

    def set_rgb_image(self, rgb: np.ndarray) -> None:
        height, width, channels = rgb.shape

        if channels != 3:
            raise ValueError("RGB画像ではありません。")

        qimage = QImage(
            rgb.data,
            width,
            height,
            width * 3,
            QImage.Format.Format_RGB888,
        ).copy()

        self.image = qimage
        self.update()

    def set_rois(self, rois: list[dict[str, int]]) -> None:
        self.rois = [
            {
                "x": int(roi["x"]),
                "y": int(roi["y"]),
                "width": int(roi["width"]),
                "height": int(roi["height"]),
            }
            for roi in rois
        ]

        self.selected_index = 0 if self.rois else -1
        self.selection_changed.emit(self.selected_index)
        self.roi_changed.emit()
        self.update()

    def clear_rois(self) -> None:
        self.rois.clear()
        self.selected_index = -1
        self.selection_changed.emit(-1)
        self.roi_changed.emit()
        self.update()

    def delete_selected(self) -> None:
        if not (0 <= self.selected_index < len(self.rois)):
            return

        del self.rois[self.selected_index]

        if self.selected_index >= len(self.rois):
            self.selected_index = len(self.rois) - 1

        self.selection_changed.emit(self.selected_index)
        self.roi_changed.emit()
        self.update()

    def set_selected_index(self, index: int) -> None:
        if 0 <= index < len(self.rois):
            self.selected_index = index
        else:
            self.selected_index = -1

        self.selection_changed.emit(self.selected_index)
        self.update()

    def set_selected_size(self, width: int, height: int) -> None:
        if not (0 <= self.selected_index < len(self.rois)):
            return

        self.rois[self.selected_index]["width"] = max(
            1,
            min(IMAGE_WIDTH, int(width)),
        )

        self.rois[self.selected_index]["height"] = max(
            1,
            min(IMAGE_HEIGHT, int(height)),
        )

        self.roi_changed.emit()
        self.update()

    def _image_target_rect(self) -> QRectF:
        if self.image is None:
            return QRectF()

        image_size = self.image.size()
        widget_width = self.width()
        widget_height = self.height()

        scale = min(
            widget_width / image_size.width(),
            widget_height / image_size.height(),
        )

        draw_width = image_size.width() * scale
        draw_height = image_size.height() * scale

        left = (widget_width - draw_width) / 2.0
        top = (widget_height - draw_height) / 2.0

        return QRectF(left, top, draw_width, draw_height)

    def _widget_to_image(self, point: QPointF) -> Optional[QPointF]:
        target = self._image_target_rect()

        if (
            target.isNull()
            or not target.contains(point)
        ):
            return None

        image_x = (
            (point.x() - target.left())
            * IMAGE_WIDTH
            / target.width()
        )

        image_y = (
            (point.y() - target.top())
            * IMAGE_HEIGHT
            / target.height()
        )

        return QPointF(
            max(0, min(IMAGE_WIDTH - 1, image_x)),
            max(0, min(IMAGE_HEIGHT - 1, image_y)),
        )

    def _image_to_widget_rect(
        self,
        roi: dict[str, int],
    ) -> QRectF:
        target = self._image_target_rect()

        x_scale = target.width() / IMAGE_WIDTH
        y_scale = target.height() / IMAGE_HEIGHT

        x1 = (
            roi["x"] - roi["width"] / 2
        ) * x_scale + target.left()

        y1 = (
            roi["y"] - roi["height"] / 2
        ) * y_scale + target.top()

        return QRectF(
            x1,
            y1,
            roi["width"] * x_scale,
            roi["height"] * y_scale,
        )

    def _find_roi_at(self, image_point: QPointF) -> int:
        x = image_point.x()
        y = image_point.y()

        for index in range(len(self.rois) - 1, -1, -1):
            roi = self.rois[index]

            if (
                roi["x"] - roi["width"] / 2 <= x
                <= roi["x"] + roi["width"] / 2
                and
                roi["y"] - roi["height"] / 2 <= y
                <= roi["y"] + roi["height"] / 2
            ):
                return index

        return -1

    def paintEvent(self, event) -> None:
        painter = QPainter(self)
        painter.fillRect(self.rect(), QColor("#202020"))

        target = self._image_target_rect()

        if self.image is not None and not target.isNull():
            painter.drawImage(target, self.image)

        for index, roi in enumerate(self.rois):
            rect = self._image_to_widget_rect(roi)
            selected = index == self.selected_index

            pen = QPen(
                QColor("#ff9800" if selected else "#ff00ff")
            )
            pen.setWidth(3 if selected else 2)

            painter.setPen(pen)
            painter.setBrush(Qt.BrushStyle.NoBrush)
            painter.drawRect(rect)

            painter.setBrush(
                QBrush(
                    QColor("#ff9800" if selected else "#ff00ff")
                )
            )

            painter.drawEllipse(
                rect.center(),
                3,
                3,
            )

            painter.drawText(
                rect.topLeft() + QPointF(2, -4),
                f"ROI{index + 1}",
            )

        if self.image is None:
            painter.setPen(QColor("white"))
            painter.drawText(
                self.rect(),
                Qt.AlignmentFlag.AlignCenter,
                "Niclaへ接続して画像を取得してください",
            )

    def mousePressEvent(self, event) -> None:
        if event.button() != Qt.MouseButton.LeftButton:
            return

        image_point = self._widget_to_image(event.position())
        if image_point is None:
            return

        hit = self._find_roi_at(image_point)

        if hit >= 0:
            self.selected_index = hit
            roi = self.rois[hit]

            self.drag_offset_x = int(image_point.x()) - roi["x"]
            self.drag_offset_y = int(image_point.y()) - roi["y"]
            self.dragging = True

            self.selection_changed.emit(hit)
            self.update()
            return

        if len(self.rois) >= ROI_COUNT:
            return

        width, height = self.default_size_for_index(len(self.rois))

        self.rois.append(
            {
                "x": int(round(image_point.x())),
                "y": int(round(image_point.y())),
                "width": width,
                "height": height,
            }
        )

        self.selected_index = len(self.rois) - 1
        self.selection_changed.emit(self.selected_index)
        self.roi_changed.emit()
        self.update()

    def mouseMoveEvent(self, event) -> None:
        if not self.dragging:
            return

        if not (0 <= self.selected_index < len(self.rois)):
            return

        image_point = self._widget_to_image(event.position())
        if image_point is None:
            return

        roi = self.rois[self.selected_index]

        roi["x"] = max(
            0,
            min(
                IMAGE_WIDTH - 1,
                int(round(image_point.x())) - self.drag_offset_x,
            ),
        )

        roi["y"] = max(
            0,
            min(
                IMAGE_HEIGHT - 1,
                int(round(image_point.y())) - self.drag_offset_y,
            ),
        )

        self.roi_changed.emit()
        self.update()

    def mouseReleaseEvent(self, event) -> None:
        self.dragging = False


class MainWindow(QMainWindow):
    HSV_NAMES = [
        "BLACK",
        "WHITE",
        "YELLOW",
        "RED",
        "BLUE",
        "GREEN",
    ]

    def __init__(self) -> None:
        super().__init__()

        self.client = NiclaSerialClient()
        self.worker: Optional[OperationThread] = None
        self.current_rgb: Optional[np.ndarray] = None
        self.simulated_rgb: Optional[np.ndarray] = None
        self.displayed_rgb: Optional[np.ndarray] = None
        self.result_handler: Optional[Callable[[object], None]] = None

        self.setWindowTitle(APP_NAME)
        self.resize(1280, 820)

        self._build_ui()
        self.refresh_ports()
        self._set_connected_ui(False)

        self.statusBar().showMessage(
            "Arduino IDEのシリアルモニターを閉じてください。"
        )

    def _build_ui(self) -> None:
        root = QWidget()
        root_layout = QVBoxLayout(root)

        connection_box = QGroupBox("Nicla Vision接続")
        connection_layout = QHBoxLayout(connection_box)

        self.port_combo = QComboBox()
        self.refresh_button = QPushButton("COM更新")
        self.connect_button = QPushButton("接続")
        self.disconnect_button = QPushButton("切断")
        self.connection_label = QLabel("未接続")

        connection_layout.addWidget(QLabel("COMポート:"))
        connection_layout.addWidget(self.port_combo, 1)
        connection_layout.addWidget(self.refresh_button)
        connection_layout.addWidget(self.connect_button)
        connection_layout.addWidget(self.disconnect_button)
        connection_layout.addWidget(self.connection_label)

        root_layout.addWidget(connection_box)

        self.tabs = QTabWidget()
        root_layout.addWidget(self.tabs, 1)

        self._build_roi_tab()
        self._build_hsv_tab()
        self._build_log_tab()

        self.setCentralWidget(root)
        self.setStatusBar(QStatusBar())

        self.refresh_button.clicked.connect(self.refresh_ports)
        self.connect_button.clicked.connect(self.connect_selected_port)
        self.disconnect_button.clicked.connect(self.disconnect_port)

    def _build_roi_tab(self) -> None:
        tab = QWidget()
        layout = QVBoxLayout(tab)

        action_row = QHBoxLayout()

        self.last_photo_button = QPushButton("直近の保存画像を表示")
        self.load_roi_button = QPushButton("NiclaのROI設定を取得")
        self.generate_config_button = QPushButton("初期設定用ROIコードを生成")
        self.read_test_button = QPushButton("色読みテスト")
        self.local_read_button = QPushButton("直近画像＋設定中ROIで色読み")
        self.save_image_button = QPushButton("画像をPNG保存")

        action_row.addWidget(self.last_photo_button)
        action_row.addWidget(self.load_roi_button)
        action_row.addWidget(self.generate_config_button)
        action_row.addWidget(self.read_test_button)
        action_row.addWidget(self.local_read_button)
        action_row.addWidget(self.save_image_button)

        layout.addLayout(action_row)

        simulation_box = QGroupBox(
            "6色化シミュレーション（HSV設定タブの現在値を使用）"
        )
        simulation_layout = QVBoxLayout(simulation_box)
        simulation_button_row = QHBoxLayout()

        self.show_original_button = QPushButton("元画像を表示")
        self.simulate_six_button = QPushButton("6色化して表示")
        self.save_six_button = QPushButton("6色化画像をPNG保存")
        self.display_mode_label = QLabel("表示: 元画像")

        simulation_button_row.addWidget(self.show_original_button)
        simulation_button_row.addWidget(self.simulate_six_button)
        simulation_button_row.addWidget(self.save_six_button)
        simulation_button_row.addWidget(self.display_mode_label)
        simulation_button_row.addStretch()

        self.simulation_summary_label = QLabel(
            "画像を取得後、「6色化して表示」を押してください。"
        )
        self.simulation_summary_label.setWordWrap(True)

        legend_label = QLabel(
            "表示色: 黒=(0,0,0) / 白=(255,255,255) / "
            "黄=(255,255,0) / 赤=(255,0,0) / "
            "青=(0,0,255) / 緑=(0,255,0)"
        )
        legend_label.setWordWrap(True)

        simulation_layout.addLayout(simulation_button_row)
        simulation_layout.addWidget(self.simulation_summary_label)
        simulation_layout.addWidget(legend_label)

        layout.addWidget(simulation_box)

        self.show_original_button.setEnabled(False)
        self.simulate_six_button.setEnabled(False)
        self.save_six_button.setEnabled(False)

        splitter = QSplitter(Qt.Orientation.Horizontal)

        self.canvas = ImageCanvas()
        splitter.addWidget(self.canvas)

        side = QWidget()
        side_layout = QVBoxLayout(side)

        selection_box = QGroupBox("選択ROI")
        selection_form = QFormLayout(selection_box)

        self.selected_label = QLabel("なし")

        self.width_spin = QSpinBox()
        self.width_spin.setRange(1, IMAGE_WIDTH)
        self.width_spin.setValue(5)

        self.height_spin = QSpinBox()
        self.height_spin.setRange(1, IMAGE_HEIGHT)
        self.height_spin.setValue(25)

        self.apply_size_button = QPushButton("サイズ反映")
        self.delete_roi_button = QPushButton("選択ROI削除")
        self.clear_roi_button = QPushButton("ROIを全部消す")

        selection_form.addRow("選択:", self.selected_label)
        selection_form.addRow("幅:", self.width_spin)
        selection_form.addRow("高さ:", self.height_spin)
        selection_form.addRow(self.apply_size_button)
        selection_form.addRow(self.delete_roi_button)
        selection_form.addRow(self.clear_roi_button)

        side_layout.addWidget(selection_box)

        self.roi_table = QTableWidget(ROI_COUNT, 5)
        self.roi_table.setHorizontalHeaderLabels(
            ["ROI", "中心X", "中心Y", "幅", "高さ"]
        )

        self.roi_table.horizontalHeader().setSectionResizeMode(
            QHeaderView.ResizeMode.Stretch
        )
        self.roi_table.setEditTriggers(
            QTableWidget.EditTrigger.NoEditTriggers
        )
        self.roi_table.setSelectionBehavior(
            QTableWidget.SelectionBehavior.SelectRows
        )

        side_layout.addWidget(self.roi_table, 1)

        result_box = QGroupBox("色読み結果")
        result_layout = QVBoxLayout(result_box)

        self.result_text = QTextEdit()
        self.result_text.setReadOnly(True)
        self.result_text.setPlaceholderText(
            "「色読みテスト」を押すと結果が表示されます。"
        )

        result_layout.addWidget(self.result_text)
        side_layout.addWidget(result_box, 1)

        splitter.addWidget(side)
        splitter.setSizes([760, 480])

        layout.addWidget(splitter, 1)
        self.tabs.addTab(tab, "画像・ROI")

        self.last_photo_button.clicked.connect(
            lambda: self.receive_image("GET_LAST_PHOTO")
        )

        self.load_roi_button.clicked.connect(self.load_roi_config)
        self.generate_config_button.clicked.connect(self.generate_initial_config)
        self.read_test_button.clicked.connect(self.run_read_test)
        self.local_read_button.clicked.connect(self.run_local_roi_read_test)
        self.save_image_button.clicked.connect(self.save_current_image)

        self.show_original_button.clicked.connect(
            self.show_original_image
        )
        self.simulate_six_button.clicked.connect(
            self.simulate_six_colors
        )
        self.save_six_button.clicked.connect(
            self.save_six_color_image
        )

        self.apply_size_button.clicked.connect(
            lambda: self.canvas.set_selected_size(
                self.width_spin.value(),
                self.height_spin.value(),
            )
        )

        self.delete_roi_button.clicked.connect(
            self.canvas.delete_selected
        )

        self.clear_roi_button.clicked.connect(self.canvas.clear_rois)

        self.canvas.selection_changed.connect(
            self.on_canvas_selection_changed
        )

        self.canvas.roi_changed.connect(self.refresh_roi_table)

        self.roi_table.cellClicked.connect(
            lambda row, column: self.canvas.set_selected_index(row)
        )

    def _build_hsv_tab(self) -> None:
        tab = QWidget()
        layout = QVBoxLayout(tab)

        button_row = QHBoxLayout()

        self.load_hsv_button = QPushButton("NiclaのHSV設定を取得")
        self.send_hsv_button = QPushButton("HSV設定をNiclaへ送信")

        button_row.addWidget(self.load_hsv_button)
        button_row.addWidget(self.send_hsv_button)
        button_row.addStretch()

        layout.addLayout(button_row)

        adjustment_box = QGroupBox(
            "6色判定前の補正（100%で補正なし）"
        )
        adjustment_form = QFormLayout(adjustment_box)

        self.brightness_spin = QSpinBox()
        self.brightness_spin.setRange(0, 400)
        self.brightness_spin.setValue(100)
        self.brightness_spin.setSuffix(" %")
        self.brightness_spin.setToolTip(
            "HSVのV値へ適用します。100%は補正なしです。"
        )

        self.saturation_spin = QSpinBox()
        self.saturation_spin.setRange(0, 400)
        self.saturation_spin.setValue(160)
        self.saturation_spin.setSuffix(" %")
        self.saturation_spin.setToolTip(
            "HSVのS値へ適用します。100%は補正なしです。"
        )

        self.load_adjust_button = QPushButton(
            "Niclaの補正設定を取得"
        )
        self.send_adjust_button = QPushButton(
            "補正設定をNiclaへ送信"
        )

        adjustment_button_widget = QWidget()
        adjustment_button_row = QHBoxLayout(adjustment_button_widget)
        adjustment_button_row.setContentsMargins(0, 0, 0, 0)
        adjustment_button_row.addWidget(self.load_adjust_button)
        adjustment_button_row.addWidget(self.send_adjust_button)
        adjustment_button_row.addStretch()

        adjustment_note = QLabel(
            "ここで変更した値は、Niclaへ送信しなくてもPC上の"
            "6色化シミュレーションへ反映されます。"
        )
        adjustment_note.setWordWrap(True)

        adjustment_form.addRow("明るさ（V）:", self.brightness_spin)
        adjustment_form.addRow("彩度（S）:", self.saturation_spin)
        adjustment_form.addRow(adjustment_button_widget)
        adjustment_form.addRow(adjustment_note)

        layout.addWidget(adjustment_box)

        checklist_box = QGroupBox(
            "READ_ROIで使用する判定対象色"
        )
        checklist_layout = QHBoxLayout(checklist_box)

        self.color_checkboxes: dict[str, QCheckBox] = {}

        for name in self.HSV_NAMES:
            checkbox = QCheckBox(name)
            checkbox.setChecked(True)
            self.color_checkboxes[name] = checkbox
            checklist_layout.addWidget(checkbox)

        checklist_layout.addStretch()

        checklist_note = QLabel(
            "ここで選択した色は自動生成コードへ埋め込まれます。"
            "通常READのGroup判定には影響しません。"
        )
        checklist_note.setWordWrap(True)

        layout.addWidget(checklist_box)
        layout.addWidget(checklist_note)

        self.hsv_table = QTableWidget(len(self.HSV_NAMES), 7)
        self.hsv_table.setHorizontalHeaderLabels(
            ["色", "H min", "H max", "S min", "S max", "V min", "V max"]
        )
        self.hsv_table.horizontalHeader().setSectionResizeMode(
            QHeaderView.ResizeMode.Stretch
        )

        defaults = {
            "BLACK": [0, 179, 0, 255, 0, 70],
            "WHITE": [0, 179, 0, 70, 140, 255],
            "YELLOW": [18, 42, 70, 255, 50, 255],
            "RED": [170, 10, 100, 255, 50, 255],
            "BLUE": [96, 128, 110, 255, 65, 255],
            "GREEN": [43, 89, 50, 255, 50, 255],
        }

        for row, name in enumerate(self.HSV_NAMES):
            name_item = QTableWidgetItem(name)
            name_item.setFlags(
                name_item.flags() & ~Qt.ItemFlag.ItemIsEditable
            )
            self.hsv_table.setItem(row, 0, name_item)

            for column, value in enumerate(defaults[name], start=1):
                self.hsv_table.setItem(
                    row,
                    column,
                    QTableWidgetItem(str(value)),
                )

        layout.addWidget(self.hsv_table)
        self.tabs.addTab(tab, "HSV設定")

        self.load_hsv_button.clicked.connect(self.load_hsv_config)
        self.send_hsv_button.clicked.connect(self.send_hsv_config)
        self.load_adjust_button.clicked.connect(self.load_color_adjust)
        self.send_adjust_button.clicked.connect(self.send_color_adjust)

    def _build_log_tab(self) -> None:
        tab = QWidget()
        layout = QVBoxLayout(tab)

        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)

        clear_button = QPushButton("ログ消去")
        clear_button.clicked.connect(self.log_text.clear)

        layout.addWidget(self.log_text)
        layout.addWidget(clear_button)

        self.tabs.addTab(tab, "通信ログ")

    def log(self, text: str) -> None:
        timestamp = time.strftime("%H:%M:%S")
        self.log_text.append(f"[{timestamp}] {text}")

    def refresh_ports(self) -> None:
        current = self.port_combo.currentData()
        self.port_combo.clear()

        ports = list(list_ports.comports())

        for port in ports:
            description = port.description or "不明なデバイス"
            self.port_combo.addItem(
                f"{port.device} — {description}",
                port.device,
            )

        if current:
            index = self.port_combo.findData(current)
            if index >= 0:
                self.port_combo.setCurrentIndex(index)

        if not ports:
            self.port_combo.addItem("利用可能なCOMポートなし", "")

    def connect_selected_port(self) -> None:
        port = self.port_combo.currentData()

        if not port:
            QMessageBox.warning(
                self,
                APP_NAME,
                "接続するCOMポートを選択してください。",
            )
            return

        self.run_operation(
            lambda log: self.client.connect(port, log),
            self.on_connected,
        )

    def on_connected(self, result: object) -> None:
        port = str(result)
        self.connection_label.setText(f"接続中: {port}")
        self._set_connected_ui(True)
        self.statusBar().showMessage(f"Niclaへ接続しました: {port}")

    def disconnect_port(self) -> None:
        if not self.client.connected:
            return

        self.run_operation(
            lambda log: self.client.disconnect(log),
            lambda result: self.on_disconnected(),
        )

    def on_disconnected(self) -> None:
        self.connection_label.setText("未接続")
        self._set_connected_ui(False)
        self.statusBar().showMessage("Niclaから切断しました。")

    def _set_connected_ui(self, connected: bool) -> None:
        self.connect_button.setEnabled(not connected)
        self.disconnect_button.setEnabled(connected)

        for button in (
            self.last_photo_button,
            self.load_roi_button,
            self.read_test_button,
            self.local_read_button,
            self.load_hsv_button,
            self.send_hsv_button,
        ):
            button.setEnabled(connected)

    def set_busy(self, busy: bool) -> None:
        self.refresh_button.setEnabled(not busy)
        self.port_combo.setEnabled(not busy)

        if busy:
            self.connect_button.setEnabled(False)
            self.disconnect_button.setEnabled(False)
        else:
            self._set_connected_ui(self.client.connected)

    def run_operation(
        self,
        task: Callable[[Callable[[str], None]], object],
        on_result: Optional[Callable[[object], None]] = None,
    ) -> None:
        if self.worker is not None and self.worker.isRunning():
            QMessageBox.information(
                self,
                APP_NAME,
                "別の通信処理を実行中です。",
            )
            return

        self.result_handler = on_result
        self.worker = OperationThread(task)

        self.worker.log.connect(self.log)
        self.worker.result.connect(self.on_worker_result)
        self.worker.error.connect(self.on_worker_error)
        self.worker.finished.connect(self.on_worker_finished)

        self.set_busy(True)
        self.worker.start()

    def on_worker_result(self, result: object) -> None:
        if self.result_handler is not None:
            self.result_handler(result)

    def on_worker_error(self, message: str) -> None:
        self.log(f"ERROR: {message}")

        QMessageBox.critical(
            self,
            APP_NAME,
            message,
        )

    def on_worker_finished(self) -> None:
        self.set_busy(False)
        self.result_handler = None

    def receive_image(self, command: str) -> None:
        self.statusBar().showMessage("画像を受信しています...")

        self.run_operation(
            lambda log: self.client.receive_image(command, log),
            self.on_image_received,
        )

    def on_image_received(self, result: object) -> None:
        if not isinstance(result, np.ndarray):
            return

        self.current_rgb = result
        self.simulated_rgb = None
        self.displayed_rgb = result

        self.canvas.set_rgb_image(result)
        self.display_mode_label.setText("表示: 元画像")
        self.simulation_summary_label.setText(
            "画像を取得しました。HSV設定を確認して、"
            "「6色化して表示」を押してください。"
        )

        self.show_original_button.setEnabled(True)
        self.simulate_six_button.setEnabled(True)
        self.save_six_button.setEnabled(False)

        self.statusBar().showMessage(
            f"画像取得完了: {LATEST_IMAGE_PATH}"
        )

    def load_roi_config(self) -> None:
        self.run_operation(
            lambda log: self.client.get_roi_config(log),
            self.on_roi_config_loaded,
        )

    def on_roi_config_loaded(self, result: object) -> None:
        if not isinstance(result, list):
            return

        self.canvas.set_rois(result)
        self.statusBar().showMessage("NiclaのROI設定を読み込みました。")

    def send_roi_config(self) -> None:
        if len(self.canvas.rois) != ROI_COUNT:
            QMessageBox.warning(
                self,
                APP_NAME,
                f"ROIを12個設定してください。現在は{len(self.canvas.rois)}個です。",
            )
            return

        rois = [dict(roi) for roi in self.canvas.rois]

        self.run_operation(
            lambda log: self.client.save_roi_config(rois, log),
            self.on_roi_config_saved,
        )

    def on_roi_config_saved(self, result: object) -> None:
        self.statusBar().showMessage("ROI送信が完了しました。")

    def generate_initial_config(self) -> None:
        if len(self.canvas.rois) != ROI_COUNT:
            QMessageBox.warning(
                self,
                APP_NAME,
                f"ROIを12個設定してください。現在は{len(self.canvas.rois)}個です。",
            )
            return

        try:
            config_text = self.build_generated_config_text()
        except (ValueError, TypeError) as error:
            QMessageBox.warning(
                self,
                APP_NAME,
                f"初期設定用コードの生成に失敗しました。\n{error}",
            )
            return

        dialog = QDialog(self)
        dialog.setWindowTitle("初期設定用コード")
        dialog.resize(760, 680)

        layout = QVBoxLayout(dialog)

        info_label = QLabel(
            "生成したコードです。そのままコピーしてNicla側のコードへ貼り付けてください。"
        )
        info_label.setWordWrap(True)
        layout.addWidget(info_label)

        code_edit = QTextEdit()
        code_edit.setReadOnly(True)
        code_edit.setPlainText(config_text)
        layout.addWidget(code_edit, 1)

        button_row = QHBoxLayout()
        copy_button = QPushButton("コードをコピー")
        close_button = QPushButton("閉じる")
        button_row.addStretch()
        button_row.addWidget(copy_button)
        button_row.addWidget(close_button)
        layout.addLayout(button_row)

        def copy_code() -> None:
            QApplication.clipboard().setText(config_text)
            self.statusBar().showMessage(
                "初期設定用コードをクリップボードへコピーしました。"
            )
            copy_button.setText("コピーしました")

        copy_button.clicked.connect(copy_code)
        close_button.clicked.connect(dialog.accept)

        code_edit.selectAll()
        dialog.exec()

    def build_generated_config_text(self) -> str:
        if len(self.canvas.rois) != ROI_COUNT:
            raise ValueError("ROIは12個必要です。")

        hsv_rows = self.read_hsv_table_rows()
        hsv_by_name = {
            str(row["name"]).upper(): row
            for row in hsv_rows
        }

        required_names = [
            "BLACK",
            "WHITE",
            "YELLOW",
            "RED",
            "BLUE",
            "GREEN",
        ]

        for name in required_names:
            if name not in hsv_by_name:
                raise ValueError(f"HSV設定に{name}がありません。")

        enabled = {
            name: self.color_checkboxes[name].isChecked()
            for name in self.HSV_NAMES
        }

        lines = [
            "// ===== NiclaRoiApp 自動生成設定 =====",
            "RoiConfig roiConfigs[ROI_COUNT] = {",
        ]

        for index, roi in enumerate(self.canvas.rois, start=1):
            comma = "," if index < ROI_COUNT else ""
            lines.append(
                f"  {{ {int(roi['x'])}, {int(roi['y'])}, "
                f"{int(roi['width'])}, {int(roi['height'])} }}"
                f"{comma}  // ROI{index}"
            )

        lines.append("};")

        variable_names = {
            "BLACK": "hsvBlack",
            "WHITE": "hsvWhite",
            "YELLOW": "hsvYellow",
            "RED": "hsvRed",
            "BLUE": "hsvBlue",
            "GREEN": "hsvGreen",
        }

        for name in required_names:
            row = hsv_by_name[name]
            lines.append(
                f"HsvRange {variable_names[name]} = {{ "
                f"{int(row['h_min'])}, {int(row['h_max'])}, "
                f"{int(row['s_min'])}, {int(row['s_max'])}, "
                f"{int(row['v_min'])}, {int(row['v_max'])} }};"
            )

        lines.extend(
            [
                f"int brightnessPercent = {self.brightness_spin.value()};",
                f"int saturationPercent = {self.saturation_spin.value()};",
                "",
                "// READ_ROI / READ_TEST用 判定対象色",
                f"bool checklistBlackEnabled = "
                f"{'true' if enabled['BLACK'] else 'false'};",
                f"bool checklistWhiteEnabled = "
                f"{'true' if enabled['WHITE'] else 'false'};",
                f"bool checklistYellowEnabled = "
                f"{'true' if enabled['YELLOW'] else 'false'};",
                f"bool checklistRedEnabled = "
                f"{'true' if enabled['RED'] else 'false'};",
                f"bool checklistBlueEnabled = "
                f"{'true' if enabled['BLUE'] else 'false'};",
                f"bool checklistGreenEnabled = "
                f"{'true' if enabled['GREEN'] else 'false'};",
                "// ===== 自動生成設定ここまで =====",
            ]
        )

        return "\n".join(lines)

    def run_local_roi_read_test(self) -> None:
        if len(self.canvas.rois) != ROI_COUNT:
            QMessageBox.warning(
                self,
                APP_NAME,
                f"ROIを12個設定してください。現在は{len(self.canvas.rois)}個です。",
            )
            return

        self.statusBar().showMessage(
            "直近画像を取得して、設定中ROIで色読みしています..."
        )

        self.run_operation(
            lambda log: self.client.receive_image("GET_LAST_PHOTO", log),
            self.on_local_roi_image_received,
        )

    def on_local_roi_image_received(self, result: object) -> None:
        if not isinstance(result, np.ndarray):
            return

        self.current_rgb = result

        try:
            rows = self.read_hsv_table_rows()
            simulated, counts_by_roi = self.classify_current_rois(
                result,
                rows,
            )
        except (ValueError, TypeError) as error:
            QMessageBox.warning(self, APP_NAME, str(error))
            return

        self.simulated_rgb = simulated
        self.displayed_rgb = simulated
        self.canvas.set_rgb_image(simulated)
        self.display_mode_label.setText("表示: 6色化")
        self.show_original_button.setEnabled(True)
        self.simulate_six_button.setEnabled(True)
        self.save_six_button.setEnabled(True)

        read_color_names = [
            name
            for name in self.HSV_NAMES
            if self.color_checkboxes[name].isChecked()
        ]

        if not read_color_names:
            QMessageBox.warning(
                self,
                APP_NAME,
                "判定対象色を1色以上チェックしてください。",
            )
            return

        result_lines: list[str] = []

        for index, counts in enumerate(counts_by_roi, start=1):
            valid_total = sum(
                counts[name]
                for name in read_color_names
            )

            if valid_total == 0:
                winner = "判定なし"
            else:
                winner = max(
                    read_color_names,
                    key=lambda name: counts[name],
                )

            count_text = " / ".join(
                f"{name}={counts[name]}"
                for name in read_color_names
            )

            result_lines.append(
                f"ROI{index}: {winner}  ({count_text})"
            )

        self.result_text.setPlainText("\n".join(result_lines))
        self.statusBar().showMessage(
            "6色化画像を使った設定中ROIの色読みが完了しました。"
        )

    def classify_current_rois(
        self,
        rgb: np.ndarray,
        rows: list[dict[str, int | str]],
    ) -> tuple[np.ndarray, list[dict[str, int]]]:
        simulated, _ = self.simulate_six_color_array(
            rgb,
            rows,
            self.brightness_spin.value(),
            self.saturation_spin.value(),
        )

        palette = {
            "BLACK": np.array([0, 0, 0], dtype=np.uint8),
            "WHITE": np.array([255, 255, 255], dtype=np.uint8),
            "YELLOW": np.array([255, 255, 0], dtype=np.uint8),
            "RED": np.array([255, 0, 0], dtype=np.uint8),
            "BLUE": np.array([0, 0, 255], dtype=np.uint8),
            "GREEN": np.array([0, 255, 0], dtype=np.uint8),
        }

        counts_by_roi: list[dict[str, int]] = []

        for roi in self.canvas.rois:
            roi_width = int(roi["width"])
            roi_height = int(roi["height"])
            center_x = int(roi["x"])
            center_y = int(roi["y"])

            x0 = center_x - roi_width // 2
            y0 = center_y - roi_height // 2
            x0 = max(0, min(IMAGE_WIDTH - roi_width, x0))
            y0 = max(0, min(IMAGE_HEIGHT - roi_height, y0))
            x1 = min(IMAGE_WIDTH, x0 + roi_width)
            y1 = min(IMAGE_HEIGHT, y0 + roi_height)

            roi_pixels = simulated[y0:y1, x0:x1]
            counts: dict[str, int] = {}

            for name in self.HSV_NAMES:
                color = palette[name]
                counts[name] = int(
                    np.count_nonzero(
                        np.all(roi_pixels == color, axis=2)
                    )
                )

            counts_by_roi.append(counts)

        return simulated, counts_by_roi

    def run_read_test(self) -> None:
        self.run_operation(
            lambda log: self.client.run_read_test(log),
            self.on_read_test_result,
        )

    def on_read_test_result(self, result: object) -> None:
        if not isinstance(result, list):
            return

        self.result_text.setPlainText("\n".join(str(line) for line in result))
        self.statusBar().showMessage("色読みテストが完了しました。")

    def read_hsv_table_rows(self) -> list[dict[str, int | str]]:
        rows: list[dict[str, int | str]] = []

        keys = [
            "h_min",
            "h_max",
            "s_min",
            "s_max",
            "v_min",
            "v_max",
        ]

        for row, name in enumerate(self.HSV_NAMES):
            config: dict[str, int | str] = {"name": name}

            for column, key in enumerate(keys, start=1):
                item = self.hsv_table.item(row, column)

                if item is None or not item.text().strip():
                    raise ValueError(
                        f"{name}の{key}が空です。"
                    )

                value = int(item.text().strip())
                maximum = 179 if key.startswith("h_") else 255

                if not 0 <= value <= maximum:
                    raise ValueError(
                        f"{name}の{key}は0～{maximum}で入力してください。"
                    )

                config[key] = value

            rows.append(config)

        return rows

    @staticmethod
    def _hsv_mask(
        hue: np.ndarray,
        saturation: np.ndarray,
        value: np.ndarray,
        config: dict[str, int | str],
    ) -> np.ndarray:
        h_min = int(config["h_min"])
        h_max = int(config["h_max"])

        if h_min <= h_max:
            hue_valid = (hue >= h_min) & (hue <= h_max)
        else:
            hue_valid = (hue >= h_min) | (hue <= h_max)

        return (
            hue_valid
            & (saturation >= int(config["s_min"]))
            & (saturation <= int(config["s_max"]))
            & (value >= int(config["v_min"]))
            & (value <= int(config["v_max"]))
        )

    @staticmethod
    def apply_percent_correction(
        values: np.ndarray,
        percent: int,
    ) -> np.ndarray:
        percent = max(0, min(400, int(percent)))

        corrected = (
            values.astype(np.uint32) * percent + 50
        ) // 100

        return np.clip(
            corrected,
            0,
            255,
        ).astype(np.uint8)

    @staticmethod
    def simulate_six_color_array(
        rgb: np.ndarray,
        rows: list[dict[str, int | str]],
        brightness_percent: int,
        saturation_percent: int,
    ) -> tuple[np.ndarray, dict[str, int]]:
        source = rgb.astype(np.float32) / 255.0

        red = source[:, :, 0]
        green = source[:, :, 1]
        blue = source[:, :, 2]

        maximum = np.maximum(red, np.maximum(green, blue))
        minimum = np.minimum(red, np.minimum(green, blue))
        difference = maximum - minimum

        hue_degrees = np.zeros_like(maximum, dtype=np.float32)
        nonzero = difference > 0.0

        red_is_max = nonzero & (maximum == red)
        green_is_max = nonzero & (maximum == green)
        blue_is_max = nonzero & (maximum == blue)

        hue_degrees[red_is_max] = (
            60.0
            * np.mod(
                (green[red_is_max] - blue[red_is_max])
                / difference[red_is_max],
                6.0,
            )
        )

        hue_degrees[green_is_max] = (
            60.0
            * (
                (
                    blue[green_is_max] - red[green_is_max]
                )
                / difference[green_is_max]
                + 2.0
            )
        )

        hue_degrees[blue_is_max] = (
            60.0
            * (
                (
                    red[blue_is_max] - green[blue_is_max]
                )
                / difference[blue_is_max]
                + 4.0
            )
        )

        hue_degrees = np.mod(hue_degrees, 360.0)

        saturation_normalized = np.zeros_like(
            maximum,
            dtype=np.float32,
        )

        positive_value = maximum > 0.0
        saturation_normalized[positive_value] = (
            difference[positive_value]
            / maximum[positive_value]
        )

        hue = np.clip(
            np.floor(hue_degrees / 2.0 + 0.5),
            0,
            179,
        ).astype(np.uint8)

        saturation = np.clip(
            np.floor(saturation_normalized * 255.0 + 0.5),
            0,
            255,
        ).astype(np.uint8)

        value = np.clip(
            np.floor(maximum * 255.0 + 0.5),
            0,
            255,
        ).astype(np.uint8)

        saturation = MainWindow.apply_percent_correction(
            saturation,
            saturation_percent,
        )

        value = MainWindow.apply_percent_correction(
            value,
            brightness_percent,
        )

        config_by_name = {
            str(row["name"]).upper(): row
            for row in rows
        }

        palette = {
            "BLACK": np.array([0, 0, 0], dtype=np.uint8),
            "WHITE": np.array([255, 255, 255], dtype=np.uint8),
            "YELLOW": np.array([255, 255, 0], dtype=np.uint8),
            "RED": np.array([255, 0, 0], dtype=np.uint8),
            "BLUE": np.array([0, 0, 255], dtype=np.uint8),
            "GREEN": np.array([0, 255, 0], dtype=np.uint8),
        }

        class_ids = {
            "BLACK": 0,
            "WHITE": 1,
            "YELLOW": 2,
            "RED": 3,
            "BLUE": 4,
            "GREEN": 5,
        }

        simulated = np.empty_like(rgb, dtype=np.uint8)
        simulated[:, :] = palette["BLACK"]

        labels = np.full(
            hue.shape,
            class_ids["BLACK"],
            dtype=np.uint8,
        )

        priority_order = [
            "BLACK",
            "WHITE",
            "YELLOW",
            "RED",
            "BLUE",
            "GREEN",
        ]

        for name in priority_order:
            config = config_by_name.get(name)

            if config is None:
                raise ValueError(
                    f"HSV設定に{name}がありません。"
                )

            mask = MainWindow._hsv_mask(
                hue,
                saturation,
                value,
                config,
            )

            simulated[mask] = palette[name]
            labels[mask] = class_ids[name]

        counts = {
            name: int(np.count_nonzero(labels == class_id))
            for name, class_id in class_ids.items()
        }

        return simulated, counts

    def show_original_image(self) -> None:
        if self.current_rgb is None:
            QMessageBox.information(
                self,
                APP_NAME,
                "元画像がありません。",
            )
            return

        self.displayed_rgb = self.current_rgb
        self.canvas.set_rgb_image(self.current_rgb)
        self.display_mode_label.setText("表示: 元画像")
        self.statusBar().showMessage("元画像を表示しました。")

    def simulate_six_colors(self) -> None:
        if self.current_rgb is None:
            QMessageBox.information(
                self,
                APP_NAME,
                "先に画像を取得してください。",
            )
            return

        try:
            rows = self.read_hsv_table_rows()
            brightness_percent = self.brightness_spin.value()
            saturation_percent = self.saturation_spin.value()

            simulated, counts = self.simulate_six_color_array(
                self.current_rgb,
                rows,
                brightness_percent,
                saturation_percent,
            )
        except (ValueError, TypeError) as error:
            QMessageBox.warning(
                self,
                APP_NAME,
                str(error),
            )
            return

        self.simulated_rgb = simulated
        self.displayed_rgb = simulated
        self.canvas.set_rgb_image(simulated)
        self.display_mode_label.setText("表示: 6色化")
        self.save_six_button.setEnabled(True)

        total = int(simulated.shape[0] * simulated.shape[1])

        summary = (
            f"補正: 明るさ={brightness_percent}% / "
            f"彩度={saturation_percent}% | "
            f"全{total}画素: "
            f"黒={counts['BLACK']} / "
            f"白={counts['WHITE']} / "
            f"黄={counts['YELLOW']} / "
            f"赤={counts['RED']} / "
            f"青={counts['BLUE']} / "
            f"緑={counts['GREEN']}"
        )

        self.simulation_summary_label.setText(summary)
        self.log(f"6色化シミュレーション: {summary}")
        self.statusBar().showMessage(
            "6色化シミュレーションを表示しました。"
        )

    def save_six_color_image(self) -> None:
        if self.simulated_rgb is None:
            QMessageBox.information(
                self,
                APP_NAME,
                "先に「6色化して表示」を実行してください。",
            )
            return

        file_name, _ = QFileDialog.getSaveFileName(
            self,
            "6色化画像を保存",
            str(DATA_DIR / "nicla_5colors.png"),
            "PNG image (*.png)",
        )

        if not file_name:
            return

        Image.fromarray(
            self.simulated_rgb,
            mode="RGB",
        ).save(file_name)

        self.statusBar().showMessage(
            f"6色化画像を保存しました: {file_name}"
        )

    def save_current_image(self) -> None:
        image_to_save = (
            self.displayed_rgb
            if self.displayed_rgb is not None
            else self.current_rgb
        )

        if image_to_save is None:
            QMessageBox.information(
                self,
                APP_NAME,
                "保存する画像がありません。",
            )
            return

        file_name, _ = QFileDialog.getSaveFileName(
            self,
            "画像を保存",
            str(DATA_DIR / "nicla_image.png"),
            "PNG image (*.png);;JPEG image (*.jpg *.jpeg)",
        )

        if not file_name:
            return

        Image.fromarray(image_to_save, mode="RGB").save(file_name)
        self.statusBar().showMessage(f"表示中の画像を保存しました: {file_name}")

    def on_canvas_selection_changed(self, index: int) -> None:
        if not (0 <= index < len(self.canvas.rois)):
            self.selected_label.setText("なし")
            return

        roi = self.canvas.rois[index]

        self.selected_label.setText(f"ROI{index + 1}")
        self.width_spin.setValue(roi["width"])
        self.height_spin.setValue(roi["height"])

        self.roi_table.selectRow(index)

    def refresh_roi_table(self) -> None:
        for row in range(ROI_COUNT):
            if row < len(self.canvas.rois):
                roi = self.canvas.rois[row]
                values = [
                    f"ROI{row + 1}",
                    str(roi["x"]),
                    str(roi["y"]),
                    str(roi["width"]),
                    str(roi["height"]),
                ]
            else:
                values = [
                    f"ROI{row + 1}",
                    "",
                    "",
                    "",
                    "",
                ]

            for column, value in enumerate(values):
                self.roi_table.setItem(
                    row,
                    column,
                    QTableWidgetItem(value),
                )

    def load_hsv_config(self) -> None:
        self.run_operation(
            lambda log: self.client.get_hsv_config(log),
            self.on_hsv_config_loaded,
        )

    def on_hsv_config_loaded(self, result: object) -> None:
        if not isinstance(result, list):
            return

        row_map = {
            name: index
            for index, name in enumerate(self.HSV_NAMES)
        }

        keys = [
            "h_min",
            "h_max",
            "s_min",
            "s_max",
            "v_min",
            "v_max",
        ]

        for config in result:
            name = str(config["name"]).upper()

            if name not in row_map:
                continue

            row = row_map[name]

            for column, key in enumerate(keys, start=1):
                self.hsv_table.setItem(
                    row,
                    column,
                    QTableWidgetItem(str(config[key])),
                )

        self.statusBar().showMessage("NiclaのHSV設定を読み込みました。")

    def send_hsv_config(self) -> None:
        try:
            rows = self.read_hsv_table_rows()

        except (ValueError, TypeError) as error:
            QMessageBox.warning(
                self,
                APP_NAME,
                str(error),
            )
            return

        self.run_operation(
            lambda log: self.client.save_hsv_config(rows, log),
            lambda result: self.statusBar().showMessage(
                "HSV設定をNiclaへ送信しました。"
            ),
        )

    def load_color_adjust(self) -> None:
        self.run_operation(
            lambda log: self.client.get_color_adjust(log),
            self.on_color_adjust_loaded,
        )

    def on_color_adjust_loaded(self, result: object) -> None:
        if not isinstance(result, tuple) or len(result) != 2:
            return

        brightness_percent = int(result[0])
        saturation_percent = int(result[1])

        self.brightness_spin.setValue(brightness_percent)
        self.saturation_spin.setValue(saturation_percent)

        self.statusBar().showMessage(
            "Niclaの補正設定を読み込みました。"
        )

    def send_color_adjust(self) -> None:
        brightness_percent = self.brightness_spin.value()
        saturation_percent = self.saturation_spin.value()

        self.run_operation(
            lambda log: self.client.save_color_adjust(
                brightness_percent,
                saturation_percent,
                log,
            ),
            lambda result: self.statusBar().showMessage(
                "補正設定をNiclaへ送信しました。"
            ),
        )

    def closeEvent(self, event) -> None:
        try:
            self.client.disconnect_silent()
        finally:
            event.accept()


def main() -> int:
    app = QApplication(sys.argv)
    app.setApplicationName(APP_NAME)

    window = MainWindow()
    window.show()

    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())