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
    QComboBox,
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
    QScrollArea,
    QSplitter,
    QStatusBar,
    QTabWidget,
    QTableWidget,
    QTableWidgetItem,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)


APP_NAME = "Nicla ROI App - BLACK/GREEN/YELLOW Learning Test"
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
        self.resize(640, 480)
        self.setMouseTracking(True)

        self.image: Optional[QImage] = None
        self.rois: list[dict[str, int]] = []
        self.selected_index = -1

        self.dragging = False
        self.drag_offset_x = 0
        self.drag_offset_y = 0

    @staticmethod
    def default_size_for_index(index: int) -> tuple[int, int]:
        if index < 4:
            return 10, 10
        if index < 8:
            return 16, 16
        return 24, 24

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

        # 320x240画像を2倍の640x480で固定表示する。
        # 表示領域より大きい場合は親のQScrollAreaでスクロールする。
        display_width = max(640, width * 2)
        display_height = max(480, height * 2)
        self.setFixedSize(display_width, display_height)
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

        # キャンバス全体へ縦横比を維持して表示する。
        # キャンバス自体は固定サイズなので、画像が見切れる場合は
        # QScrollArea側にスクロールバーが表示される。
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


class LearningCanvas(QWidget):
    """色学習専用の最小テスト用キャンバス。既存ROI編集には触れない。"""

    selection_changed = Signal()

    def __init__(self) -> None:
        super().__init__()
        self.setMinimumSize(640, 480)
        self.setFixedSize(640, 480)
        self.image: Optional[QImage] = None
        self.drag_start: Optional[QPointF] = None
        self.drag_end: Optional[QPointF] = None
        self.dragging = False

    def set_rgb_image(self, rgb: np.ndarray) -> None:
        height, width, channels = rgb.shape
        if channels != 3:
            raise ValueError("RGB画像ではありません。")
        self.image = QImage(
            rgb.data,
            width,
            height,
            width * 3,
            QImage.Format.Format_RGB888,
        ).copy()
        self.setFixedSize(max(640, width * 2), max(480, height * 2))
        self.clear_selection()
        self.update()

    def _image_target_rect(self) -> QRectF:
        if self.image is None:
            return QRectF()
        image_size = self.image.size()
        scale = min(
            self.width() / image_size.width(),
            self.height() / image_size.height(),
        )
        draw_width = image_size.width() * scale
        draw_height = image_size.height() * scale
        left = (self.width() - draw_width) / 2.0
        top = (self.height() - draw_height) / 2.0
        return QRectF(left, top, draw_width, draw_height)

    def _widget_to_image(self, point: QPointF) -> Optional[QPointF]:
        target = self._image_target_rect()
        if target.isNull() or not target.contains(point):
            return None
        x = (point.x() - target.left()) * IMAGE_WIDTH / target.width()
        y = (point.y() - target.top()) * IMAGE_HEIGHT / target.height()
        return QPointF(
            max(0, min(IMAGE_WIDTH - 1, x)),
            max(0, min(IMAGE_HEIGHT - 1, y)),
        )

    def clear_selection(self) -> None:
        self.drag_start = None
        self.drag_end = None
        self.dragging = False
        self.selection_changed.emit()
        self.update()

    def selection_bounds(self) -> Optional[tuple[int, int, int, int]]:
        if self.drag_start is None or self.drag_end is None:
            return None
        x1 = int(round(min(self.drag_start.x(), self.drag_end.x())))
        y1 = int(round(min(self.drag_start.y(), self.drag_end.y())))
        x2 = int(round(max(self.drag_start.x(), self.drag_end.x())))
        y2 = int(round(max(self.drag_start.y(), self.drag_end.y())))
        x1 = max(0, min(IMAGE_WIDTH - 1, x1))
        y1 = max(0, min(IMAGE_HEIGHT - 1, y1))
        x2 = max(0, min(IMAGE_WIDTH, x2 + 1))
        y2 = max(0, min(IMAGE_HEIGHT, y2 + 1))
        if x2 <= x1 or y2 <= y1:
            return None
        return x1, y1, x2 - x1, y2 - y1

    def paintEvent(self, event) -> None:
        painter = QPainter(self)
        painter.fillRect(self.rect(), QColor("#202020"))
        target = self._image_target_rect()
        if self.image is not None and not target.isNull():
            painter.drawImage(target, self.image)
        if self.drag_start is not None and self.drag_end is not None and not target.isNull():
            x_scale = target.width() / IMAGE_WIDTH
            y_scale = target.height() / IMAGE_HEIGHT
            x1 = min(self.drag_start.x(), self.drag_end.x()) * x_scale + target.left()
            y1 = min(self.drag_start.y(), self.drag_end.y()) * y_scale + target.top()
            x2 = max(self.drag_start.x(), self.drag_end.x()) * x_scale + target.left()
            y2 = max(self.drag_start.y(), self.drag_end.y()) * y_scale + target.top()
            pen = QPen(QColor("#ff3030"))
            pen.setWidth(3)
            painter.setPen(pen)
            painter.setBrush(Qt.BrushStyle.NoBrush)
            painter.drawRect(QRectF(x1, y1, max(1.0, x2-x1), max(1.0, y2-y1)))
        if self.image is None:
            painter.setPen(QColor("white"))
            painter.drawText(
                self.rect(),
                Qt.AlignmentFlag.AlignCenter,
                "画像・ROIタブで画像を取得してください",
            )

    def mousePressEvent(self, event) -> None:
        if event.button() != Qt.MouseButton.LeftButton:
            return
        point = self._widget_to_image(event.position())
        if point is None:
            return
        self.drag_start = point
        self.drag_end = point
        self.dragging = True
        self.update()

    def mouseMoveEvent(self, event) -> None:
        if not self.dragging:
            return
        point = self._widget_to_image(event.position())
        if point is None:
            return
        self.drag_end = point
        self.update()

    def mouseReleaseEvent(self, event) -> None:
        if event.button() != Qt.MouseButton.LeftButton:
            return
        if self.dragging:
            point = self._widget_to_image(event.position())
            if point is not None:
                self.drag_end = point
        self.dragging = False
        self.selection_changed.emit()
        self.update()


class MainWindow(QMainWindow):
    HSV_NAMES = [
        "BLACK",
        "WHITE",
        "YELLOW",
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

        # 色学習テスト用。既存ROI/HSV処理とは独立して保持する。
        self.learning_samples: dict[str, list[tuple[int, int, int, int]]] = {
            name: [] for name in self.HSV_NAMES
        }

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
        self._build_learning_tab()
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

        self.capture_button = QPushButton("新しく撮影して表示")
        self.last_photo_button = QPushButton("直近の保存画像を表示")
        self.load_roi_button = QPushButton("NiclaのROI設定を取得")
        self.send_roi_button = QPushButton("12個のROIをNiclaへ送信")
        self.read_test_button = QPushButton("色読みテスト")
        self.save_image_button = QPushButton("画像をPNG保存")

        action_row.addWidget(self.capture_button)
        action_row.addWidget(self.last_photo_button)
        action_row.addWidget(self.load_roi_button)
        action_row.addWidget(self.send_roi_button)
        action_row.addWidget(self.read_test_button)
        action_row.addWidget(self.save_image_button)

        layout.addLayout(action_row)

        simulation_box = QGroupBox(
            "5色化シミュレーション（HSV設定タブの現在値を使用）"
        )
        simulation_layout = QVBoxLayout(simulation_box)
        simulation_button_row = QHBoxLayout()

        self.show_original_button = QPushButton("元画像を表示")
        self.simulate_six_button = QPushButton("5色化して表示")
        self.save_six_button = QPushButton("5色化画像をPNG保存")
        self.display_mode_label = QLabel("表示: 元画像")

        simulation_button_row.addWidget(self.show_original_button)
        simulation_button_row.addWidget(self.simulate_six_button)
        simulation_button_row.addWidget(self.save_six_button)
        simulation_button_row.addWidget(self.display_mode_label)
        simulation_button_row.addStretch()

        self.simulation_summary_label = QLabel(
            "画像を取得後、「5色化して表示」を押してください。"
        )
        self.simulation_summary_label.setWordWrap(True)

        legend_label = QLabel(
            "表示色: 黒=(0,0,0) / 白=(255,255,255) / "
            "黄=(255,255,0) / 青=(0,0,255) / "
            "緑=(0,255,0)"
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

        self.image_scroll_area = QScrollArea()
        self.image_scroll_area.setWidget(self.canvas)
        self.image_scroll_area.setWidgetResizable(False)
        self.image_scroll_area.setAlignment(
            Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignTop
        )
        self.image_scroll_area.setHorizontalScrollBarPolicy(
            Qt.ScrollBarPolicy.ScrollBarAsNeeded
        )
        self.image_scroll_area.setVerticalScrollBarPolicy(
            Qt.ScrollBarPolicy.ScrollBarAsNeeded
        )
        self.image_scroll_area.setMinimumSize(420, 320)
        splitter.addWidget(self.image_scroll_area)

        side = QWidget()
        side_layout = QVBoxLayout(side)

        selection_box = QGroupBox("選択ROI")
        selection_form = QFormLayout(selection_box)

        self.selected_label = QLabel("なし")

        self.width_spin = QSpinBox()
        self.width_spin.setRange(1, IMAGE_WIDTH)
        self.width_spin.setValue(10)

        self.height_spin = QSpinBox()
        self.height_spin.setRange(1, IMAGE_HEIGHT)
        self.height_spin.setValue(10)

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

        self.capture_button.clicked.connect(
            lambda: self.receive_image("CAPTURE_AND_GET")
        )

        self.last_photo_button.clicked.connect(
            lambda: self.receive_image("GET_LAST_PHOTO")
        )

        self.load_roi_button.clicked.connect(self.load_roi_config)
        self.send_roi_button.clicked.connect(self.send_roi_config)
        self.read_test_button.clicked.connect(self.run_read_test)
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
            "5色判定前の補正（100%で補正なし）"
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
            "5色化シミュレーションへ反映されます。"
        )
        adjustment_note.setWordWrap(True)

        adjustment_form.addRow(
            "明るさ（V）:",
            self.brightness_spin,
        )
        adjustment_form.addRow(
            "彩度（S）:",
            self.saturation_spin,
        )
        adjustment_form.addRow(adjustment_button_widget)
        adjustment_form.addRow(adjustment_note)

        layout.addWidget(adjustment_box)

        self.hsv_table = QTableWidget(len(self.HSV_NAMES), 7)
        self.hsv_table.setHorizontalHeaderLabels(
            ["色", "H min", "H max", "S min", "S max", "V min", "V max"]
        )

        self.hsv_table.horizontalHeader().setSectionResizeMode(
            QHeaderView.ResizeMode.Stretch
        )

        defaults = {
            "BLACK":  [0, 179, 0, 255, 0, 70],
            "WHITE":  [0, 179, 0, 70, 140, 255],
            "YELLOW": [18, 42, 70, 255, 50, 255],
            "BLUE":   [96, 128, 110, 255, 90, 255],
            "GREEN":  [43, 89, 50, 255, 50, 255],
        }

        for row, name in enumerate(self.HSV_NAMES):
            name_item = QTableWidgetItem(name)
            name_item.setFlags(name_item.flags() & ~Qt.ItemFlag.ItemIsEditable)
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
        self.load_adjust_button.clicked.connect(
            self.load_color_adjust
        )
        self.send_adjust_button.clicked.connect(
            self.send_color_adjust
        )

    def _build_learning_tab(self) -> None:
        tab = QWidget()
        layout = QVBoxLayout(tab)

        note = QLabel(
            "最小テスト版: 画像上をドラッグして範囲を選び、色ラベルを付けます。"
            "登録した範囲の画素からHSVを抽出し、HSV設定表へ反映できます。"
        )
        note.setWordWrap(True)
        layout.addWidget(note)

        controls = QHBoxLayout()
        self.learning_color_combo = QComboBox()
        self.learning_color_combo.addItems(self.HSV_NAMES)
        self.add_learning_sample_button = QPushButton("選択範囲をサンプル追加")
        self.clear_learning_color_button = QPushButton("選択色のサンプル全削除")
        self.generate_learning_hsv_button = QPushButton("選択色のHSVを生成→表へ反映")
        self.adjust_green_yellow_button = QPushButton("GREEN/YELLOW境界を自動調整")
        self.generate_code_button = QPushButton("Nicla設定コード生成")
        self.copy_code_button = QPushButton("生成コードをコピー")

        controls.addWidget(QLabel("ラベル:"))
        controls.addWidget(self.learning_color_combo)
        controls.addWidget(self.add_learning_sample_button)
        controls.addWidget(self.clear_learning_color_button)
        controls.addWidget(self.generate_learning_hsv_button)
        controls.addWidget(self.adjust_green_yellow_button)
        controls.addWidget(self.generate_code_button)
        controls.addWidget(self.copy_code_button)
        controls.addStretch()
        layout.addLayout(controls)

        splitter = QSplitter(Qt.Orientation.Horizontal)
        self.learning_canvas = LearningCanvas()
        learning_scroll = QScrollArea()
        learning_scroll.setWidget(self.learning_canvas)
        learning_scroll.setWidgetResizable(False)
        learning_scroll.setAlignment(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignTop)
        splitter.addWidget(learning_scroll)

        right = QWidget()
        right_layout = QVBoxLayout(right)
        self.learning_summary = QTextEdit()
        self.learning_summary.setReadOnly(True)
        self.learning_summary.setPlaceholderText(
            "画像上をドラッグ → ラベルを選択 → 「選択範囲をサンプル追加」"
        )
        self.generated_code_text = QTextEdit()
        self.generated_code_text.setReadOnly(True)
        self.generated_code_text.setPlaceholderText(
            "「Nicla設定コード生成」を押すと、.inoへ貼り付ける設定コードを表示します。"
        )
        right_layout.addWidget(QLabel("学習サンプル / 抽出結果"))
        right_layout.addWidget(self.learning_summary, 1)
        right_layout.addWidget(QLabel("Nicla貼り付け用コード"))
        right_layout.addWidget(self.generated_code_text, 1)
        splitter.addWidget(right)
        splitter.setSizes([760, 500])
        layout.addWidget(splitter, 1)
        self.tabs.addTab(tab, "色学習テスト")

        self.add_learning_sample_button.clicked.connect(self.add_learning_sample)
        self.clear_learning_color_button.clicked.connect(self.clear_learning_samples_for_color)
        self.generate_learning_hsv_button.clicked.connect(self.generate_hsv_from_learning_samples)
        self.adjust_green_yellow_button.clicked.connect(self.adjust_green_yellow_boundary)
        self.generate_code_button.clicked.connect(self.generate_nicla_setting_code)
        self.copy_code_button.clicked.connect(self.copy_generated_code)
        self.learning_color_combo.currentTextChanged.connect(
            lambda _: self.refresh_learning_summary()
        )

    def refresh_learning_summary(self, extra: str = "") -> None:
        lines: list[str] = []
        for name in self.HSV_NAMES:
            samples = self.learning_samples.get(name, [])
            lines.append(f"{name}: {len(samples)}個")
            for index, (x, y, w, h) in enumerate(samples, start=1):
                lines.append(f"  {index}: x={x}, y={y}, w={w}, h={h}")
        if extra:
            lines.extend(["", extra])
        self.learning_summary.setPlainText("\n".join(lines))

    def add_learning_sample(self) -> None:
        if self.current_rgb is None:
            QMessageBox.information(self, APP_NAME, "先に画像を取得してください。")
            return
        bounds = self.learning_canvas.selection_bounds()
        if bounds is None:
            QMessageBox.information(self, APP_NAME, "画像上をドラッグして範囲を選択してください。")
            return
        name = self.learning_color_combo.currentText().upper()
        self.learning_samples[name].append(bounds)
        self.refresh_learning_summary(f"{name}へ選択範囲を追加しました。")

    def clear_learning_samples_for_color(self) -> None:
        name = self.learning_color_combo.currentText().upper()
        self.learning_samples[name].clear()
        self.refresh_learning_summary(f"{name}のサンプルを削除しました。")

    @staticmethod
    def rgb_to_adjusted_hsv_arrays(
        rgb: np.ndarray,
        brightness_percent: int,
        saturation_percent: int,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """既存5色シミュレーションと同じ計算でH/S/Vを作る。"""
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
        hue_degrees[red_is_max] = 60.0 * np.mod(
            (green[red_is_max] - blue[red_is_max]) / difference[red_is_max], 6.0
        )
        hue_degrees[green_is_max] = 60.0 * (
            (blue[green_is_max] - red[green_is_max]) / difference[green_is_max] + 2.0
        )
        hue_degrees[blue_is_max] = 60.0 * (
            (red[blue_is_max] - green[blue_is_max]) / difference[blue_is_max] + 4.0
        )
        hue_degrees = np.mod(hue_degrees, 360.0)

        saturation_normalized = np.zeros_like(maximum, dtype=np.float32)
        positive_value = maximum > 0.0
        saturation_normalized[positive_value] = (
            difference[positive_value] / maximum[positive_value]
        )

        hue = np.clip(np.floor(hue_degrees / 2.0 + 0.5), 0, 179).astype(np.uint8)
        saturation = np.clip(
            np.floor(saturation_normalized * 255.0 + 0.5), 0, 255
        ).astype(np.uint8)
        value = np.clip(np.floor(maximum * 255.0 + 0.5), 0, 255).astype(np.uint8)

        saturation = MainWindow.apply_percent_correction(
            saturation, saturation_percent
        )
        value = MainWindow.apply_percent_correction(value, brightness_percent)
        return hue, saturation, value

    def generate_hsv_from_learning_samples(self) -> None:
        if self.current_rgb is None:
            QMessageBox.information(self, APP_NAME, "先に画像を取得してください。")
            return

        name = self.learning_color_combo.currentText().upper()
        samples = self.learning_samples.get(name, [])
        if not samples:
            QMessageBox.information(self, APP_NAME, f"{name}のサンプルがありません。")
            return

        hue, saturation, value = self.rgb_to_adjusted_hsv_arrays(
            self.current_rgb,
            self.brightness_spin.value(),
            self.saturation_spin.value(),
        )

        h_values: list[np.ndarray] = []
        s_values: list[np.ndarray] = []
        v_values: list[np.ndarray] = []

        for x, y, w, h in samples:
            h_values.append(hue[y:y+h, x:x+w].reshape(-1))
            s_values.append(saturation[y:y+h, x:x+w].reshape(-1))
            v_values.append(value[y:y+h, x:x+w].reshape(-1))

        hs = np.concatenate(h_values).astype(np.int16)
        ss = np.concatenate(s_values).astype(np.int16)
        vs = np.concatenate(v_values).astype(np.int16)

        if name in ("BLACK", "WHITE"):
            # 黒・白は暗部でHが不安定になるため、Hは判定に使わない。
            h_min, h_max = 0, 179
            s_min = int(np.floor(np.percentile(ss, 2.5)))
            s_max = int(np.ceil(np.percentile(ss, 97.5)))
            v_min = int(np.floor(np.percentile(vs, 2.5)))
            v_max = int(np.ceil(np.percentile(vs, 97.5)))
            extraction_note = "Hは全域、S/Vは2.5～97.5 percentile"

        else:
            # 色付き画素では、暗い黒のノイズをそのまま学習しない。
            # 低彩度画素を除外し、Hの最頻値（主要な色相）の周辺だけを採用する。
            # これにより、GREENを囲ったときにH=0～150のように広がるのを防ぐ。
            chromatic_mask = ss >= 60

            if np.count_nonzero(chromatic_mask) < max(5, int(len(ss) * 0.10)):
                QMessageBox.warning(
                    self,
                    APP_NAME,
                    f"{name}として使える彩度の高い画素が少なすぎます。\n"
                    "色部分だけをもう少し狭く囲ってください。",
                )
                return

            color_h = hs[chromatic_mask]
            color_s = ss[chromatic_mask]
            color_v = vs[chromatic_mask]

            histogram = np.bincount(color_h, minlength=180).astype(np.int64)

            # 近接するHをまとめて主要な山を探す。
            smooth = np.zeros(180, dtype=np.int64)
            for offset in range(-3, 4):
                smooth += np.roll(histogram, offset)

            dominant_h = int(np.argmax(smooth))

            # 最頻Hの±10以内だけを「その色」として採用する。
            # GREENとYELLOWの境界が重なりやすいため、以前の±18より狭くする。
            delta = np.abs(color_h - dominant_h)
            circular_delta = np.minimum(delta, 180 - delta)
            main_mask = circular_delta <= 10

            selected_h = color_h[main_mask]
            selected_s = color_s[main_mask]
            selected_v = color_v[main_mask]

            if selected_h.size < 5:
                QMessageBox.warning(
                    self,
                    APP_NAME,
                    f"{name}の主要な色相を十分に抽出できませんでした。\n"
                    "色部分だけを囲い直してください。",
                )
                return

            h_min = max(0, int(np.floor(np.percentile(selected_h, 2.5))) - 3)
            h_max = min(179, int(np.ceil(np.percentile(selected_h, 97.5))) + 3)

            # GREEN/YELLOW/BLUEはS=0付近を許可すると黒を拾いやすいため、
            # 学習結果でも最低60を維持する。
            s_min = max(
                60,
                int(np.floor(np.percentile(selected_s, 5.0))) - 10,
            )
            s_max = min(
                255,
                int(np.ceil(np.percentile(selected_s, 97.5))) + 10,
            )

            # 暗い緑を残すためV下限は厳しくしすぎない。
            v_min = max(
                0,
                int(np.floor(np.percentile(selected_v, 2.5))) - 5,
            )
            v_max = min(
                255,
                int(np.ceil(np.percentile(selected_v, 97.5))) + 5,
            )

            extraction_note = (
                f"主要H={dominant_h} / S>=60の画素から主要色相±10を抽出"
            )

        row = self.HSV_NAMES.index(name)
        values = [h_min, h_max, s_min, s_max, v_min, v_max]

        for column, number in enumerate(values, start=1):
            self.hsv_table.setItem(
                row,
                column,
                QTableWidgetItem(str(number)),
            )

        self.refresh_learning_summary(
            f"{name} 抽出結果（補正後HSV）\n"
            f"H={h_min}～{h_max}, S={s_min}～{s_max}, V={v_min}～{v_max}\n"
            f"{extraction_note}\n"
            "HSV設定タブへ反映しました。"
        )
        self.statusBar().showMessage(
            f"{name}のHSVを画像サンプルから生成しました。"
        )


    def adjust_green_yellow_boundary(self) -> None:
        """
        GREENとYELLOWの登録済み学習サンプルを比較し、
        主にH（色相）の境界を自動調整する。

        目的:
        - 暗い緑はGREENとして残す
        - YELLOWとGREENのH範囲が重ならないようにする
        - S/Vは各色で生成済みの値をできるだけ維持する
        """
        if self.current_rgb is None:
            QMessageBox.information(
                self,
                APP_NAME,
                "先に画像を取得してください。",
            )
            return

        green_samples = self.learning_samples.get("GREEN", [])
        yellow_samples = self.learning_samples.get("YELLOW", [])

        if not green_samples or not yellow_samples:
            QMessageBox.information(
                self,
                APP_NAME,
                "GREENとYELLOWの両方に学習サンプルを登録してください。",
            )
            return

        hue, saturation, value = self.rgb_to_adjusted_hsv_arrays(
            self.current_rgb,
            self.brightness_spin.value(),
            self.saturation_spin.value(),
        )

        def collect(samples):
            hs_parts = []
            ss_parts = []
            vs_parts = []
            for x, y, w, h in samples:
                hs_parts.append(hue[y:y+h, x:x+w].reshape(-1))
                ss_parts.append(saturation[y:y+h, x:x+w].reshape(-1))
                vs_parts.append(value[y:y+h, x:x+w].reshape(-1))
            hs = np.concatenate(hs_parts).astype(np.int16)
            ss = np.concatenate(ss_parts).astype(np.int16)
            vs = np.concatenate(vs_parts).astype(np.int16)

            # 色付き画素だけを優先。黒や灰色の混入を抑える。
            mask = ss >= 60
            if np.count_nonzero(mask) < 5:
                return None

            return hs[mask], ss[mask], vs[mask]

        green_data = collect(green_samples)
        yellow_data = collect(yellow_samples)

        if green_data is None or yellow_data is None:
            QMessageBox.warning(
                self,
                APP_NAME,
                "GREENまたはYELLOWで彩度の高い画素を十分に抽出できませんでした。\n"
                "色部分だけをもう少し狭く囲ってください。",
            )
            return

        green_h, green_s, green_v = green_data
        yellow_h, yellow_s, yellow_v = yellow_data

        # 主要Hを取得
        def dominant_hue(hs):
            hist = np.bincount(hs, minlength=180).astype(np.int64)
            smooth = np.zeros(180, dtype=np.int64)
            for offset in range(-3, 4):
                smooth += np.roll(hist, offset)
            return int(np.argmax(smooth))

        g_dom = dominant_hue(green_h)
        y_dom = dominant_hue(yellow_h)

        # 今回の用途では通常 YELLOW < GREEN のH関係を想定。
        # 逆転している場合は自動調整せず、学習範囲の見直しを促す。
        if not (y_dom < g_dom):
            QMessageBox.warning(
                self,
                APP_NAME,
                f"GREEN/YELLOWの主要Hの順序が想定と異なります。\n"
                f"YELLOW主要H={y_dom}, GREEN主要H={g_dom}\n"
                "選択範囲に別の色が混ざっていないか確認してください。",
            )
            return

        # 各色の主要H周辺±10のみを使って分布端を求める
        def main_cluster(hs, dom):
            delta = np.abs(hs - dom)
            circular_delta = np.minimum(delta, 180 - delta)
            return hs[circular_delta <= 10]

        g_main = main_cluster(green_h, g_dom)
        y_main = main_cluster(yellow_h, y_dom)

        if g_main.size < 5 or y_main.size < 5:
            QMessageBox.warning(
                self,
                APP_NAME,
                "GREEN/YELLOWの主要H帯を十分に抽出できませんでした。",
            )
            return

        y_high = int(np.ceil(np.percentile(y_main, 97.5)))
        g_low = int(np.floor(np.percentile(g_main, 2.5)))

        # 分布の間に隙間があればその中央、重なる場合でも主要値の中央で切る
        if y_high < g_low:
            boundary = (y_high + g_low) // 2
        else:
            boundary = (y_dom + g_dom) // 2

        # 1つのH値が両方に入らないよう、YELLOW max=boundary、
        # GREEN min=boundary+1 とする。
        yellow_h_min = max(
            0,
            int(np.floor(np.percentile(y_main, 2.5))) - 2,
        )
        yellow_h_max = max(yellow_h_min, min(178, boundary))

        green_h_min = min(179, max(1, boundary + 1))
        green_h_max = min(
            179,
            int(np.ceil(np.percentile(g_main, 97.5))) + 2,
        )
        green_h_max = max(green_h_min, green_h_max)

        yellow_row = self.HSV_NAMES.index("YELLOW")
        green_row = self.HSV_NAMES.index("GREEN")

        # Hだけ自動調整。S/Vは個別学習で生成した現在値を維持する。
        self.hsv_table.setItem(
            yellow_row, 1, QTableWidgetItem(str(yellow_h_min))
        )
        self.hsv_table.setItem(
            yellow_row, 2, QTableWidgetItem(str(yellow_h_max))
        )
        self.hsv_table.setItem(
            green_row, 1, QTableWidgetItem(str(green_h_min))
        )
        self.hsv_table.setItem(
            green_row, 2, QTableWidgetItem(str(green_h_max))
        )

        # S下限が低すぎると黒を拾いやすいので、最低60を保証
        for row in (yellow_row, green_row):
            item = self.hsv_table.item(row, 3)
            if item is not None and item.text().strip():
                current_s_min = int(item.text().strip())
                if current_s_min < 60:
                    self.hsv_table.setItem(
                        row, 3, QTableWidgetItem("60")
                    )

        self.refresh_learning_summary(
            "GREEN/YELLOW境界を自動調整しました。\n"
            f"YELLOW主要H={y_dom}, GREEN主要H={g_dom}\n"
            f"境界H={boundary}\n"
            f"YELLOW H={yellow_h_min}～{yellow_h_max}\n"
            f"GREEN H={green_h_min}～{green_h_max}\n"
            "S/Vは各色の現在値を維持しています。"
        )
        self.statusBar().showMessage(
            "GREENとYELLOWのH境界を自動調整しました。"
        )

    def generate_nicla_setting_code(self) -> None:
        try:
            rows = self.read_hsv_table_rows()
        except (ValueError, TypeError) as error:
            QMessageBox.warning(self, APP_NAME, str(error))
            return

        if len(self.canvas.rois) != ROI_COUNT:
            QMessageBox.warning(
                self,
                APP_NAME,
                f"ROIを12個設定またはNiclaから取得してください。現在は{len(self.canvas.rois)}個です。",
            )
            return

        lines: list[str] = []
        lines.append("// ===== NiclaRoiApp 自動生成設定 =====")
        lines.append("")
        lines.append("RoiConfig roiConfigs[ROI_COUNT] = {")
        for index, roi in enumerate(self.canvas.rois):
            comma = "," if index < ROI_COUNT - 1 else ""
            lines.append(
                f"  {{ {roi['x']}, {roi['y']}, {roi['width']}, {roi['height']} }}{comma}  // ROI{index + 1}"
            )
        lines.append("};")
        lines.append("")
        name_to_var = {
            "BLACK": "hsvBlack",
            "WHITE": "hsvWhite",
            "YELLOW": "hsvYellow",
            "BLUE": "hsvBlue",
            "GREEN": "hsvGreen",
        }
        for row in rows:
            name = str(row["name"]).upper()
            lines.append(
                f"HsvRange {name_to_var[name]} = {{ "
                f"{row['h_min']}, {row['h_max']}, {row['s_min']}, {row['s_max']}, "
                f"{row['v_min']}, {row['v_max']} }};"
            )
        lines.append("")
        lines.append(f"int brightnessPercent = {self.brightness_spin.value()};")
        lines.append(f"int saturationPercent = {self.saturation_spin.value()};")
        lines.append("")
        lines.append("// ===== 自動生成設定ここまで =====")
        self.generated_code_text.setPlainText("\n".join(lines))
        self.statusBar().showMessage("Nicla貼り付け用の設定コードを生成しました。")

    def copy_generated_code(self) -> None:
        text = self.generated_code_text.toPlainText()
        if not text.strip():
            QMessageBox.information(self, APP_NAME, "先に設定コードを生成してください。")
            return
        QApplication.clipboard().setText(text)
        self.statusBar().showMessage("生成コードをクリップボードへコピーしました。")

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
            self.capture_button,
            self.last_photo_button,
            self.load_roi_button,
            self.send_roi_button,
            self.read_test_button,
            self.load_hsv_button,
            self.send_hsv_button,
            self.load_adjust_button,
            self.send_adjust_button,
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
        self.learning_canvas.set_rgb_image(result)
        self.display_mode_label.setText("表示: 元画像")
        self.simulation_summary_label.setText(
            "画像を取得しました。HSV設定を確認して、"
            "「5色化して表示」を押してください。"
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
            lambda result: self.statusBar().showMessage(
                "12個のROI設定をNiclaへ送信しました。"
            ),
        )

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

        # 最大値は255 * 400 + 50 = 102050なので、
        # uint16ではなくuint32を使用してオーバーフローを防ぐ。
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

        # Nicla側C++のroundf()に合わせ、正数はfloor(x+0.5)
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

        # Nicla側と同じ整数式で補正する。
        # corrected = (original * percent + 50) // 100
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
            "BLUE": np.array([0, 0, 255], dtype=np.uint8),
            "GREEN": np.array([0, 255, 0], dtype=np.uint8),
        }

        class_ids = {
            "BLACK": 0,
            "WHITE": 1,
            "YELLOW": 2,
            "BLUE": 3,
            "GREEN": 4,
        }

        # Nicla側と同じく、未一致画素はBLACKとして扱う
        simulated = np.empty_like(rgb, dtype=np.uint8)
        simulated[:, :] = palette["BLACK"]

        labels = np.full(
            hue.shape,
            class_ids["BLACK"],
            dtype=np.uint8,
        )

        # Nicla側と同じ上書き順
        priority_order = [
            "BLACK",
            "WHITE",
            "YELLOW",
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
        self.display_mode_label.setText("表示: 5色化")
        self.save_six_button.setEnabled(True)

        total = int(simulated.shape[0] * simulated.shape[1])

        summary = (
            f"補正: 明るさ={brightness_percent}% / "
            f"彩度={saturation_percent}% | "
            f"全{total}画素: "
            f"黒={counts['BLACK']} / "
            f"白={counts['WHITE']} / "
            f"黄={counts['YELLOW']} / "
            f"青={counts['BLUE']} / "
            f"緑={counts['GREEN']}"
        )

        self.simulation_summary_label.setText(summary)
        self.log(f"5色化シミュレーション: {summary}")
        self.statusBar().showMessage(
            "5色化シミュレーションを表示しました。"
        )

    def save_six_color_image(self) -> None:
        if self.simulated_rgb is None:
            QMessageBox.information(
                self,
                APP_NAME,
                "先に「5色化して表示」を実行してください。",
            )
            return

        file_name, _ = QFileDialog.getSaveFileName(
            self,
            "5色化画像を保存",
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
            f"5色化画像を保存しました: {file_name}"
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
        if (
            not isinstance(result, tuple)
            or len(result) != 2
        ):
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