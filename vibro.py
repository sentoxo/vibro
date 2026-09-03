# AI Generated code for real-time vibration monitoring and ESC control.

import os
import sys
import re
import struct
import threading
import time
from collections import deque
import numpy as np
import psutil
import serial
from serial.tools import list_ports

from PyQt6.QtCore import QThread, pyqtSignal, QTimer, Qt
from PyQt6.QtWidgets import (
    QApplication, QComboBox, QDialog, QDialogButtonBox, QDoubleSpinBox,
    QGridLayout, QGroupBox, QHBoxLayout, QLabel, QLineEdit, QMainWindow,
    QPushButton, QSpinBox, QVBoxLayout, QWidget
)
import pyqtgraph as pg

# === CONFIGURATION ===
BAUD_RATE = 921600
FC_BAUD_RATE = 115200
FS = 800.0                       
WINDOW_SECONDS = 2.0             
BUFFER_SIZE = int(FS * WINDOW_SECONDS)
LSB_TO_MS2 = 0.0039 * 9.80665    
FFT_Y_MAX = 7.0
TIME_Y_MIN_RANGE = 2.0
DEFAULT_FFT_REFRESH_HZ = 5.0
FFT_UPDATE_INTERVAL = 1.0 / DEFAULT_FFT_REFRESH_HZ
MAX_FFT_POINTS = 512

SEQ_LINE = re.compile(r"^S(\d),(\d+),(-?\d+),(-?\d+),(-?\d+)\s*$")
PREFERRED_PORT_DESCRIPTION = "USB_SERIAL CH340"


def find_preferred_port():
    preferred = PREFERRED_PORT_DESCRIPTION.replace("_", " ").lower()
    for port in list_ports.comports():
        description = (port.description or "").replace("-", " ").replace("_", " ").lower()
        if preferred in description:
            return port.device
    return None


class PortDialog(QDialog):
    def __init__(self, title="Select ESP32 serial port", message=None, excluded_port=None):
        super().__init__()
        self.excluded_port = excluded_port
        self.setWindowTitle(title)
        self.setMinimumWidth(440)

        layout = QVBoxLayout(self)
        layout.addWidget(QLabel(message or "Select a connected COM port:"))

        self.port_combo = QComboBox()
        layout.addWidget(self.port_combo)

        buttons = QDialogButtonBox(
            QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Cancel
        )
        buttons.accepted.connect(self.accept_selected_port)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)

        self.refresh_timer = QTimer(self)
        self.refresh_timer.timeout.connect(self.refresh_ports)
        self.refresh_timer.start(1000)
        self.refresh_ports()

    def refresh_ports(self):
        selected_port = self.port_combo.currentData()
        ports = sorted(list_ports.comports(), key=lambda port: port.device)

        self.port_combo.blockSignals(True)
        self.port_combo.clear()
        for port in ports:
            if port.device == self.excluded_port:
                continue
            description = port.description or "Unknown device"
            self.port_combo.addItem(f"{port.device} - {description}", port.device)

        if selected_port:
            selected_index = self.port_combo.findData(selected_port)
            if selected_index >= 0:
                self.port_combo.setCurrentIndex(selected_index)
        self.port_combo.blockSignals(False)

    def accept_selected_port(self):
        if self.port_combo.currentData():
            self.accept()

    def selected_port(self):
        return self.port_combo.currentData()

class SerialWorker(QThread):
    data_received = pyqtSignal(int, int, float, float, float)
    calibration_finished = pyqtSignal()

    def __init__(self, port, baudrate):
        super().__init__()
        self.port = port
        self.baudrate = baudrate
        self.running = True
        self.ser = None
        self.calibration_requested = threading.Event()

    def run(self):
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
            self.ser.write(b"call all\n")
            time.sleep(3)
            self.ser.write(b"start stream\n")

            while self.running:
                if self.calibration_requested.is_set():
                    self.ser.write(b"stop stream\n")
                    time.sleep(1.5)
                    self.ser.write(b"call all\n")
                    time.sleep(2.5)
                    self.ser.write(b"start stream\n")
                    self.calibration_requested.clear()
                    self.calibration_finished.emit()

                line = self.ser.readline().decode("utf-8", errors="replace").strip()
                if not line:
                    continue
                match = SEQ_LINE.match(line)
                if match:
                    sid, seq, raw_x, raw_y, raw_z = map(int, match.groups())
                    self.data_received.emit(
                        sid, seq,
                        raw_x * LSB_TO_MS2,
                        raw_y * LSB_TO_MS2,
                        raw_z * LSB_TO_MS2,
                    )

            if self.ser and self.ser.is_open:
                try:
                    self.ser.write(b"stop stream\n")
                except Exception:
                    pass
                self.ser.close()
        except Exception as e:
            print(f"Serial error: {e}")

    def stop(self):
        self.running = False
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except Exception:
                pass
        self.wait(2000)

    def request_calibration(self):
        self.calibration_requested.set()


class FCTelemetryReader(QThread):
    rpm_updated = pyqtSignal(float)
    MSP_MOTOR_TELEMETRY = 139
    MOTOR_RECORD_SIZE = 13

    def __init__(self, serial_port):
        super().__init__()
        self.ser = serial_port
        self.running = True
        self.buffer = bytearray()

    @classmethod
    def _build_request(cls):
        command = cls.MSP_MOTOR_TELEMETRY
        return b"$M<\x00" + bytes((command, command))

    @classmethod
    def _parse_packet(cls, buffer):
        while len(buffer) >= 6:
            start = buffer.find(b"$M>")
            if start < 0:
                del buffer[:-2]
                return None
            if start:
                del buffer[:start]
            payload_size = buffer[3]
            packet_size = payload_size + 6
            if len(buffer) < packet_size:
                return None

            packet = bytes(buffer[:packet_size])
            del buffer[:packet_size]
            checksum = 0
            for byte in packet[3:-1]:
                checksum ^= byte
            if checksum == packet[-1] and packet[4] == cls.MSP_MOTOR_TELEMETRY:
                return packet[5:-1]
        return None

    @classmethod
    def _parse_motor_3_rpm(cls, payload):
        if not payload:
            return None
        motor_count = payload[0]
        motor_offset = 1 + 2 * cls.MOTOR_RECORD_SIZE
        if motor_count < 3 or len(payload) < motor_offset + 4:
            return None
        return struct.unpack_from("<I", payload, 1 + 2 * cls.MOTOR_RECORD_SIZE)[0]

    def run(self):
        next_request = 0.0
        while self.running and self.ser and self.ser.is_open:
            try:
                now = time.monotonic()
                if now >= next_request:
                    self.ser.write(self._build_request())
                    next_request = now + 0.1

                if self.ser.in_waiting > 0:
                    self.buffer.extend(self.ser.read(self.ser.in_waiting))
                    payload = self._parse_packet(self.buffer)
                    while payload is not None:
                        rpm_value = self._parse_motor_3_rpm(payload)
                        if rpm_value is not None:
                            self.rpm_updated.emit(float(rpm_value))
                        payload = self._parse_packet(self.buffer)
                else:
                    self.msleep(10)
            except Exception:
                self.msleep(50)

    def stop(self):
        self.running = False
        self.wait(500)


class FFTWorker(QThread):
    result_ready = pyqtSignal(int, object, object)

    def __init__(self):
        super().__init__()
        self.queue = deque()
        self.lock = threading.Lock()

    def submit(self, axis_index, signal_arr):
        with self.lock:
            self.queue.append((axis_index, signal_arr))

    def run(self):
        while not self.isInterruptionRequested():
            item = None
            with self.lock:
                if self.queue:
                    item = self.queue.popleft()

            if item is None:
                self.msleep(10)
                continue

            axis_index, signal_arr = item
            freqs, amps = self.compute_fft(signal_arr)
            self.result_ready.emit(axis_index, freqs, amps)

    @staticmethod
    def compute_fft(signal_arr):
        arr = np.asarray(signal_arr, dtype=np.float64)
        n = len(arr)
        if n < 64:
            return np.array([]), np.array([])

        n = min(n, MAX_FFT_POINTS)
        if len(arr) > n:
            arr = arr[-n:]

        sig = arr - np.mean(arr)
        window = np.hanning(n)
        scale = np.sum(window) / n

        yf = np.fft.rfft(sig * window)
        freqs = np.fft.rfftfreq(n, 1.0 / FS)
        amp = (np.abs(yf) / n) / scale
        if n > 1:
            amp[1:-1] *= 2.0

        return freqs, amp


class RealtimeVibeApp(QMainWindow):
    def __init__(self, serial_port):
        super().__init__()
        self.setWindowTitle("VibroApp")
        self.resize(1600, 900)

        self.time_window_seconds = 2.0
        self.buf_x = deque(maxlen=int(FS * self.time_window_seconds))
        self.buf_y = deque(maxlen=int(FS * self.time_window_seconds))
        self.buf_z = deque(maxlen=int(FS * self.time_window_seconds))
        self.time_axes = ['X', 'Y', 'Z']
        self.esp_port = serial_port
        self.fc_serial = None
        self.fc_port = None
        self.esc_running = False
        self.process = psutil.Process()
        self.cpu_count = max(psutil.cpu_count() or 1, 1)
        self.cpu_history = deque(maxlen=10)
        self.process.cpu_percent(None)
        self.fc_diag_last_message = "FC not initialized"
        self.fc_rpm = 0.0
        self.fc_telemetry_thread = None
        self.fft_worker = FFTWorker()
        self.fft_worker.result_ready.connect(self.handle_fft_result)
        self.fft_worker.start()
        self.fft_cache = [None, None, None]
        self.last_fft_update = 0.0
        self.fft_refresh_hz = DEFAULT_FFT_REFRESH_HZ
        self.fft_update_interval = FFT_UPDATE_INTERVAL

        self.init_ui()

        self.serial_thread = SerialWorker(serial_port, BAUD_RATE)
        self.serial_thread.data_received.connect(self.handle_sample)
        self.serial_thread.calibration_finished.connect(self.calibration_finished)
        self.serial_thread.start()

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_plots)
        self.set_fft_refresh_rate(self.fft_refresh_hz)
        self.set_time_zoom(False)
        self.timer.start(int(1000.0 / self.fft_refresh_hz))

    def init_ui(self):
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QVBoxLayout(main_widget)

        statistics_box = QGroupBox("Diagnostics")
        statistics_box.setStyleSheet(
            "QGroupBox { margin-top: 0px; padding-top: 8px; border: 1px solid #444; }"
        )
        statistics_layout = QGridLayout(statistics_box)
        statistics_layout.setContentsMargins(8, 12, 8, 8)
        statistics_layout.setVerticalSpacing(4)
        self.average_stat_label = QLabel("Average vibration: 0.0 m/s²")
        statistics_layout.addWidget(self.average_stat_label, 0, 0, 1, 3)
        statistics_layout.addWidget(QLabel("Dominant FFT Peaks"), 1, 0, 1, 3)
        self.peak_stat_labels = []
        for peak_number in range(3):
            peak_label = QLabel(self.format_peak_statistics(peak_number, [[], [], []]))
            self.peak_stat_labels.append(peak_label)
            statistics_layout.addWidget(peak_label, peak_number + 2, 0, 1, 3)
        self.rpm_label = QLabel("Motor RPM: --")
        self.rpm_label.setStyleSheet("QLabel { font-weight: 600; }")
        statistics_layout.addWidget(self.rpm_label, 5, 0, 1, 3)

        self.rpm_hz_label = QLabel("Motor Hz: --")
        self.rpm_hz_label.setStyleSheet("QLabel { font-weight: 600; }")
        statistics_layout.addWidget(self.rpm_hz_label, 6, 0, 1, 3)

        self.cpu_stat_label = QLabel("CPU usage: 0%")
        statistics_layout.addWidget(self.cpu_stat_label, 7, 0, 1, 3)
        statistics_layout.addWidget(QLabel(""), 8, 0, 1, 3)

        settings_box = QGroupBox("Settings")
        settings_box.setStyleSheet(
            "QGroupBox { margin-top: 0px; padding-top: 8px; border: 1px solid #444; }"
        )
        settings_layout = QVBoxLayout(settings_box)
        settings_layout.setContentsMargins(8, 12, 8, 8)
        settings_layout.setSpacing(6)

        self.btn_calibration = QPushButton("Calibration")
        self.btn_calibration.clicked.connect(self.calibrate)
        self.btn_calibration.setFixedWidth(200)
        settings_layout.addWidget(self.btn_calibration)

        self.btn_reconnect_esp = QPushButton("Reconnect to esp")
        self.btn_reconnect_esp.clicked.connect(self.reconnect_esp)
        self.btn_reconnect_esp.setFixedWidth(200)
        settings_layout.addWidget(self.btn_reconnect_esp)

        self.btn_zoom_timescale = QPushButton("Zoom")
        self.btn_zoom_timescale.setCheckable(True)
        self.btn_zoom_timescale.setFixedWidth(200)
        self.btn_zoom_timescale.clicked.connect(self.toggle_time_zoom)
        settings_layout.addWidget(self.btn_zoom_timescale)

        refresh_label = QLabel("FFT refresh speed:")
        settings_layout.addWidget(refresh_label)

        self.fft_refresh_input = QDoubleSpinBox()
        self.fft_refresh_input.setRange(1.0, 30.0)
        self.fft_refresh_input.setDecimals(1)
        self.fft_refresh_input.setSingleStep(0.5)
        self.fft_refresh_input.setSuffix(" Hz")
        self.fft_refresh_input.setValue(self.fft_refresh_hz)
        self.fft_refresh_input.valueChanged.connect(self.set_fft_refresh_rate)
        settings_layout.addWidget(self.fft_refresh_input)

        settings_layout.addStretch(1)
        settings_box.setFixedHeight(200)
        settings_box.setFixedWidth(220)

        action_box = QGroupBox("File")
        action_box.setStyleSheet(
            "QGroupBox { margin-top: 0px; padding-top: 8px; border: 1px solid #444; }"
        )
        action_layout = QVBoxLayout(action_box)
        action_layout.setContentsMargins(8, 12, 8, 8)
        action_layout.setSpacing(6)

        self.filename_input = QLineEdit()
        self.filename_input.setPlaceholderText("PNG file name")
        self.filename_input.setFixedWidth(220)
        action_layout.addWidget(self.filename_input)

        self.filename_index_input = QSpinBox()
        self.filename_index_input.setRange(1, 99)
        self.filename_index_input.setValue(1)
        self.filename_index_input.setFixedWidth(220)
        action_layout.addWidget(self.filename_index_input)

        self.btn_save = QPushButton("Save PNG Snapshot")
        self.btn_save.clicked.connect(self.save_png)
        self.btn_save.setFixedWidth(220)
        action_layout.addWidget(self.btn_save)

        self.save_status_label = QLabel("")
        self.save_status_label.setWordWrap(True)
        self.save_status_label.setStyleSheet("QLabel { color: #2d6b2d; font-size: 12px; }")
        self.save_status_label.setFixedWidth(220)
        action_layout.addWidget(self.save_status_label)

        action_layout.addStretch(1)
        action_box.setFixedHeight(200)
        action_box.setFixedWidth(260)

        esc_box = QGroupBox("ESC control")
        esc_box.setStyleSheet(
            "QGroupBox { margin-top: 0px; padding-top: 8px; border: 1px solid #444; }"
        )
        esc_layout = QVBoxLayout(esc_box)
        esc_layout.setContentsMargins(8, 12, 8, 8)
        esc_layout.setSpacing(4)
        self.fc_status_label = QLabel("FC: disconnected")
        esc_layout.addWidget(self.fc_status_label)
        self.fc_diag_label = QLabel("FC diag: not initialized")
        esc_layout.addWidget(self.fc_diag_label)
        self.btn_esc_start = QPushButton("START")
        self.btn_esc_start.setFixedWidth(170)
        self.btn_esc_start.clicked.connect(self.start_esc)
        esc_layout.addWidget(self.btn_esc_start, alignment=Qt.AlignmentFlag.AlignLeft)
        self.btn_esc_stop = QPushButton("STOP")
        self.btn_esc_stop.setFixedWidth(170)
        self.btn_esc_stop.clicked.connect(self.stop_esc)
        esc_layout.addWidget(self.btn_esc_stop, alignment=Qt.AlignmentFlag.AlignLeft)

        power_label = QLabel("Power level:")
        power_label.setFixedHeight(18)
        power_label.setContentsMargins(0, 0, 0, 0)
        power_label.setStyleSheet("QLabel { margin: 0px; padding: 0px; }")
        esc_layout.addWidget(power_label, alignment=Qt.AlignmentFlag.AlignLeft)

        power_row_layout = QHBoxLayout()
        power_row_layout.setContentsMargins(0, 0, 0, 0)
        power_row_layout.setSpacing(6)

        self.esc_power_input = QDoubleSpinBox()
        self.esc_power_input.setRange(0.0, 100.0)
        self.esc_power_input.setDecimals(1)
        self.esc_power_input.setSingleStep(0.1)
        self.esc_power_input.setSuffix(" %")
        self.esc_power_input.setValue(0.0)
        self.esc_power_input.setFixedWidth(95)
        self.esc_power_input.setFixedHeight(24)
        self.esc_power_input.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.esc_power_input.valueChanged.connect(self.update_esc_power)
        power_row_layout.addWidget(self.esc_power_input, alignment=Qt.AlignmentFlag.AlignLeft)

        self.btn_power_minus_01 = QPushButton("-1%")
        self.btn_power_minus_01.setFixedWidth(50)
        self.btn_power_minus_01.setFixedHeight(24)
        self.btn_power_minus_01.clicked.connect(lambda: self.adjust_esc_power(-1))
        power_row_layout.addWidget(self.btn_power_minus_01, alignment=Qt.AlignmentFlag.AlignLeft)

        self.btn_power_plus_01 = QPushButton("+1%")
        self.btn_power_plus_01.setFixedWidth(50)
        self.btn_power_plus_01.setFixedHeight(24)
        self.btn_power_plus_01.clicked.connect(lambda: self.adjust_esc_power(1))
        power_row_layout.addWidget(self.btn_power_plus_01, alignment=Qt.AlignmentFlag.AlignLeft)

        self.btn_power_minus = QPushButton("-10%")
        self.btn_power_minus.setFixedWidth(50)
        self.btn_power_minus.setFixedHeight(24)
        self.btn_power_minus.clicked.connect(lambda: self.adjust_esc_power(-10))
        power_row_layout.addWidget(self.btn_power_minus, alignment=Qt.AlignmentFlag.AlignLeft)

        self.btn_power_plus = QPushButton("+10%")
        self.btn_power_plus.setFixedWidth(50)
        self.btn_power_plus.setFixedHeight(24)
        self.btn_power_plus.clicked.connect(lambda: self.adjust_esc_power(10))
        power_row_layout.addWidget(self.btn_power_plus, alignment=Qt.AlignmentFlag.AlignLeft)

        power_row_layout.addStretch(1)
        esc_layout.addLayout(power_row_layout)

        self.btn_reconnect_fc = QPushButton("Reconnect to FC")
        self.btn_reconnect_fc.setFixedWidth(170)
        self.btn_reconnect_fc.clicked.connect(self.reconnect_fc)
        esc_layout.addWidget(self.btn_reconnect_fc, alignment=Qt.AlignmentFlag.AlignLeft)
        statistics_box.setFixedHeight(200)
        settings_box.setFixedHeight(200)
        action_box.setFixedHeight(200)
        esc_box.setFixedHeight(200)
        esc_box.setFixedWidth(360)

        statistics_and_esc_layout = QHBoxLayout()
        statistics_and_esc_layout.addWidget(statistics_box, 1)
        statistics_and_esc_layout.addWidget(settings_box, 0)
        statistics_and_esc_layout.addWidget(action_box, 0)
        statistics_and_esc_layout.addWidget(esc_box, 1)
        main_layout.addLayout(statistics_and_esc_layout)

        pg.setConfigOptions(antialias=True, background='w', foreground='k')
        self.graphics_layout = pg.GraphicsLayoutWidget()
        main_layout.addWidget(self.graphics_layout)

        # Build 3x2 Grid (Rows: X, Y, Z | Cols: Time, FFT)
        self.curves_time = []
        self.curves_fft = []
        self.fft_rpm_lines = []
        
        colors = ['#1f77b4', '#1f77b4', '#1f77b4'] 
        
        self.plots_time = []
        self.plots_fft = []

        for i, axis in enumerate(self.time_axes):
            # Time Plot
            p_time = self.graphics_layout.addPlot(
                title=f"Czasówki - Oś {axis} - 0.0 m/s² avg"
            )
            p_time.showGrid(x=True, y=True)
            p_time.setLabel('left', '[m/s²]')
            p_time.setLimits(minYRange=TIME_Y_MIN_RANGE)
            p_time.enableAutoRange(axis='y', enable=False)
            if i == 2: p_time.setLabel('bottom', 'Czas [s]')
            c_time = p_time.plot(pen=pg.mkPen(colors[i], width=1))
            self.curves_time.append(c_time)
            self.plots_time.append(p_time)

            # FFT Plot
            p_fft = self.graphics_layout.addPlot(title=f"Widma FFT - Oś {axis}")
            p_fft.showGrid(x=True, y=True)
            p_fft.setLabel('left', '[m/s²]')
            p_fft.setYRange(0, FFT_Y_MAX)
            p_fft.setXRange(0, FS / 2)
            if i == 2: p_fft.setLabel('bottom', 'Częstotliwość [Hz]')
            c_fft = p_fft.plot(pen=pg.mkPen(colors[i], width=1.2))
            rpm_line = pg.InfiniteLine(
                angle=90,
                movable=False,
                pen=pg.mkPen('#d62728', width=2),
            )
            double_rpm_line = pg.InfiniteLine(
                angle=90,
                movable=False,
                pen=pg.mkPen('#e6b800', width=2),
            )
            rpm_line.setVisible(False)
            double_rpm_line.setVisible(False)
            p_fft.addItem(rpm_line)
            p_fft.addItem(double_rpm_line)
            self.fft_rpm_lines.append((rpm_line, double_rpm_line))
            self.curves_fft.append(c_fft)
            self.plots_fft.append(p_fft)

            self.graphics_layout.nextRow()

        # Link X-axes for unified zooming/panning
        self.plots_time[1].setXLink(self.plots_time[0])
        self.plots_time[2].setXLink(self.plots_time[0])
        self.plots_fft[1].setXLink(self.plots_fft[0])
        self.plots_fft[2].setXLink(self.plots_fft[0])

    def handle_sample(self, sid, seq, x, y, z):
        if sid == 0: 
            self.buf_x.append(x)
            self.buf_y.append(y)
            self.buf_z.append(z)

    def toggle_time_zoom(self):
        zoom_enabled = self.btn_zoom_timescale.isChecked()
        self.set_time_zoom(zoom_enabled)

    def set_time_zoom(self, zoom_enabled):
        if zoom_enabled:
            self.time_window_seconds = 0.2
            self.btn_zoom_timescale.setText("Zoom ON")
            self.btn_zoom_timescale.setChecked(True)
            self.btn_zoom_timescale.setStyleSheet("QPushButton { background-color: #d9f7d9; color: #124b12; font-weight: bold; }")
        else:
            self.time_window_seconds = 2.0
            self.btn_zoom_timescale.setText("Zoom")
            self.btn_zoom_timescale.setChecked(False)
            self.btn_zoom_timescale.setStyleSheet("")

        self.buf_x = deque(self.buf_x, maxlen=int(FS * self.time_window_seconds))
        self.buf_y = deque(self.buf_y, maxlen=int(FS * self.time_window_seconds))
        self.buf_z = deque(self.buf_z, maxlen=int(FS * self.time_window_seconds))

        self.update_plots()

    def calibration_finished(self):
        self.btn_calibration.setEnabled(True)

    def handle_fc_rpm_update(self, rpm_value):
        self.fc_rpm = max(0.0, float(rpm_value))
        if self.fc_rpm <= 0:
            self.rpm_label.setText("RPM: --")
            self.rpm_hz_label.setText("Hz: --")
            self.update_fft_rpm_indicators(0.0)
            return

        hz_value = self.fc_rpm / 60.0
        self.rpm_label.setText(f"RPM: {self.fc_rpm:.0f}")
        self.rpm_hz_label.setText(f"Hz: {hz_value:.2f}")
        self.update_fft_rpm_indicators(hz_value)

    def update_fft_rpm_indicators(self, motor_hz):
        frequencies = (float(motor_hz), float(motor_hz) * 2.0)
        for rpm_line, double_rpm_line in self.fft_rpm_lines:
            for line, frequency in zip((rpm_line, double_rpm_line), frequencies):
                line.setPos(frequency)
                line.setVisible(0.0 < frequency <= FS / 2)

    def reconnect_esp(self):
        if hasattr(self, "serial_thread"):
            self.serial_thread.stop()

        self.serial_thread = SerialWorker(self.esp_port, BAUD_RATE)
        self.serial_thread.data_received.connect(self.handle_sample)
        self.serial_thread.calibration_finished.connect(self.calibration_finished)
        self.serial_thread.start()
        self.btn_calibration.setEnabled(True)

    def log_fc_diagnostic(self, message, level="INFO"):
        timestamp = time.strftime("%H:%M:%S")
        text = f"[FC {timestamp}] {level}: {message}"
        self.fc_diag_last_message = text
        print(text, flush=True)
        if hasattr(self, "fc_diag_label"):
            self.fc_diag_label.setText(text)

    def send_motor_values(self, motor_values):
        if self.fc_serial is None:
            self.log_fc_diagnostic("FC serial object is None; no connection was opened.", "ERROR")
            self.fc_status_label.setText("FC: disconnected")
            return False

        if not self.fc_serial.is_open:
            self.log_fc_diagnostic(f"FC serial port is closed; cannot send {motor_values}.", "ERROR")
            self.fc_status_label.setText("FC: disconnected")
            return False

        motor_values = [int(round(value)) for value in motor_values]
        payload = b"".join(struct.pack("<H", value) for value in motor_values)
        packet = b"$M<" + bytes((len(payload), 214)) + payload
        checksum = 0
        for byte in packet[3:]:
            checksum ^= byte
        full_packet = packet + bytes((checksum,))

        self.log_fc_diagnostic(
            f"Sending ESC packet for channels={motor_values}, payload_len={len(payload)}, checksum=0x{checksum:02X}, bytes={full_packet.hex()}",
            "INFO",
        )

        try:
            bytes_written = self.fc_serial.write(full_packet)
            self.fc_serial.flush()
            self.log_fc_diagnostic(f"FC write succeeded: {bytes_written} bytes sent.", "OK")
            return True
        except serial.SerialException as error:
            self.log_fc_diagnostic(f"FC write failed: {error}", "ERROR")
            self.fc_status_label.setText(f"FC error: {error}")
            self.fc_serial.close()
            self.fc_serial = None
            self.fc_port = None
            self.esc_running = False
            return False

    def start_esc(self):
        self._stop_ramp_active = False
        power_value = self.get_esc_power_value()
        throttle = 1000 + power_value * 10
        command = [throttle, throttle, throttle, throttle]
        self.log_fc_diagnostic(f"start_esc() requested power={power_value}%, command={command}", "INFO")
        if self.send_motor_values(command):
            self.esc_running = True
            self.log_fc_diagnostic("ESC start command accepted by FC writer.", "OK")
        else:
            self.esc_running = False
            self.log_fc_diagnostic("ESC start failed; FC is not connected or rejected the write.", "ERROR")

    def _run_esc_stop_ramp(self, power_value):
        if not getattr(self, "_stop_ramp_active", False):
            return

        if power_value <= 0:
            command = [1000, 1000, 1000, 1000]
            self.send_motor_values(command)
            self.set_esc_power_value(0)
            self.esc_running = False
            self._stop_ramp_active = False
            self.log_fc_diagnostic("ESC stop ramp completed; motors are at zero throttle.", "OK")
            return

        throttle = 1000 + power_value * 10
        command = [throttle, throttle, throttle, throttle]
        self.send_motor_values(command)
        self.esc_running = False
        self.set_esc_power_value(power_value)
        self.log_fc_diagnostic(f"ESC stop ramp: {power_value}% for 50 ms.", "INFO")
        QTimer.singleShot(50, lambda: self._run_esc_stop_ramp(power_value - 5))

    def stop_esc(self):
        if self.fc_serial is None or not self.fc_serial.is_open:
            self.log_fc_diagnostic("ESC stop requested, but FC is not connected.", "ERROR")
            return

        self._stop_ramp_active = True
        current_power = max(0, min(100, self.get_esc_power_value()))
        self.log_fc_diagnostic(f"stop_esc() started smooth ramp from {current_power}% down to 0%.", "INFO")
        self._run_esc_stop_ramp(current_power)

    def get_esc_power_value(self):
        return max(0.0, min(100.0, float(self.esc_power_input.value())))

    def set_esc_power_value(self, value):
        new_value = max(0.0, min(100.0, float(value)))
        self.esc_power_input.setValue(new_value)

    def set_fft_refresh_rate(self, hz_value):
        value = max(1.0, float(hz_value))
        self.fft_refresh_hz = value
        self.fft_update_interval = 1.0 / self.fft_refresh_hz
        if hasattr(self, "timer"):
            self.timer.setInterval(int(max(50, 1000.0 / self.fft_refresh_hz)))
        if hasattr(self, "fft_refresh_input") and abs(self.fft_refresh_input.value() - self.fft_refresh_hz) > 1e-9:
            self.fft_refresh_input.blockSignals(True)
            self.fft_refresh_input.setValue(self.fft_refresh_hz)
            self.fft_refresh_input.blockSignals(False)

    def update_esc_power(self):
        if self.esc_running:
            self.start_esc()

    def adjust_esc_power(self, delta):
        new_value = max(0.0, min(100.0, self.get_esc_power_value() + float(delta)))
        self.set_esc_power_value(new_value)
        if self.esc_running:
            self.start_esc()

    def _start_fc_telemetry(self):
        if self.fc_serial is None or not self.fc_serial.is_open:
            return

        if self.fc_telemetry_thread is not None:
            self.fc_telemetry_thread.stop()
            self.fc_telemetry_thread = None

        self.fc_telemetry_thread = FCTelemetryReader(self.fc_serial)
        self.fc_telemetry_thread.rpm_updated.connect(self.handle_fc_rpm_update)
        self.fc_telemetry_thread.start()

    def reconnect_fc(self):
        if self.fc_serial is not None and self.fc_serial.is_open:
            self.stop_esc()
            self.fc_serial.close()
            self.fc_serial = None
            self.fc_port = None

        dialog = PortDialog(
            title="Select Betaflight FC serial port",
            message="Select the COM port used by your Betaflight flight controller:",
            excluded_port=self.esp_port,
        )
        if dialog.exec() != QDialog.DialogCode.Accepted:
            self.log_fc_diagnostic("FC reconnect canceled by user.", "INFO")
            return

        selected_port = dialog.selected_port()
        self.log_fc_diagnostic(
            f"Attempting FC connection on {selected_port} at {FC_BAUD_RATE} baud.",
            "INFO",
        )
        try:
            self.fc_serial = serial.Serial(selected_port, FC_BAUD_RATE, timeout=0.1)
            self.fc_port = selected_port
            self.fc_status_label.setText(f"FC: connected ({selected_port})")
            self._start_fc_telemetry()
            self.log_fc_diagnostic(f"FC connected successfully on {selected_port}.", "OK")
        except serial.SerialException as error:
            self.fc_status_label.setText(f"FC connection failed: {error}")
            self.log_fc_diagnostic(f"FC connection failed on {selected_port}: {error}", "ERROR")

    def disconnect_fc(self):
        self.log_fc_diagnostic("Disconnecting FC port and resetting ESC state.", "INFO")
        if self.fc_telemetry_thread is not None:
            self.fc_telemetry_thread.stop()
            self.fc_telemetry_thread = None
        if self.fc_serial is not None and self.fc_serial.is_open:
            self.send_motor_values([1000, 1000, 1000, 1000])
            self.fc_serial.close()
        self.fc_serial = None
        self.fc_port = None
        self.esc_running = False
        self.fc_status_label.setText("FC: disconnected")
        self.rpm_label.setText("Motor RPM: --")
        self.rpm_hz_label.setText("Motor Hz: --")
        self.update_fft_rpm_indicators(0.0)
        self.log_fc_diagnostic("FC disconnected.", "OK")

    def format_peak_statistics(self, peak_number, peaks):
        axis_values = []
        for axis, axis_peaks in zip(self.time_axes, peaks):
            if peak_number < len(axis_peaks):
                frequency, amplitude = axis_peaks[peak_number]
                axis_values.append(f"{axis}: {frequency:.0f} Hz {amplitude:.1f} m/s²")
            else:
                axis_values.append(f"{axis}: --")
        return f"Peak {peak_number + 1}: " + " | ".join(axis_values)

    def find_fft_peaks(self, frequencies, amplitudes):
        if len(frequencies) < 2:
            return []
        non_dc_indices = np.arange(1, len(frequencies))
        peak_indices = non_dc_indices[
            np.argsort(amplitudes[non_dc_indices])[-3:][::-1]
        ]
        return [
            (frequencies[index], amplitudes[index]) for index in peak_indices
        ]

    def handle_fft_result(self, axis_index, freqs, amps):
        self.fft_cache[axis_index] = (freqs, amps)
        if len(freqs) > 0:
            self.curves_fft[axis_index].setData(freqs, amps)

        fft_peaks = []
        for axis_idx in range(3):
            data = self.fft_cache[axis_idx]
            if data is None:
                fft_peaks.append([])
            else:
                freqs, amps = data
                if len(freqs) > 0:
                    fft_peaks.append(self.find_fft_peaks(freqs, amps))
                else:
                    fft_peaks.append([])

        for peak_number, peak_label in enumerate(self.peak_stat_labels):
            peak_label.setText(self.format_peak_statistics(peak_number, fft_peaks))

    def update_plots(self):
        if len(self.buf_x) < 64:
            return

        data = [np.array(self.buf_x), np.array(self.buf_y), np.array(self.buf_z)]
        t = np.arange(len(data[0])) / FS
        self.average_stat_label.setText(
            f"Average vibration: {np.mean(np.abs(np.concatenate(data))):.1f} m/s²"
        )
        cpu_usage = self.process.cpu_percent() #/ self.cpu_count
        self.cpu_history.append(max(0.0, min(100.0, cpu_usage)))
        average_cpu_usage = np.mean(self.cpu_history)
        self.cpu_stat_label.setText(f"CPU usage: {average_cpu_usage:.0f}%")

        for i in range(3):
            self.curves_time[i].setData(t, data[i])
            time_y_limit = max(
                TIME_Y_MIN_RANGE / 2,
                np.max(np.abs(data[i])) * 1.1,
            )
            self.plots_time[i].setYRange(-time_y_limit, time_y_limit, padding=0)
            self.plots_time[i].setXRange(0, self.time_window_seconds, padding=0)
            average_level = np.mean(np.abs(data[i]))
            self.plots_time[i].setTitle(
                f"Czasówki - Oś {self.time_axes[i]} - {average_level:.1f} m/s² avg"
            )

        now = time.perf_counter()
        if now - self.last_fft_update >= self.fft_update_interval:
            self.last_fft_update = now
            for i in range(3):
                self.fft_worker.submit(i, data[i])

    def calibrate(self):
        self.buf_x.clear()
        self.buf_y.clear()
        self.buf_z.clear()
        self.fft_cache = [None, None, None]
        for curve in self.curves_time + self.curves_fft:
            curve.setData([], [])
        for i, axis in enumerate(self.time_axes):
            self.plots_time[i].setTitle(f"Czasówki - Oś {axis} - 0.0 m/s² avg")
        self.average_stat_label.setText("Average vibration: 0.0 m/s²")
        for peak_number, peak_label in enumerate(self.peak_stat_labels):
            peak_label.setText(self.format_peak_statistics(peak_number, [[], [], []]))
        self.cpu_stat_label.setText("CPU usage: 0%")
        self.btn_calibration.setEnabled(False)
        self.serial_thread.request_calibration()

    def save_png(self):
        log_dir = os.path.join(os.getcwd(), "logs")
        os.makedirs(log_dir, exist_ok=True)

        base_name = self.filename_input.text().strip()
        if not base_name:
            base_name = "snapshot"
        base_name = os.path.splitext(base_name)[0].strip()
        if not base_name:
            base_name = "snapshot"

        power_level = int(round(self.get_esc_power_value()))
        index_value = int(self.filename_index_input.value())
        requested_name = f"{base_name}{index_value:02d}_{power_level}.png"

        filename = os.path.join(log_dir, requested_name)
        original_name = os.path.basename(filename)
        if os.path.exists(filename):
            base_name_only, ext = os.path.splitext(filename)
            counter = 2
            while True:
                new_name = f"{base_name_only}_{counter:02d}{ext}"
                if not os.path.exists(new_name):
                    filename = new_name
                    break
                counter += 1
            saved_name = os.path.basename(filename)
            message = f"{original_name} exists already, saved as {saved_name}"
        else:
            saved_name = os.path.basename(filename)
            message = f"{saved_name} saved"

        exporter = pg.exporters.ImageExporter(self.graphics_layout.scene())
        exporter.parameters()['width'] = 1920
        exporter.export(filename)

        if hasattr(self, "save_status_label"):
            self.save_status_label.setText(message)
        print(f"Snapshot saved to {filename}")

    def closeEvent(self, event):
        self.timer.stop()
        self.disconnect_fc()

        if hasattr(self, "serial_thread"):
            self.serial_thread.stop()

        if hasattr(self, "fc_telemetry_thread") and self.fc_telemetry_thread is not None:
            self.fc_telemetry_thread.stop()

        if hasattr(self, "fft_worker"):
            self.fft_worker.requestInterruption()
            self.fft_worker.quit()
            self.fft_worker.wait(1000)

        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    port = find_preferred_port()
    if port is None:
        port_dialog = PortDialog()
        if port_dialog.exec() != QDialog.DialogCode.Accepted:
            sys.exit(0)
        port = port_dialog.selected_port()

    win = RealtimeVibeApp(port)
    win.show()
    sys.exit(app.exec())