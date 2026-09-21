# AI Generated code for real-time vibration monitoring and ESC control.

# The on-disk burst format (little-endian):
#   magic "DVB1" | version u16 | axes u16 | sample_rate u32 |
#   sample_count u32 | timestamp_ns u64 | sequence u64 | int16 samples[count][3]

import argparse
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
    QFileDialog, QGridLayout, QGroupBox, QHBoxLayout, QLabel, QLineEdit,
    QMainWindow, QPushButton, QSpinBox, QVBoxLayout, QWidget
)
import pyqtgraph as pg
import pyqtgraph.exporters as pg_exporters

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

# === DVB1 BINARY BURST FORMAT ===
DVB1_MAGIC = b"DVB1"
DVB1_HEADER_SIZE = 4 + 2 + 2 + 4 + 4 + 8 + 8  # magic|version|axes|rate|count|ts|seq
DVB1_SAMPLE_RATE = 3200.0                      # Hz
DVB1_SAMPLE_COUNT = 6400                       # samples per burst
DVB1_LSB_TO_MS2 = 0.0039 * 9.80665              # ADXL345 8g mode: 3.9 mg/LSB
FFT_MIN_HZ = 4.0                               # FFT display range
FFT_MAX_FREQ_CHOICES = [200, 400, 600, 800, 1000, 1200, 1600]  # Hz, FFT max-frequency options
DEFAULT_FFT_MAX_FREQ = 800.0                     # Hz, default FFT max frequency


def remove_gravity_component(axis_data):
    # Remove the constant (DC / gravity) component from each axis.
    return [axis - np.mean(axis) for axis in axis_data]


def parse_dvb1_file(path):
    # Read whole file and split it into bursts (DVB1 format)
    with open(path, "rb") as f:
        data = f.read()

    bursts = []
    offset = 0
    while offset + DVB1_HEADER_SIZE <= len(data):
        if data[offset:offset + 4] != DVB1_MAGIC:
            # Skip garbage byte, search for next magic
            next_magic = data.find(DVB1_MAGIC, offset + 1)
            if next_magic < 0:
                break
            offset = next_magic
            continue

        version, axes, sample_rate, sample_count = struct.unpack_from(
            "<HHII", data, offset + 4
        )
        timestamp_ns, sequence = struct.unpack_from("<QQ", data, offset + 16)
        samples_bytes = sample_count * axes * 2
        if offset + DVB1_HEADER_SIZE + samples_bytes > len(data):
            break  # truncated burst

        raw = np.frombuffer(
            data, dtype="<i2",
            count=sample_count * axes,
            offset=offset + DVB1_HEADER_SIZE,
        ).reshape(sample_count, axes)

        bursts.append({
            "version": version,
            "axes": axes,
            "sample_rate": float(sample_rate),
            "sample_count": sample_count,
            "timestamp_ns": timestamp_ns,
            "sequence": sequence,
            "samples": raw[:, :3].astype(np.float64) * DVB1_LSB_TO_MS2,
        })
        offset += DVB1_HEADER_SIZE + samples_bytes

    return bursts


class ModeDialog(QDialog):
    # Startup dialog: choose serial port, demo mode, or binary file mode
    MODE_SERIAL = "serial"
    MODE_DEMO = "demo"
    MODE_FILE = "file"

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Select mode")
        self.setMinimumWidth(480)
        self.mode = None
        self.port = None
        self.file_path = None

        layout = QVBoxLayout(self)
        layout.addWidget(QLabel("Select data source:"))

        self.port_combo = QComboBox()
        layout.addWidget(self.port_combo)

        self.file_row = QHBoxLayout()
        self.file_input = QLineEdit()
        self.file_input.setPlaceholderText("Binary burst file (.dvb)")
        self.file_row.addWidget(self.file_input)
        browse_button = QPushButton("Browse...")
        browse_button.clicked.connect(self.browse_file)
        self.file_row.addWidget(browse_button)
        layout.addLayout(self.file_row)

        buttons = QDialogButtonBox(
            QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Cancel
        )
        buttons.accepted.connect(self.accept_mode)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)

        demo_button = buttons.addButton("Demo", QDialogButtonBox.ButtonRole.ActionRole)
        demo_button.clicked.connect(self.accept_demo)

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
            description = port.description or "Unknown device"
            self.port_combo.addItem(f"{port.device} - {description}", port.device)
        if selected_port:
            selected_index = self.port_combo.findData(selected_port)
            if selected_index >= 0:
                self.port_combo.setCurrentIndex(selected_index)
        self.port_combo.blockSignals(False)

    def browse_file(self):
        path, _ = QFileDialog.getOpenFileName(
            self, "Select binary burst file", "",
            "Burst files (*.dvb *.bin);;All files (*)"
        )
        if path:
            self.file_input.setText(path)

    def accept_mode(self):
        # OK button: file mode if a file is chosen, otherwise serial port
        path = self.file_input.text().strip()
        if path:
            self.mode = self.MODE_FILE
            self.file_path = path
        elif self.port_combo.currentData():
            self.mode = self.MODE_SERIAL
            self.port = self.port_combo.currentData()
        else:
            return
        self.accept()

    def accept_demo(self):
        self.mode = self.MODE_DEMO
        self.accept()


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
        self.demo_mode = False
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

        demo_button = buttons.addButton("Demo", QDialogButtonBox.ButtonRole.ActionRole)
        demo_button.clicked.connect(self.accept_demo)

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

    def accept_demo(self):
        self.demo_mode = True
        self.accept()

class SerialWorker(QThread):
    data_received = pyqtSignal(int, int, float, float, float)
    calibration_finished = pyqtSignal()

    def __init__(self, port, baudrate, lsb_per_g=256.0):
        super().__init__()
        self.port = port
        self.baudrate = baudrate
        # Sensor sensitivity in LSB per g. Converted to m/s^2 per LSB so the
        # rest of the app always works in m/s^2 regardless of the hardware.
        self.lsb_per_g = lsb_per_g
        self.ms2_per_lsb = (1.0 / self.lsb_per_g) * 9.80665
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
                        raw_x * self.ms2_per_lsb,
                        raw_y * self.ms2_per_lsb,
                        raw_z * self.ms2_per_lsb,
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

    def __init__(self, serial_port, motor_channel=3):
        super().__init__()
        self.ser = serial_port
        self.running = True
        self.buffer = bytearray()
        self.motor_channel = max(1, int(motor_channel))

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
    def _parse_motor_rpm(cls, payload, motor_channel):
        if not payload:
            return None
        motor_count = payload[0]
        if motor_channel < 1 or motor_count < motor_channel:
            return None
        motor_offset = 1 + (motor_channel - 1) * cls.MOTOR_RECORD_SIZE
        if len(payload) < motor_offset + 4:
            return None
        return struct.unpack_from("<I", payload, motor_offset)[0]

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
                        rpm_value = self._parse_motor_rpm(payload, self.motor_channel)
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

    def set_motor_channel(self, motor_channel):
        self.motor_channel = max(1, int(motor_channel))


class FFTWorker(QThread):
    result_ready = pyqtSignal(int, object, object)

    def __init__(self):
        super().__init__()
        self.queue = deque()
        self.lock = threading.Lock()

    def submit(self, axis_index, signal_arr, sample_rate=FS, full=False):
        with self.lock:
            self.queue.append((axis_index, signal_arr, sample_rate, full))

    def run(self):
        while not self.isInterruptionRequested():
            item = None
            with self.lock:
                if self.queue:
                    item = self.queue.popleft()

            if item is None:
                self.msleep(10)
                continue

            axis_index, signal_arr, sample_rate, full = item
            freqs, amps = self.compute_fft(signal_arr, sample_rate, full)
            self.result_ready.emit(axis_index, freqs, amps)

    @staticmethod
    def compute_fft(signal_arr, sample_rate=FS, full=False):
        arr = np.asarray(signal_arr, dtype=np.float64)
        n = len(arr)
        if n < 64:
            return np.array([]), np.array([])

        if not full:
            # Live mode: cap FFT length for speed
            n = min(n, MAX_FFT_POINTS)
            if len(arr) > n:
                arr = arr[-n:]

        sig = arr - np.mean(arr)
        window = np.hanning(n)
        scale = np.sum(window) / n

        yf = np.fft.rfft(sig * window)
        freqs = np.fft.rfftfreq(n, 1.0 / sample_rate)
        amp = (np.abs(yf) / n) / scale
        if n > 1:
            amp[1:-1] *= 2.0

        return freqs, amp


class RealtimeVibeApp(QMainWindow):
    def __init__(self, serial_port=None, demo_mode=False, file_path=None):
        super().__init__()
        self.setWindowTitle("VibroApp")
        self.resize(1600, 900)

        self.time_window_seconds = 2.0
        # FFT max frequency (Hz) also sets the live Nyquist: live_sample_rate = 2 * fft_max_freq.
        self.fft_max_freq = DEFAULT_FFT_MAX_FREQ
        self.live_sample_rate = 2.0 * self.fft_max_freq
        self.buf_x = deque(maxlen=int(self.live_sample_rate * self.time_window_seconds))
        self.buf_y = deque(maxlen=int(self.live_sample_rate * self.time_window_seconds))
        self.buf_z = deque(maxlen=int(self.live_sample_rate * self.time_window_seconds))
        self.time_axes = ['X', 'Y', 'Z']
        self.esp_port = serial_port
        self.demo_mode = demo_mode
        self.file_mode = file_path is not None
        self.file_path = file_path
        self.file_bursts = []
        self.current_burst_index = 0
        self.demo_sample_index = 0
        self.fc_serial = None
        self.fc_port = None
        self.esc_running = False
        self.process = psutil.Process()
        self.cpu_count = max(psutil.cpu_count() or 1, 1)
        self.cpu_history = deque(maxlen=10)
        self.process.cpu_percent(None)

        # Incoming-data statistics (live serial mode only).
        self.packet_count = 0          # packets received in the current 1 s window
        self.last_seq = None           # last seen sequence id (packet index)
        self.lost_packets = 0          # cumulative lost packets (seq gaps)
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
        # Default to the old ADXL345 (256 LSB/g). Changed via the Settings
        # dropdown; also applied live to a running serial worker.
        self.lsb_per_g = 256.0

        self.init_ui()

        # Report incoming-data frequency and lost packets once per second.
        self.packet_stats_timer = QTimer()
        self.packet_stats_timer.timeout.connect(self.update_packet_stats)
        self.packet_stats_timer.start(1000)

        if self.file_mode:
            # File mode: load bursts and show the first one
            self.load_burst_file(self.file_path)
        elif self.demo_mode:
            # Demo mode: generate synthetic bursts, selectable from list box
            self.btn_reconnect_esp.setVisible(False)
            self.generate_demo_bursts()
        else:
            self.serial_thread = SerialWorker(serial_port, BAUD_RATE, self.lsb_per_g)
            self.serial_thread.data_received.connect(self.handle_sample)
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

        # CPU usage (live, smoothed average over the last 10 samples)
        self.cpu_stat_label = QLabel("CPU usage: 0%")
        statistics_layout.addWidget(self.cpu_stat_label, 6, 0, 1, 3)

        # Incoming-data frequency (packets per second) and lost-packet count.
        self.packet_freq_label = QLabel("Incoming frequency: 0 Hz")
        statistics_layout.addWidget(self.packet_freq_label, 7, 0, 1, 3)
        self.lost_packets_label = QLabel("Lost packets: 0")
        statistics_layout.addWidget(self.lost_packets_label, 8, 0, 1, 3)

        # Burst metadata (file mode only)
        self.burst_info_label = QLabel("")
        self.burst_info_label.setWordWrap(True)
        statistics_layout.addWidget(self.burst_info_label, 5, 0, 1, 3)
        statistics_layout.addWidget(QLabel(""), 9, 0, 1, 3)

        settings_box = QGroupBox("Settings")
        settings_box.setStyleSheet(
            "QGroupBox { margin-top: 0px; padding-top: 8px; border: 1px solid #444; }"
        )
        settings_layout = QVBoxLayout(settings_box)
        settings_layout.setContentsMargins(8, 12, 8, 8)
        settings_layout.setSpacing(6)

        self.btn_reconnect_esp = QPushButton("Reconnect to esp")
        self.btn_reconnect_esp.clicked.connect(self.reconnect_esp)
        self.btn_reconnect_esp.setFixedWidth(200)
        settings_layout.addWidget(self.btn_reconnect_esp)
        if self.demo_mode or self.file_mode:
            # Hide ESP reconnect button in demo and file modes
            self.btn_reconnect_esp.setVisible(False)

        # Burst selector for demo / file modes
        self.burst_combo = QComboBox()
        self.burst_combo.setFixedWidth(200)
        self.burst_combo.currentIndexChanged.connect(self.on_burst_selected)
        settings_layout.addWidget(self.burst_combo)
        if not self.demo_mode and not self.file_mode:
            self.burst_combo.setVisible(False)

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

        # FFT graph maximum frequency selector.  The chosen value sets both the
        # FFT display range and the live sample rate (Nyquist = sample_rate / 2).
        fft_max_label = QLabel("FFT max frequency:")
        settings_layout.addWidget(fft_max_label)

        self.fft_max_freq_input = QComboBox()
        self.fft_max_freq_input.setFixedWidth(200)
        for _f in FFT_MAX_FREQ_CHOICES:
            self.fft_max_freq_input.addItem(str(_f), int(_f))
        self.fft_max_freq_input.setCurrentIndex(FFT_MAX_FREQ_CHOICES.index(DEFAULT_FFT_MAX_FREQ))
        self.fft_max_freq_input.currentIndexChanged.connect(self.on_fft_max_freq_changed)
        settings_layout.addWidget(self.fft_max_freq_input)

        # Sensor sensitivity selector: choose the LSB/g converter for the
        # connected accelerometer. 256 = old ADXL345, 2048 = new sensor.
        sensor_label = QLabel("Sensor sensitivity:")
        settings_layout.addWidget(sensor_label)

        self.sensor_sensitivity_combo = QComboBox()
        self.sensor_sensitivity_combo.addItem("256 (old ADXL345)", 256)
        self.sensor_sensitivity_combo.addItem("2048 (new sensor)", 2048)
        self.sensor_sensitivity_combo.setCurrentIndex(0)
        self.sensor_sensitivity_combo.setFixedWidth(200)
        self.sensor_sensitivity_combo.currentIndexChanged.connect(self.on_sensor_sensitivity_changed)
        settings_layout.addWidget(self.sensor_sensitivity_combo)

        settings_layout.addStretch(1)
        settings_box.setFixedHeight(260)
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

        rpm_channel_label = QLabel("RPM motor channel:")
        esc_layout.addWidget(rpm_channel_label)
        self.rpm_motor_channel_input = QSpinBox()
        self.rpm_motor_channel_input.setRange(1, 16)
        self.rpm_motor_channel_input.setValue(3)
        self.rpm_motor_channel_input.setSuffix(" (1-based)")
        self.rpm_motor_channel_input.setFixedWidth(170)
        self.rpm_motor_channel_input.valueChanged.connect(self.set_rpm_motor_channel)
        esc_layout.addWidget(self.rpm_motor_channel_input)

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
        esc_box.setFixedHeight(250)
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
            # FFT range: 4 Hz .. selected max frequency (see Settings).
            p_fft.setXRange(FFT_MIN_HZ, self.fft_max_freq, padding=0)
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
        # Every incoming line counts toward the incoming-data frequency.
        self.packet_count += 1
        if sid == 0:
            # Lost-packet detection: the sequence id should increase by exactly 1.
            if self.last_seq is not None:
                gap = seq - self.last_seq
                if gap > 1:
                    # A gap means packets were dropped on the way.
                    self.lost_packets += gap - 1
                # gap <= 0 means the sequence rewound (e.g. reconnect); ignore.
            self.last_seq = seq
            self.buf_x.append(x)
            self.buf_y.append(y)
            self.buf_z.append(z)

    def update_packet_stats(self):
        # Called once per second: report the packet rate, then restart the window.
        self.packet_freq_label.setText(f"Incoming frequency: {self.packet_count} Hz")
        self.lost_packets_label.setText(f"Lost packets: {self.lost_packets}")
        self.packet_count = 0

    def generate_demo_bursts(self):
        # Build synthetic bursts matching the DVB1 format (3200 Hz, 6400 samples)
        demo_rates = [48.0, 72.0, 110.0, 160.0]
        base_timestamp = int(time.time() * 1e9)
        for burst_number, demo_hz in enumerate(demo_rates):
            t = np.arange(DVB1_SAMPLE_COUNT) / DVB1_SAMPLE_RATE
            x = 1.1 * np.sin(2.0 * np.pi * demo_hz * t)
            y = 0.8 * np.sin(2.0 * np.pi * demo_hz * 1.5 * t + 0.7)
            z = 0.6 * np.sin(2.0 * np.pi * demo_hz * 2.0 * t + 1.4)
            self.file_bursts.append({
                "version": 1,
                "axes": 3,
                "sample_rate": int(DVB1_SAMPLE_RATE),
                "sample_count": DVB1_SAMPLE_COUNT,
                "timestamp_ns": base_timestamp + burst_number * 1_000_000_000,
                "sequence": burst_number + 1,
                "samples": np.column_stack((x, y, z)),
            })

        self.burst_combo.blockSignals(True)
        for index, burst in enumerate(self.file_bursts):
            self.burst_combo.addItem(f"Burst {index + 1} (seq {burst['sequence']})", index)
        self.burst_combo.blockSignals(False)
        self.burst_combo.setCurrentIndex(0)
        self.display_burst(0)

    def load_burst_file(self, path):
        # Read binary file and split it into DVB1 bursts
        try:
            self.file_bursts = parse_dvb1_file(path)
        except Exception as error:
            self.file_bursts = []
            self.burst_info_label.setText(f"File error: {error}")
            return

        self.burst_combo.blockSignals(True)
        self.burst_combo.clear()
        for index, burst in enumerate(self.file_bursts):
            self.burst_combo.addItem(f"Burst {index + 1} (seq {burst['sequence']})", index)
        self.burst_combo.blockSignals(False)

        if self.file_bursts:
            self.current_burst_index = 0
            self.burst_combo.setCurrentIndex(0)
            self.display_burst(0)
        else:
            self.burst_info_label.setText("No bursts found in file")

    def on_burst_selected(self, index):
        # User picked a different burst from the list box
        if 0 <= index < len(self.file_bursts):
            self.display_burst(index)

    def display_burst(self, burst_index):
        # Show one full burst in time plots and compute its FFT
        burst = self.file_bursts[burst_index]
        self.current_burst_index = burst_index
        self.current_fs = burst["sample_rate"] or DVB1_SAMPLE_RATE

        samples = burst["samples"]
        count = len(samples)
        if count == 0:
            return  # nothing to draw for an empty burst
        t = np.arange(count) / self.current_fs

        # Use the burst array directly (buffers are too small for 6400 samples)
        data = [samples[:, 0], samples[:, 1], samples[:, 2]]

        # Remove the constant gravity component so only vibration remains
        data = remove_gravity_component(data)

        # Human-readable timestamp
        timestamp_seconds = burst["timestamp_ns"] / 1e9
        timestamp_text = time.strftime(
            "%Y-%m-%d %H:%M:%S", time.localtime(timestamp_seconds)
        )
        milliseconds = int(burst["timestamp_ns"] % 1_000_000_000 / 1_000_000)
        self.burst_info_label.setText(
            f"Burst {burst_index + 1}/{len(self.file_bursts)} | "
            f"samples: {burst['sample_count']} | "
            f"timestamp: {timestamp_text}.{milliseconds:03d} | "
            f"sequence: {burst['sequence']}"
        )

        # Draw time plots immediately
        for i in range(3):
            self.curves_time[i].setData(t, data[i])
            time_y_limit = max(TIME_Y_MIN_RANGE / 2, np.max(np.abs(data[i])) * 1.1)
            self.plots_time[i].setYRange(-time_y_limit, time_y_limit, padding=0)
            self.plots_time[i].setXRange(0, count / self.current_fs, padding=0)
            average_level = np.mean(np.abs(data[i]))
            self.plots_time[i].setTitle(
                f"Czasówki - Oś {self.time_axes[i]} - {average_level:.1f} m/s² avg"
            )
        self.average_stat_label.setText(
            f"Average vibration: {np.mean(np.abs(np.concatenate(data))):.1f} m/s²"
        )

        # Submit full burst for FFT (no MAX_FFT_POINTS downsampling for file mode)
        for i in range(3):
            self.fft_worker.submit(i, data[i], self.current_fs, full=True)
        self.fft_cache = [None, None, None]

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

    def handle_fc_rpm_update(self, rpm_value):
        self.fc_rpm = max(0.0, float(rpm_value))
        hz_value = self.fc_rpm / 60.0
        self.update_fft_rpm_indicators(hz_value)

    def set_rpm_motor_channel(self, motor_channel):
        if self.fc_telemetry_thread is not None:
            self.fc_telemetry_thread.set_motor_channel(motor_channel)
        self.log_fc_diagnostic(f"RPM telemetry channel set to motor {motor_channel}.", "INFO")

    def update_fft_rpm_indicators(self, motor_hz):
        max_frequency = DVB1_SAMPLE_RATE / 2.0  # file-mode sensor rate cap
        frequencies = (float(motor_hz), float(motor_hz) * 2.0)
        for rpm_line, double_rpm_line in self.fft_rpm_lines:
            for line, frequency in zip((rpm_line, double_rpm_line), frequencies):
                line.setPos(frequency)
                line.setVisible(0.0 < frequency <= max_frequency)

    def reconnect_esp(self):
        if self.demo_mode:
            return

        if hasattr(self, "serial_thread"):
            self.serial_thread.stop()

        self.serial_thread = SerialWorker(self.esp_port, BAUD_RATE, self.lsb_per_g)
        self.serial_thread.data_received.connect(self.handle_sample)
        self.serial_thread.start()

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

    def on_sensor_sensitivity_changed(self, index):
        # Update the physical conversion factor from the Settings dropdown and
        # push it to a running serial worker so live data re-scales immediately.
        value = self.sensor_sensitivity_combo.currentData()
        if value is None:
            return
        self.lsb_per_g = float(value)
        if hasattr(self, "serial_thread") and self.serial_thread is not None:
            self.serial_thread.lsb_per_g = float(value)
            self.serial_thread.ms2_per_lsb = (1.0 / float(value)) * 9.80665

    def on_fft_max_freq_changed(self, index):
        # Wrapper so the combo's currentIndexChanged delivers the stored value.
        hz = int(self.fft_max_freq_input.currentData())
        self.set_fft_max_frequency(hz)

    def set_fft_max_frequency(self, hz):
        # Set the FFT display max frequency (Hz) and the live sample rate used to
        # compute the FFT frequency axis. Nyquist = sample_rate / 2, so the live
        # sample rate is twice the chosen max frequency.
        self.fft_max_freq = float(hz)
        self.live_sample_rate = 2.0 * self.fft_max_freq
        # Rebuild the time-domain buffers so they still hold ~time_window_seconds.
        self.buf_x = deque(maxlen=int(self.live_sample_rate * self.time_window_seconds))
        self.buf_y = deque(maxlen=int(self.live_sample_rate * self.time_window_seconds))
        self.buf_z = deque(maxlen=int(self.live_sample_rate * self.time_window_seconds))
        # Widen/shrink the FFT display range on all three axes if they exist.
        if hasattr(self, "plots_fft"):
            for p_fft in self.plots_fft:
                p_fft.setXRange(FFT_MIN_HZ, self.fft_max_freq, padding=0)

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

        self.fc_telemetry_thread = FCTelemetryReader(
            self.fc_serial,
            self.rpm_motor_channel_input.value(),
        )
        self.fc_telemetry_thread.rpm_updated.connect(self.handle_fc_rpm_update)
        self.fc_telemetry_thread.start()

    def reconnect_fc(self):
        if self.fc_serial is not None and self.fc_serial.is_open:
            self._stop_ramp_active = False
            if self.fc_telemetry_thread is not None:
                self.fc_telemetry_thread.stop()
                self.fc_telemetry_thread = None
            self.send_motor_values([1000, 1000, 1000, 1000])
            if self.fc_serial is not None and self.fc_serial.is_open:
                self.fc_serial.close()
            self.fc_serial = None
            self.fc_port = None
            self.esc_running = False

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
        if self.file_mode:
            return  # file mode plots are drawn once per burst
        if len(self.buf_x) < 64:
            return

        data = [np.array(self.buf_x), np.array(self.buf_y), np.array(self.buf_z)]
        # Remove the constant gravity component so only vibration remains
        data = remove_gravity_component(data)
        t = np.arange(len(data[0])) / self.live_sample_rate
        self.average_stat_label.setText(
            f"Average vibration: {np.mean(np.abs(np.concatenate(data))):.1f} m/s²"
        )

        # Sample CPU usage and show a smoothed average over recent samples.
        self.cpu_history.append(max(0.0, min(100.0, self.process.cpu_percent())))
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
            # Pass the live sample rate so the FFT frequency axis matches the
            # currently selected FFT max frequency (Nyquist = sample_rate / 2).
            for i in range(3):
                self.fft_worker.submit(i, data[i], self.live_sample_rate)

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

        try:
            exporter = pg_exporters.ImageExporter(self.graphics_layout.scene())
            exporter.parameters()['width'] = 1920
            exporter.export(filename)
        except Exception as error:
            error_message = f"PNG export failed: {error}"
            self.save_status_label.setStyleSheet(
                "QLabel { color: #9b1c1c; font-size: 12px; }"
            )
            self.save_status_label.setText(error_message)
            print(error_message)
            return

        self.save_status_label.setStyleSheet(
            "QLabel { color: #2d6b2d; font-size: 12px; }"
        )
        self.save_status_label.setText(message)
        print(f"Snapshot saved to {filename}")

    def closeEvent(self, event):
        self.timer.stop()
        self.disconnect_fc()

        if hasattr(self, "demo_timer"):
            self.demo_timer.stop()


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
    parser = argparse.ArgumentParser(description="VibroApp")
    parser.add_argument(
        "--file", metavar="PATH",
        help="open a binary burst file (DVB1) instead of asking for a mode",
    )
    parser.add_argument(
        "--debug", action="store_true",
        help="debug mode: validate the file (with --file) and exit without GUI",
    )
    args = parser.parse_args()

    # Debug mode: just check the file and report, no GUI
    if args.debug:
        if not args.file:
            print("ERROR: --debug requires --file PATH")
            sys.exit(2)
        try:
            bursts = parse_dvb1_file(args.file)
        except Exception as error:
            print(f"ERROR: could not open {args.file}: {error}")
            sys.exit(1)
        if not bursts:
            print(f"ERROR: no DVB1 bursts found in {args.file}")
            sys.exit(1)
        for index, burst in enumerate(bursts):
            print(
                f"Burst {index + 1}: seq={burst['sequence']} "
                f"rate={burst['sample_rate']}Hz samples={burst['sample_count']} "
                f"ts_ns={burst['timestamp_ns']}"
            )
        print(f"OK: {len(bursts)} burst(s) loaded from {args.file}")
        sys.exit(0)

    app = QApplication(sys.argv)
    port = None
    demo_mode = False
    file_path = None

    if args.file:
        # CLI file mode: skip the mode dialog
        file_path = args.file
    else:
        # Show mode selection dialog at startup
        mode_dialog = ModeDialog()
        if mode_dialog.exec() != QDialog.DialogCode.Accepted:
            sys.exit(0)
        if mode_dialog.mode == ModeDialog.MODE_DEMO:
            demo_mode = True
        elif mode_dialog.mode == ModeDialog.MODE_FILE:
            file_path = mode_dialog.file_path
        else:
            port = mode_dialog.port

    win = RealtimeVibeApp(port, demo_mode=demo_mode, file_path=file_path)
    win.show()
    sys.exit(app.exec())
