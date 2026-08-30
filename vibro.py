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

from PyQt6.QtCore import QThread, pyqtSignal, QTimer
from PyQt6.QtWidgets import (
    QApplication, QComboBox, QDialog, QDialogButtonBox, QGridLayout,
    QGroupBox, QHBoxLayout, QLabel, QLineEdit, QMainWindow, QPushButton,
    QSpinBox, QVBoxLayout, QWidget
)
import pyqtgraph as pg
import pyqtgraph.exporters

# === CONFIGURATION ===
BAUD_RATE = 921600
FC_BAUD_RATE = 115200
FS = 800.0                       
WINDOW_SECONDS = 2.0             
BUFFER_SIZE = int(FS * WINDOW_SECONDS)
LSB_TO_MS2 = 0.0039 * 9.80665    
FFT_Y_MAX = 7.0
TIME_Y_MIN_RANGE = 2.0

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
            if port.device == excluded_port:
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
        self.calibration_requested = threading.Event()

    def run(self):
        try:
            ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
            ser.write(b"call all\n")
            time.sleep(3)
            ser.write(b"start stream\n")
            
            while self.running:
                if self.calibration_requested.is_set():
                    ser.write(b"stop stream\n")
                    time.sleep(1)
                    ser.write(b"call all\n")
                    time.sleep(3)
                    ser.write(b"start stream\n")
                    self.calibration_requested.clear()
                    self.calibration_finished.emit()

                line = ser.readline().decode("utf-8", errors="replace").strip()
                if not line:
                    continue
                match = SEQ_LINE.match(line)
                if match:
                    sid, seq, raw_x, raw_y, raw_z = map(int, match.groups())
                    self.data_received.emit(
                        sid, seq, 
                        raw_x * LSB_TO_MS2, 
                        raw_y * LSB_TO_MS2, 
                        raw_z * LSB_TO_MS2
                    )
            
            ser.write(b"stop stream\n")
            ser.close()
        except Exception as e:
            print(f"Serial error: {e}")

    def stop(self):
        self.running = False
        self.wait()

    def request_calibration(self):
        self.calibration_requested.set()


class RealtimeVibeApp(QMainWindow):
    def __init__(self, serial_port):
        super().__init__()
        self.setWindowTitle("VibroApp")
        self.resize(1600, 900)

        self.buf_x = deque(maxlen=BUFFER_SIZE)
        self.buf_y = deque(maxlen=BUFFER_SIZE)
        self.buf_z = deque(maxlen=BUFFER_SIZE)
        self.time_axes = ['X', 'Y', 'Z']
        self.esp_port = serial_port
        self.fc_serial = None
        self.fc_port = None
        self.esc_running = False
        self.process = psutil.Process()
        self.cpu_count = max(psutil.cpu_count() or 1, 1)
        self.cpu_history = deque(maxlen=10)
        self.process.cpu_percent(None)

        self.init_ui()

        self.serial_thread = SerialWorker(serial_port, BAUD_RATE)
        self.serial_thread.data_received.connect(self.handle_sample)
        self.serial_thread.calibration_finished.connect(self.calibration_finished)
        self.serial_thread.start()

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_plots)
        self.timer.start(50) 

    def init_ui(self):
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QVBoxLayout(main_widget)

        control_layout = QHBoxLayout()
        self.filename_input = QLineEdit()
        self.filename_input.setPlaceholderText("PNG file name")
        self.filename_input.setFixedWidth(300)
        control_layout.addWidget(self.filename_input)
        self.btn_save = QPushButton("Save PNG Snapshot")
        self.btn_save.clicked.connect(self.save_png)
        control_layout.addWidget(self.btn_save)
        self.btn_calibration = QPushButton("Calibration")
        self.btn_calibration.clicked.connect(self.calibrate)
        control_layout.addWidget(self.btn_calibration)
        main_layout.addLayout(control_layout)

        statistics_box = QGroupBox("Diagnostics")
        statistics_layout = QGridLayout(statistics_box)
        self.average_stat_label = QLabel("Average vibration: 0.0 m/s²")
        statistics_layout.addWidget(self.average_stat_label, 0, 0, 1, 3)
        statistics_layout.addWidget(QLabel("Dominant FFT Peaks"), 1, 0, 1, 3)
        self.peak_stat_labels = []
        for peak_number in range(3):
            peak_label = QLabel(self.format_peak_statistics(peak_number, [[], [], []]))
            self.peak_stat_labels.append(peak_label)
            statistics_layout.addWidget(peak_label, peak_number + 2, 0, 1, 3)
        self.cpu_stat_label = QLabel("CPU usage: 0%")
        statistics_layout.addWidget(self.cpu_stat_label, 5, 0, 1, 3)

        esc_box = QGroupBox("ESC control")
        esc_layout = QVBoxLayout(esc_box)
        self.fc_status_label = QLabel("FC: disconnected")
        esc_layout.addWidget(self.fc_status_label)
        self.btn_esc_start = QPushButton("START")
        self.btn_esc_start.clicked.connect(self.start_esc)
        esc_layout.addWidget(self.btn_esc_start)
        self.btn_esc_stop = QPushButton("STOP")
        self.btn_esc_stop.clicked.connect(self.stop_esc)
        esc_layout.addWidget(self.btn_esc_stop)
        power_layout = QHBoxLayout()
        power_layout.addWidget(QLabel("Power level:"))
        self.esc_power_input = QSpinBox()
        self.esc_power_input.setRange(0, 100)
        self.esc_power_input.setSuffix(" %")
        self.esc_power_input.valueChanged.connect(self.update_esc_power)
        power_layout.addWidget(self.esc_power_input)
        esc_layout.addLayout(power_layout)
        self.btn_reconnect_fc = QPushButton("Reconnect to FC")
        self.btn_reconnect_fc.clicked.connect(self.reconnect_fc)
        esc_layout.addWidget(self.btn_reconnect_fc)
        esc_layout.addStretch()
        statistics_box.setFixedHeight(180)
        esc_box.setFixedHeight(180)

        statistics_and_esc_layout = QHBoxLayout()
        statistics_and_esc_layout.addWidget(statistics_box, 1)
        statistics_and_esc_layout.addWidget(esc_box, 1)
        main_layout.addLayout(statistics_and_esc_layout)

        pg.setConfigOptions(antialias=True, background='w', foreground='k')
        self.graphics_layout = pg.GraphicsLayoutWidget()
        main_layout.addWidget(self.graphics_layout)

        # Build 3x2 Grid (Rows: X, Y, Z | Cols: Time, FFT)
        self.curves_time = []
        self.curves_fft = []
        
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

    def calibrate(self):
        self.buf_x.clear()
        self.buf_y.clear()
        self.buf_z.clear()
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

    def calibration_finished(self):
        self.btn_calibration.setEnabled(True)

    def send_motor_values(self, motor_values):
        if self.fc_serial is None or not self.fc_serial.is_open:
            self.fc_status_label.setText("FC: disconnected")
            return False

        payload = b"".join(struct.pack("<H", value) for value in motor_values)
        packet = b"$M<" + bytes((len(payload), 214)) + payload
        checksum = 0
        for byte in packet[3:]:
            checksum ^= byte
        try:
            self.fc_serial.write(packet + bytes((checksum,)))
            self.fc_serial.flush()
            return True
        except serial.SerialException as error:
            self.fc_status_label.setText(f"FC error: {error}")
            self.fc_serial.close()
            self.fc_serial = None
            self.fc_port = None
            self.esc_running = False
            return False

    def start_esc(self):
        if self.send_motor_values([1000 + self.esc_power_input.value() * 10, 1000, 1000, 1000]):
            self.esc_running = True

    def stop_esc(self):
        if self.send_motor_values([1000, 1000, 1000, 1000]):
            self.esc_running = False

    def update_esc_power(self):
        if self.esc_running:
            self.start_esc()

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
            return

        selected_port = dialog.selected_port()
        try:
            self.fc_serial = serial.Serial(selected_port, FC_BAUD_RATE, timeout=0.1)
            self.fc_port = selected_port
            self.fc_status_label.setText(f"FC: connected ({selected_port})")
        except serial.SerialException as error:
            self.fc_status_label.setText(f"FC connection failed: {error}")

    def disconnect_fc(self):
        if self.fc_serial is not None and self.fc_serial.is_open:
            self.send_motor_values([1000, 1000, 1000, 1000])
            self.fc_serial.close()
        self.fc_serial = None
        self.fc_port = None
        self.esc_running = False

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

    def compute_fft(self, signal_arr):
        n = len(signal_arr)
        if n < 64:
            return np.array([]), np.array([])
        
        sig = signal_arr - np.mean(signal_arr)
        window = np.hanning(n)
        scale = np.sum(window) / n
        
        yf = np.fft.rfft(sig * window)
        freqs = np.fft.rfftfreq(n, 1.0 / FS)
        
        amp = (np.abs(yf) / n) / scale
        if n > 1:
            amp[1:-1] *= 2.0
            
        return freqs, amp

    def update_plots(self):
        if len(self.buf_x) < 64:
            return

        data = [np.array(self.buf_x), np.array(self.buf_y), np.array(self.buf_z)]
        t = np.arange(len(data[0])) / FS
        fft_peaks = []
        self.average_stat_label.setText(
            f"Average vibration: {np.mean(np.abs(np.concatenate(data))):.1f} m/s²"
        )
        cpu_usage = self.process.cpu_percent() / self.cpu_count
        self.cpu_history.append(max(0.0, min(100.0, cpu_usage)))
        average_cpu_usage = np.mean(self.cpu_history)
        self.cpu_stat_label.setText(f"CPU usage: {average_cpu_usage:.0f}%")

        for i in range(3):
            # Update Time
            self.curves_time[i].setData(t, data[i])
            time_y_limit = max(
                TIME_Y_MIN_RANGE / 2,
                np.max(np.abs(data[i])) * 1.1,
            )
            self.plots_time[i].setYRange(-time_y_limit, time_y_limit, padding=0)
            average_level = np.mean(np.abs(data[i]))
            self.plots_time[i].setTitle(
                f"Czasówki - Oś {self.time_axes[i]} - {average_level:.1f} m/s² avg"
            )
            
            # Update FFT
            f, a = self.compute_fft(data[i])
            if len(f) > 0:
                self.curves_fft[i].setData(f, a)
                fft_peaks.append(self.find_fft_peaks(f, a))
            else:
                fft_peaks.append([])

        for peak_number, peak_label in enumerate(self.peak_stat_labels):
            peak_label.setText(self.format_peak_statistics(peak_number, fft_peaks))

    def save_png(self):
        filename = self.filename_input.text().strip()
        if not filename:
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            filename = f"Realtime_Snapshot_{timestamp}.png"
        elif not filename.lower().endswith(".png"):
            filename += ".png"
        exporter = pg.exporters.ImageExporter(self.graphics_layout.scene())
        exporter.parameters()['width'] = 1920
        exporter.export(filename)
        print(f"Snapshot saved to {filename}")

    def closeEvent(self, event):
        self.disconnect_fc()
        self.serial_thread.stop()
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