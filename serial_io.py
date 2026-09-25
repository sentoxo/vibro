# Serial worker thread: reads ESC32 burst lines and emits vibration samples.

import serial
import threading
import time

from PyQt6.QtCore import QThread, pyqtSignal

import config


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
        # Buffer for the bulk-read loop (see run()). Reading all pending bytes
        # at once and splitting into lines here is ~20x cheaper than calling
        # ser.readline() per line, which was pegging ~75% of a core.
        self.buffer = bytearray()

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

                # Bulk read: pull every pending byte in one call and split into
                # complete lines. This replaces the old per-line ser.readline(),
                # which issued a syscall + timeout per packet and consumed the
                # vast majority of CPU. in_waiting==0 falls through to the sleep
                # below so the loop stays idle (not busy-spinning) between bursts.
                if self.ser.in_waiting:
                    self.buffer.extend(self.ser.read(self.ser.in_waiting))
                while b"\n" in self.buffer:
                    line, self.buffer = self.buffer.split(b"\n", 1)
                    text = line.decode("utf-8", errors="replace").strip()
                    if not text:
                        continue
                    match = config.SEQ_LINE.match(text)
                    if match:
                        sid, seq, raw_x, raw_y, raw_z = map(int, match.groups())
                        self.data_received.emit(
                            sid, seq,
                            raw_x * self.ms2_per_lsb,
                            raw_y * self.ms2_per_lsb,
                            raw_z * self.ms2_per_lsb,
                        )
                else:
                    self.msleep(1)

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
