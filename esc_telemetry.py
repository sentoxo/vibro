# Flight-controller (Betaflight) motor-RPM telemetry reader thread.

import serial
import struct
import threading
import time

from PyQt6.QtCore import QThread, pyqtSignal


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
