# Startup and port-selection dialogs.

from PyQt6.QtCore import QTimer
from PyQt6.QtWidgets import (
    QComboBox, QDialog, QDialogButtonBox, QFileDialog, QHBoxLayout, QLabel,
    QLineEdit, QPushButton, QVBoxLayout,
)
from serial.tools import list_ports


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
