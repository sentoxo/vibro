# Startup and port-selection dialogs.

from PyQt6.QtCore import QTimer
from PyQt6.QtWidgets import (
    QComboBox, QDialog, QDialogButtonBox, QFileDialog, QHBoxLayout, QLabel,
    QLineEdit, QMessageBox, QPushButton, QVBoxLayout,
)
from serial.tools import list_ports

import config
import updater


class ModeDialog(QDialog):
    # Startup dialog: choose serial port, demo mode, or binary file mode
    MODE_SERIAL = "serial"
    MODE_DEMO = "demo"
    MODE_FILE = "file"

    def __init__(self):
        super().__init__()
        self.setWindowTitle("VibroApp - Select mode")
        self.setMinimumWidth(480)
        self.mode = None
        self.port = None
        self.file_path = None
        self._update_workers = []
        self.update_applied = False

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

        self.check_updates_button = QPushButton("Check for updates")
        self.check_updates_button.clicked.connect(self.check_for_updates)
        layout.addWidget(self.check_updates_button)
        self.update_status = QLabel(f"Current version: v{config.APP_VERSION}")
        layout.addWidget(self.update_status)

        self.mode_buttons = QDialogButtonBox(
            QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Cancel
        )
        self.mode_buttons.accepted.connect(self.accept_mode)
        self.mode_buttons.rejected.connect(self.reject)
        layout.addWidget(self.mode_buttons)

        self.demo_button = self.mode_buttons.addButton(
            "Demo", QDialogButtonBox.ButtonRole.ActionRole
        )
        self.demo_button.clicked.connect(self.accept_demo)

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

    def _start_update_worker(self, action):
        worker = updater.UpdateWorker(action)
        worker.setParent(self)
        self._update_workers.append(worker)
        if action == "check":
            worker.check_finished.connect(self.handle_update_check)
        else:
            worker.update_finished.connect(self.handle_update_result)
        worker.finished.connect(self._release_finished_worker)
        self.mode_buttons.setEnabled(False)
        self.demo_button.setEnabled(False)
        self.check_updates_button.setEnabled(False)
        worker.start()

    def _release_finished_worker(self):
        # Use a dialog-bound slot so thread cleanup and widget changes run on the UI thread.
        worker = self.sender()
        if worker in self._update_workers:
            self._update_workers.remove(worker)
        worker.deleteLater()
        if self.update_applied:
            if not any(active_worker.isRunning() for active_worker in self._update_workers):
                self.reject()
        elif not self._update_workers:
            self.mode_buttons.setEnabled(True)
            self.demo_button.setEnabled(True)
            self.check_updates_button.setEnabled(True)

    def reject(self):
        # A running Git operation must finish before its QThread parent can be destroyed.
        if any(worker.isRunning() for worker in self._update_workers):
            return
        super().reject()

    def check_for_updates(self):
        self.update_status.setText("Checking GitHub for updates...")
        self._start_update_worker("check")

    def handle_update_check(self, status, version, message):
        if status == "error":
            self.update_status.setText("Update check failed.")
            QMessageBox.warning(self, "Update check failed", message)
        elif status == "current":
            self.update_status.setText(f"Current version: v{config.APP_VERSION}")
            QMessageBox.information(self, "No update available", message)
        else:
            self.update_status.setText(f"Version {version} is available.")
            answer = QMessageBox.question(
                self,
                "Update available",
                f"Version {version} is available. Update this clone now?\n\n"
                "The updater will run a fast-forward-only Git pull. The app will close "
                "afterward so you can restart it with the updated code.",
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
                QMessageBox.StandardButton.No,
            )
            if answer == QMessageBox.StandardButton.Yes:
                self.check_updates_button.setEnabled(False)
                self.update_status.setText("Updating from GitHub...")
                self._start_update_worker("update")
                return
    def handle_update_result(self, succeeded, message):
        if not succeeded:
            self.update_status.setText("Update failed.")
            QMessageBox.warning(self, "Update failed", message)
            return

        self.update_applied = True
        QMessageBox.information(
            self,
            "Update applied",
            f"{message}\n\nClose this message, then restart VibroApp to use the update.",
        )
        # The entry point imported old modules; _release_finished_worker exits after Git stops.
        self.reject()

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
