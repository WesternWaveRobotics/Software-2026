# main_window.py - Main GUI for ROV control and live camera feed display.

import cv2
import pygame
import serial
from PyQt6.QtCore import Qt, QThread
from PyQt6.QtGui import QImage, QPixmap
from PyQt6.QtWidgets import (
    QLabel,
    QMainWindow,
    QPlainTextEdit,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from src.workers import CameraWorker, ControllerWorker


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        # GUI Setup
        self.setWindowTitle("ROV Control and Camera Feed")
        self.resize(1000, 800)

        # TODO: Update to multiple column layout
        self.layout = QVBoxLayout()

        widget = QWidget()
        widget.setLayout(self.layout)
        self.setCentralWidget(widget)

        # Camera Setup
        self.video_label = QLabel("VIDEO STREAM")
        self.video_label.setMinimumSize(640, 480)
        self.layout.addWidget(self.video_label, alignment=Qt.AlignmentFlag.AlignCenter)

        self.camera_thread = QThread()
        self.camera_worker = CameraWorker(fps=30)
        self.camera_worker.moveToThread(self.camera_thread)

        self.camera_thread.started.connect(self.camera_worker.run)
        self.camera_worker.camera_ready.connect(self.update_frame)
        self.camera_worker.error.connect(self.handle_camera_error)

        self.camera_thread.start()

        # Controller Setup
        self.thruster_display = QPlainTextEdit()
        self.thruster_display.setReadOnly(True)
        self.layout.addWidget(self.thruster_display)

        self.controller_thread = QThread()
        self.controller_worker = ControllerWorker()
        self.controller_worker.moveToThread(self.controller_thread)

        self.controller_thread.started.connect(self.controller_worker.run)
        self.controller_worker.controller_ready.connect(self.send_command)
        self.controller_worker.status.connect(self.update_controller_status)

        self.controller_thread.start()

        # Arduino setup
        self.ser = None
        self.serial_btn = QPushButton("Connect to Arduino")
        self.serial_btn.clicked.connect(self.connect_serial)
        self.layout.addWidget(self.serial_btn)

        # Debug output
        self.debug = QPlainTextEdit()
        self.layout.addWidget(self.debug)

    # QT Slot - Camera
    def update_frame(self, frame):
        """Display live video feed from the camera."""

        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_frame.shape
        bytes_per_line = ch * w

        qt_image = QImage(rgb_frame.data, w, h, bytes_per_line, QImage.Format.Format_RGB888)

        pixmap = QPixmap.fromImage(qt_image)

        # Scale to fit label (keeps aspect ratio)
        scaled_pixmap = pixmap.scaled(
            self.video_label.size(),
            Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.SmoothTransformation,
        )

        self.video_label.setPixmap(scaled_pixmap)

    def handle_camera_error(self, msg):
        self.video_label.setText(f"Camera Error:\n{msg}")

    # QT Slot - Controller
    def send_command(self, cntrl_data):
        """Send scaled motor control data to arduino"""

        # Update the GUI display with the latest controller motor values.
        lines = [f"{key}: {value}" for key, value in cntrl_data.items()]
        self.thruster_display.setPlainText("\n".join(lines))

        # Send motor values to Arduino if connected
        if self.ser and self.ser.is_open:
            command = f"{int(cntrl_data['motorFL'])} {int(cntrl_data['motorFR'])} {int(cntrl_data['motorBL'])} {int(cntrl_data['motorBR'])} {int(cntrl_data['motorUPL'])} {int(cntrl_data['motorUPR'])}\n"
            self.ser.write(command.encode("utf-8"))

    def update_controller_status(self, status):
        """Update the controller status label."""

        self.debug.appendPlainText(status)
        if "Disconnected" in status:
            self.thruster_display.clear()

    def connect_serial(self):
        """Attempt to connect to the Arduino via serial port."""

        try:
            self.ser = serial.Serial(port="COM4", baudrate=9600)
            self.debug.appendPlainText("Arduino Connected")
            self.serial_btn.setText("Connected")
            self.serial_btn.setEnabled(False)

        except Exception as e:
            self.debug.appendPlainText(f"Serial Error:\n{str(e)}")
            self.serial_btn.setText("Connect to Arduino")
            self.serial_btn.setEnabled(True)

    def closeEvent(self, event):
        """Release resources when the GUI is closed."""

        # Close camera thread
        self.camera_worker.stop()
        self.camera_thread.quit()
        self.camera_thread.wait()

        # Close controller thread
        self.controller_worker.stop()
        self.controller_thread.quit()
        self.controller_thread.wait()

        # others
        if self.ser:
            self.ser.close()
        pygame.quit()
        event.accept()
