import sys
import time

import cv2
import numpy as np
import pygame
import serial
from PyQt6.QtCore import QObject, QSize, Qt, QThread, QTimer, pyqtSignal
from PyQt6.QtGui import QImage, QPixmap
from PyQt6.QtWidgets import QApplication, QLabel, QMainWindow, QVBoxLayout, QWidget


class CameraWorker(QObject):
    # Signals
    camera_ready = pyqtSignal(np.ndarray)
    error = pyqtSignal(str)

    def __init__(self, fps=30):
        super().__init__()
        self.fps = fps
        self.read_cam = True

    def run(self):
        frame_time = 1.0 / self.fps
        try:
            cap = cv2.VideoCapture(0)

            if not cap.isOpened():
                raise Exception("Camera not found")

            while self.read_cam:
                start_time = time.time()

                ret, frame = cap.read()

                if not ret:
                    self.error.emit("Error Reading Frame")
                    break
                else:
                    self.camera_ready.emit(frame)

                # Limit fps
                elapsed = time.time() - start_time  # Elapsed time to load single frame
                time.sleep(max(0, frame_time - elapsed))  # Subtract elapsed time for stable fps

            cap.release()

        except Exception as e:
            self.error.emit(str(e))

        finally:
            if cap:
                cap.release()

    def stop(self):
        self.read_cam = False


class ControllerWorker(QObject):
    # Signal
    controller_ready = pyqtSignal(dict)
    error = pyqtSignal(str)

    def __init__(self, poll_rate=30):
        super().__init__()
        self.rate = poll_rate
        self.cntrl_data = {}
        self.read_controller = True

    def run(self):
        pygame.init()
        pygame.joystick.init()

        interval = 1 / self.rate

        try:
            if pygame.joystick.get_count() == 0:
                raise Exception("No Controller Connected")

            controller = pygame.joystick.Joystick(0)

            while self.read_controller:
                start_time = time.time()
                pygame.event.pump()

                # Joysticks
                left_stick_x = controller.get_axis(0)
                left_stick_y = controller.get_axis(1)
                right_stick_x = controller.get_axis(2)
                right_stick_y = controller.get_axis(3)

                # Triggers
                left_trigger = controller.get_axis(4)
                right_trigger = controller.get_axis(5)

                # D-pad
                dpad_x, dpad_y = controller.get_hat(0)
                # Buttons
                # A = controller.get_button(0)
                # B = controller.get_button(1)
                # X = controller.get_button(2)
                # Y = controller.get_button(3)

                # Bumpers
                # LB = controller.get_button(4)
                # RB = controller.get_button(5)

                surge = self.apply_deadzone(-left_stick_y)
                yaw = self.apply_deadzone(left_stick_x)
                sway = self.apply_deadzone(right_stick_x)
                heave = self.apply_deadzone(-right_stick_y)

                motorFL, motorFR, motorBL, motorBR, motorUPL, motorUPR = self.calculate_thrust(
                    surge, sway, yaw, heave
                )

                self.cntrl_data["motorFL"] = self.scale(motorFL)
                self.cntrl_data["motorFR"] = self.scale(motorFR)
                self.cntrl_data["motorBL"] = self.scale(motorBL)
                self.cntrl_data["motorBR"] = self.scale(motorBR)
                self.cntrl_data["motorUPL"] = self.scale(motorUPL)
                self.cntrl_data["motorUPR"] = self.scale(motorUPR)

                self.controller_ready.emit(self.cntrl_data.copy())

                # limit polling rate (similar to fps)
                elapsed = time.time() - start_time  # Elapsed time to load single frame
                time.sleep(max(0, interval - elapsed))

        except Exception as e:
            self.error.emit(str(e))

    def apply_deadzone(self, value, deadzone=0.0):
        if abs(value) < deadzone:
            return 0.0  # Neutral value within the target range
        else:
            return value

    def calculate_thrust(self, surge, sway, yaw, heave):
        motorFL = max(-1.0, min(1.0, surge + sway + yaw))
        motorFR = max(-1.0, min(1.0, surge - sway - yaw))
        motorBL = max(-1.0, min(1.0, surge - sway + yaw))
        motorBR = max(-1.0, min(1.0, surge + sway - yaw))
        motorUPL = heave
        motorUPR = heave

        return motorFL, motorFR, motorBL, motorBR, motorUPL, motorUPR

    def scale(self, value, from_range=(-1.0, 1.0), to_range=(1000, 2000), neutral_range=(1450, 1550)):
        """Scale value from one range to another with an optional deadzone and neutral range."""

        from_min, from_max = from_range
        to_min, to_max = to_range
        scaled_value = ((value - from_min) / (from_max - from_min)) * (to_max - to_min) + to_min

        # Apply neutral range deadzone between 1475 and 1575
        if neutral_range[0] <= scaled_value <= neutral_range[1]:
            return sum(neutral_range) / 2

        return round(scaled_value, 1)

    def stop(self):
        self.read_controller = False


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        # GUI Setup
        self.setWindowTitle("ROV Control and Camera Feed")
        self.resize(800, 600)

        self.layout = QVBoxLayout()

        widget = QWidget()
        widget.setLayout(self.layout)
        self.setCentralWidget(widget)

        # Camera Setup
        self.video_label = QLabel("VIDEO")
        self.video_label.setMinimumSize(640, 480)
        self.layout.addWidget(self.video_label)

        self.camera_thread = QThread()
        self.camera_worker = CameraWorker(fps=30)

        self.camera_worker.moveToThread(self.camera_thread)

        self.camera_thread.started.connect(self.camera_worker.run)
        self.camera_worker.camera_ready.connect(self.update_frame)
        self.camera_worker.error.connect(self.handle_camera_error)

        self.camera_thread.start()

        # Controller Setup
        self.controller_label = QLabel("CONTROLLER")
        self.layout.addWidget(self.controller_label)

        self.controller_thread = QThread()
        self.controller_worker = ControllerWorker()

        self.controller_thread.started.connect(self.controller_worker.run)
        self.controller_worker.controller_ready.connect(self.read_controller)
        self.controller_worker.error.connect(self.handle_controller_error)

        self.controller_thread.start()
        # Arduino setup
        # self.ser = serial.Serial("COM4", 9600)

    # QT Slot - Camera
    def update_frame(self, frame):
        """Display live video feed from the camera"""
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

    # QT Slot - Controller
    def read_controller(self, cntrl_data):
        """Send scaled motor control data to arduino"""
        print(cntrl_data)

        # command = f"{int(cntrl_data["motorFL"])} {cntrl_data["motorFR"]} {cntrl_data["motorBL"]} {cntrl_data["motorBR"]} {cntrl_data["motorUPL"]} {cntrl_data["motorUPR"]}\n"
        # self.ser.write(command.encode("utf-8"))

    def handle_camera_error(self, msg):
        self.video_label.setText(f"Camera Error:\n{msg}")

    def handle_controller_error(self, msg):
        self.controller_label.setText(f"Controller Error:\n{msg}")

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
        pygame.quit()
        event.accept()


# Run the GUI application
if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())
