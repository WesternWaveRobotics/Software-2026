import sys
import time

import cv2
import pygame
import serial
from PyQt5.QtCore import QTimer
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import QApplication, QLabel, QVBoxLayout, QWidget

# Initialize Pygame for joystick handling
pygame.init()
pygame.joystick.init()
controller = pygame.joystick.Joystick(0)

# Set up serial communication (adjust port as needed)
ser = serial.Serial("COM3", 9600)

# Open the USB camera (adjust index if needed)
cap = cv2.VideoCapture(1)


def calculate_thrust(surge, sway, yaw):
    """Calculate the thrust for each motor based on input axes."""
    motorFL = surge + sway + yaw
    motorFR = surge - sway - yaw
    motorBL = surge - sway + yaw
    motorBR = surge + sway - yaw
    return motorFL, motorFR, motorBL, motorBR


def clamp_motor_values(motorFL, motorFR, motorBL, motorBR):
    """Clamp thrust values between -1.0 and 1.0."""
    motorFL = max(-1.0, min(1.0, motorFL))
    motorFR = max(-1.0, min(1.0, motorFR))
    motorBL = max(-1.0, min(1.0, motorBL))
    motorBR = max(-1.0, min(1.0, motorBR))
    return motorFL, motorFR, motorBL, motorBR


def scale(value, from_range, to_range, deadzone=0.0, neutral_range=(1475, 1575)):
    """Scale value from one range to another with an optional deadzone and neutral range."""
    if abs(value) < deadzone:
        return (to_range[0] + to_range[1]) / 2  # Neutral value within the target range

    from_min, from_max = from_range
    to_min, to_max = to_range
    scaled_value = ((value - from_min) / (from_max - from_min)) * (to_max - to_min) + to_min

    # Apply neutral range deadzone between 1475 and 1575
    if neutral_range[0] <= scaled_value <= neutral_range[1]:
        return (neutral_range[0] + neutral_range[1]) / 2

    return scaled_value


class ROV_GUI(QWidget):
    def __init__(self):
        super().__init__()
        # GUI setup
        self.setWindowTitle("ROV Control and Camera Feed")
        self.setGeometry(100, 100, 1536, 864)

        # Video feed display
        self.label = QLabel(self)
        self.label.setScaledContents(True)  # Allow scaling
        self.layout = QVBoxLayout()
        self.layout.addWidget(self.label)
        self.setLayout(self.layout)

        # Timer to update video feed
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(30)  # 30 ms interval (33fps)

        # Timer to read Xbox controller inputs
        self.controller_timer = QTimer()
        self.controller_timer.timeout.connect(self.read_controller)
        self.controller_timer.start(100)  # 100 ms interval

    def update_frame(self):
        """Capture and display the video feed from the camera."""
        ret, frame = cap.read()
        if not ret:
            return  # Exit if frame is not read correctly
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        height, width = self.label.height(), self.label.width()  # Get QLabel size
        frame = cv2.resize(frame, (width, height), interpolation=cv2.INTER_LINEAR)  # Resize to fit window
        qimg = QImage(frame.data, width, height, 3 * width, QImage.Format_RGB888)
        self.label.setPixmap(QPixmap.fromImage(qimg))

    def read_controller(self):
        """Read Xbox controller inputs and send motor commands."""
        pygame.event.pump()

        # Get joystick axes
        left_stick_x = controller.get_axis(1)  # Left stick horizontal (yaw)
        left_stick_y = controller.get_axis(0)  # Left stick vertical (surge)
        right_stick_x = controller.get_axis(3)  # Right stick horizontal (sway)

        # Determine trigger axis for heave
        trigger_axis = controller.get_axis(2)  # Adjust based on testing
        heave = scale(trigger_axis, (-1, 1), (-1.0, 1.0), deadzone=0.1)

        # Scale joystick inputs with dead zones
        surge = scale(-left_stick_y, (-1, 1), (-1.0, 1.0), deadzone=0.1)
        yaw = scale(left_stick_x, (-1, 1), (-1.0, 1.0), deadzone=0.1)
        sway = scale(right_stick_x, (-1, 1), (-1.0, 1.0), deadzone=0.1)

        # Get D-pad input
        dpad_x, dpad_y = controller.get_hat(0)

        # Initialize motor values in case D-pad is not pressed
        motorFL, motorFR, motorBL, motorBR = 0, 0, 0, 0

        # Check for D-pad strafe input (left or right)
        if dpad_x == 1:  # D-pad Right → Only move right motors
            motorFL, motorFR, motorBL, motorBR = (
                surge,
                -1,
                sway,
                1,
            )  # Front right goes backward, back right goes forward
        elif dpad_x == -1:  # D-pad Left → Only move left motors
            motorFL, motorFR, motorBL, motorBR = (
                1,
                surge,
                -1,
                sway,
            )  # Front left goes backward, back left goes forward
        else:
            # Normal movement calculations for other axes
            motorFL, motorFR, motorBL, motorBR = calculate_thrust(surge, sway, yaw)

        # Clamp values
        motorFL, motorFR, motorBL, motorBR = clamp_motor_values(motorFL, motorFR, motorBL, motorBR)

        # Scale thrusts to ESC range (1000 to 2000) with neutral range deadzone
        motorFL_speed = scale(motorFL, (-1.0, 1.0), (1000, 2000), neutral_range=(1475, 1575))
        motorFR_speed = scale(motorFR, (-1.0, 1.0), (1000, 2000), neutral_range=(1475, 1575))
        motorBL_speed = scale(motorBL, (-1.0, 1.0), (1000, 2000), neutral_range=(1475, 1575))
        motorBR_speed = scale(motorBR, (-1.0, 1.0), (1000, 2000), neutral_range=(1475, 1575))
        motorUp1_speed = scale(heave, (-1.0, 1.0), (1000, 2000), neutral_range=(1475, 1575))
        motorUp2_speed = scale(-heave, (-1.0, 1.0), (1000, 2000), neutral_range=(1475, 1575))

        # Send motor commands to Arduino
        command = f"{int(motorFL_speed)} {int(motorFR_speed)} {int(motorBL_speed)} {int(motorBR_speed)} {int(motorUp1_speed)} {int(motorUp2_speed)}\n"
        ser.write(command.encode("utf-8"))

    def closeEvent(self, event):
        """Release resources when the GUI is closed."""
        self.timer.stop()
        self.controller_timer.stop()
        cap.release()
        pygame.quit()
        event.accept()


# Run the GUI application
if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = ROV_GUI()
    window.show()
    sys.exit(app.exec_())
