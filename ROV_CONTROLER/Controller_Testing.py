import sys
import pygame
import serial
import time
import cv2
from PyQt5.QtWidgets import QApplication, QLabel, QVBoxLayout, QWidget
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import QTimer

# ---------------------------------------------------------
#  INITIALIZE PYGAME JOYSTICK
# ---------------------------------------------------------
pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    print("ERROR: No controller detected. Plug in a USB controller and restart.")
    sys.exit()

controller = pygame.joystick.Joystick(0)
controller.init()
print(f"Controller detected: {controller.get_name()}")

# ---------------------------------------------------------
#  HELPER FUNCTIONS
# ---------------------------------------------------------

def calculate_thrust(surge, sway, yaw):
    """
    Mix surge (forward/back), sway (left/right), and yaw (rotation)
    into 4 motor outputs.
    """
    motorFL = surge + sway + yaw
    motorFR = surge - sway - yaw
    motorBL = surge - sway + yaw
    motorBR = surge + sway - yaw
    return motorFL, motorFR, motorBL, motorBR


def clamp_motor_values(*motors):
    """Clamp all motor values to the range [-1.0, 1.0]."""
    return tuple(max(-1.0, min(1.0, m)) for m in motors)


def scale(value, from_range, to_range, deadzone=0.0, neutral_range=(1475, 1575)):
    """
    Scale joystick input to ESC PWM range.
    Includes:
      - deadzone for joystick
      - neutral band for ESCs
    """
    if abs(value) < deadzone:
        return (to_range[0] + to_range[1]) / 2

    from_min, from_max = from_range
    to_min, to_max = to_range

    scaled_value = ((value - from_min) / (from_max - from_min)) * (to_max - to_min) + to_min

    if neutral_range[0] <= scaled_value <= neutral_range[1]:
        return (neutral_range[0] + neutral_range[1]) / 2

    return scaled_value


# ---------------------------------------------------------
#  MAIN GUI CLASS
# ---------------------------------------------------------

class ROV_GUI(QWidget):
    def __init__(self):
        super().__init__()

        # ---------------- GUI SETUP ----------------
        self.setWindowTitle("ROV Control and Camera Feed")
        self.setGeometry(100, 100, 1536, 864)

        self.label = QLabel(self)
        self.label.setScaledContents(True)

        layout = QVBoxLayout()
        layout.addWidget(self.label)
        self.setLayout(layout)

        # ---------------- CONTROLLER TIMER ----------------
        self.controller_timer = QTimer()
        self.controller_timer.timeout.connect(self.read_controller)
        self.controller_timer.start(50)  # 20 Hz update rate

    # -----------------------------------------------------
    #  READ CONTROLLER INPUTS
    # -----------------------------------------------------
    def read_controller(self):
        pygame.event.pump()  # Required for updating joystick state

        # ---------------- AXIS MAPPING (Correct for Xbox) ----------------
        left_x  = controller.get_axis(0)   # Left stick horizontal (yaw)
        left_y  = controller.get_axis(1)   # Left stick vertical (surge)
        right_x = controller.get_axis(3)   # Right stick horizontal (sway)
        trigger = controller.get_axis(2)   # Combined triggers (-1 to +1)

        # ---------------- APPLY DEADZONES & SCALE ----------------
        surge = scale(-left_y, (-1, 1), (-1, 1), deadzone=0.1)
        yaw   = scale(left_x, (-1, 1), (-1, 1), deadzone=0.1)
        sway  = scale(right_x, (-1, 1), (-1, 1), deadzone=0.1)
        heave = scale(trigger, (-1, 1), (-1, 1), deadzone=0.1)

        # ---------------- D-PAD INPUT ----------------
        dpad_x, dpad_y = controller.get_hat(0)

        # Default motor values
        motorFL = motorFR = motorBL = motorBR = 0

        if dpad_x == 1:
            # Strafe right using only right motors
            motorFL, motorFR, motorBL, motorBR = surge, -1, sway, 1
        elif dpad_x == -1:
            # Strafe left using only left motors
            motorFL, motorFR, motorBL, motorBR = 1, surge, -1, sway
        else:
            # Normal movement
            motorFL, motorFR, motorBL, motorBR = calculate_thrust(surge, sway, yaw)

        # ---------------- CLAMP ----------------
        motorFL, motorFR, motorBL, motorBR = clamp_motor_values(
            motorFL, motorFR, motorBL, motorBR
        )

        # ---------------- SCALE TO ESC PWM ----------------
        motorFL_pwm = scale(motorFL, (-1, 1), (1000, 2000))
        motorFR_pwm = scale(motorFR, (-1, 1), (1000, 2000))
        motorBL_pwm = scale(motorBL, (-1, 1), (1000, 2000))
        motorBR_pwm = scale(motorBR, (-1, 1), (1000, 2000))
        motorUp1_pwm = scale(heave, (-1, 1), (1000, 2000))
        motorUp2_pwm = scale(-heave, (-1, 1), (1000, 2000))

        # ---------------- DEBUG OUTPUT ----------------
        print(f"Axes: {[controller.get_axis(i) for i in range(controller.get_numaxes())]}")
        print(f"Motors: {int(motorFL_pwm)} {int(motorFR_pwm)} {int(motorBL_pwm)} {int(motorBR_pwm)} {int(motorUp1_pwm)} {int(motorUp2_pwm)}")

        # ---------------- SEND TO ARDUINO ----------------
        command = f"{int(motorFL_pwm)} {int(motorFR_pwm)} {int(motorBL_pwm)} {int(motorBR_pwm)} {int(motorUp1_pwm)} {int(motorUp2_pwm)}\n"
        # ser.write(command.encode())

    # -----------------------------------------------------
    #  CLEANUP
    # -----------------------------------------------------
    def closeEvent(self, event):
        pygame.quit()
        event.accept()


# ---------------------------------------------------------
#  RUN APPLICATION
# ---------------------------------------------------------
if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = ROV_GUI()
    window.show()
    sys.exit(app.exec_())
