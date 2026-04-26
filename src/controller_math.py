# controller_math.py - Utility functions for calculating motor thrusts and scaling controller inputs.


def apply_deadzone(value, deadzone=0.1):
    """Apply a deadzone for controller joysick inputs"""
    if abs(value) < deadzone:
        return 0.0
    else:
        return value


def calculate_thrust(surge, sway, yaw, heave):
    """Calculate individual motor thrusts based on controller inputs."""

    motorFL = max(-1.0, min(1.0, surge + sway + yaw))
    motorFR = -max(-1.0, min(1.0, surge - sway - yaw))  # NEGATIVE bc CCW motors :(
    motorBL = -max(-1.0, min(1.0, surge - sway + yaw))  # NEGATIVE bc CCW motors :(
    motorBR = max(-1.0, min(1.0, surge + sway - yaw))
    motorUPL = -heave  # NEGATIVE bc CCW motors :(
    motorUPR = heave

    return motorFL, motorFR, motorBL, motorBR, motorUPL, motorUPR


def scale(value, input_range=(-1.0, 1.0), esc_range=(1000, 2000), neutral_range=(1475, 1525)):
    """Scale controller input to motor output range, with a neutral deadzone."""

    from_min, from_max = input_range
    to_min, to_max = esc_range

    scaled_value = ((value - from_min) / (from_max - from_min)) * (to_max - to_min) + to_min

    # Apply neutral range deadzone between 1475 and 1525
    if neutral_range[0] <= scaled_value <= neutral_range[1]:
        return 1500  # ESC neutral value

    return int(scaled_value)
