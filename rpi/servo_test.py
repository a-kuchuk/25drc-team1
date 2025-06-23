import pigpio
import time

# --- Servo Setup ---
SERVO_GPIO = 26            # BCM pin 26 (physical pin 37)
CENTER_PW = 1520           # Center position (0°)
MAX_ANGLE = 45             # Stay within ±45° for safety
MIN_PW = 1070              # ~ -45°
MAX_PW = 1970              # ~ +45°
DEGREE_SPAN = 90           # Total angle range being used

# Connect to pigpio daemon
pi = pigpio.pi()
if not pi.connected:
    raise RuntimeError("Failed to connect to pigpio daemon. Use: sudo pigpiod")

def angle_to_pulse(angle_deg):
    """
    Convert angle in degrees (−45° to +45°) to pulse width in µs.
    Clamps outside angles for safety.
    """
    angle = max(-MAX_ANGLE, min(MAX_ANGLE, angle_deg))
    scale = (MAX_PW - MIN_PW) / DEGREE_SPAN
    return int(CENTER_PW + angle * scale)

try:
    print("Testing CM509MG Servo Safely...")

    print("Moving to -45°")
    pi.set_servo_pulsewidth(SERVO_GPIO, angle_to_pulse(-45))
    time.sleep(1)

    print("Moving to 0°")
    pi.set_servo_pulsewidth(SERVO_GPIO, angle_to_pulse(0))
    time.sleep(1)

    print("Moving to +45°")
    pi.set_servo_pulsewidth(SERVO_GPIO, angle_to_pulse(50))
    time.sleep(1)

    # print("Sweeping between ±45°")
    # for angle in range(-45, 46, 5):
    #     pi.set_servo_pulsewidth(SERVO_GPIO, angle_to_pulse(angle))
    #     time.sleep(0.03)

    # for angle in range(45, -46, -5):
    #     pi.set_servo_pulsewidth(SERVO_GPIO, angle_to_pulse(angle))
    #     time.sleep(0.03)

finally:
    print("Stopping servo signal...")
    pi.set_servo_pulsewidth(SERVO_GPIO, 0)  # stop PWM signal
    pi.stop()
    print("Test complete.")
