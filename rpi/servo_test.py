import pigpio
import time

SERVO_GPIO = 26           # BCM pin 26 (physical pin 37)
CENTER_PW = 1520          # Neutral pulse width
DEGREE_SPAN = 100         # Total servo range: ±50° = 100°
MIN_PW = 1020             # ~-50°
MAX_PW = 2020             # ~+50°

def angle_to_pulse(angle_deg):
    """Convert angle (-50 to +50) to pulse width in µs."""
    angle = max(-50, min(50, angle_deg))  # clamp to ±50
    scale = (MAX_PW - MIN_PW) / DEGREE_SPAN
    return int(CENTER_PW + angle * scale)

pi = pigpio.pi()
if not pi.connected:
    raise RuntimeError("pigpio daemon not running. Start with: sudo pigpiod")

try:
    print("Moving to -50°")
    pi.set_servo_pulsewidth(SERVO_GPIO, angle_to_pulse(-50))
    time.sleep(1)

    print("Moving to 0°")
    pi.set_servo_pulsewidth(SERVO_GPIO, angle_to_pulse(0))
    time.sleep(1)

    print("Moving to +50°")
    pi.set_servo_pulsewidth(SERVO_GPIO, angle_to_pulse(50))
    time.sleep(1)

    print("Sweeping...")
    for angle in range(-50, 51, 5):
        pi.set_servo_pulsewidth(SERVO_GPIO, angle_to_pulse(angle))
        time.sleep(0.04)

    for angle in range(50, -51, -5):
        pi.set_servo_pulsewidth(SERVO_GPIO, angle_to_pulse(angle))
        time.sleep(0.04)

finally:
    pi.set_servo_pulsewidth(SERVO_GPIO, 0)
    pi.stop()
    print("Servo test complete.")
