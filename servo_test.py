import pigpio
import time

# Constants
SERVO_GPIO = 18  # Use GPIO18 (PIN 12 on Pi header)
MIN_PW = 500     # Minimum pulse width in microseconds (~0°)
MAX_PW = 2500    # Maximum pulse width in microseconds (~180°)

# Initialize pigpio
pi = pigpio.pi()
if not pi.connected:
    raise RuntimeError("Failed to connect to pigpio daemon. Is 'pigpiod' running?")

try:
    print("Moving to 0°")
    pi.set_servo_pulsewidth(SERVO_GPIO, MIN_PW)
    time.sleep(1)

    print("Moving to 90°")
    pi.set_servo_pulsewidth(SERVO_GPIO, (MIN_PW + MAX_PW) // 2)
    time.sleep(1)

    print("Moving to 180°")
    pi.set_servo_pulsewidth(SERVO_GPIO, MAX_PW)
    time.sleep(1)

    # Sweep back and forth
    print("Sweeping...")
    for angle in range(0, 181, 10):
        pulse = MIN_PW + (MAX_PW - MIN_PW) * angle // 180
        pi.set_servo_pulsewidth(SERVO_GPIO, pulse)
        time.sleep(0.05)

    for angle in range(180, -1, -10):
        pulse = MIN_PW + (MAX_PW - MIN_PW) * angle // 180
        pi.set_servo_pulsewidth(SERVO_GPIO, pulse)
        time.sleep(0.05)

finally:
    # Turn off servo pulses
    pi.set_servo_pulsewidth(SERVO_GPIO, 0)
    pi.stop()
    print("Servo test complete.")
