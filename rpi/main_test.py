import time
from motor import Motor
from steering import SteeringController
from pid import PID

# Initialize hardware controllers
motor = Motor()
steering = SteeringController()
pid = PID(Kp=1.0, Ki=0.0, Kd=0.0)  # No integral/derivative for test

# List of test scenarios: (description, steering_angle_in_degrees)
test_sequence = [
    ("Straight", 0),
    ("Gentle Left", -10),
    ("Gentle Right", 10),
    ("Sharp Left", -25),
    ("Sharp Right", 25),
    ("Straight", 0),
]

def test_drive():
    print("=== Starting Ackermann Drive Test ===")
    time.sleep(2)

    for description, angle in test_sequence:
        print(f"\n[Test] {description} | Steering Angle: {angle:.1f}°")

        # Apply steering angle
        steering.set_steering_angle(angle)

        # Scale motor speed based on steering angle
        motor.move_scaled(angle, steering.max_steering_angle_deg)

        time.sleep(3)  # run each scenario for 3 seconds

    print("\nTest complete. Stopping motors.")
    motor.stop()
    steering.cleanup()

if __name__ == "__main__":
    try:
        test_drive()
    except KeyboardInterrupt:
        print("\nInterrupted by user. Shutting down...")
        motor.stop()
        steering.cleanup()
