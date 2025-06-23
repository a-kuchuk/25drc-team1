import time
from Control import Motor
from Control import SteeringController

def main():
    try:
        # === Init ===
        base_speed = 60     # base motor speed (0-100)
        min_speed = 30      # minimum motor speed when turning
        motor = Motor(base_speed=base_speed, min_speed=min_speed)
        steering = SteeringController()

        max_angle = steering.max_steering_angle_deg  # ~27.6 degrees

        print("=== Driving forward with center steering ===")
        steering.set_steering_angle(0)  # center
        motor.move_scaled(steering_angle=0, max_steering_angle=max_angle)
        time.sleep(3)
        motor.stop()
        time.sleep(1)

        print("=== Steering full left ===")
        steering.set_steering_angle(-max_angle)
        time.sleep(1)

        print("=== Steering center ===")
        steering.set_steering_angle(0)
        time.sleep(1)

        print("=== Steering full right ===")
        steering.set_steering_angle(max_angle)
        time.sleep(1)

        print("=== Steering center ===")
        steering.set_steering_angle(0)
        time.sleep(1)

        print("=== Drive test with left turn ===")
        steering.set_steering_angle(-max_angle)
        motor.move_scaled(steering_angle=-max_angle, max_steering_angle=max_angle)
        time.sleep(2)
        motor.stop()
        time.sleep(1)

        print("=== Drive test with right turn ===")
        steering.set_steering_angle(max_angle)
        motor.move_scaled(steering_angle=max_angle, max_steering_angle=max_angle)
        time.sleep(2)
        motor.stop()
        time.sleep(1)

    finally:
        print("=== Cleaning up ===")
        motor.cleanup()
        steering.cleanup()

if __name__ == "__main__":
    main()
