import time
from Control.motor import *
from Control.steering import *

def main():
    try:
        # === Init ===
        base_speed = 80     # base motor speed (0-100)
        min_speed = 30      # minimum motor speed when turning
        motor = Motor(base_speed, min_speed)
        steering = SteeringController()

        max_angle = steering.max_steering_angle_deg 

        print("=== Driving forward with center steering ===")
        # steering.set_steering_angle(0)  # center
        steering.set_steering_angle(-5) # center with offset
        # motor.move_scaled(0, max_angle)
        motor.forward(base_speed)
        time.sleep(1)
        motor.stop()
        time.sleep(1)

        print("=== Drive test with left turn ===")
        steering.set_steering_angle(-max_angle)
        # motor.move_scaled(-max_angle, max_angle)
        motor.left(base_speed, False)
        time.sleep(2)
        motor.stop()
        time.sleep(1)

        print("=== Drive test with right turn ===")
        steering.set_steering_angle(max_angle)
        motor.right(base_speed, False)
        time.sleep(2)
        motor.stop()
        time.sleep(1)

        print("=== Driving forward with center steering ===")
        steering.set_steering_angle(-5)  # center with offset
        # motor.move_scaled(0, max_angle)
        motor.forward(base_speed)
        time.sleep(1)
        motor.stop()
        time.sleep(1)

        print("dkjlfjkldjfkldf")
        motor.forward(base_speed)
        steering.set_steering_angle(20)
        time.sleep(1)
        steering.set_steering_angle(40)
        time.sleep(1)
        steering.set_steering_angle(0)
        time.sleep(1)
        steering.set_steering_angle(-20)
        time.sleep(1)
        steering.set_steering_angle(-40)
        time.sleep(1)
        steering.set_steering_angle(0)
        time.sleep(1)

    finally:
        print("=== Cleaning up ===")
        motor.cleanup()
        steering.cleanup()

if __name__ == "__main__":
    main()
