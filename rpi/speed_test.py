import time
from Control.motor import *
from Control.steering import *

def main():
    try:
        # === Init ===
        max_speed = 90
        base_speed = 60     # base motor speed (0-100)
        min_speed = 30      # minimum motor speed when turning
        motor = Motor(base_speed, min_speed)
        steering = SteeringController()

        max_angle = steering.max_steering_angle_deg 

        print("=== Min speed ===")
        motor.forward(30)
        time.sleep(2)
        motor.stop()
        time.sleep(1)

        print("=== base speed ===")
        motor.forward(60)
        time.sleep(2)
        motor.stop()
        time.sleep(1)

        print("=== max speed ===")
        motor.forward(90)
        time.sleep(2)
        motor.stop()
        time.sleep(1)

        print("=== wait 20 (turn car) ===")
        time.sleep(60)

        print("slow acceleration")
        motor.forward(30)
        motor.forward(40)
        motor.forward(50)
        motor.forward(60)
        motor.forward(70)
        time.sleep(2)
        motor.stop()
        time.sleep(1)

        print("fast acceleration")
        motor.forward(30)
        motor.forward(80)
        time.sleep(2)
        motor.stop()
        time.sleep(1)

    finally:
        print("=== Cleaning up ===")
        motor.cleanup()
        steering.cleanup()

if __name__ == "__main__":
    main()
