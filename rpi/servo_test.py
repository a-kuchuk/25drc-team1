from time import sleep
from Control import *  # assuming your class is in steering.py

def main():
    steering = SteeringController()

    try:
        print("Testing SteeringController with ±45° range")

        # Turn center
        print("Centering (0°)")
        steering.set_steering_angle(0)
        sleep(1)

        # # Turn full left
        # print("Turning to -45°")
        # steering.set_steering_angle(-45)
        # sleep(1)

        # # Turn center
        # print("Centering (0°)")
        # steering.set_steering_angle(0)
        # sleep(1)

        # # Turn full right
        # print("Turning to +45°")
        # steering.set_steering_angle(45)
        # sleep(1)

        # # Sweep from -45° to +45°
        # print("Sweeping from -45° to +45°")
        # for angle in range(-45, 46, 5):
        #     print(f"Angle: {angle}°")
        #     steering.set_steering_angle(angle)
        #     sleep(0.05)

        # # Sweep back
        # print("Sweeping from +45° to -45°")
        # for angle in range(45, -46, -5):
        #     print(f"Angle: {angle}°")
        #     steering.set_steering_angle(angle)
        #     sleep(0.05)

        # # Center at the end
        # print("Centering again")
        # steering.set_steering_angle(0)
        # sleep(1)

    except KeyboardInterrupt:
        print("\nTest interrupted.")

    finally:
        print("Cleaning up...")
        steering.cleanup()
        print("Test complete.")

if __name__ == "__main__":
    main()
