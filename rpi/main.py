# import os
# os.environ["QT_QPA_PLATFORM"] = "offscreen"

import cv2
import numpy as np
import time
import signal
import sys

from LaneDetection.lane_detection import *
from ObjectDetection import *
from colours import *
import utils
from Control.pid import PID
from Control.motor import Motor
from Control.steering import SteeringController
from ArrowDetection import *

# import RPi.GPIO as GPIO

# --- Constants ---
BASE_SPEED = 50
MIN_SPEED = 30
FRAME_WIDTH = 480
FRAME_HEIGHT = 240
LOOKAHEAD_Y = 0

# --- Global Controllers ---
motor = None
steering = None

# --- Lane and Object Color Trackers ---
left = TapeYellow()
right = TapeBlue()
purple = PaintPurple()
finish = TapeGreen()

arrow_state = None
arrow_cooldown = 0

def drive(steering_angle=-5, speed=BASE_SPEED, timeout=0.05):
    steering.set_steering_angle(steering_angle)
    motor.forward(speed)
    time.sleep(timeout)
    return

def main_loop():
    global arrow_state, arrow_cooldown

    success, img = cap.read()
    if not success:
        print("Camera read failed.")
        return

    img = cv2.resize(img, (FRAME_WIDTH, FRAME_HEIGHT))
    # cv2.imshow('vid', img)
    # cv2.waitKey(1)

    h, w, c = img.shape
    # points = utils.trackbar_val()
    # print(points)
    img_warp = utils.img_warp(img, np.float32([(0, 179), (480, 179), (0, 240), (480, 240)]), w, h)    
    # img_warp = utils.img_warp(img, points, w, h)
    # cv2.imshow('warp', img_warp)

    # if arrow_cooldown == 0 and arrow_state is None:
    #     direction = utils.detect_arrow_direction(img)
    #     if direction:
    #         arrow_state = direction
    #         arrow_cooldown = 30
    #         print(f"Arrow detected: {direction.upper()}")
    # if arrow_cooldown > 0:
    #     arrow_cooldown -= 1

    left_mask = getLane(img_warp, left, "left", -1)
    right_mask = getLane(img_warp, right, "right", 1)
    object_mask = getLane(img_warp, purple, "object")
    obj_x = utils.get_lane_centroid_x(object_mask[LOOKAHEAD_Y]) if object_mask is not None else None

    if obj_x is not None:
        if obj_x < FRAME_WIDTH // 2:
            print("Object on left — turning right to avoid")
            drive(30, BASE_SPEED, 0.2)
        else:
            print("Object on right — turning left to avoid")
            drive(-30, BASE_SPEED, 0.2)
        return

    # if arrow_state == 'left':
    #     print("Following arrow left")
    #     drive(-30, BASE_SPEED, 1.5)
    #     return
    # elif arrow_state == 'right':
    #     print("Following arrow right")
    #     drive(30, BASE_SPEED, 1.5)
    #     return

    fin_lane = getLane(img_warp, finish, "finish")
    height = fin_lane.shape[0]
    roi = fin_lane[int(height * 0.75):, :]
    row_sums = np.sum(roi == 255, axis=1)
    min_width_ratio = 0.3
    min_white_pixels = int(fin_lane.shape[1] * min_width_ratio)

    if np.any(row_sums >= min_white_pixels):
        print("FIN")
        print("reset to forward?")
        drive(timeout=2)
        return

    left_x = utils.get_lane_centroid_x(left_mask[LOOKAHEAD_Y]) if left_mask is not None else None
    right_x = utils.get_lane_centroid_x(right_mask[LOOKAHEAD_Y]) if right_mask is not None else None
    frame_center = FRAME_WIDTH // 2
    THRESHOLD = 20

    if left_x is not None and right_x is not None:
        # lane_center = (left_x + right_x) // 2
        # error = lane_center - frame_center
        # if error > THRESHOLD:
        #     print("Slight Right")
        #     drive(steering_angle=10)
        # elif error < -THRESHOLD:
        #     print("Slight Left")
        #     drive(steering_angle=-10)
        # else:
        #     print("Go Straight")
            # drive()
        min_left_x = utils.get_leftmost_lane_x(left_mask)
        min_right_x = utils.get_leftmost_lane_x(right_mask)
        # print(min_left_x)
        # print(min_right_x)
        if min_left_x > min_right_x:
            print("Fork detected — yellow is right of blue. Turning hard left")
            drive(steering_angle=-30, timeout=0.2)
            return
        print("Forward")
        drive()
    elif left_x is not None:
        print("Only left lane found — turning right")
        drive(steering_angle=15, timeout=0.15)
    elif right_x is not None:
        print("Only right lane found — turning left")
        drive(steering_angle=-15, timeout=0.15)
    else:
        print("No lanes found. slight forward then stop")
        drive(speed=MIN_SPEED, timeout=0.03)

    # cv2.waitKey(1)

if __name__ == '__main__':
    cap = cv2.VideoCapture(0)
    # init_trackbar_vals = [000, 157, 000, 155]
    # utils.trackbar_init(init_trackbar_vals)

    try:
        motor = Motor(BASE_SPEED, MIN_SPEED)
        steering = SteeringController()
        steering.set_steering_angle(0)
        while True:
            main_loop()
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received. Cleaning up and exiting...")
    except Exception as e:
        print(f"\nUnexpected error: {e}")
    finally:
        print("Stopping robot and cleaning up GPIO...")
        if motor:
            motor.stop()
            motor.cleanup()
        if steering:
            steering.cleanup()
        cap.release()
        # cv2.destroyAllWindows()
        sys.exit(0)
