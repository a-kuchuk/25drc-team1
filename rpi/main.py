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
BASE_SPEED = 87
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

def drive(steering_angle=0, speed=BASE_SPEED, timeout=0.05):
    steering.set_steering_angle(steering_angle-5)
    motor.forward(speed)
    return

def drive_right(steering_angle=0, speed=BASE_SPEED, timeout=0.05):
    steering.set_steering_angle(steering_angle-5)
    # steering.set_steering_angle(steering_angle)
    motor.right(speed, False)
    return

def drive_left(steering_angle=0, speed=BASE_SPEED, timeout=0.05):
    steering.set_steering_angle(steering_angle-5)
    # steering.set_steering_angle(steering_angle)
    motor.left(speed, False)
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
    
    # best so far
    img_warp = utils.img_warp(img, np.float32([(0, 61), (480, 61), (0, 240), (480, 240)]), w, h) 


    # img_warp = utils.img_warp(img, np.float32([(0, 80), (480, 80), (0, 240), (480, 240)]), w, h) 

    # img_warp = utils.img_warp(img, points, w, h)
    # cv2.imshow('warp', img_warp)

    left_mask = getLane(img_warp, left, "left", -1)
    right_mask = getLane(img_warp, right, "right", 1)
    object_mask = getLane(img_warp, purple, "object")
    obj_x = utils.get_lane_centroid_x(object_mask[LOOKAHEAD_Y]) if object_mask is not None else None
    fin_mask = getLane(img_warp, finish, "finish")
    fin_top = utils.get_highest_lane_y(fin_mask)
    fin_area = cv2.countNonZero(fin_mask)

    MIN_Y_THRESHOLD = 125
    MIN_FINISH_AREA = 1000  

    if fin_top is not None and fin_top > MIN_Y_THRESHOLD and fin_area > MIN_FINISH_AREA:
        print("fin")
        drive()
        time.sleep(0.5)
        motor.stop()
        time.sleep(10)
        return

    left_points = utils.get_leftmost_lane_x(left_mask)
    right_points = utils.get_leftmost_lane_x(right_mask)

    # if obj_x is not None:
    #     if obj_x < FRAME_WIDTH // 2 and right_points:
    #         print("object turn left")
    #         # time.sleep(0.2)
    #         drive_left(35, BASE_SPEED, 0.2)
    #     else:
    #         print("object turn right")
    #         # time.sleep(0.2)
    #         drive_right(35, BASE_SPEED, 0.2)
    #     return

    if left_points is not None and right_points is not None:
        print("forward")
        drive()
        return
    elif left_points is not None:
        print("right")
        drive_right(steering_angle=35)
        return
    elif right_points is not None:
        print("left")
        drive_left(steering_angle=-35)
        return
    


if __name__ == '__main__':

    motor = Motor(BASE_SPEED, MIN_SPEED)
    steering = SteeringController()
    steering.set_steering_angle(0)
    # motor.forward(BASE_SPEED)
    drive(speed=100)

    cap = cv2.VideoCapture(0)
    # init_trackbar_vals = [000, 157, 000, 155]
    # utils.trackbar_init(init_trackbar_vals)

    try:
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
