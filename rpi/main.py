import cv2
import numpy as np
import time
from Control import *
from LaneDetection import *
from ObjectDetection import *
import utils

from pid import PID
from motor import Motor
from steering import SteeringController

# --- Constants ---
BASE_SPEED = 60
MIN_SPEED = 30
FRAME_WIDTH = 480
FRAME_HEIGHT = 240

# --- Initialize Controllers ---
steering = SteeringController()
motor = Motor(base_speed=BASE_SPEED, min_speed=MIN_SPEED)
pid = PID(Kp=0.6, Ki=0.05, Kd=0.1)

# --- OpenCV Camera ---
cap = cv2.VideoCapture(0)
utils.trackbar_init([100, 103, 000, 240])

# --- Main Loop ---
def main():
    success, img = cap.read()
    if not success:
        print("Camera read failed.")
        return

    img = cv2.resize(img, (FRAME_WIDTH, FRAME_HEIGHT))
    cv2.imshow('Original', img)

    # Image warping for top-down view
    h, w, c = img.shape
    points = utils.trackbar_val()
    img_warp = utils.img_warp(img, points, w, h)
    cv2.imshow('Warped', img_warp)

    # Lane processing
    left_mask = detect_lane_color(img_warp, color='yellow')  # You implement this
    right_mask = detect_lane_color(img_warp, color='blue')

    # Get lane centroids
    left_x = get_lane_centroid_x(left_mask)
    right_x = get_lane_centroid_x(right_mask)

    # Calculate center error
    center_x = FRAME_WIDTH // 2
    if left_x is not None and right_x is not None:
        lane_center = (left_x + right_x) // 2
        error = center_x - lane_center
    else:
        print("One or both lanes not detected")
        error = 0

    # --- PID computation ---
    correction_angle = pid.compute(error)

    # --- Actuation ---
    steering.set_steering_angle(correction_angle)
    motor.move_scaled(correction_angle, steering.max_steering_angle)

    cv2.waitKey(1)

# --- Cleanup and Run ---
try:
    while True:
        main()

except KeyboardInterrupt:
    print("Stopping robot...")
    motor.stop()
    steering.cleanup()
    cap.release()
    cv2.destroyAllWindows()

