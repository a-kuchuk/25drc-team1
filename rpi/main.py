import cv2
import numpy as np
import time
from Control import *
from LaneDetection.lane_detection import *
from ObjectDetection import *
import utils
import colours

from Control.pid import PID
from Control.motor import Motor
from Control.steering import SteeringController

from ArrowDetection import *

# --- Constants ---
BASE_SPEED = 60
MIN_SPEED = 30
FRAME_WIDTH = 480
FRAME_HEIGHT = 240
LOOKAHEAD_Y = 150

# --- Initialize Controllers ---
steering = SteeringController()
motor = Motor(base_speed=BASE_SPEED, min_speed=MIN_SPEED)
pid = PID(Kp=0.6, Ki=0.05, Kd=0.1)

# arrow_mode_triggered = False  # To prevent repeated detection
# --- OpenCV Camera ---
# cap = cv2.VideoCapture(0)
#utils.trackbar_init([100, 103, 000, 240])

# --- Main Loop ---
def main():
    '''
    main function
    current steps:
        - read image
        - resize image
        - warp image
    '''
    # global arrow_mode_triggered

    success, img = cap.read()
    if not success:
        print("Camera read failed.")
        return

    img = cv2.resize(img, (FRAME_WIDTH, FRAME_HEIGHT))
    #cv2.imshow('vid', img)
    cv2.waitKey(1)

    # IMAGE WARPING STEP
    h, w, c = img.shape
    # points = utils.trackbar_val()
    img_warp = utils.img_warp(img, np.float32([(93, 188), (480 - 93, 188), (0, 240), (480 - 0, 240)]), w, h)
    if (img_warp):
        print("Great success")
    else:
        print("kms")
    #cv2.imshow('warp', img_warp)

    # --- Lane Detection ---
    left_mask = getLane(img_warp, colours.TapeYellow, "left")
    right_mask = getLane(img_warp, colours.TapeBlue, "right")

    if (left_mask):
        print("left lane success")
    if (right_mask):
        print("right lane succes")
    else:
        print("either left or right lane fail")

    # For each frame:
    #   Apply colour thresholding to extract left (yellow) and right (blue) lane masks.
    #   For each mask:
    #       Sample multiple horizontal scanlines (e.g., at 5 or more y-values)
    #       Extract x-coordinates of centroids at those y-values
    #       This gives you a set of x-y points per lane → enables curve fitting.

    # Get lane point samples
    left_points = utils.get_lane_points(left_mask)
    right_points = utils.get_lane_points(right_mask)
    # Fit curves
    left_poly = utils.fit_poly(left_points)
    right_poly = utils.fit_poly(right_points)

    # Determine lane centre and heading
    if left_poly is not None and right_poly is not None:
        left_x = utils.evaluate_poly(left_poly, LOOKAHEAD_Y)
        right_x = utils.evaluate_poly(right_poly, LOOKAHEAD_Y)
        lane_center = (left_x + right_x) // 2
    
        # --- Error Calculation ---
        lateral_error = (FRAME_WIDTH // 2) - lane_center
        heading_gradient = (utils.derivative_at(left_poly, LOOKAHEAD_Y) + utils.derivative_at(right_poly, LOOKAHEAD_Y)) / 2
        heading_error = np.degrees(np.arctan(heading_gradient))

        # PID computation (mix lateral + heading)
        # Lateral error = horizontal distance between center of image and lane midpoint
        # Heading error = angle between robot’s current direction and lane tangent
        total_error = lateral_error + heading_error
        correction = pid.compute(total_error)

        # Driving correction from PID
        steering.set_steering_angle(correction)
        motor.move_scaled(correction, steering.max_steering_angle_deg)

        # --- Visualization ---
        debug = cv2.cvtColor(img_warp, cv2.COLOR_GRAY2BGR)
        for x, y in left_points:
            cv2.circle(debug, (x, y), 2, (255, 0, 0), -1)
        for x, y in right_points:
            cv2.circle(debug, (x, y), 2, (0, 255, 0), -1)
        cv2.line(debug, (lane_center, 0), (lane_center, FRAME_HEIGHT), (0, 255, 255), 2)
        cv2.line(debug, (FRAME_WIDTH // 2, 0), (FRAME_WIDTH // 2, FRAME_HEIGHT), (0, 0, 255), 2)
        #cv2.imshow("Lane Debug", debug)

    else:
        print("Lane fitting failed :(")
        motor.stop()

    # OVJECT DETECTION (might need to be modified)
    objectMask = getLane(img_warp, colours.HighlighterPink, "object")

    # ARROW DETECTION STEP
    
    cv2.waitKey(1)
    #except KeyboardInterrupt:

# # --- Cleanup and Run ---
# try:
#     while True:
#         main()


if __name__ == '__main__':
    cap = cv2.VideoCapture(0)
    init_trackbar_vals = [93, 188, 000, 240]
    utils.trackbar_init(init_trackbar_vals)
    while True:
        main()
    # print("Stopping robot...")
    # motor.stop()
    # steering.cleanup()
    # cap.release()
    # cv2.destroyAllWindows()
