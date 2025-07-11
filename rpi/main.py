import cv2
import numpy as np
import time
# from Control import *
from LaneDetection.lane_detection import *
from ObjectDetection import *
import utils
from colours import *

# from Control.pid import PID
# from Control.motor import Motor
# from Control.steering import SteeringController

from ArrowDetection import *

# --- Constants ---
BASE_SPEED = 70
MIN_SPEED = 30
FRAME_WIDTH = 480
FRAME_HEIGHT = 240
LOOKAHEAD_Y = 150

# --- Initialize Controllers ---
# steering = SteeringController()
# motor = Motor(base_speed=BASE_SPEED, min_speed=MIN_SPEED)
# pid = PID(Kp=0.6, Ki=0.05, Kd=0.1)

left = TapeYellow()
right = TapeBlue()
finish = TapeGreen

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
    cv2.imshow('vid', img)
    cv2.waitKey(1)

    # IMAGE WARPING STEP
    h, w, c = img.shape
    # points = utils.trackbar_val()
    # print(points)
    img_warp = utils.img_warp(img, np.float32([(0, 64), (480, 64), (0, 177), (480, 177)]), w, h)
    # img_warp = utils.img_warp(img, points, w, h)
    # cv2.imshow('warp', img_warp)

    left_mask = getLane(img, left, "left")

    right_mask = getLane(img, right, "right")

    left_highest_y = utils.get_highest_lane_y(left_mask)
    right_highest_y = utils.get_highest_lane_y(right_mask)

    if left_highest_y is None or right_highest_y is None:
        print("Could not find valid lane pixels for one or both sides.")
        return

    print("Highest point (Y) of left lane:", left_highest_y)
    print("Highest point (Y) of right lane:", right_highest_y)


    # CHANGE LOGIC TO ONLY DETECT SHIT IN NTEH BOTTOM ~1/4OF THE IMAGE TO DISCOUNT RANDOM SHADOWS/ETC
    fin_lane = getLane(img, finish, "finish")

    # Only consider the bottom 1/4 of the image
    height = fin_lane.shape[0]
    roi = fin_lane[int(height * 0.75):, :]  # Region Of Interest

    # Count how many pixels in each row are white (i.e. part of the lane)
    row_sums = np.sum(roi == 255, axis=1)

    # Define a threshold: how wide must a line be to count as a valid lane
    # For example, require a white segment that's at least 30% of image width in at least one row
    min_width_ratio = 0.3
    min_white_pixels = int(fin_lane.shape[1] * min_width_ratio)

    # Trigger only if any row in the ROI has enough contiguous white pixels
    if np.any(row_sums >= min_white_pixels):
        print("FIN")
        time.sleep(2)
        return
    
    # For each frame:
    #   Apply colour thresholding to extract left (yellow) and right (blue) lane masks.
    #   For each mask:
    #       Sample multiple horizontal scanlines (e.g., at 5 or more y-values)
    #       Extract x-coordinates of centroids at those y-values
    #       This gives you a set of x-y points per lane → enables curve fitting.

    # Get lane point samples

    # Test code for other lane detection contour method
    left_points = utils.detectLinePoly(left_mask)
    right_points = utils.detectLinePoly(right_mask)
    # left_points = utils.get_lane_points(left_mask, left_highest_y)
    # right_points = utils.get_lane_points(right_mask, right_highest_y)
    print(f"POINTS \n {left_points} \n {right_points}")
    print(f"Left points: {len(left_points)}, Right points: {len(right_points)}")

    # Fit curves
    left_poly = utils.fit_poly(left_points)
    right_poly = utils.fit_poly(right_points)
    # print(f"POLYS \n {left_poly} \n {right_poly}")

    # Determine lane centre and heading
    if left_poly is not None and right_poly is not None:
        left_x = utils.evaluate_poly(left_poly, LOOKAHEAD_Y)
        right_x = utils.evaluate_poly(right_poly, LOOKAHEAD_Y)
        lane_center = (left_x + right_x) // 2
    
        # --- Error Calculation ---
        lateral_error = (FRAME_WIDTH // 2) - lane_center
        heading_gradient = (utils.derivative_at(left_poly, LOOKAHEAD_Y) + utils.derivative_at(right_poly, LOOKAHEAD_Y)) / 2
        heading_error = np.degrees(np.arctan(heading_gradient))

        # # PID computation (mix lateral + heading)
        # # Lateral error = horizontal distance between center of image and lane midpoint
        # # Heading error = angle between robot’s current direction and lane tangent
        # total_error = lateral_error + heading_error
        # correction = pid.compute(total_error)

        # # Driving correction from PID
        # steering.set_steering_angle(correction)
        # # motor.move_scaled(correction, steering.max_steering_angle_deg)
        # motor.forward(BASE_SPEED)

        # Display debugging visuals
        utils.display_debug(img, left_poly, right_poly, lateral_error, heading_error, LOOKAHEAD_Y, left_highest_y, right_highest_y)

    # breaks loop if green lane detected
    # if np.any(bottom_quarter):
    #     print("FINISH")
    #     time.sleep(2)
    #     return
    
    else:
        print("Lane fitting failed :(")
        # motor.stop()

    # OVJECT DETECTION (might need to be modified)
    # objectMask = getLane(img_warp, colours.HighlighterPink, "object")

    # ARROW DETECTION STEP
    
    cv2.waitKey(1)
    #except KeyboardInterrupt:

if __name__ == '__main__':
    # ls /dev/video*
    cap = cv2.VideoCapture(0)
    init_trackbar_vals = [000, 157, 000, 155]
    utils.trackbar_init(init_trackbar_vals)
    while True:
        main()
    print("Stopping robot...")
    # motor.stop()
    # steering.cleanup()
    cap.release()
    cv2.destroyAllWindows()
