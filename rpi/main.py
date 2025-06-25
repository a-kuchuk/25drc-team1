import cv2
import numpy as np
import time
from Control import *
from LaneDetection.lane_detection import *
from ObjectDetection import *
import utils
from colours import *

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
    #cv2.imshow('vid', img)
    cv2.waitKey(1)

    # IMAGE WARPING STEP
    h, w, c = img.shape
    # points = utils.trackbar_val()
    img_warp = utils.img_warp(img, np.float32([(0, 157), (480, 157), (0, 155), (480, 155)]), w, h)

    left_mask = getLane(img_warp, left, "left")

    right_mask = getLane(img_warp, right, "right")

    fin_lane = getLane(img_warp, finish, "finish")
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

        # # PID computation (mix lateral + heading)
        # # Lateral error = horizontal distance between center of image and lane midpoint
        # # Heading error = angle between robot’s current direction and lane tangent
        # total_error = lateral_error + heading_error
        # correction = pid.compute(total_error)

        # # Driving correction from PID
        # steering.set_steering_angle(correction)
        # motor.move_scaled(correction, steering.max_steering_angle_deg)

        # Display debugging visuals
        display_debug(img, left_poly, right_poly, lateral_error, heading_error, LOOKAHEAD_Y)

    # breaks loop if green lane detected
    if fin_lane:
        time.sleep(2)
        return
    
    else:
        print("Lane fitting failed :(")
        motor.stop()

    # OVJECT DETECTION (might need to be modified)
    # objectMask = getLane(img_warp, colours.HighlighterPink, "object")

    # ARROW DETECTION STEP
    
    cv2.waitKey(1)
    #except KeyboardInterrupt:


def display_debug(img, left_poly, right_poly, lateral_error, heading_error, lookahead_y=150):
    debug_img = img.copy()
    h, w, _ = img.shape

    # Draw fitted polynomials as points
    for y in range(lookahead_y, h, 5):
        if left_poly is not None:
            lx = int(utils.evaluate_poly(left_poly, y))
            cv2.circle(debug_img, (lx, y), 3, (0, 255, 255), -1)  # yellow

        if right_poly is not None:
            rx = int(utils.evaluate_poly(right_poly, y))
            cv2.circle(debug_img, (rx, y), 3, (255, 0, 0), -1)  # blue

    # Draw heading vector (if both polynomials are valid)
    if left_poly is not None and right_poly is not None:
        left_x = utils.evaluate_poly(left_poly, lookahead_y)
        right_x = utils.evaluate_poly(right_poly, lookahead_y)
        center_x = int((left_x + right_x) // 2)
        center_y = lookahead_y

        heading_gradient = (utils.derivative_at(left_poly, lookahead_y) + 
                            utils.derivative_at(right_poly, lookahead_y)) / 2
        angle_rad = np.arctan(heading_gradient)

        direction_len = 40
        dx = int(direction_len * np.cos(angle_rad))
        dy = int(direction_len * np.sin(angle_rad))

        # Draw heading arrow
        cv2.arrowedLine(debug_img, (center_x, center_y),
                        (center_x + dx, center_y - dy), (0, 0, 255), 2)

        # Draw center to lookahead point line
        cv2.line(debug_img, (w // 2, h), (center_x, lookahead_y), (0, 255, 0), 2)

    # Add debug text
    cv2.putText(debug_img, f"Lateral Error: {lateral_error}", (10, 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    cv2.putText(debug_img, f"Heading Error: {heading_error:.2f}", (10, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    # Show debug window
    cv2.imshow("Debug View", debug_img)


if __name__ == '__main__':
    cap = cv2.VideoCapture(0)
    init_trackbar_vals = [000, 157, 000, 155]
    utils.trackbar_init(init_trackbar_vals)
    while True:
        main()
    print("Stopping robot...")
    motor.stop()
    steering.cleanup()
    cap.release()
    cv2.destroyAllWindows()
