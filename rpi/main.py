import cv2
import numpy as np
import time
# from Control import *
from LaneDetection.lane_detection import *
from ObjectDetection import *
import utils
from colours import *

# from Control.pid import PID
from Control.motor import Motor
from Control.steering import SteeringController

from ArrowDetection import *

# --- Constants ---
BASE_SPEED = 70
MIN_SPEED = 30
FRAME_WIDTH = 480
FRAME_HEIGHT = 240

# change to 0 when wartping?
LOOKAHEAD_Y = 100

# --- Initialize Controllers ---
# steering = SteeringController()
# motor = Motor(base_speed=BASE_SPEED, min_speed=MIN_SPEED)
# pid = PID(Kp=0.6, Ki=0.05, Kd=0.1)

left = TapeYellow()
right = TapeBlue()
purple = PaintPurple()
finish = TapeGreen

steering = SteeringController()
motor = Motor(BASE_SPEED, MIN_SPEED)

# arrow_mode_triggered = False  # To prevent repeated detection
# --- OpenCV Camera ---
# cap = cv2.VideoCapture(0)
#utils.trackbar_init([100, 103, 000, 240])

arrow_state = None  # global state to remember arrow direction
arrow_cooldown = 0

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
    global arrow_state, arrow_cooldown

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
    img_warp = utils.img_warp(img, np.float32([(0, 130), (480, 130), (0, 240), (480, 240)]), w, h)
    # img_warp = utils.img_warp(img, points, w, h)
    cv2.imshow('warp', img_warp)

    # Ensuring sees arrow only once
    if arrow_cooldown == 0 and arrow_state is None:
        direction = utils.detect_arrow_direction(img)
        if direction:
            arrow_state = direction
            arrow_cooldown = 30  # avoid re-detecting for 30 frames
            print(f"Arrow detected: {direction.upper()}")
    if arrow_cooldown > 0:
        arrow_cooldown -= 1

    # --- Lane Detection ---
    left_mask = getLane(img_warp, left, "left")
    right_mask = getLane(img_warp, right, "right")
    
    # --- Object Detection ---
    object_mask = getLane(img_warp, purple, "object")  # Define TapePurple in colours.py
    obj_x = utils.get_lane_centroid_x(object_mask[LOOKAHEAD_Y]) if object_mask is not None else None

    # --- Object Avoidance ---
    if obj_x is not None:
        if obj_x < FRAME_WIDTH // 2:
            print("Object on left — turning right to avoid")
            drive(30, BASE_SPEED, 0.5)
        else:
            print("Object on right — turning left to avoid")
            drive(-30,BASE_SPEED,0.5)
        return  # object avoidance overrides lane following
    
    # --- Turning Challenge ---
    if arrow_state == 'left':
        print("Following arrow left")
        drive(-30, BASE_SPEED, 1.5)
        return
    elif arrow_state == 'right':
        print("Following arrow right")
        drive(30,BASE_SPEED,1.5)
        return

    # # --- Finish Lane Detection & Execution ---
    fin_lane = getLane(img_warp, finish, "finish")
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
        print("reset to forward?")
        drive(timeout=2)
        return

    # Get centroid X positions
    left_x = utils.get_lane_centroid_x(left_mask[LOOKAHEAD_Y]) if left_mask is not None else None
    right_x = utils.get_lane_centroid_x(right_mask[LOOKAHEAD_Y]) if right_mask is not None else None
    frame_center = FRAME_WIDTH // 2
    THRESHOLD = 20

    # --- Bang Bang Control ---
    if left_x is not None and right_x is not None:
        lane_center = (left_x + right_x) // 2
        error = lane_center - frame_center

        if error > THRESHOLD:
            print("Slight Right")
            drive(steering_angle=10)

        elif error < -THRESHOLD:
            print("Slight Left")
            drive(steering_angle=-10)
        else:
            print("Go Straight")
            drive()

    elif left_x is not None:
        # Only left lane detected → drive **right** until right lane appears
        print("Only left lane found — turning right")       # Make full right turn
        drive(steering_angle=30)

    elif right_x is not None:
        # Only right lane detected → drive **left** until left lane appears
        print("Only right lane found — turning left")
        drive(steering_angle=-30)

    else:
        print("No lanes found. slight forward then stop")
        drive(speed=MIN_SPEED)

    cv2.waitKey(1)

if __name__ == '__main__':
    # ls /dev/video*
    try:
        cap = cv2.VideoCapture(0)
        init_trackbar_vals = [000, 157, 000, 155]
        utils.trackbar_init(init_trackbar_vals)
        while True:
            main()
    except KeyboardInterrupt:
        print("Interrupted by user.")
    finally:
        print("Stopping robot...")
        motor.cleanup()
        steering.cleanup()
        cap.release()
        cv2.destroyAllWindows()


def drive(steering_angle=0, speed=BASE_SPEED, timeout=0.5):
    steering.set_servo_angle(-steering_angle)
    motor.forward(speed)
    time.sleep(timeout)
