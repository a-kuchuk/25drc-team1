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
LOOKAHEAD_Y = 200

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
    img_warp = utils.img_warp(img, np.float32([(0, 94), (480, 94), (0, 159), (480, 159)]), w, h)
    # img_warp = utils.img_warp(img, points, w, h)
    # cv2.imshow('warp', img_warp)

    left_mask = getLane(img, left, "left")

    right_mask = getLane(img, right, "right")

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
    

    # Get centroid X positions
    left_x = utils.get_lane_centroid_x(left_mask[LOOKAHEAD_Y]) if left_mask is not None else None
    right_x = utils.get_lane_centroid_x(right_mask[LOOKAHEAD_Y]) if right_mask is not None else None

    frame_center = FRAME_WIDTH // 2
    THRESHOLD = 20

    if left_x is not None and right_x is not None:
        lane_center = (left_x + right_x) // 2
        error = lane_center - frame_center

        if error > THRESHOLD:
            print("Turn Right")
            # motor.right()
        elif error < -THRESHOLD:
            print("Turn Left")
            # motor.left()
        else:
            print("Go Straight")
            # motor.forward(BASE_SPEED)

    elif left_x is not None:
        # Only left lane detected → drive **right** until right lane appears
        print("Only left lane found — turning right")
        # motor.right()

    elif right_x is not None:
        # Only right lane detected → drive **left** until left lane appears
        print("Only right lane found — turning left")
        # motor.left()

    else:
        print("No lanes found — stopping")
        # motor.stop()

    
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
