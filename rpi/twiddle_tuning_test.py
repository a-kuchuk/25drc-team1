import cv2
import numpy as np
import time
from Control.pid import PID
# from Control.motor import Motor
# from Control.steering import SteeringController
from LaneDetection.lane_detection import getLane
from colours import TapeYellow, TapeBlue, TapeGreen
import utils

# Constants
BASE_SPEED = 70
MIN_SPEED = 30
FRAME_WIDTH = 480
FRAME_HEIGHT = 240
LOOKAHEAD_Y = 150
MAX_FRAMES = 100  # How many frames per trial
USE_VISUAL_DEBUG = True

# Lane colours
left = TapeYellow()
right = TapeBlue()
finish = TapeGreen

cap = cv2.VideoCapture(0)
utils.trackbar_init([0, 157, 0, 155])

# --- Error Evaluation Function ---
def run_episode(p):
    pid = PID(*p)
    total_error = 0
    frame_count = 0

    while frame_count < MAX_FRAMES:
        success, img = cap.read()
        if not success:
            break

        img = cv2.resize(img, (FRAME_WIDTH, FRAME_HEIGHT))
        h, w, _ = img.shape
        img_warp = utils.img_warp(img, np.float32([(0, 94), (480, 94), (0, 159), (480, 159)]), w, h)

        left_mask = getLane(img_warp, left, "left")
        right_mask = getLane(img_warp, right, "right")

        left_points = utils.get_lane_points(left_mask)
        right_points = utils.get_lane_points(right_mask)

        left_poly = utils.fit_poly(left_points)
        right_poly = utils.fit_poly(right_points)

        if left_poly is not None and right_poly is not None:
            left_x = utils.evaluate_poly(left_poly, LOOKAHEAD_Y)
            right_x = utils.evaluate_poly(right_poly, LOOKAHEAD_Y)
            lane_center = (left_x + right_x) // 2

            lateral_error = (FRAME_WIDTH // 2) - lane_center
            heading_gradient = (utils.derivative_at(left_poly, LOOKAHEAD_Y) + 
                                utils.derivative_at(right_poly, LOOKAHEAD_Y)) / 2
            heading_error = np.degrees(np.arctan(heading_gradient))

            error = lateral_error + heading_error
            _ = pid.compute(error)  # PID output not used for motion here
            total_error += abs(error)

            if USE_VISUAL_DEBUG:
                display_debug(img_warp, left_poly, right_poly, lateral_error, heading_error, LOOKAHEAD_Y)

        frame_count += 1
        cv2.waitKey(1)
    print(f"frame_count = {frame_count}")
    return total_error / max(1, frame_count)

# --- Twiddle Optimization ---
def twiddle(tol=0.01):
    p = [0.2, 0.0, 0.0]  # Kp, Ki, Kd
    dp = [0.1, 0.01, 0.01]
    best_err = run_episode(p)
    print(f"Initial error: {best_err:.2f}")

    while sum(dp) > tol:
        print('inside')
        for i in range(3):
            print(f"in for loop {i}")
            p[i] += dp[i]
            err = run_episode(p)

            if err < best_err:
                best_err = err
                dp[i] *= 1.1
            else:
                p[i] -= 2 * dp[i]
                err = run_episode(p)

                if err < best_err:
                    best_err = err
                    dp[i] *= 1.05
                else:
                    p[i] += dp[i]
                    dp[i] *= 0.95

            print(f"PID: {p}, Error: {err:.2f}, Best: {best_err:.2f}, Steps: {dp}")
    print("befor ereturn")
    return p

# --- Debug Drawing ---
def display_debug(img, left_poly, right_poly, lateral_error, heading_error, lookahead_y):
    debug_img = img.copy()
    h, w, _ = img.shape

    for y in range(lookahead_y, h, 5):
        if left_poly is not None:
            lx = int(utils.evaluate_poly(left_poly, y))
            cv2.circle(debug_img, (lx, y), 2, (0, 255, 255), -1)

        if right_poly is not None:
            rx = int(utils.evaluate_poly(right_poly, y))
            cv2.circle(debug_img, (rx, y), 2, (255, 0, 0), -1)

    if left_poly is not None and right_poly is not None:
        left_x = utils.evaluate_poly(left_poly, lookahead_y)
        right_x = utils.evaluate_poly(right_poly, lookahead_y)
        center_x = int((left_x + right_x) // 2)
        heading_gradient = (utils.derivative_at(left_poly, lookahead_y) + 
                            utils.derivative_at(right_poly, lookahead_y)) / 2
        angle_rad = np.arctan(heading_gradient)

        dx = int(40 * np.cos(angle_rad))
        dy = int(40 * np.sin(angle_rad))

        cv2.arrowedLine(debug_img, (center_x, lookahead_y),
                        (center_x + dx, lookahead_y - dy), (0, 0, 255), 2)

        cv2.line(debug_img, (w // 2, h), (center_x, lookahead_y), (0, 255, 0), 2)

    cv2.putText(debug_img, f"Lateral Error: {lateral_error}", (10, 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    cv2.putText(debug_img, f"Heading Error: {heading_error:.2f}", (10, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    cv2.imshow("Twiddle Debug", debug_img)
    cv2.imshow("Og Debug", img)

# --- Run the test ---
if __name__ == "__main__":
    try:
        best_pid = twiddle()
        print(f"\n✅ Best PID gains found: Kp={best_pid[0]:.3f}, Ki={best_pid[1]:.3f}, Kd={best_pid[2]:.3f}")
    finally:
        cap.release()
        cv2.destroyAllWindows()