import cv2
import numpy as np
import imutils
import colours
from main import LOOKAHEAD_Y, FRAME_HEIGHT

# LOOKAHEAD_Y = 150  # Y-coordinate to calculate steering target
# FRAME_HEIGHT = 240

def img_warp(img, points, w, h):
    """main warping function using transformation matrix built into cv2

    Args:
        img (image): image
        points (float32): warp parameters
        w (int): width
        h (int): height

    Returns:
        img_warp (image): warped image
    """

    pts1 = np.float32(points)
    pts2 = np.float32([[0,0], [w,0], [0,h], [w,h]])
    matrix = cv2.getPerspectiveTransform(pts1, pts2)
    img_warp = cv2.warpPerspective(img, matrix, (w,h))
    return img_warp

def nothing(a):
    pass

def trackbar_init(initTVals, wt=480, ht=240):
    """initialises trackbar window (called trackbars) with values width top/bottom and height top/bottom
    currently only used in perspective warp

    Args:
        initTVals (array): initial trackbar values
        wt (int, optional): width. Defaults to 480.
        ht (int, optional): height. Defaults to 240.
    """
    cv2.namedWindow("trackbars")
    cv2.resizeWindow("trackbars", 360, 240)
    cv2.createTrackbar("width top", "trackbars", initTVals[0], wt//2, nothing)
    cv2.createTrackbar("height top", "trackbars", initTVals[1], ht, nothing)
    cv2.createTrackbar("width bottom", "trackbars", initTVals[2], wt//2, nothing)
    cv2.createTrackbar("height bottom", "trackbars", initTVals[3], ht, nothing)

def trackbar_val(wt=480, ht=240):
    '''
    retrieves data from trackbars window
    currently only used in perspective warp
    '''
    width_top = cv2.getTrackbarPos("width top", "trackbars")
    height_top = cv2.getTrackbarPos("height top", "trackbars")
    width_bottom = cv2.getTrackbarPos("width bottom", "trackbars")
    height_bottom = cv2.getTrackbarPos("height bottom", "trackbars")
    points = np.float32([(width_top, height_top), (wt - width_top, height_top), 
                         (width_bottom, height_bottom), (wt - width_bottom, height_bottom)])

    # print("POINTS")
    # print(points)
    print(f"POINTS \n {width_top} \n {height_top} \n {width_bottom} \n {height_bottom}")
    return points

# def detect_arrow(img, arrow):


def thresholding(img, colour):
    imgHsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower = np.array([colour.h_min, colour.s_min, colour.v_min])
    upper = np.array([colour.h_max, colour.s_max, colour.v_max])
    mask = cv2.inRange(imgHsv, lower, upper)
    # print(f"in utils \n {testBlue}")
    height = mask.shape[0]
    top_cutoff = int(height / 3)
    mask[:top_cutoff, :] = 0  # Set top 1/3 to black

    return mask


def find_centroid(img):
    # convert image to greyscale
    grey = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # convert grey scale image to binary image
    ret, thresh = cv2.threshold
    return


# https://learnopencv.com/find-center-of-blob-centroid-using-opencv-cpp-python/
def get_lane_centroid_x(mask):
    M = cv2.moments(mask)
    if M["m00"] > 0:
        cx = int(M["m10"] / M["m00"])
        return cx
    return None

def get_lane_centroid_y(mask):
    M = cv2.moments(mask)
    if M["m00"] > 0:
        cy = int(M["01"] / M["00"])
        return cy
    return None

def get_lane_points(mask, step=20):
    """
    Sample centroids across multiple horizontal scanlines (y-axis slices)
    to collect (x, y) points representing the lane.
    """
    if mask is None:
        return []

    points = []
    for y in range(LOOKAHEAD_Y, FRAME_HEIGHT, step):
        row = mask[y, :]
        x_vals = np.where(row > 0)[0]
        if len(x_vals) > 0:
            cx = int(np.mean(x_vals))
            points.append((cx, y))
    return points

def fit_poly(points):
    """
    Fit a 2nd degree polynomial to a list of (x, y) points.
    Returns coefficients of the polynomial x = f(y).
    """
    if len(points) >= 3:
        x, y = zip(*points)
        return np.polyfit(y, x, 2)
    return None

def evaluate_poly(poly_coeffs, y):
    """
    Evaluate the polynomial at a specific y-value to get x.
    """
    return int(np.polyval(poly_coeffs, y))

def derivative_at(poly_coeffs, y):
    """
    Get the derivative (dx/dy) at a given y.
    Used to estimate the heading angle.
    """
    d = np.polyder(poly_coeffs)
    return np.polyval(d, y)
