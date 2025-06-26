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
    # print(f"POINTS \n {width_top} \n {height_top} \n {width_bottom} \n {height_bottom}")
    return points

# def detect_arrow(img, arrow):


def thresholding(img, colour):
    imgHsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower = np.array([colour.h_min, colour.s_min, colour.v_min])
    upper = np.array([colour.h_max, colour.s_max, colour.v_max])
    mask = cv2.inRange(imgHsv, lower, upper)

    return mask

import numpy as np

def get_highest_lane_y(mask):
    """Returns the min (highest on screen) Y coordinate of lane pixels, with outliers removed."""
    # Get all y-coordinates of white pixels
    coords = np.column_stack(np.where(mask == 255))
    
    if coords.size == 0:
        return None

    y_coords = coords[:, 0]  # rows are y, cols are x

    # IQR filtering to remove outliers
    q1 = np.percentile(y_coords, 25)
    q3 = np.percentile(y_coords, 75)
    iqr = q3 - q1
    lower_bound = q1 - 1.5 * iqr
    upper_bound = q3 + 1.5 * iqr

    filtered_y = y_coords[(y_coords >= lower_bound) & (y_coords <= upper_bound)]
    
    if filtered_y.size == 0:
        return None

    return int(np.min(filtered_y))  # highest point on screen



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

def get_lane_points(mask,highest_y, step=20):
    """
    Sample centroids across multiple horizontal scanlines (y-axis slices)
    to collect (x, y) points representing the lane.
    """
    if mask is None:
        return []

    points = []

    # Scaning upwards toward top of image 
    # for y in range(LOOKAHEAD_Y - step, 100, -step):
    #     row = mask[y, :]
    #     x_vals = np.where(row > 0)[0]
    #     if len(x_vals) > 0:
    #         cx = int(np.mean(x_vals))
    #         points.append((cx, y))

    # # Scanning points from lookahead_y down to bottom of image - increasing y coord 
    # for y in range(LOOKAHEAD_Y, FRAME_HEIGHT-step, step):
    #     row = mask[y, :]                                # Takes horizontal row at height y 
    #     x_vals = np.where(row > 0)[0]                   # Find indices, x coords, where mask is nonzero
    #     if len(x_vals) > 0:
    #         cx = int(np.mean(x_vals))
    #         points.append((cx, y))

    y_start = min(highest_y, LOOKAHEAD_Y)
    y_end = max(highest_y, LOOKAHEAD_Y)

    for y in range (y_start, y_end, step):
        row = mask[y, :]                                # Takes horizontal row at height y 
        x_vals = np.where(row > 0)[0]                   # Find indices, x coords, where mask is nonzero
        if len(x_vals) > 0:
            cx = int(np.mean(x_vals))
            points.append((cx, y))
    return points

def fit_poly(points):
    """
    Fit a 3rd degree polynomial to a list of (x, y) points.
    Returns coefficients of the polynomial x = f(y).
    """
    if len(points) >= 4:        # Need at least 4 points for cubic
        x, y = zip(*points)
        return np.polyfit(y, x, 3)
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


def draw_fitted_curve(img, poly, y_start, y_end, color):
    if poly is None:
        return

    points = []
    for y in range(y_start, y_end, 2):  # smooth curve with small step
        x = evaluate_poly(poly, y)
        points.append([x, y])

    # Convert to proper shape for cv2.polylines
    points = np.array(points, dtype=np.int32).reshape((-1, 1, 2))
    cv2.polylines(img, [points], isClosed=False, color=color, thickness=2)

def display_debug(img, left_poly, right_poly, lateral_error, heading_error, lookahead_y, left_highest_y, right_highest_y):
    debug_img = img.copy()
    h, w, _ = img.shape

    # Draw fitted polynomials as points
    # for y in range(lookahead_y, h, 5):
    #     if left_poly is not None:
    #         lx = int(evaluate_poly(left_poly, y))
    #         cv2.circle(debug_img, (lx, y), 3, (0, 255, 255), -1)  # yellow

    #     if right_poly is not None:
    #         rx = int(evaluate_poly(right_poly, y))
    #         cv2.circle(debug_img, (rx, y), 3, (255, 0, 0), -1)  # blue

    # for y in range(left_highest_y, lookahead_y, 5):
    #     if left_poly is not None:
    #         lx = int(evaluate_poly(left_poly, y))
    #         cv2.circle(debug_img, (lx, y), 3, (0, 255, 255), -1)  # yellow

    # for y in range(right_highest_y, lookahead_y, 5):
    #     if right_poly is not None:
    #         rx = int(evaluate_poly(right_poly, y))
    #         cv2.circle(debug_img, (rx, y), 3, (255, 0, 0), -1)  # blue

    # Draw smooth polynomial curves using cv2.polylines
    draw_fitted_curve(debug_img, left_poly, left_highest_y, lookahead_y, (0, 255, 255))  # Yellow
    draw_fitted_curve(debug_img, right_poly, right_highest_y, lookahead_y, (255, 0, 0))  # Blue

    # Draw heading vector (if both polynomials are valid)
    if left_poly is not None and right_poly is not None:
        left_x = evaluate_poly(left_poly, lookahead_y)
        right_x = evaluate_poly(right_poly, lookahead_y)
        center_x = int((left_x + right_x) // 2)
        center_y = lookahead_y

        heading_gradient = (derivative_at(left_poly, lookahead_y) + 
                            derivative_at(right_poly, lookahead_y)) / 2
        angle_rad = np.arctan(heading_gradient)

        direction_len = 40      # length of red arrow
        dx = int(direction_len * np.sin(angle_rad))     # horizontal comp of arrow
        dy = int(direction_len * np.cos(angle_rad))     # vertical comp of arrow

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


# Old code from pervious team modified to output polynomial fitting of lane 
def detectLinePoly(img, pixel_count=50, min_points=10):
    # grey = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    # if cv2.countNonZero(grey) < pixel_count:
    #     return None, []

    edge_image = cv2.Canny(img, 200, 250)
    contours, _ = cv2.findContours(edge_image, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) == 0:
        return None, []

    # Choose longest contour
    lengths = [len(c) for c in contours]
    j = np.argmax(lengths)
    line = contours[j].reshape(-1, 2)

    # Sort points by y (top to bottom)
    line = line[line[:, 1].argsort()]

    if len(line) < min_points:
        return None, []

    return line

