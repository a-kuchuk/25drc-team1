import cv2
import numpy as np
from Control import *
from LaneDetection.lane_detection import *
from ObjectDetection import *
import utils
import colours

def main():
    '''
    main function
    current steps:
        - read image
        - resize image
        - warp image
    '''

    success, img = cap.read()
    img = cv2.resize(img, (480, 240))
    cv2.imshow('vid', img)
    cv2.waitKey(1)

    # IMAGE WARPING STEP
    h, w, c = img.shape
    points = utils.trackbar_val()
    img_warp = utils.img_warp(img, points, w, h)
    cv2.imshow('warp', img_warp)

    # LANE DETECTION STEP
    leftLane = getLane(img_warp, colours.TapeBlue, "left")
    rightLane = getLane(img_warp, colours.TapeYellow, "right")

    # OVJECT DETECTION (might need to be modified)
    leftLane = getLane(img_warp, colours.HighlighterPink, "object")

    # ARROW DETECTION STEP
    
    

if __name__ == '__main__':
    cap = cv2.VideoCapture(0)
    init_trackbar_vals = [93, 188, 000, 240]
    utils.trackbar_init(init_trackbar_vals)
    while True:
        main()
    cap.release()
    cv2.destroyAllWindows()