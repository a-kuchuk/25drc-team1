import cv2
import numpy as np
from Control import *
from LaneDetection import *
from ObjectDetection import *
import utils

def main():
    success, img = cap.read()
    img = cv2.resize(img, (480, 240))
    cv2.imshow('vid', img)
    cv2.waitKey(1)

    # IMAGE WARPING STEP
    h, w, c = img.shape
    points = utils.trackbar_val()
    img_warp = utils.img_warp(img, points, w, h)
    cv2.imshow('warp', img_warp)
    

if __name__ == '__main__':
    cap = cv2.VideoCapture(0)
    init_trackbar_vals = [171, 103, 000, 240]
    utils.trackbar_init(init_trackbar_vals)
    while True:
        main()