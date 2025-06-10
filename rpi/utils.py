import cv2
import numpy as np

def img_warp(img, points, w, h):
    '''
    main warping function using transformation matrix built into cv2
    '''
    pts1 = np.float32(points)
    pts2 = np.float32([[0,0], [w,0], [0,h], [w,h]])
    matrix = cv2.getPerspectiveTransform(pts1, pts2)
    img_warp = cv2.warpPerspective(img, matrix, (w,h))
    return img_warp

def nothing(a):
    pass

def trackbar_init(initTVals, wt=480, ht=240):
    cv2.namedWindow("trackbars")
    cv2.resizeWindow("trackbars", 360, 240)
    cv2.createTrackbar("width top", "trackbars", initTVals[0], wt//2, nothing)
    cv2.createTrackbar("height top", "trackbars", initTVals[1], ht, nothing)
    cv2.createTrackbar("width bottom", "trackbars", initTVals[2], wt//2, nothing)
    cv2.createTrackbar("height bottom", "trackbars", initTVals[3], ht, nothing)

def trackbar_val(wt=480, ht=240):
    width_top = cv2.getTrackbarPos("width top", "trackbars")
    height_top = cv2.getTrackbarPos("height top", "trackbars")
    width_bottom = cv2.getTrackbarPos("width bottom", "trackbars")
    height_bottom = cv2.getTrackbarPos("height bottom", "trackbars")
    points = np.float32([(width_top, height_top), (wt - width_top, height_top), (width_bottom, height_bottom), (wt - width_bottom, height_bottom)])

    return points