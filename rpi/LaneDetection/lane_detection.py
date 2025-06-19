import cv2
import numpy as np
import imutils
import utils
import colours

def getLane(img, colour, name):
    
    imgThresh = utils.thresholding(img, colour)
    cv2.imshow(name, imgThresh)

    return None
