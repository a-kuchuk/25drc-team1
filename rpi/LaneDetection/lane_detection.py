import cv2
import numpy as np
import imutils
import utils
import colours

def getLane(img, colour, name):
    """generates "lane" based on an isolated colour parameter

    Args:
        img (img): image being analysed
        colour (class): taarget colour as a class)
        name (string): name desplayed on popup window

    Returns:
        None
    """
    return utils.thresholding(img, colour)
    # cv2.imshow(name, imgThresh)