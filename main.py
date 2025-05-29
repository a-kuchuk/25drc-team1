import cv2
import numpy as np
from Control import *
from LaneDetection import *
from ObjectDetection import *

def main():
    success, img = cap.read()
    cv2.imshow('vid', img)
    cv2.waitKey(1)
    

if __name__ == '__main__':
    cap = cv2.VideoCapture(0)
    while True:
        main()