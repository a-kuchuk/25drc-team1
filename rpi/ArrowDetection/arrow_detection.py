import cv2
import numpy as np

def detect_arrow_direction(frame):
    """
    Detects a black arrow in the image and determines if it's pointing left or right.
    Returns:
        - "left", "right", or None
    """

    # Step 1: Preprocess
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Step 2: Black mask in HSV
    lower_black = np.array([0, 0, 0])
    upper_black = np.array([180, 255, 50])
    mask = cv2.inRange(hsv, lower_black, upper_black)

    # Step 3: Morphology cleanup
    kernel = np.ones((5,5),np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.dilate(mask, kernel, iterations=1)

    # Step 4: Contour detection
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 1000:  # Filter small objects
            approx = cv2.approxPolyDP(cnt, 0.03 * cv2.arcLength(cnt, True), True)

            x, y, w, h = cv2.boundingRect(approx)
            arrow_roi = mask[y:y+h, x:x+w]

            # Step 5: Compute left vs right half black pixel count
            left_half = arrow_roi[:, :w//2]
            right_half = arrow_roi[:, w//2:]
            left_black = cv2.countNonZero(left_half)
            right_black = cv2.countNonZero(right_half)

            # Optional: draw debug
            cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 0), 2)
            cv2.putText(frame, f"L: {left_black} R: {right_black}", (x, y-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)

            if left_black > right_black * 1.3:
                return "left"
            elif right_black > left_black * 1.3:
                return "right"

    return None