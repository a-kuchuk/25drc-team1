import cv2
import numpy as np

def split_image(image):
    """
    Split the input image into left and right halves
    Args:
        image: Input image in BGR format
    Returns:
        tuple: (left_half, right_half) of the image
    """
    height, width = image.shape[:2]
    mid_point = width // 2
    
    # Split the image into left and right halves
    left_half = image[:, :mid_point]
    right_half = image[:, mid_point:]
    
    return left_half, right_half

def detect_color_lane(image, lower_bound, upper_bound):
    """
    Detect lanes of a specific color using HSV thresholding
    Args:
        image: Input image in BGR format
        lower_bound: Lower HSV threshold values
        upper_bound: Upper HSV threshold values
    Returns:
        mask: Binary mask of detected lane
    """
    # Convert to HSV color space
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    # Create mask for color detection
    mask = cv2.inRange(hsv, lower_bound, upper_bound)
    
    return mask

def process_lanes(image):
    """
    Main function for lane detection processing
    Args:
        image: Input image in BGR format
    Returns:
        processed_image: Image with detected lanes
    """
    # Split the image
    left_half, right_half = split_image(image)
    
    # Define HSV ranges for yellow and blue
    # Note: These values are placeholders - use calibrate.py to find exact values
    yellow_lower = np.array([20, 100, 100])  # Placeholder for yellow lower bound
    yellow_upper = np.array([30, 255, 255])  # Placeholder for yellow upper bound
    
    blue_lower = np.array([100, 100, 100])   # Placeholder for blue lower bound
    blue_upper = np.array([130, 255, 255])   # Placeholder for blue upper bound
    
    # Detect yellow lane in left half
    yellow_mask = detect_color_lane(left_half, yellow_lower, yellow_upper)
    
    # Detect blue lane in right half
    blue_mask = detect_color_lane(right_half, blue_lower, blue_upper)
    
    # Apply masks to original image halves
    yellow_result = cv2.bitwise_and(left_half, left_half, mask=yellow_mask)
    blue_result = cv2.bitwise_and(right_half, right_half, mask=blue_mask)
    
    # Combine the results
    result = np.hstack((yellow_result, blue_result))
    
    return result

def main():
    """
    Main function for testing image splitting
    """
    cap = cv2.VideoCapture(0)
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
            
        # Split the image
        left_half, right_half = split_image(frame)
        
        # Combine halves side by side for visualization
        combined = np.hstack((left_half, right_half))
        
        # Show original and split images
        cv2.imshow('Original', frame)
        cv2.imshow('Split View', combined)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
