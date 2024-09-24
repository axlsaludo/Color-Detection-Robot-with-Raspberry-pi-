# ----------------------------------------------------
# Color Tresholder for HSV Color Calibration
# by Logic Laboratories
# ----------------------------------------------------
# To simply use the program run it in the terminal and 
# use the shortcut "q" to quit.
# ----------------------------------------------------
# This program is used to determine the HSV color and Calibrate 
# for the color detection robot. The program will display the
# original frame, mask, and result of the color detection.
# ----------------------------------------------------


import cv2
import numpy as np

# Callback function for trackbars
def nothing(x):
    pass

# Create a window
cv2.namedWindow('Threshold Adjuster')

# Create trackbars for HSV thresholds
cv2.createTrackbar('H min', 'Threshold Adjuster', 0, 179, nothing)
cv2.createTrackbar('H max', 'Threshold Adjuster', 179, 179, nothing)
cv2.createTrackbar('S min', 'Threshold Adjuster', 0, 255, nothing)
cv2.createTrackbar('S max', 'Threshold Adjuster', 255, 255, nothing)
cv2.createTrackbar('V min', 'Threshold Adjuster', 0, 255, nothing)
cv2.createTrackbar('V max', 'Threshold Adjuster', 255, 255, nothing)

# Start video capture from webcam
cap = cv2.VideoCapture(0)

# Infinite loop to keep the window open
while True:
    # Capture frame-by-frame
    ret, frame = cap.read()
    if not ret:
        print("Failed to capture image")
        break

    # Convert image to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Get the current positions of the trackbars
    h_min = cv2.getTrackbarPos('H min', 'Threshold Adjuster')
    h_max = cv2.getTrackbarPos('H max', 'Threshold Adjuster')
    s_min = cv2.getTrackbarPos('S min', 'Threshold Adjuster')
    s_max = cv2.getTrackbarPos('S max', 'Threshold Adjuster')
    v_min = cv2.getTrackbarPos('V min', 'Threshold Adjuster')
    v_max = cv2.getTrackbarPos('V max', 'Threshold Adjuster')

    # Set the threshold for HSV values
    lower_bound = np.array([h_min, s_min, v_min])
    upper_bound = np.array([h_max, s_max, v_max])

    # Create a mask and result
    mask = cv2.inRange(hsv, lower_bound, upper_bound)
    result = cv2.bitwise_and(frame, frame, mask=mask)

    # Display the original frame, mask, and result
    cv2.imshow('Original', frame)
    cv2.imshow('Mask', mask)
    cv2.imshow('Result', result)

    # Display current threshold values
    threshold_text = f'H min: {h_min}, H max: {h_max}, S min: {s_min}, S max: {s_max}, V min: {v_min}, V max: {v_max}'
    cv2.putText(frame, threshold_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

    # Break the loop when 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Export the thresholds to a text file
with open('color_thresholds.txt', 'w') as f:
    f.write(f'H min: {h_min}\n')
    f.write(f'H max: {h_max}\n')
    f.write(f'S min: {s_min}\n')
    f.write(f'S max: {s_max}\n')
    f.write(f'V min: {v_min}\n')
    f.write(f'V max: {v_max}\n')

# Release the capture and clean up
cap.release()
cv2.destroyAllWindows()
