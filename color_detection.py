import cv2
import numpy as np
import time

# create a video capture object to read from the camera
cap = cv2.VideoCapture(0)

# flag to indicate whether or not red has been found
red_found = False

# start time for measuring the 10-second interval
start_time = time.time()

while True:
    # read a frame from the camera
    ret, frame = cap.read()

    # convert the color space from BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # define a range of red color in HSV color space
    lower_red = np.array([0, 50, 50])
    upper_red = np.array([10, 255, 255])
    mask1 = cv2.inRange(hsv, lower_red, upper_red)

    lower_red = np.array([170, 50, 50])
    upper_red = np.array([180, 255, 255])
    mask2 = cv2.inRange(hsv, lower_red, upper_red)

    # combine the masks to get the final mask
    mask = cv2.bitwise_or(mask1, mask2)

    # find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # find the contour with the largest area
    max_area = 0
    max_contour = None
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > max_area:
            max_area = area
            max_contour = contour

    # if a red object is found
    if max_contour is not None:
        # calculate the center of the contour
        M = cv2.moments(max_contour)
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])

        if time.time() - start_time >= 10:
        # print the coordinates of the center
          print("Center of red object: ({}, {})".format(cx, cy))

        # check if the center of the object is in the middle of the frame
        
        if time.time() - start_time >= 10:
         if cx < frame.shape[1] / 2:
            print("Object is to the left of center")
         elif cx > frame.shape[1] / 2:
            print("Object is to the right of center")
         else:
            print("Object is at the center")

        # bitwise AND the mask with the frame to show the red areas only
        res = cv2.bitwise_and(frame, frame, mask=mask)

        # calculate the percentage of red pixels in the mask
        num_pixels = mask.shape[0] * mask.shape[1]
        num_red_pixels = cv2.countNonZero(mask)
        red_percentage = num_red_pixels / num_pixels * 100

        # check if any red pixel is present in the mask
        if red_percentage >= 70 and not red_found:
            print("70 percent is red now")
            red_found = True

    else:
        res = frame

    # check if 10 seconds have passed since the last print
    if time.time() - start_time >= 10:
        print("Current percentage of red: {:.2f}%".format(red_percentage))
        start_time = time.time()

    # display the resulting image
    cv2.imshow('Frame', res)

    # wait for key press
    key = cv2.waitKey(1)

    # exit if the 'q' key is pressed
    if key & 0xFF == ord('q'):
        break

# release the video capture object and close all windows
cap.release()
cv2.destroyAllWindows()
