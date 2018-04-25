# Code sourced and modified from http://opencv-python-tutroals.readthedocs.io and Geeks4Geeks
# All rights go to original creators
import cv2
import numpy as np

cap = cv2.VideoCapture(0)

while(True):

   # Take each frame
    ret, frame = cap.read()

    # Convert BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # define range of blue color in HSV
	# 0,0,0 and 220, 64, 255
    lower_red = np.array([0,0,150], dtype=np.uint8)
    upper_red = np.array([220,40,255], dtype=np.uint8)
	
    # Threshold the HSV image to get only blue colors
    mask = cv2.inRange(hsv, lower_red, upper_red)

    # Bitwise-AND mask and original image
    res = cv2.bitwise_and(frame,frame, mask= mask)

    cv2.imshow('frame',frame)
    cv2.imshow('mask',mask)
    cv2.imshow('res',res)
    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break

cv2.destroyAllWindows()
cap.release()