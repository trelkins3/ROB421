# Code sourced and modified from http://opencv-python-tutroals.readthedocs.io and Geeks4Geeks
# All rights go to original creators
import cv2
import numpy as np

cap = cv2.VideoCapture(0)

while(True):
	# Take each frame
	ret, frame = cap.read()

    # Convert BGR to gray
	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	
	blur = cv2.bilateralFilter(gray,9,75,75)
	thresh = cv2.adaptiveThreshold(blur,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY_INV,11,2)

	kernel = np.ones((5,5),np.uint8)
	modThresh = cv2.erode(thresh,kernel,iterations = 1)
	modThresh = cv2.dilate(modThresh, kernel, iterations = 2)
	image,contours,hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	#opening = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
	
	frame = cv2.drawContours(frame, contours, -1, (0,255,0),3)
	
	cv2.imshow('frame',frame)
	cv2.imshow('erosion', modThresh)
	cv2.imshow('mask',image)

	k = cv2.waitKey(5) & 0xFF
	if k == 27:
		break
		
cv2.destroyAllWindows()
cap.release()