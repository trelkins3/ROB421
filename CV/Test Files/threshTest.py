# Code sourced and modified from http://opencv-python-tutroals.readthedocs.io and Geeks4Geeks
# All rights go to original creators
import cv2
import numpy as np


cap = cv2.VideoCapture(0)

while(True):
	# Take each frame
	ret, frame = cap.read()

    # Convert BGR to HSV
	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	
	# define range of blue color in HSV
	_,thresh = cv2.threshold(gray,180,255,cv2.THRESH_BINARY)
	
	# bilateral seems to have less noise but the edges aren't as sharp...
	# gaussian has more noise but sharper edges
	blur = cv2.bilateralFilter(gray,9,75,75)
	#blur = cv2.GaussianBlur(gray,(5,5),0)
	
	# same goes with mean and gaussian
	thresh2 = cv2.adaptiveThreshold(blur,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY_INV,11,2)
	ret3,thresh3 = cv2.threshold(blur,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)

	#cv2.imshow('frame',frame)
	#cv2.imshow('mask',thresh)
	cv2.imshow('thresh2',thresh2)
	cv2.imshow('thresh3',thresh3)
	#cv2.imshow('thresh3',thresh3)
	k = cv2.waitKey(5) & 0xFF
	if k == 27:
		break
		
cv2.destroyAllWindows()
cap.release()