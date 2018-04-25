# Code by Donald "Trey" Elkins, 2018
# Sourced/Modified from Kyle Hounslow, Geeks4Geeks, and opencv-python-tutorials.com
import cv2
import numpy as np

cap = cv2.VideoCapture(0)

# Need code here somewhere to stop the frame feed until the robot stops moving?

while(True):
	# Take two consecutive frames
	grabbed, frame1 = cap.read()
	grabbed2, frame2 = cap.read()
	
	# Make sure frames were fetched
	if not (grabbed and grabbed2):
		break
	
	# Initial calculations
	gray1 = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)	
	gray2 = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)
	frameDelta = cv2.absdiff(gray1, gray2)
	
	# Thresholding
	ret,thresh = cv2.threshold(frameDelta, 20, 255, cv2.THRESH_BINARY)
	
	# Dunno about this code, might be kinda high profile
	blur = cv2.blur(thresh,(5,5))
	ret,thresh = cv2.threshold(blur, 20, 255, cv2.THRESH_BINARY)
	
	# Find contours
	_,contours,_ = cv2.findContours(thresh.copy(),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

	# Output contours to frame
	for c in contours:
		print cv2.contourArea(c)
		# Value below can be changed to generate more/less contours
		if (cv2.contourArea(c) > 1000):
			#(x, y, w, h) = cv2.boundingRect(c)
			#cv2.rectangle(frame1,(x,y),(x+w,y+h),(0,255,0),2)
			(x,y),radius = cv2.minEnclosingCircle(c)
			center = (int(x),int(y))
			radius = int(radius)
			cv2.circle(frame1,center,radius,(255,0,0),2)
			
	
	# Keep as many of these commented out as possible to prevent lag
	cv2.imshow('frame',frame1)
	#cv2.imshow('difference', frameDelta)
	#cv2.imshow('mask',thresh)

	key = cv2.waitKey(1) & 0xFF
	if key == ord("q"):
		break
		
cv2.destroyAllWindows()
cap.release()