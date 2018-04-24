# Code sourced and modified from http://opencv-python-tutroals.readthedocs.io and Geeks4Geeks
# All rights go to original creators
import cv2
import numpy as np 
 
# Webcamera no 0 is used to capture the frames
cap = cv2.VideoCapture(0) 
 
# This drives the program into an infinite loop.
while(True):       
	# Captures the live stream frame-by-frame
	_, frame = cap.read() 
	# Converts images from BGR to HSV
	hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
	lower_red = np.array([110,50,50])
	upper_red = np.array([200,255,255])
 
	# Here we are defining range of bluecolor in HSV
	# This creates a mask of blue coloured 
	# objects found in the frame.
	mask = cv2.inRange(hsv, lower_red, upper_red)
	# The bitwise and of the frame and mask is done so 
	# that only the blue coloured objects are highlighted 
	# and stored in res
	res = cv2.bitwise_and(frame,frame, mask= mask)
	cv2.imshow('frame',frame)
	cv2.imshow('mask',mask)
	cv2.imshow('res',res)
	
	# This displays the frame, mask 
	# and res which we created in 3 separate windows.
	# Quit if user presses 'q'
	if cv2.waitKey(1) & 0xFF == ord('q'):
		break
 
# Destroys all of the HighGUI windows.
cv2.destroyAllWindows()
 
# release the captured frame
cap.release()
