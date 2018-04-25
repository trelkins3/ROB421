# NEXT STEPS:
# 1. Implement serial transmission - transmit the byte package and leave encoder feedback open
# 2. Figure out byte package data - 8 bits, 4 for sides, 4 for corners?
# 3. Streamline camera detection and circle generation
# 4. Comb over code to fix any glaring performance issues
# 5. Encoder data integration

import cv2
import numpy

cap = cv2.VideoCapture(0)

# Streamline with window size read code, otherwise use this pattern to make a new array
borderContours = [numpy.array([[0,0],[0,750],[200,750],[200,0]], dtype=numpy.int32),\
			numpy.array([[1280,0],[1280,750],[1080,750],[1080,0]], dtype=numpy.int32),\
			numpy.array([[0,0],[0,150],[1280,150],[1280,0]], dtype=numpy.int32),\
			numpy.array([[0,550],[0,750],[1280,750],[1280,550]], dtype=numpy.int32)]
			
while(True):
	# Take two consecutive frames
	grabbed, frame1 = cap.read()
	grabbed2, frame2 = cap.read()
	
	for b in borderContours:
		(x, y, w, h) = cv2.boundingRect(b)
		cv2.rectangle(frame1,(x,y),(x+w,y+h),(0,255,0),2)
		
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
		#print cv2.contourArea(c)
		# Value below can be changed to generate more/less contours
		if ((cv2.contourArea(c) > 1000) and (cv2.contourArea(c) < 3000)):
			(x,y),radius = cv2.minEnclosingCircle(c)
			center = (int(x),int(y))
			radius = int(radius)
			# Create the circle on the screen if it falls within an area that justifies robot movement
			for b in borderContours:
				if (cv2.pointPolygonTest(b, (x,y), False) == 1.0):
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