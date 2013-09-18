#!/usr/bin/env python
import cv2.cv as cv

dims = (9, 6) 					# 9x6 chessboard
boards = 8						# number of boards to be collected
npoints = dims[0] * dims[1]		# Number of points on chessboard
capture = cv.CaptureFromCAM(1)	# Capture an image
primaryWarning = False			# If the first warning has been issued
warningDisplayed = False		# If the normal warning has been issued
successes = 0					# Number of successful collections
intrinsics = cv.CreateMat(3, 3, cv.CV_64FC1)
distortion = cv.CreateMat(4, 1, cv.CV_64FC1)

# Make a general-purpose frame
cv.NamedWindow("Calibration", cv.CV_WINDOW_AUTOSIZE)

# Load the blank and checkmark to display later
blank_path = "images/blank.png"
blank_img = cv.LoadImage(blank_path)
cv.ShowImage("Calibration", blank_img)
cv.WaitKey(75)

while successes < boards:
	# Get a frame while we have less than 8 successes
	frame = cv.QueryFrame(capture)
	# Attempt to find the chessboard corners
	found, points=cv.FindChessboardCorners(frame,dims,cv.CV_CALIB_CB_ADAPTIVE_THRESH)
	if found != 0:
		successes = successes + 1
	### Jason,
	### This is the portion that we do not understand
	### Some of it is syntactically incorrect, and we could not
	### correct it because we do not understand what is supposed
	### to be happening here. I have commented in our best guesses
	### 	source: http://www.neuroforge.co.uk/index.php/camera-calibration

		# Model points
		#opts = cv.CreateMat(nimages * npoints, 3, cv.CV_32FC1)
		# Puts all model points into matrix
		#ipts = cv.CreateMat(opts * (npoints*1.0), 2, cv.CV_32FC1)
		# Unknown
		#npts = cv.CreateMat(nimages, 1, cv.CV_32SC1)

		#cv.SetZero(intrinsics)
		#cv.SetZero(distortion)

		#cv.SetZero(intrinsics2)
		#cv.SetZero(distortion2)
		
		#size=cv.GetSize(frame)
		#cv.CalibrateCamera2(opts, ipts, npts, size,intrinsics, distortion, cv.CreateMat(points), 3, cv.CV_32FC1,cv.CreateMat(lenpoints), 3, cv.CV_32FC1,flags = 0)

		#mapx = cv.CreateImage((mat_w,mat_h), cv.IPL_DEPTH_32F, 1)
		#mapy = cv.CreateImage((mat_w,mat_h), cv.IPL_DEPTH_32F, 1)
		#cv.InitUndistortMap(intrinsics, distortion, mapx, mapy)
		#r = cv.CloneImage(img)
		#cv.Remap(img, r, mapx, mapy)

		cv.DrawChessboardCorners(frame, dims, points, found)
		cv.ShowImage("Calibration", frame)
		cv.WaitKey(75)

		warningDisplayed = False
	else:
		if primaryWarning == False:
			primaryWarning = True
			print("Checkerboard not found (yet)")
			warningDisplayed = True

		elif warningDisplayed == True:
			pass

		else:
			print("Checkerboard lost")
			warningDisplayed = True
		cv.ShowImage("Calibration", blank_img)
		cv.WaitKey(75)

cv.DestroyWindow("Calibration")

print("Calibration complete")


