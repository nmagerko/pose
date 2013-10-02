#!/usr/bin/env python
import cv2.cv as cv
import time, sys

dims = (9, 6) 					# 9x6 chessboard
boards = 20						# number of boards to be collected
npoints = dims[0] * dims[1]		# Number of points on chessboard
successes = 0					# Number of successful collections

# Points of object from 3D image
objectPoints = cv.CreateMat(boards*npoints, 3, cv.CV_32FC1)
# Points of object in 2D image
imagePoints = cv.CreateMat(boards*npoints, 2, cv.CV_32FC1)
# Points on the board
points = cv.CreateMat(boards, 1, cv.CV_32FC1)

#Load necessary data from file
intrinsics = cv.Load("output/intrinsics" + str(dims[0])+"x"+str(dims[1]) + ".xml")
distortion = cv.Load("output/distortion" + str(dims[0])+"x"+str(dims[1]) + ".xml")
print("Reloaded all parameters successfully. Preparing to calculate extrinsics... \n")
time.sleep(1)

print("Press the spacebar to collect an image")

# Make a general-purpose frame
cv.NamedWindow("Calibration", cv.CV_WINDOW_AUTOSIZE)

while True and successes != boards:
	k = cv.WaitKey()
	capture = None

	if k%256 == 32:
		# Capture an image
		capture = cv.CaptureFromCAM(0)
		# Get a frame while we have less than 8 successes
		image = cv.QueryFrame(capture)
		# Create a grayscale image
		grayImage = cv.CreateImage(cv.GetSize(image), 8, 1)
		cv.CvtColor(image, grayImage, cv.CV_BGR2GRAY)
		# Attempt to find the chessboard corners
		found, corners=cv.FindChessboardCorners(grayImage,dims,cv.CV_CALIB_CB_ADAPTIVE_THRESH)
		# Find the corners
		corners = cv.FindCornerSubPix(grayImage, corners, (11, 11), (-1,-1), (cv.CV_TERMCRIT_EPS+cv.CV_TERMCRIT_ITER,30,0.1))	
	
		# If the chessboard was not found
		if found == 0:
			cv.ShowImage("Calibration", image)
		else:
			# Display the image with the corners shown
			cv.DrawChessboardCorners(image, dims, corners, found)
			cv.SaveImage("output/calibration-images/calibration"+str(dims[0])+"x"+str(dims[1])+ "-"+str(successes+1)+".jpg", image)
			cv.ShowImage("Calibration", image)

			# Number of corners
			ncorners = len(corners)
		
			# If the amount of corners is correct (good image)
			if ncorners == npoints:
				print("Found frame {0}".format(successes+1))
				step = successes*npoints
				for j in range(npoints):
					# Assign points to their respective matrix
					cv.Set2D(imagePoints, step, 0, corners[j][0])
					cv.Set2D(imagePoints, step, 1, corners[j][1])
					cv.Set2D(objectPoints, step, 0, float(j)/float(dims[0]))
					cv.Set2D(objectPoints, step, 1, float(j)%float(dims[0]))
					cv.Set2D(objectPoints, step, 2, 0.0)
					step = step + 1
				cv.Set2D(points, successes, 0, npoints)
				successes = successes + 1	

	elif k%256 == 27:
		cv.DestroyWindow("Calibration")
		sys.exit()	

cv.DestroyWindow("Calibration")

# Prepare new matricies 
objectPoints2 = cv.CreateMat(successes*npoints, 3, cv.CV_32FC1)
imagePoints2 = cv.CreateMat(successes*npoints, 2, cv.CV_32FC1)
points2 = cv.CreateMat(successes, 1, cv.CV_32SC1)

# Assign points to their respective matrix
for i in range(successes*npoints):
	cv.Set2D(imagePoints2, i, 0, cv.Get2D(imagePoints, i, 0))
	cv.Set2D(imagePoints2, i, 1, cv.Get2D(imagePoints, i, 1))
	cv.Set2D(objectPoints2, i, 0, cv.Get2D(objectPoints, i, 0))
	cv.Set2D(objectPoints2, i, 1, cv.Get2D(objectPoints, i, 1))
	cv.Set2D(objectPoints2, i, 2, cv.Get2D(objectPoints, i, 2))

# Rotation and translation
rvec = cv.CreateMat(3, 1, cv.CV_32FC1)
tvec = cv.CreateMat(3, 1, cv.CV_32FC1)

cv.FindExtrinsicCameraParams2(objectPoints2, imagePoints2, intrinsics, distortion, rvec, tvec)

cv.Save("rvec.xml", rvec)
cv.Save("tvec.xml", tvec)

#Work on plotting points


