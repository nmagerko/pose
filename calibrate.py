#!/usr/bin/env python
import cv2.cv as cv
import time, sys

dims = (4, 4) 					# 9x6 chessboard
boards = 20						# number of boards to be collected
npoints = dims[0] * dims[1]		# Number of points on chessboard
primaryWarning = False			# If the first warning has been issued
warningDisplayed = False		# If the normal warning has been issued
successes = 0					# Number of successful collections

# Points of object from 3D image
objectPoints = cv.CreateMat(boards*npoints, 3, cv.CV_32FC1)
# Points of object in 2D image
imagePoints = cv.CreateMat(boards*npoints, 2, cv.CV_32FC1)
# Points on the board
points = cv.CreateMat(boards, 1, cv.CV_32FC1)
# The 3x3 intrinsic matrix
intrinsics = cv.CreateMat(3, 3, cv.CV_32FC1)
# Distortion coefficient
distortionOutput = cv.CreateMat(5, 1, cv.CV_32FC1)

# Make a general-purpose frame
cv.NamedWindow("Calibration", cv.CV_WINDOW_AUTOSIZE)

while True and successes != boards:
	k = cv.WaitKey()
	capture = None
	if k%256 == 32:
		capture = cv.CaptureFromCAM(0)	# Capture an image
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
			if primaryWarning == False:
				primaryWarning = True
				print("Checkerboard not found (yet) \n")
				warningDisplayed = True

			elif warningDisplayed == True:
				pass

			else:
				print("Checkerboard lost")
				warningDisplayed = True
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

			warningDisplayed = False

	elif k%256 == 27:
		cv.DestroyWindow("Calibration")
		sys.exit()	

cv.DestroyWindow("Calibration")

print("All frames found. Starting calibration...")

# Prepare new matricies to assign to viewCount
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

for i in range(successes):
	cv.Set2D(points2, i, 0, cv.Get2D(points, i ,0))

cv.Set2D(intrinsics, 0, 0, 1.0)
cv.Set2D(intrinsics, 1, 1, 1.0)

# Rotation and translation
rcv = cv.CreateMat(boards, 3, cv.CV_32FC1)
tcv = cv.CreateMat(boards, 3, cv.CV_32FC1)

print("Finished. Checking camera calibration... (This might take a while) \n")

# Try to calibrate the camera
cv.CalibrateCamera2(objectPoints2, imagePoints2, points2, cv.GetSize(image), intrinsics, distortionOutput, rcv, tcv, flags=0)

print("OK - Saving...")

# Store results in XML files
cv.Save("output/intrinsics" + str(dims[0])+"x"+str(dims[1]) + ".xml", intrinsics)
cv.Save("output/distortion" + str(dims[0])+"x"+str(dims[1]) + ".xml", distortionOutput)

# Loading from XML files
intrinsic = cv.Load("output/intrinsics" + str(dims[0])+"x"+str(dims[1]) + ".xml")
distortion = cv.Load("output/distortion" + str(dims[0])+"x"+str(dims[1]) + ".xml")
print "Saved. Reloaded all distortion parameters successfully \n"

mapx = cv.CreateImage(cv.GetSize(image), cv.IPL_DEPTH_32F, 1 );
mapy = cv.CreateImage(cv.GetSize(image), cv.IPL_DEPTH_32F, 1 );
cv.InitUndistortMap(intrinsic,distortion,mapx,mapy)
cv.NamedWindow("Undistorted")

print("Mapping complete")
print("Please wait, camera switching on... \n")
time.sleep(1)

print("Camera online")
while(2):
	image=cv.QueryFrame(capture)
	t = cv.CloneImage(image);
	cv.ShowImage( "Original View", image )
	cv.Remap( t, image, mapx, mapy )
	cv.ShowImage("Undistorted View", image)
	c = cv.WaitKey(33)
	if (c == 1048688):		# Enter 'p' key to pause for some time
		cv.WaitKey(2000)
	elif c==1048603:		# Enter esc key to exit
		break

print("Program closed - calibration complete")


