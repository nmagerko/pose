#!/usr/bin/env python
import numpy as np
import cv2, time, sys

# Modified from: https://opencv-python-tutroals.readthedocs.org/en/latest/py_tutorials/py_calib3d/py_calibration/py_calibration.html
# Data checked with http://vision.in.tum.de/data/datasets/rgbd-dataset/file_formats

dims = (9, 6) 					# 9x6 chessboard (see images folder)
boards = 20				        # number of boards to be collected
npoints = dims[0] * dims[1]		        # number of points on chessboard
successes = 0					# number of successful collections

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((9*6,3), np.float32)
objp[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2)

# lists to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

# alert the user that collection is starting
print("Preparing to calculate camera intrinsics...")
time.sleep(1)

# collect images only when the user presses space
print("Press the spacebar to collect an image")

# make a general-purpose window to display output
cv2.namedWindow('Calibration')

# while the number of boards to be collected has not been reached
while successes != boards:
        # collect the key from the user
	k = cv2.waitKey()
	capture = None

        # if the spacebar was pressed
	if k%256 == 32:
                # note that the following process may be changed to work with libfreenect
                # in order to maintain consistancy
		# capture an image
		capture = cv2.VideoCapture(1)
		# get a frame while we have less than 8 successes
		_, image = capture.read()
		# create a grayscale image
		gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
		# attempt to find the chessboard corners
		ret, corners = cv2.findChessboardCorners(gray, dims)

		# if found, add object points, image points (after refining them)
		if ret:
                        # tell the user that the calibration object was found and add standard points to list
			print("Found frame {0}".format(successes+1))
			objpoints.append(objp)

                        # find where the points are on the calibration object, and append these points to appropriate list
			cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
			imgpoints.append(corners)

                        # draw and display the corners, saving the image and incrementing the success counter
			cv2.drawChessboardCorners(image, dims, corners, True)
			cv2.imwrite("output/calibration-images/calibration"+str(dims[0])+"x"+str(dims[1])+ "-"+str(successes+1)+".jpg", image)
			cv2.imshow('Calibration', image)
			successes += 1
		else:
                        # otherwise, simply show the image collected
			cv2.imshow('Calibration', image)
	elif k%256 == 27:
                # if the user presses escape, quit
		cv2.destroyAllWindows()
		sys.exit()	

# once collection has been completed, destroy the windows
cv2.destroyAllWindows()
print("All frames found. Starting calibration....")

# assign output to OpenCV's calibration function to the appropriate variables
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)

# using the image that was stored and original calibration information, calculate the optimal intrinsics matrix
img = cv2.imread("output/calibration-images/calibration"+str(dims[0])+"x"+str(dims[1])+"-1.jpg")
h,  w = img.shape[:2]
newcameramtx, roi=cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),1,(w,h))

# output the distortion and new intrinsics matrix to a file
np.savetxt("output/intrinsics" + str(dims[0])+"x"+str(dims[1]) + ".txt", newcameramtx)
np.savetxt("output/distortion" + str(dims[0])+"x"+str(dims[1]) + ".txt", dist)

# tell the user that the process was successful, and exit
print("Calibration complete. Intrinisics and distortion saved to output directory")
print("Exiting")
sys.exit()
