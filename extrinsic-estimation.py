#!/usr/bin/env python
import numpy as np
import cv2, time, sys
from numpy import linalg

dims = (9, 6) 					# 9x6 chessboard
boards = 1						# number of boards to be collected
npoints = dims[0] * dims[1]		# Number of points on chessboard
successes = 0					# Number of successful collections

def null(A, eps=1e-15):
    u, s, vh = np.linalg.svd(A)
    null_mask = (s <= eps)
    null_space = np.compress(null_mask, vh, axis=0)
    return np.transpose(null_space)

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((9*6,3), np.float32)
objp[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2)

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

# Rotation and translation vectors
rvec = []
tvec = []

# Calibration output
mtx = np.loadtxt("output/intrinsics" + str(dims[0])+"x"+str(dims[1]) + ".txt")
dst = np.loadtxt("output/distortion" + str(dims[0])+"x"+str(dims[1]) + ".txt")

print("Preparing to calculate camera extrinsics...")
time.sleep(1)

print("Press the spacebar to collect an image")

# Make a general-purpose frame
cv2.namedWindow('Calculate Extrinsics')

while True and successes != boards:
	k = cv2.waitKey()
	capture = None

	if k%256 == 32:
		# Capture an image
		capture = cv2.VideoCapture(1)
		# Get a frame while we have less than 8 successes
		_, image = capture.read()
		# Create a grayscale image
		gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
		# Attempt to find the chessboard corners
		ret, corners = cv2.findChessboardCorners(gray, dims)

		# If found, add object points, image points (after refining them)
		if ret:
			print("Found chessboard")
			objpoints.append(objp)

			cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
			imgpoints.append(corners)

	        # Draw and display the corners
			cv2.drawChessboardCorners(image, dims, corners, True)
 			cv2.imshow('Calibration', image)
			successes += 1
		else:
			cv2.imshow('Calculate Extrinsics', image)
	elif k%256 == 27:
		cv2.destroyAllWindows()
		sys.exit()	

cv2.destroyAllWindows()

# Create new imgpoints matrix to match shape required by solvePnP
imgpoints2 = np.zeros(shape=(npoints, 2))
for i in range(0, 54):
	imgpoints2[i] = imgpoints[0][i][0]

ret, rvec, tvec = cv2.solvePnP(np.array(objpoints[0]), imgpoints2, mtx, dst)

# Calculate rotation matrix and create [R|t]
rmtx = cv2.Rodrigues(rvec)[0]
r_t = np.zeros(shape=(4, 4))
for i in range(0, 4):
	for j in range (0, 4):
		if i < 3:
			if j < 3:
				r_t[i][j] = rmtx[i][j]
			else:
				r_t[i][j] = tvec[i]
		else:
			r_t[i][j] = 0

nullspace = null(r_t)
coordinates = nullspace/nullspace[3]

if ret:

	print("\n" + "Rotation")
	print(rvec)
	print("\n" + "Translation")
	print(tvec)
	print("\n" + "Coordinates:")
	print("x: \t" + str(coordinates[0]))
	print("y: \t" + str(coordinates[1]))
	print("z: \t" + str(coordinates[2]))
else:
	print("\n" + "Failure")
#Work on plotting points