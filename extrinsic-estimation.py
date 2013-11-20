#!/usr/bin/env python
import numpy as np
import cv2, time, sys
from numpy import linalg

dims = (9, 6) 					# 9x6 chessboard (see images directory)
boards = 1					# number of boards to be collected
npoints = dims[0] * dims[1]		        # Number of points on chessboard
successes = 0					# Number of successful collections

# calculates the nullspace of an inputted matrix
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

# lists to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

# rotation and translation vectors
rvec = []
tvec = []

# calibration output, loaded from file
mtx = np.loadtxt("output/intrinsics" + str(dims[0])+"x"+str(dims[1]) + ".txt")
dst = np.loadtxt("output/distortion" + str(dims[0])+"x"+str(dims[1]) + ".txt")

# from here, the process is essentially the same as calibrate.py
# comments are ommitted for that reason
print("Preparing to calculate camera extrinsics...")
time.sleep(1)

print("Press the spacebar to collect an image")

cv2.namedWindow('Calculate Extrinsics')

while True and successes != boards:
	k = cv2.waitKey()
	capture = None

	if k%256 == 32:
		capture = cv2.VideoCapture(1)
		_, image = capture.read()
		gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
		ret, corners = cv2.findChessboardCorners(gray, dims)

		if ret:
			print("Found chessboard")
			objpoints.append(objp)

			cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
			imgpoints.append(corners)

			cv2.drawChessboardCorners(image, dims, corners, True)
 			cv2.imshow('Calibration', image)
			successes += 1
		else:
			cv2.imshow('Calculate Extrinsics', image)
	elif k%256 == 27:
		cv2.destroyAllWindows()
		sys.exit()	

cv2.destroyAllWindows()

# here, a new image matrix is created to match the shape of that which is created by solvePnp
imgpoints2 = np.zeros(shape=(npoints, 2))
for i in range(0, 54):
	imgpoints2[i] = imgpoints[0][i][0]

# solvePnp is called, and the output is appropriately stored
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

# calculate the nullspace of this resulting matrix, and call it the coordinates
nullspace = null(r_t)
coordinates = nullspace/nullspace[3]

# if all went as planned, print the output
if ret:
	print("\n" + "Rotation")
	print(rvec)
	print("\n" + "Translation")
	print(tvec)
	print("\n" + "Coordinates:")
	print("x: \t" + str(coordinates[0]))
	print("y: \t" + str(coordinates[1]))
	print("z: \t" + str(coordinates[2]))
# otehrwise, say so
else:
	print("\n" + "Failure")
