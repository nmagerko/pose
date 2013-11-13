import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from math import tan
import cv2
import freenect
import time
import pickle

dims = (9, 6) 					# 9x6 chessboard
boards = 1						# number of boards to be collected
npoints = dims[0] * dims[1]		# Number of points on chessboard

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

fig = plt.figure()
clouds = []

# Calibration output
mtx = np.loadtxt("output/intrinsics" + str(dims[0])+"x"+str(dims[1]) + ".txt")
dst = np.loadtxt("output/distortion" + str(dims[0])+"x"+str(dims[1]) + ".txt")

def CalculateRT():
	successes = 0					# Number of successful collections
	while True and successes != boards:
		capture = None

		# Capture an image
		(image, _) = freenect.sync_get_video()
		# Conver to grayscale
		gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
		# Attempt to find the chessboard corners
		ret, corners = cv2.findChessboardCorners(gray, dims)

		# If found, add object points, image points (after refining them)
		if ret:
			objpoints.append(objp)

			cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
			imgpoints.append(corners)
			successes += 1

	# Create new imgpoints matrix to match shape required by solvePnP
	imgpoints2 = np.zeros(shape=(npoints, 2))
	for i in range(0, 54):
		imgpoints2[i] = imgpoints[0][i][0]

	ret, rvec, tvec = cv2.solvePnP(np.array(objpoints[0]), imgpoints2, mtx, dst)

	# Calculate rotation matrix and create [R|t]
	rmtx = cv2.Rodrigues(rvec)[0]

	return rmtx, tvec

def RawDepthToMeters(depthValue):
	# http://pille.iwr.uni-heidelberg.de/~kinect01/doc/classdescription.html#kinectcloud-section
	return 1/(depthValue * (-0.0030711016) + 3.3309495161)

def DepthToWorld(x, y, depthValue):

	fx_d = mtx[0,0]
	fy_d = mtx[1,1]
	cx_d = mtx[0,2]
	cy_d = mtx[1,2]

	depth = RawDepthToMeters(depthValue)
	# http://pille.iwr.uni-heidelberg.de/~kinect01/doc/classdescription.html#kinectcloud-section
	resultZ = float(depth)
	resultX = float((x - cx_d) * resultZ)/fx_d
	resultY = float((y - cy_d) * resultZ)/fy_d
	
	result = [resultX, resultY, resultZ]
	return result

def GenerateCloud():
	print "Getting depth...",
	(depth,_) = freenect.sync_get_depth()
	print " \t \t \t OK"
	print "Waiting for calibration object..."
	rotate, translate = CalculateRT()
	print "Rotation and translation calculated"
	# Collects 12288 points/frame
	worldCoordinates = np.arange(36864, dtype = np.float64).reshape(96, 128, 3)
	for i in range(0, 480, 5):
		for j in range(0, 640, 5):
			depthValue = depth[i,j]
			if depthValue < 2047:
				values = DepthToWorld(i, j, depthValue)
				worldCoordinates[i/5, j/5, 0] = values[0]
				worldCoordinates[i/5, j/5, 1] = values[1]
				worldCoordinates[i/5, j/5, 2] = values[2]
			else:
				worldCoordinates[i/5, j/5, 0] = 0
				worldCoordinates[i/5, j/5, 1] = 0
				worldCoordinates[i/5, j/5, 2] = 0
	x, y, z = [], [], []
	for row in worldCoordinates:
		for point in row:
			if str(point) != "[ 0.  0.  0.]":
				point = rotate.dot(np.array(point).reshape(3, 1)) + translate
				x.append(float(point[0]))
				y.append(float(point[1]))
				z.append(float(point[2]))

	clouds.append([x,y,z])

print "\n" + "KINECT ONLINE \n"
for i in range(0, 1):
	if i == 0:
		raw_input("\n" + "Press any key to the LAST frame")
		GenerateCloud()
	elif i > 0:
		raw_input("Press any key to get another frame")
		GenerateCloud()
	else:
		raw_input("\n" + "Press any key to get the first frame")
		GenerateCloud()

with open("output/clouds.asc", "w") as f:
	for cloud in clouds:
		for i in range(0, len(cloud[0])):
			f.write(str(cloud[0][i]) + "," + str(cloud[1][i]) + "," + str(cloud[2][i]) + "\n")