import numpy as np
from math import tan
import cv2
import freenect
import time

dims = (9, 6) 					# 9x6 chessboard (see images directory)
npoints = dims[0] * dims[1]			# Number of points on chessboard

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
# 3d point in real world space
objpoints = np.zeros( (np.prod(dims), 3), np.float32 )
objpoints[:,:2] = np.indices(dims).T.reshape(-1, 2)

# rotation and translation vectors
rvec = []
tvec = []

# list to store each point cloud generated
clouds = []

# calibration output - from file
mtx = np.loadtxt("output/intrinsics9x6.txt")
dst = np.loadtxt("output/distortion9x6.txt")

def CalculateRT(image):
	# convert to grayscale
	gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	# attempt to find the chessboard corners
	ret, corners = cv2.findChessboardCorners(gray, dims)

	# if found, refine the corners' location, and solve for rotation and translation
	if ret:
		cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)

		# create matrix to match shape required by solvePnP
		# 2d points in image plane.
		imgpoints = corners.reshape(npoints, 2)

		ret, rvec, tvec = cv2.solvePnP(objpoints, imgpoints, mtx, dst)

		# Calculate rotation matrix
		rmtx = cv2.Rodrigues(rvec)[0]

		# return the rotation and translation
		return rmtx, tvec
	else:
		# otherwise, return both as "Error"
		return "Error", "Error"

def RawDepthToMeters(depthValue):
	# http://pille.iwr.uni-heidelberg.de/~kinect01/doc/classdescription.html#kinectcloud-section
	return 1/(depthValue * (-0.0030711016) + 3.3309495161)

# converts the depth data from the Kinect to meters
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
	while (True):
		image = None
		depth = None
		rotate = None
		translate = None
		# get a frame using libfreenect
		(image, _) = freenect.sync_get_video()
		(depth,_) = freenect.sync_get_depth()
		rotate, translate = CalculateRT(image)
		if rotate != "Error":
			break;
		else:
			print "Grid not found"
	print "Camera Center: "
	print (np.dot(-np.transpose(rotate),translate))
	print "Rotation: "
	print rotate
	print "Translation :"
	print translate
	# Collects 12288 points/frame
	worldCoordinates = np.arange(36864, dtype = np.float64).reshape(96, 128, 3)
	for i in range(0, 480, 5):
		for j in range(0, 640, 5):
			depthValue = depth[i,j]
			if depthValue < 2047:
				# if the depth value is small enough, convert it
				# to depth in meters
				values = DepthToWorld(i, j, depthValue)
				worldCoordinates[i/5, j/5, 0] = values[0]
				worldCoordinates[i/5, j/5, 1] = values[1]
				worldCoordinates[i/5, j/5, 2] = values[2]
			else:
				# otherwise, assign the value to zero
				worldCoordinates[i/5, j/5, 0] = 0
				worldCoordinates[i/5, j/5, 1] = 0
				worldCoordinates[i/5, j/5, 2] = 0
	x, y, z = [], [], []
	for row in worldCoordinates:
		for point in row:
			if str(point) != "[ 0.  0.  0.]":
				# apply the rotation and translation to each x, y and z coordinate
				point = rotate.dot(np.array(point).reshape(3, 1)) + translate
				x.append(float(point[0]))
				y.append(float(point[1]))
				z.append(float(point[2]))
	# append the x, y and z lists to clouds as a list of lists,
	# creating a point cloud
	clouds.append([x,y,z])

print "\n" + "KINECT ONLINE \n"
for i in range(0, 5):
	GenerateCloud()

# output point clouds so that meshlab can read them
with open("output/clouds.asc", "w") as f:
	for cloud in clouds:
		for i in range(0, len(cloud[0])):
			f.write(str(cloud[0][i]) + "," + str(cloud[1][i]) + "," + str(cloud[2][i]) + "\n")
