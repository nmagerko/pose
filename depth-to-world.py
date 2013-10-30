import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from math import tan
import cv2
from freenect import sync_get_depth as get_depth

x = []
y = []
z = []

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

# Calibration output
mtx = np.loadtxt("output/intrinsics" + str(dims[0])+"x"+str(dims[1]) + ".txt")
dst = np.loadtxt("output/distortion" + str(dims[0])+"x"+str(dims[1]) + ".txt")

def CalculateRT():
	successes = 0					# Number of successful collections
	while True and successes != boards:
		capture = None

		# Capture an image
		capture = cv2.VideoCapture(cv2.CAP_ANY)
		# Get a frame while we have less than 8 successes
		_, image = capture.read()
		if (image.any()):
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
	
	result = np.array([[resultX], [resultY], [resultZ]])
	return result

def zoom_factory(ax, base_scale = 2.):
    def zoom_fun(event):
        # get the current x and y limits
        cur_xlim = ax.get_xlim()
        cur_ylim = ax.get_ylim()
        cur_xrange = (cur_xlim[1] - cur_xlim[0])*.5
        cur_yrange = (cur_ylim[1] - cur_ylim[0])*.5
        xdata = event.xdata # get event x location
        ydata = event.ydata # get event y location
        if event.button == 'up':
            # deal with zoom in
            scale_factor = 1/base_scale
        elif event.button == 'down':
            # deal with zoom out
            scale_factor = base_scale
        else:
            # deal with something that should never happen
            scale_factor = 1
            print event.button
        # set new limits
        ax.set_xlim([xdata - cur_xrange*scale_factor,
                     xdata + cur_xrange*scale_factor])
        ax.set_ylim([ydata - cur_yrange*scale_factor,
                     ydata + cur_yrange*scale_factor])
        plt.draw() # force re-draw

    fig = ax.get_figure() # get the figure of interest
    # attach the call back
    fig.canvas.mpl_connect('scroll_event',zoom_fun)

    #return the function
    return zoom_fun

def GenerateCloud():
	print("Getting depth")
	(depth,_) = get_depth()
	print("Got depth")
	for i in range(0, 480, 5):
		for j in range(0, 640, 5):
			depthValue = depth[i,j]
			if depthValue < 2047:
				worldCoordinates = DepthToWorld(i, j, depthValue)
				rotate, translate = CalculateRT()
				print("Got RT")
				newWorldCoordinates = rotate * worldCoordinates + translate
				#print(newWorldCoordinates)
				#x.append(newWorldCoordinates[0])
				#y.append(newWorldCoordinates[1])
				#z.append(newWorldCoordinates[2])

	fig = plt.figure()
	ax = Axes3D(fig)
	ax.scatter(x, y, z)
	f = zoom_factory(ax, base_scale=1.25)
	plt.show()

for i in range(0, 2):
	print("Program started")
	GenerateCloud()
