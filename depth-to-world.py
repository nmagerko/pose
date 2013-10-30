import numpy as np
import matplotlib.pyplot as plt
from math import tan
from mpl_toolkits.mplot3d import Axes3D
from freenect import sync_get_depth as get_depth

x = []
y = []
z = []

def RawDepthToMeters(depthValue):
	if depthValue < 2047:
		# http://pille.iwr.uni-heidelberg.de/~kinect01/doc/classdescription.html#kinectcloud-section
		return 1/(depthValue * (-0.0030711016) + 3.3309495161)
	return 0.0

def DepthToWorld(x, y, mtx, depthValue):

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

(depth,_) = get_depth()
mtx = np.loadtxt("output/intrinsics9x6.txt")
for i in range(0, 480, 5):
	for j in range(0, 640, 5):
		depthValue = depth[i,j]
		if RawDepthToMeters(depthValue) > 0:
			worldCoordinates = DepthToWorld(i, j, mtx, depthValue)
			x.append(worldCoordinates[0])
			y.append(worldCoordinates[1])
			z.append(worldCoordinates[2])

fig = plt.figure()
ax = Axes3D(fig)
ax.scatter(x, y, z)
plt.show()