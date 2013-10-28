import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from freenect import sync_get_depth as get_depth

def RawDepthToMeters(depthValue):
	if depthValue < 2047:
		return (1.0/float(depthValue)) * -0.0030711016 + 3.3309495161
	return 0.0

def DepthToWorld(x, y, mtx, depthValue):

	fx_d = 1.0 / mtx[0,0]
	fy_d = 1.0 / mtx[1,1]
	cx_d = mtx[0,2]
	cy_d = mtx[1,2]

	depth = RawDepthToMeters(depthValue)
	resultX = float((x - cx_d) * depth * fx_d)
	resultY = float((y - cy_d) * depth * fy_d)
	resultZ = float(depth)
	result = [resultX, resultY, resultZ]
	return result

def DepthLoop():
	global depth
	(depth,_) = get_depth()
	mtx = np.loadtxt("output/intrinsics9x6.txt")
	x = []
	y = []
	z = []
	fig = plt.figure()
	ax = Axes3D(fig)

	for i in range(0, 480):
		for j in range(0, 640):
			depthValue = depth[i,j]
			if RawDepthToMeters(depthValue) > 0:
				worldCoordinates = DepthToWorld(i, j, mtx, depthValue)
				x.append(worldCoordinates[0])
				y.append(worldCoordinates[1])
				z.append(worldCoordinates[2])

	ax.scatter(x, y, z)
	plt.show()
	
	#w.tofile("output/d2w.txt", "\n")
	
DepthLoop()
