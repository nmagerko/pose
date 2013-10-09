import numpy as np
from freenect import sync_get_depth as get_depth

def RawDepthToMeters(depthValue):
	if depthValue < 2047:
		return (1.0/float(depthValue)) * -0.0030711016 + 3.3309495161
	return 0.0

def DepthToWorld(x, y, depthValue):

	mtx = np.loadtxt("output/intrinsics9x6.txt")

	fx_d = 1.0 / mtx[0,0]
	fy_d = 1.0 / mtx[1,1]
	cx_d = mtx[0,2]
	cy_d = mtx[1,2]

	depth = RawDepthToMeters(depthValue)
	resultX = float((x - cx_d) * depth * fx_d)
	resultY = float((y - cy_d) * depth * fy_d)
	resultZ = float(depth)
	result = np.array([resultX, resultY, resultZ])
	return result

def DepthLoop():
	global depth
	(depth,_) = get_depth()
	
	#for i in range(0, 640):
	#	for j in range(0, 480):
	#
	## loop through the depth points (currently finding point 250,250)
	print(DepthToWorld(250, 250, depth[250,250]))

DepthLoop()
