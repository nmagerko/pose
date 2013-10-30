import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from math import tan
#import oct2py
from freenect import sync_get_depth as get_depth

x = []
y = []
z = []

def RawDepthToMeters(depthValue):
	# http://pille.iwr.uni-heidelberg.de/~kinect01/doc/classdescription.html#kinectcloud-section
	return 1/(depthValue * (-0.0030711016) + 3.3309495161)

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

def zoom_factory(ax,base_scale = 2.):
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

(depth,_) = get_depth()
mtx = np.loadtxt("output/intrinsics9x6.txt")
for i in range(0, 480, 5):
	for j in range(0, 640, 5):
		depthValue = depth[i,j]
		if depthValue < 2047:
			worldCoordinates = DepthToWorld(i, j, mtx, depthValue)
			x.append(worldCoordinates[0])
			y.append(worldCoordinates[1])
			z.append(worldCoordinates[2])

fig = plt.figure()
ax = Axes3D(fig)
ax.scatter(x, y, z)
f = zoom_factory(ax, base_scale=1.25)
plt.show()