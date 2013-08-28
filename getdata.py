###
# Using the depth and RGB data provided from the kinect,
# this program outputs each frame into a file which 
# separates frames by a !!! character pattern, and 
# the depth/rgb values by a $$$ character pattern.
#
# WARNING: this program will overwrite any file that
# is given as the output name
#
# Nick Magerko, Jon Reynolds 2013
###

from freenect import sync_get_depth as get_depth, sync_get_video as get_video
import cv  
import numpy as np
import sys
  
filename = sys.argv[1]

def getData():
    global depth, rgb, data 
    data = ""

    i = 0
    #for the first ten frames
    while i < 10:
        # Get a fresh frame
		(depth,_), (rgb,_) = get_depth(), get_video()

		data = data + str(depth) + " \n $$$ \n" + str(rgb) + "\n !!! \n"

		# Build a two panel color image
		d3 = np.dstack((depth,depth,depth)).astype(np.uint8)
		da = np.hstack((d3,rgb))

		# Simple Downsample
		cv.ShowImage('Depth and RGB',cv.fromarray(np.array(da[::2,::2,::-1])))
		cv.WaitKey(5)
		i = i + 1
        
getData()
output = file(filename, 'w')
output.write(data)
output.close()
