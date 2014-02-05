#!/usr/bin/env python

import freenect
import numpy as np

def RawDepthToMeters(depthValue):
    # http://pille.iwr.uni-heidelberg.de/~kinect01/doc/classdescription.html#kinectcloud-section
    return 1/(depthValue * (-0.0030711016) + 3.3309495161)

def DepthToWorld(x, y, depthValue):

    fx_d = 5.9421434211923247e+02
    fy_d = 5.9104053696870778e+02
    cx_d = 3.3930780975300314e+02
    cy_d = 2.4273913761751615e+02

    depth = RawDepthToMeters(depthValue)
    # http://pille.iwr.uni-heidelberg.de/~kinect01/doc/classdescription.html#kinectcloud-section
    resultZ = float(depth)
    resultX = float((x - cx_d) * resultZ)/fx_d
    resultY = float((y - cy_d) * resultZ)/fy_d
    
    result = [resultX, resultY, resultZ]
    return result

def GenerateCloud():
    (depth,_) = freenect.sync_get_depth()
    # Collects 4800 points/frame
    worldCoordinates = np.arange(14400, dtype = np.float64).reshape(60, 80, 3)
    for i in range(0, 480, 8):
        for j in range(0, 640, 8):
            depthValue = depth[i,j]
            if depthValue < 2047:
                # if the depth value is small enough, convert it
                # to depth in meters
                values = DepthToWorld(i, j, depthValue)
                worldCoordinates[i/8, j/8, 0] = values[0]
                worldCoordinates[i/8, j/8, 1] = values[1]
                worldCoordinates[i/8, j/8, 2] = values[2]
            else:
                # otherwise, assign the value to zero
                worldCoordinates[i/8, j/8, 0] = 0
                worldCoordinates[i/8, j/8, 1] = 0
                worldCoordinates[i/8, j/8, 2] = 0
    cloud = []
    for row in worldCoordinates:
        for point in row:
            if str(point) != "[ 0.  0.  0.]":
                cloud.append([point[0],point[1],point[2]])
    return cloud

print "\n" + "KINECT ONLINE"
pointCloud = GenerateCloud()
rowCount = len(pointCloud)
columnCount = 3

filename = raw_input("Cloud generated. Enter filename for output (e.g., RANSAC.mat): \t")
print "Saving to output/" + filename + "... \t",
with open("output/RANSAC.mat", "w") as f:
    f.write("# Created by Nick Magerko and Jon Reynolds\n")
    f.write("# name: cloud\n")
    f.write("# type: matrix\n")
    f.write("# rows: " + str(rowCount) + "\n")
    f.write("# columns: " + str(columnCount) + "\n")
    for point in pointCloud:
        f.write(" " + str(point[0]) + " " + str(point[1]) + " " + str(point[2]) + "\n")
print "DONE \n"
