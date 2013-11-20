#!/usr/bin/env python

import numpy as np
import cv2
import os
import sys
import freenect

# modified from opencv samples

square_size = 1.0

pattern_size = (9, 6)
pattern_points = np.zeros( (np.prod(pattern_size), 3), np.float32 )
pattern_points[:,:2] = np.indices(pattern_size).T.reshape(-1, 2)
pattern_points *= square_size

obj_points = []
img_points = []

h, w = 0, 0
i = 0
cv2.namedWindow('Calibration')
while i < 20:
    k = cv2.waitKey()
    if k%256 == 32:
        (img, _) = freenect.sync_get_video()
        if img is None:
          print "Failed to load"
          continue

        h, w = img.shape[:2]
        #vis = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        found, corners = cv2.findChessboardCorners(img, pattern_size)
        if found:
            term = ( cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 30, 0.1 )
            #cv2.cornerSubPix(img, corners, (5, 5), (-1, -1), term)
            
            cv2.drawChessboardCorners(img, pattern_size, corners, found)
            cv2.imwrite("output/calibration-images/calibration"+"-"+str(i+1)+".jpg", img)
        if not found:
            print 'chessboard not found'
            continue
        img_points.append(corners.reshape(-1, 2))
        obj_points.append(pattern_points)
        i += 1

        print 'ok'

rms, camera_matrix, dist_coefs, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, (w, h), None, None)
print "RMS:", rms
print "camera matrix:\n", camera_matrix
print "distortion coefficients: ", dist_coefs.ravel()
cv2.destroyAllWindows()

np.savetxt("output/intrinsics9x6.txt", camera_matrix)
np.savetxt("output/distortion9x6.txt", dist_coefs)