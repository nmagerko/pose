###
# Using the data provided from the output of getdata, 
# this script splits the data into frames by the !!!
# character pattern, followed by the depth and RGB
# data, which is split by the $$$ pattern
#
# Jon Reynolds, Nick Magerko 2013
###

import sys
import pickle

filename = sys.argv[1]

with open(filename + '.pk', 'rb') as file_output:
    kinect_data = pickle.load(file_output)

print(kinect_data)
    
##f = file(filename, 'r')
##rawdata = f.readlines()
##f.close()

##data = ""
##images = []
##depth = []
##RGB= []
##
##for line in rawdata:
##    data = data + line.strip()
##
##splitimages = data.split('!!!')
##for image in splitimages:
##    images.append(image)
##
##
##for sensordata in images:
##    if sensordata != "":
##        try:
##            depth.append(sensordata.split('$$$')[0])
##            RGB.append(sensordata.split('$$$')[1])
##        except IndexError:
##	    pass
##
##print(str(depth) + "\n" + str(RGB))
