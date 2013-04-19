#!/usr/bin/env python

import cloudprocpy, cv2
import numpy as np



cmap = np.zeros((256, 3),dtype='uint8')
cmap[:,0] = range(256)
cmap[:,2] = range(256)[::-1]
cmap[0] = [0,0,0]

grabber = cloudprocpy.CloudGrabber()
grabber.startRGBD()


try:
    while True:
        rgb, depth = grabber.getRGBD()
        cv2.imshow("rgb", rgb)
        cv2.imshow("depth", cmap[np.fmin((depth*.064).astype('int'), 255)])
        cv2.waitKey(30)
except KeyboardInterrupt:
    print "got Control-C"
