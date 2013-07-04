#!/usr/bin/env python

import argparse
parser = argparse.ArgumentParser()
parser.add_argument("h5file")
parser.add_argument("--pattern")
args = parser.parse_args()

import h5py
import rapprentice.cv_plot_utils as cpu
import numpy as np
import cv2
import fnmatch

hdf = h5py.File(args.h5file,"r")
all_imgnames = [(np.asarray(seg["rgb"]),name) for (name,seg) in hdf.items() if (args.pattern is None) or fnmatch.fnmatch(name, args.pattern)]

nrows = 7
chunksize = nrows**2



for i in xrange(0,len(all_imgnames),chunksize):
    imgnames = all_imgnames[i:i+chunksize]
    imgs = []
    for (img, name) in imgnames:
        cv2.putText(img, name,(30,30), cv2.FONT_HERSHEY_PLAIN, 3, (255, 0, 255), thickness = 3) 

        imgs.append(img)
    bigimg = cpu.tile_images(imgs, nrows, nrows)
    cv2.imshow("bigimg", bigimg)
    cv2.waitKey()